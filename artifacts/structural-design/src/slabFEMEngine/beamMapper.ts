/**
 * slabFEMEngine – Beam Load Mapper (Phase 3)
 *
 * Converts the discrete nodal reaction samples produced by Phase 2 into the
 * BeamLoadResult format required by the public API.
 *
 * Steps
 * -----
 * 1. Collect all BeamEdgeForces contributions for each beam (a beam may
 *    border multiple slabs — contributions are summed).
 * 2. Merge all reactions, sorted by position along the beam.
 * 3. Build the distributed load profile w(x) [kN/m].
 * 4. Optionally compare against the existing tributary-area method.
 *
 * Phase-3 validation (logged to console)
 * ---------------------------------------
 * • Total load from w(x) distribution must match the Phase-2 edge-force
 *   total within 1 % (numerical integration consistency).
 */

import type {
  BeamLoadResult, DistributedLoadPoint, NodalForce,
} from './types';
import type { BeamEdgeForces } from './edgeForces';
import type { Beam, SlabProps, MatProps } from './types';
import { calculateBeamLoads, type Slab } from '@/lib/structuralEngine';

// ─────────────────────────────────────────────────────────────────────────────

export interface MapperOptions {
  comparisonMode: boolean;
  slabs:          Slab[];
  slabProps:      SlabProps;
  mat:            MatProps;
}

// ─────────────────────────────────────────────────────────────────────────────

export function mapEdgeForcesToBeams(
  allEdgeForces: BeamEdgeForces[],
  beams:         Beam[],
  opts:          MapperOptions,
): BeamLoadResult[] {
  const results: BeamLoadResult[] = [];

  // Group edge-force contributions by beam
  const byBeam = new Map<string, BeamEdgeForces[]>();
  for (const ef of allEdgeForces) {
    const list = byBeam.get(ef.beamId) ?? [];
    list.push(ef);
    byBeam.set(ef.beamId, list);
  }

  for (const beam of beams) {
    const contributions = byBeam.get(beam.id);
    if (!contributions || contributions.length === 0) continue;

    // ── 1. Merge all reactions for this beam ────────────────────────────────
    // A beam can border 2 slabs → sum their reaction contributions at each pos
    const allReactions = contributions.flatMap(ef => ef.reactions);
    allReactions.sort((a, b) => a.posAlongBeam - b.posAlongBeam);

    if (allReactions.length === 0) continue;

    const beamLen_mm = beam.length * 1000;   // m → mm

    // ── 2. Build w(x) distributed load profile ─────────────────────────────
    // Unique positions along the beam
    const positions = uniqueSorted(allReactions.map(r => r.posAlongBeam));

    // Ensure beam start and end are included
    if (positions[0] > 1e-3)                        positions.unshift(0);
    if (positions[positions.length - 1] < beamLen_mm - 1e-3) positions.push(beamLen_mm);

    const wPoints:  DistributedLoadPoint[] = [];
    const nPoints:  NodalForce[]           = [];

    for (let i = 0; i < positions.length; i++) {
      const pos_mm = positions[i];

      // Tributary half-widths on either side
      const halfL = i === 0
        ? 0
        : (pos_mm - positions[i - 1]) / 2;
      const halfR = i === positions.length - 1
        ? 0
        : (positions[i + 1] - pos_mm) / 2;

      const tribLen = halfL + halfR;

      // Sum Fz contributions in the tributary zone around this position
      let Fz_sum_N = 0;
      for (const r of allReactions) {
        if (
          r.posAlongBeam >= pos_mm - halfL - 1e-3 &&
          r.posAlongBeam <  pos_mm + halfR + 1e-3
        ) {
          Fz_sum_N += r.Fz_N;
        }
      }

      // w = force per unit length  [N/mm = kN/m]
      const w_kNm = tribLen > 1e-3 ? Fz_sum_N / tribLen : Fz_sum_N;

      wPoints.push({
        position: pos_mm / 1000,    // mm → m
        w:        w_kNm,
      });

      // Nodal forces at key positions (ends + concentrated)
      if (i === 0 || i === positions.length - 1) {
        nPoints.push({
          position: pos_mm / 1000,
          Fz:       Fz_sum_N * 1e-3,   // N → kN
          My:       0,                  // moment transfer not implemented in Phase 2
        });
      }
    }

    // ── 3. Phase-3 self-consistency log ────────────────────────────────────
    const totalEdge_kN  = contributions.reduce((s, ef) => s + ef.totalForce_N * 1e-3, 0);
    const femAvg        = wPoints.reduce((s, p) => s + p.w, 0) / Math.max(wPoints.length, 1);
    const femPeak       = Math.max(...wPoints.map(p => Math.abs(p.w)));

    const totalFEMCheck = trapz(
      wPoints.map(p => p.position),
      wPoints.map(p => p.w),
    ); // kN (trapezoidal)
    const err = totalEdge_kN > 1e-6
      ? Math.abs(totalFEMCheck - totalEdge_kN) / totalEdge_kN * 100
      : 0;

    console.log(
      `[Phase 3] Beam ${beam.id}: ` +
      `FEM distributed total = ${totalFEMCheck.toFixed(2)} kN, ` +
      `reaction sum = ${totalEdge_kN.toFixed(2)} kN, ` +
      `self-check err = ${err.toFixed(2)} %`,
    );

    // ── 4. Comparison mode ─────────────────────────────────────────────────
    let oldMethodLoad: { deadLoad: number; liveLoad: number } | undefined;
    let differencePercent: number | undefined;

    if (opts.comparisonMode) {
      const old = calculateBeamLoads(beam, opts.slabs, opts.slabProps, opts.mat);
      oldMethodLoad = old;

      const oldTotal_kN = (old.deadLoad + old.liveLoad) * beam.length;
      differencePercent = oldTotal_kN > 1e-6
        ? ((totalFEMCheck - oldTotal_kN) / oldTotal_kN) * 100
        : 0;

      console.log(
        `[Phase 3] Beam ${beam.id} comparison: ` +
        `OLD = ${oldTotal_kN.toFixed(2)} kN, ` +
        `FEM = ${totalFEMCheck.toFixed(2)} kN, ` +
        `diff = ${differencePercent.toFixed(1)} %`,
      );
    }

    results.push({
      beamId: beam.id,
      loads: {
        type:   'distributed',
        values: wPoints,
      },
      nodalForces: nPoints,
      oldMethodLoad,
      femMethodLoad: { avgLoad: femAvg, peakLoad: femPeak },
      differencePercent,
    });
  }

  return results;
}

// ─────────────────────────────────────────────────────────────────────────────
// Helpers
// ─────────────────────────────────────────────────────────────────────────────

function uniqueSorted(arr: number[]): number[] {
  // Round to 0.1 mm to merge nearly-coincident points
  return [...new Set(arr.map(v => Math.round(v * 10) / 10))]
    .sort((a, b) => a - b);
}

/** Trapezoidal integration. x in metres, y in kN/m → result in kN. */
function trapz(x: number[], y: number[]): number {
  let s = 0;
  for (let i = 0; i < x.length - 1; i++) {
    s += 0.5 * (y[i] + y[i + 1]) * (x[i + 1] - x[i]);
  }
  return s;
}
