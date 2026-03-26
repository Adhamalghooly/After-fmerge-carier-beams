/**
 * slabFEMEngine – Phase 4: Stress-Based Edge Load Transfer
 * ══════════════════════════════════════════════════════════
 *
 * Computes slab-to-beam load transfer using the fundamental stress formula:
 *
 *   t_z = −(Qx · n_x + Qy · n_y)          [kN/m]
 *
 * and 1-D Gauss integration along each element edge:
 *
 *   f_edge = ∫ N^T · t_z  dL              [N]
 *
 * where
 *   Qx, Qy  = Mindlin transverse shear resultants   [kN/m]
 *   n       = outward normal to the element edge     (dimensionless)
 *   N       = linear edge shape functions            [N₁=(1−ξ)/2, N₂=(1+ξ)/2]
 *   L       = element edge length                    [mm]
 *
 * Unit algebra: [kN/m] × [mm] = [N]  (exact — no extra conversion needed).
 *
 * ── Stress Smoothing ────────────────────────────────────────────────────────
 * Raw Gauss-point stresses (4 per element) are smoothed to nodes by simple
 * nodal averaging: each node accumulates the stresses of all surrounding
 * elements and divides by the count.  This is the minimum requirement for
 * stable edge extraction (avoids Gauss-point aliasing at boundaries).
 *
 * ── Sign Convention ─────────────────────────────────────────────────────────
 *   t_z > 0  → downward load on beam  (gravity direction, consistent with Phase 2)
 *   t_z = −(Q · n)  because:
 *     • Q · n  is the shear traction ON THE SLAB from outside
 *     • If support pushes slab UP: Q·n < 0 (inward) → −(Q·n) > 0 on beam ✓
 *
 * ── Output Format ────────────────────────────────────────────────────────────
 * Returns BeamEdgeForces[] — EXACTLY the same interface consumed by Phase 3
 * (mapEdgeForcesToBeams).  Phase 3 remains unchanged.
 *
 * ── Key Difference from Phase 2 ─────────────────────────────────────────────
 * Phase 2 (reaction-based):  works only where DOFs are constrained (beam nodes).
 * Phase 4 (stress-based):    works for ANY edge — including FREE edges where
 *                            Phase 2 gives zero by definition.
 */

import type { SlabMesh, FEMElement, FEMNode, Beam, SlabProps, MatProps } from './types';
import type { BeamEdgeForces, BeamNodeReaction } from './edgeForces';
import { computeInternalForces } from './internalForces';
import type { ElementForceResult } from './types';

// ─────────────────────────────────────────────────────────────────────────────
// Internal types
// ─────────────────────────────────────────────────────────────────────────────

interface NodeStress {
  Qx: number;   // kN/m
  Qy: number;   // kN/m
  Mx: number;   // kN·m/m  (kept for potential Phase-5 use)
  My: number;
  Mxy: number;
  count: number;
}

const EPS = 1e-3; // mm — coordinate matching tolerance

// ─────────────────────────────────────────────────────────────────────────────
// Step 1 – Stress smoothing: Gauss-point → nodes
// ─────────────────────────────────────────────────────────────────────────────

/**
 * For each element:
 *   • 4 Gauss points provide Qx, Qy at their physical coordinates (x, y in mm).
 *   • We assign the ELEMENT AVERAGE to all 4 of its corner nodes.
 *   • After all elements, each node divides by contribution count (simple average).
 *
 * This is Nodal Averaging — the minimum valid smoothing per the prompt.
 */
function smoothStressesToNodes(
  mesh:         SlabMesh,
  gaussResults: ElementForceResult[],
): Map<number, NodeStress> {
  // nodeId → accumulator
  const acc = new Map<number, NodeStress>();

  const initNode = (): NodeStress => ({ Qx: 0, Qy: 0, Mx: 0, My: 0, Mxy: 0, count: 0 });
  const ensure = (id: number) => {
    if (!acc.has(id)) acc.set(id, initNode());
    return acc.get(id)!;
  };

  // Group Gauss results by elementId
  const byElem = new Map<number, ElementForceResult[]>();
  for (const r of gaussResults) {
    const list = byElem.get(r.elementId) ?? [];
    list.push(r);
    byElem.set(r.elementId, list);
  }

  for (const elem of mesh.elements) {
    const gpList = byElem.get(elem.id);
    if (!gpList || gpList.length === 0) continue;

    // Element average stress
    let avgQx = 0, avgQy = 0, avgMx = 0, avgMy = 0, avgMxy = 0;
    for (const gp of gpList) {
      avgQx  += gp.resultants.Qx;
      avgQy  += gp.resultants.Qy;
      avgMx  += gp.resultants.Mx;
      avgMy  += gp.resultants.My;
      avgMxy += gp.resultants.Mxy;
    }
    const n = gpList.length;
    avgQx /= n; avgQy /= n; avgMx /= n; avgMy /= n; avgMxy /= n;

    // Distribute to all 4 corner nodes
    for (const nodeId of elem.nodeIds) {
      const nd = ensure(nodeId);
      nd.Qx  += avgQx;
      nd.Qy  += avgQy;
      nd.Mx  += avgMx;
      nd.My  += avgMy;
      nd.Mxy += avgMxy;
      nd.count++;
    }
  }

  // Finalise averages
  for (const [, nd] of acc) {
    if (nd.count > 0) {
      nd.Qx  /= nd.count;
      nd.Qy  /= nd.count;
      nd.Mx  /= nd.count;
      nd.My  /= nd.count;
      nd.Mxy /= nd.count;
    }
  }

  return acc;
}

// ─────────────────────────────────────────────────────────────────────────────
// Step 2 – Edge detection helpers
// ─────────────────────────────────────────────────────────────────────────────

/**
 * 4 edges per element.  Index maps to:
 *   0 → bottom (n0→n1), outward n = (0, −1)
 *   1 → right  (n1→n2), outward n = (+1, 0)
 *   2 → top    (n2→n3), outward n = (0, +1)
 *   3 → left   (n3→n0), outward n = (−1, 0)
 * Nodes are in CCW order: n0=(xmin,ymin), n1=(xmax,ymin), n2=(xmax,ymax), n3=(xmin,ymax)
 */
const EDGE_DEF = [
  { iA: 0, iB: 1, nx:  0, ny: -1 },   // bottom
  { iA: 1, iB: 2, nx:  1, ny:  0 },   // right
  { iA: 2, iB: 3, nx:  0, ny:  1 },   // top
  { iA: 3, iB: 0, nx: -1, ny:  0 },   // left
] as const;

/**
 * Returns true if both edge endpoints lie on the beam line (horizontal or vertical).
 * Coordinates in mm, matching tolerance EPS.
 */
function edgeLiesOnBeam(
  na: FEMNode, nb: FEMNode,
  beam: Beam,
): boolean {
  if (beam.direction === 'horizontal') {
    const by = beam.y1;
    return Math.abs(na.y - by) < EPS && Math.abs(nb.y - by) < EPS
        && Math.min(na.x, nb.x) >= Math.min(beam.x1, beam.x2) - EPS
        && Math.max(na.x, nb.x) <= Math.max(beam.x1, beam.x2) + EPS;
  } else {
    const bx = beam.x1;
    return Math.abs(na.x - bx) < EPS && Math.abs(nb.x - bx) < EPS
        && Math.min(na.y, nb.y) >= Math.min(beam.y1, beam.y2) - EPS
        && Math.max(na.y, nb.y) <= Math.max(beam.y1, beam.y2) + EPS;
  }
}

/**
 * Signed position of a node along the beam axis (mm from beam start).
 */
function posAlongBeam(node: FEMNode, beam: Beam): number {
  if (beam.direction === 'horizontal') {
    return node.x - Math.min(beam.x1, beam.x2);
  } else {
    return node.y - Math.min(beam.y1, beam.y2);
  }
}

// ─────────────────────────────────────────────────────────────────────────────
// Steps 3 + 4 – Compute edge traction and integrate
// ─────────────────────────────────────────────────────────────────────────────

const GP1D = 1 / Math.sqrt(3); // Gauss point for 2-point rule
const GAUSS_1D = [
  { xi: -GP1D, w: 1.0 },
  { xi:  GP1D, w: 1.0 },
] as const;

/**
 * Computes the equivalent nodal forces at two edge nodes (na, nb) due to
 * traction t_z along the edge using 2-point Gauss quadrature.
 *
 * t_z [kN/m], L [mm] → f [N]   (unit exact: kN/m × mm = N)
 */
function integrateEdge(
  tz_a: number, tz_b: number,
  L_mm: number,
): { fa: number; fb: number } {
  let fa = 0, fb = 0;
  const half = L_mm / 2;

  for (const gp of GAUSS_1D) {
    const N1 = (1 - gp.xi) / 2;   // shape function at node a
    const N2 = (1 + gp.xi) / 2;   // shape function at node b

    const tz = N1 * tz_a + N2 * tz_b;   // interpolated traction

    fa += N1 * tz * gp.w * half;
    fb += N2 * tz * gp.w * half;
  }

  return { fa, fb };
}

// ─────────────────────────────────────────────────────────────────────────────
// Public: Phase-4 entry point
// ─────────────────────────────────────────────────────────────────────────────

/**
 * extractStressEdgeForces
 * ────────────────────────
 * Returns BeamEdgeForces[] in the same format as Phase 2 (extractBeamEdgeForces).
 *
 * Input coordinates: MILLIMETRES (the caller scales from metres if necessary).
 * Output: nodal forces in NEWTONS, positions in mm (same as Phase 2).
 */
export function extractStressEdgeForces(
  mesh:      SlabMesh,
  d_full:    number[],
  slabProps: SlabProps,
  mat:       MatProps,
  beams:     Beam[],
): BeamEdgeForces[] {

  // ── Step 1: Gauss-point stresses ────────────────────────────────────────
  const gaussResults = computeInternalForces(mesh, d_full, slabProps, mat);

  // ── Step 1b: Smooth to nodes ─────────────────────────────────────────────
  const nodeStress = smoothStressesToNodes(mesh, gaussResults);

  // Fast lookup: nodeId → FEMNode
  const nodeById = new Map<number, FEMNode>();
  for (const nd of mesh.nodes) nodeById.set(nd.id, nd);

  // ── Step 2–4: For each beam, scan element edges ──────────────────────────

  // Accumulator: beamId → { posAlongBeam → Fz_N }
  const beamAcc = new Map<string, Map<number, number>>();

  for (const beam of beams) {
    if (!beam.slabs.includes(mesh.slabId)) continue;
    beamAcc.set(beam.id, new Map<number, number>());
  }

  if (beamAcc.size === 0) return [];

  for (const elem of mesh.elements) {
    const nodeIds = elem.nodeIds;

    for (const edgeDef of EDGE_DEF) {
      const idA = nodeIds[edgeDef.iA];
      const idB = nodeIds[edgeDef.iB];
      const na  = nodeById.get(idA)!;
      const nb  = nodeById.get(idB)!;

      // Edge length [mm]
      const L_mm = Math.hypot(nb.x - na.x, nb.y - na.y);
      if (L_mm < EPS) continue;

      // Check all relevant beams
      for (const [beamId, posMap] of beamAcc) {
        const beam = beams.find(b => b.id === beamId)!;

        if (!edgeLiesOnBeam(na, nb, beam)) continue;

        // Outward normal from element (points toward beam)
        const nx = edgeDef.nx;
        const ny = edgeDef.ny;

        // Smoothed shear at edge nodes [kN/m]
        const sa = nodeStress.get(idA) ?? { Qx: 0, Qy: 0 };
        const sb = nodeStress.get(idB) ?? { Qx: 0, Qy: 0 };

        // Traction ON BEAM = −(Q · n)  [kN/m]
        //   Q·n < 0 (inward, beam supports slab up) → −(Q·n) > 0 (downward load on beam) ✓
        const tz_a = -(sa.Qx * nx + sa.Qy * ny);
        const tz_b = -(sb.Qx * nx + sb.Qy * ny);

        // 2-point Gauss integration → nodal forces [N]
        const { fa, fb } = integrateEdge(tz_a, tz_b, L_mm);

        // Accumulate by position along beam (mm)
        const posA = posAlongBeam(na, beam);
        const posB = posAlongBeam(nb, beam);

        posMap.set(posA, (posMap.get(posA) ?? 0) + fa);
        posMap.set(posB, (posMap.get(posB) ?? 0) + fb);
      }
    }
  }

  // ── Step 5: Build BeamEdgeForces[] ─────────────────────────────────────
  const results: BeamEdgeForces[] = [];

  for (const [beamId, posMap] of beamAcc) {
    if (posMap.size === 0) continue;
    const beam = beams.find(b => b.id === beamId)!;

    const span_mm = beam.direction === 'horizontal'
      ? Math.abs(beam.x2 - beam.x1)
      : Math.abs(beam.y2 - beam.y1);

    const reactions: BeamNodeReaction[] = [];

    for (const [pos_mm, Fz_N] of posMap) {
      // Approximate tributary length (refined by Phase 3 Voronoi step)
      const tribLen = span_mm / Math.max(posMap.size, 1);
      reactions.push({
        nodeId:       -1,              // stress method — no mesh nodeId
        posAlongBeam: pos_mm,
        tributaryLen: tribLen,
        Fz_N,
        w_kNm:        tribLen > EPS ? Fz_N / tribLen : 0,
      });
    }

    reactions.sort((a, b) => a.posAlongBeam - b.posAlongBeam);

    const totalForce_N = reactions.reduce((s, r) => s + r.Fz_N, 0);
    results.push({ beamId, reactions, totalForce_N });
  }

  return results;
}

// ─────────────────────────────────────────────────────────────────────────────
// Debug summary helper
// ─────────────────────────────────────────────────────────────────────────────

export function summariseStressExtraction(
  results: BeamEdgeForces[],
  totalApplied_kN: number,
): void {
  const totalBeam_kN = results.reduce((s, r) => s + r.totalForce_N * 1e-3, 0);
  const err = totalApplied_kN > 1e-6
    ? Math.abs(totalApplied_kN - totalBeam_kN) / totalApplied_kN * 100
    : 0;

  console.group('[Phase 4] Stress-Based Edge Transfer');
  console.log(`  Applied = ${totalApplied_kN.toFixed(2)} kN`);
  console.log(`  Beam loads total = ${totalBeam_kN.toFixed(2)} kN`);
  console.log(`  Equilibrium error = ${err.toFixed(2)} %`);
  for (const r of results) {
    console.log(
      `  Beam ${r.beamId}: ${(r.totalForce_N * 1e-3).toFixed(2)} kN  ` +
      `(${r.reactions.length} nodes)`,
    );
  }
  console.groupEnd();
}
