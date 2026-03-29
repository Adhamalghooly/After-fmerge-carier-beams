/**
 * استخراج أحمال الأعمدة من التحليل ثلاثي الأبعاد (3D Frame Analysis)
 * لاستخدامها في التصميم بدلاً من الطريقة التقريبية (2D)
 *
 * المحاور: للأعمدة الرأسية:
 *   - Local Y = Global X → momentY = Mx (عزم حول المحور العالمي X)
 *   - Local Z = Global Y → momentZ = My (عزم حول المحور العالمي Y)
 *   - nodeI = أسفل العمود (Bot), nodeJ = أعلى العمود (Top)
 */

import type { Beam, Column, Frame, FrameResult, MatProps, BeamOnBeamConnection, Slab, SlabProps } from '@/lib/structuralEngine';
import { analyze3DFrame, type Node3D, type Element3D, type Model3D, type LoadCase3D } from '@/lib/solver3D';

export interface ColumnLoads3D {
  Pu: number;
  PuMin: number;   // min axial (may be tension for edge columns under eccentric live load)
  Mx: number;   // max |momentY| (global X moment)
  My: number;   // max |momentZ| (global Y moment)
  MxTop: number; // momentY at top
  MxBot: number; // momentY at bottom
  MyTop: number; // momentZ at top
  MyBot: number; // momentZ at bottom
  Vu: number;    // max shear
}

interface BeamEnvelope3D {
  shearYMax: number;
  shearYI: number;
  shearYJ: number;
  momentZI: number;
  momentZJ: number;
  momentZmid: number;
}

interface ColumnEnvelope3D {
  axialMax: number; // max compression (positive)
  axialMin: number; // min (may be tension — negative)
  shearMax: number;
  momentYI: number;
  momentYJ: number;
  momentYmax: number;
  momentZI: number;
  momentZJ: number;
  momentZmax: number;
}

type EndReleaseMap = Record<string, {
  nodeI: { ux: boolean; uy: boolean; uz: boolean; rx: boolean; ry: boolean; rz: boolean };
  nodeJ: { ux: boolean; uy: boolean; uz: boolean; rx: boolean; ry: boolean; rz: boolean };
}>;

/**
 * Build the 3D global stiffness model with pattern loading cases.
 *
 * Beam-on-Beam handling (ETABS-equivalent):
 * For each beam-on-beam connection the PRIMARY (carrier) beam is split at the
 * bearing point into two sub-elements sharing an intermediate node.  The
 * SECONDARY (carried) beams have their removed-column end reconnected to that
 * same intermediate node, and a moment release (hinge) is applied there so
 * only shear is transferred — exactly as ETABS models a Gerber beam.
 * This is a true FEM solution: both distributed loads AND the carried beam
 * reaction are resolved simultaneously in the global stiffness matrix.
 * No iteration or approximation is needed.
 */
function build3DModelWithPatternLoading(
  frames: Frame[],
  beams: Beam[],
  columns: Column[],
  mat: MatProps,
  frameEndReleases?: EndReleaseMap,
  beamOnBeamConnections?: BeamOnBeamConnection[],
  slabs?: Slab[],
  slabProps?: SlabProps,
): { model: Model3D; patternCases: LoadCase3D[]; primaryBeamSplitIds: Map<string, string> } {
  const beamsMap = new Map(beams.map(b => [b.id, b]));
  const E = 4700 * Math.sqrt(mat.fc) * 1000; // MPa → kPa (kN/m²) — consistent with kN/m loads
  const G = E / (2 * (1 + 0.2));

  const nodesMap = new Map<string, Node3D>();
  const elements3d: Element3D[] = [];

  // Helper: get or create node by position
  const getOrCreateNode = (
    x: number,
    y: number,
    z: number,
    restraints: [boolean, boolean, boolean, boolean, boolean, boolean],
  ): string => {
    const key = `N_${x.toFixed(0)}_${y.toFixed(0)}_${z.toFixed(0)}`;
    if (!nodesMap.has(key)) {
      nodesMap.set(key, { id: key, x, y, z, restraints });
    }
    return key;
  };

  // Determine ground level
  let minZ = Infinity;
  for (const col of columns) {
    if (col.isRemoved) continue;
    const zBot = col.zBottom ?? 0;
    if (zBot < minZ) minZ = zBot;
  }

  const colTopNodeMap = new Map<string, string>();

  for (const col of columns) {
    if (col.isRemoved) continue;
    const zBot = col.zBottom ?? 0;
    const zTop = col.zTop ?? (zBot + col.L);
    const xMm = col.x * 1000;
    const yMm = col.y * 1000;

    const isGroundLevel = Math.abs(zBot - minZ) < 1;
    let botRestraints: [boolean, boolean, boolean, boolean, boolean, boolean];
    if (isGroundLevel) {
      const isPinned = col.bottomEndCondition === 'P';
      botRestraints = isPinned
        ? [true, true, true, false, false, false]
        : [true, true, true, true, true, true];
    } else {
      botRestraints = [false, false, false, false, false, false];
    }

    const botId = getOrCreateNode(xMm, yMm, zBot, botRestraints);
    const topId = getOrCreateNode(xMm, yMm, zTop, [false, false, false, false, false, false]);

    colTopNodeMap.set(col.id, topId);

    elements3d.push({
      id: `col_${col.id}`,
      type: 'column',
      nodeI: botId,
      nodeJ: topId,
      b: col.b,
      h: col.h,
      E,
      G,
      wLocal: { wx: -1.2 * mat.gamma * (col.b * col.h) / 1e6, wy: 0, wz: 0 },
      stiffnessModifier: 0.65,
    });
  }

  // ── Build beam elements ──────────────────────────────────────────────────
  // We keep track of per-element dead/live (factored UDL) for load cases.
  // Key = element id (possibly `beam_X_A` / `beam_X_B` for split elements).
  const beamDeadLoads = new Map<string, number>(); // 1.2*wD UDL (kN/m)
  const beamLiveLoads = new Map<string, number>(); // 1.6*wL UDL (kN/m)
  // Ordered per-frame list of element IDs for per-frame pattern loading.
  // Map: frameId → ordered list of elemIds in that frame
  const frameBeamElemIds = new Map<string, string[]>();
  const allBeamElemIds: string[] = [];
  const processedBeams = new Set<string>();

  for (const frame of frames) {
    const frameElemIds: string[] = [];
    for (const beamId of frame.beamIds) {
      if (processedBeams.has(beamId)) {
        // already added — just reference the element id for this frame's list
        const eid = `beam_${beamId}`;
        if (!frameElemIds.includes(eid)) frameElemIds.push(eid);
        continue;
      }
      processedBeams.add(beamId);

      const beam = beamsMap.get(beamId);
      if (!beam) continue;

      const fromCol = columns.find(c => c.id === beam.fromCol);
      const toCol = columns.find(c => c.id === beam.toCol);
      if (!fromCol || !toCol) continue;

      // Both columns must have top-nodes in the map (i.e. not removed).
      // If either is removed, this beam is a secondary beam — it will be
      // reconnected via the bearing intermediate node below.
      const nodeIId = colTopNodeMap.get(fromCol.id);
      const nodeJId = colTopNodeMap.get(toCol.id);
      // Skip beams whose column nodes don't exist yet — will be handled in BoB pass
      if (!nodeIId && !nodeJId) continue;

      const elemId = `beam_${beamId}`;

      // Look up user-defined end releases
      let releases: Element3D['releases'] | undefined;
      if (frameEndReleases) {
        const posKey = `${fromCol.x.toFixed(3)}_${fromCol.y.toFixed(3)}_${toCol.x.toFixed(3)}_${toCol.y.toFixed(3)}`;
        const posKeyRev = `${toCol.x.toFixed(3)}_${toCol.y.toFixed(3)}_${fromCol.x.toFixed(3)}_${fromCol.y.toFixed(3)}`;
        const rel = frameEndReleases[posKey] || frameEndReleases[posKeyRev];
        if (rel) {
          const isReversed = !!frameEndReleases[posKeyRev] && !frameEndReleases[posKey];
          const ni = isReversed ? rel.nodeJ : rel.nodeI;
          const nj = isReversed ? rel.nodeI : rel.nodeJ;
          releases = {
            nodeI: { ux: ni.ux, uy: ni.uy, uz: ni.uz, mx: ni.rx, my: ni.ry, mz: ni.rz },
            nodeJ: { ux: nj.ux, uy: nj.uy, uz: nj.uz, mx: nj.rx, my: nj.ry, mz: nj.rz },
          };
        }
      }

      // Both ends present → standard beam (includes primary/carrier beams)
      if (nodeIId && nodeJId) {
        elements3d.push({
          id: elemId,
          type: 'beam',
          nodeI: nodeIId,
          nodeJ: nodeJId,
          b: beam.b,
          h: beam.h,
          E,
          G,
          wLocal: { wx: 0, wy: 0, wz: 0 },
          stiffnessModifier: 0.35,
          releases,
        });
        beamDeadLoads.set(elemId, 1.2 * beam.deadLoad);
        beamLiveLoads.set(elemId, 1.6 * beam.liveLoad);
        frameElemIds.push(elemId);
        allBeamElemIds.push(elemId);
      }
      // Beams with one missing-column end → secondary beams → handled in BoB pass below
    }
    frameBeamElemIds.set(frame.id, frameElemIds);
  }

  // ── Beam-on-Beam: split primary beams and reconnect secondary beams ──────
  // Map: originalBeamId → 'split' (so getFrameResults3D can merge _A/_B results)
  const primaryBeamSplitIds = new Map<string, string>(); // beamId → `${beamId}_A,${beamId}_B`

  if (beamOnBeamConnections && beamOnBeamConnections.length > 0) {
    for (const conn of beamOnBeamConnections) {
      const primaryBeamElemId = `beam_${conn.primaryBeamId}`;
      const primaryElemIndex = elements3d.findIndex(e => e.id === primaryBeamElemId);
      if (primaryElemIndex < 0) continue;

      const primaryElem = elements3d[primaryElemIndex];
      const nodeI = nodesMap.get(primaryElem.nodeI);
      const nodeJ = nodesMap.get(primaryElem.nodeJ);
      if (!nodeI || !nodeJ) continue;

      // Compute bearing point in 3D space by linear interpolation
      const totalLenMm = Math.sqrt(
        Math.pow(nodeJ.x - nodeI.x, 2) +
        Math.pow(nodeJ.y - nodeI.y, 2) +
        Math.pow(nodeJ.z - nodeI.z, 2),
      );
      // distanceOnPrimary is in meters; totalLenMm in mm
      const ratio = totalLenMm > 0 ? Math.min(Math.max((conn.distanceOnPrimary * 1000) / totalLenMm, 0.01), 0.99) : 0.5;
      const bx = nodeI.x + ratio * (nodeJ.x - nodeI.x);
      const by = nodeI.y + ratio * (nodeJ.y - nodeI.y);
      const bz = nodeI.z + ratio * (nodeJ.z - nodeI.z);

      const midNodeId = getOrCreateNode(bx, by, bz, [false, false, false, false, false, false]);

      // Sub-element A: nodeI → midNode
      const subElemA: Element3D = {
        ...primaryElem,
        id: `${primaryBeamElemId}_A`,
        nodeI: primaryElem.nodeI,
        nodeJ: midNodeId,
        releases: primaryElem.releases
          ? { ...primaryElem.releases, nodeJ: { ux: false, uy: false, uz: false, mx: false, my: false, mz: false } }
          : undefined,
      };
      // Sub-element B: midNode → nodeJ
      const subElemB: Element3D = {
        ...primaryElem,
        id: `${primaryBeamElemId}_B`,
        nodeI: midNodeId,
        nodeJ: primaryElem.nodeJ,
        releases: primaryElem.releases
          ? { ...primaryElem.releases, nodeI: { ux: false, uy: false, uz: false, mx: false, my: false, mz: false } }
          : undefined,
      };

      // Replace original element with two sub-elements
      elements3d.splice(primaryElemIndex, 1, subElemA, subElemB);

      // Distribute loads (UDL stays same — it's per unit length)
      const origDead = beamDeadLoads.get(primaryBeamElemId) ?? 0;
      const origLive = beamLiveLoads.get(primaryBeamElemId) ?? 0;
      beamDeadLoads.set(`${primaryBeamElemId}_A`, origDead);
      beamDeadLoads.set(`${primaryBeamElemId}_B`, origDead);
      beamLiveLoads.set(`${primaryBeamElemId}_A`, origLive);
      beamLiveLoads.set(`${primaryBeamElemId}_B`, origLive);
      beamDeadLoads.delete(primaryBeamElemId);
      beamLiveLoads.delete(primaryBeamElemId);

      // Update per-frame element id lists
      for (const [fid, fEids] of frameBeamElemIds) {
        const idx = fEids.indexOf(primaryBeamElemId);
        if (idx >= 0) fEids.splice(idx, 1, `${primaryBeamElemId}_A`, `${primaryBeamElemId}_B`);
        frameBeamElemIds.set(fid, fEids);
      }
      const gIdx = allBeamElemIds.indexOf(primaryBeamElemId);
      if (gIdx >= 0) allBeamElemIds.splice(gIdx, 1, `${primaryBeamElemId}_A`, `${primaryBeamElemId}_B`);

      primaryBeamSplitIds.set(conn.primaryBeamId, `${conn.primaryBeamId}_A,${conn.primaryBeamId}_B`);

      // Reconnect secondary (carried) beams to the intermediate bearing node
      for (const secBeamId of conn.secondaryBeamIds) {
        const secBeam = beamsMap.get(secBeamId);
        if (!secBeam) continue;

        const secFromCol = columns.find(c => c.id === secBeam.fromCol);
        const secToCol = columns.find(c => c.id === secBeam.toCol);

        // Determine which end connects to the removed column
        const isAtStart = secBeam.fromCol === conn.removedColumnId;
        const otherCol = isAtStart ? secToCol : secFromCol;
        if (!otherCol) continue;

        const otherNodeId = colTopNodeMap.get(otherCol.id);
        if (!otherNodeId) continue;

        const secElemId = `beam_${secBeamId}`;

        // Hinge at the bearing end (mz and my released — biaxial)
        const hingeRelease = { ux: false, uy: false, uz: false, mx: false, my: true, mz: true };
        const noRelease    = { ux: false, uy: false, uz: false, mx: false, my: false, mz: false };

        const secElem: Element3D = {
          id: secElemId,
          type: 'beam',
          nodeI: isAtStart ? midNodeId : otherNodeId,
          nodeJ: isAtStart ? otherNodeId : midNodeId,
          b: secBeam.b,
          h: secBeam.h,
          E,
          G,
          wLocal: { wx: 0, wy: 0, wz: 0 },
          stiffnessModifier: 0.35,
          releases: {
            nodeI: isAtStart ? hingeRelease : noRelease,
            nodeJ: isAtStart ? noRelease    : hingeRelease,
          },
        };

        // Add or replace secondary beam element
        const existingIdx = elements3d.findIndex(e => e.id === secElemId);
        if (existingIdx >= 0) {
          elements3d[existingIdx] = secElem;
        } else {
          elements3d.push(secElem);
        }

        // Add secondary beam loads if not already tracked
        if (!beamDeadLoads.has(secElemId)) {
          beamDeadLoads.set(secElemId, 1.2 * secBeam.deadLoad);
          beamLiveLoads.set(secElemId, 1.6 * secBeam.liveLoad);
          // Register in frames that contain this secondary beam
          for (const frame of frames) {
            if (frame.beamIds.includes(secBeamId)) {
              const fEids = frameBeamElemIds.get(frame.id) ?? [];
              if (!fEids.includes(secElemId)) {
                fEids.push(secElemId);
                frameBeamElemIds.set(frame.id, fEids);
              }
              if (!allBeamElemIds.includes(secElemId)) {
                allBeamElemIds.push(secElemId);
              }
            }
          }
        }
      }
    }
  }

  const model: Model3D = { nodes: Array.from(nodesMap.values()), elements: elements3d };

  // ── ETABS-equivalent slab load profiles (non-uniform FEF correction) ──────
  // For two-way slab beams (β ≤ 2), the slab load distribution is:
  //   Long-side beams  → Trapezoidal  (zero at corners → max at plateau → zero)
  //   Short-side beams → Triangular   (zero at ends → peak at midspan → zero)
  // Using equivalent UDL over-estimates fixed-end moments by ~7–11%.
  // We now track the ACTUAL peak load intensity and normalised profile shape for
  // each qualifying element so the solver can apply Gauss-quadrature FEF.
  //
  // Only applied to:
  //   • Non-split (no _A/_B suffix) beam elements
  //   • Beams bordering exactly ONE slab
  //   • Contact ratio ≥ 0.85 (essentially full-contact)
  //   • Two-way slabs (β = ly/lx ≤ 2.0)
  // ──────────────────────────────────────────────────────────────────────────
  interface ElemSlabProfile {
    /** DL peak intensity at SERVICE level (kN/m) — at the point of max tributary width */
    wPeak_DL_service: number;
    /** LL peak intensity at SERVICE level (kN/m) */
    wPeak_LL_service: number;
    /** Factored UNIFORM dead load (1.2 × [beamSW + wallLoad]), replaces beamDeadLoads for UDL */
    uniformDL_factored: number;
    /** Normalised profile shape — t ∈ [0,1], m = multiplier (peak = 1.0) */
    shape: Array<{ t: number; m: number }>;
  }

  const elemSlabProfiles = new Map<string, ElemSlabProfile>();

  if (slabs && slabs.length > 0 && slabProps) {
    // Service-level intensities (kN/m²)
    const wDL_service = (slabProps.thickness / 1000) * mat.gamma + slabProps.finishLoad;
    const wLL_service = slabProps.liveLoad;

    for (const elem of elements3d) {
      if (elem.type !== 'beam') continue;
      // Skip split sub-elements — profile remapping across a split is non-trivial;
      // the UDL approximation is retained for these elements.
      if (elem.id.endsWith('_A') || elem.id.endsWith('_B')) continue;

      const baseBeamId = elem.id.replace(/^beam_/, '');
      const beam = beamsMap.get(baseBeamId);
      if (!beam || beam.slabs.length !== 1) continue;

      const slab = slabs.find(s => s.id === beam.slabs[0]);
      if (!slab) continue;

      const slabW = Math.abs(slab.x2 - slab.x1); // m
      const slabH = Math.abs(slab.y2 - slab.y1); // m
      const lx = Math.min(slabW, slabH);           // short span (m)
      const ly = Math.max(slabW, slabH);            // long span (m)
      if (lx < 0.1) continue;

      const beta = ly / lx;
      if (beta > 2.0) continue; // one-way slab — UDL is already exact for these

      // Determine which edge of the slab this beam spans
      const slabEdge = beam.direction === 'horizontal' ? slabW : slabH;
      const isLongSide = slabEdge >= ly - 0.01;

      // Contact ratio: fraction of beam length that overlaps the slab
      let contactLen: number;
      if (beam.direction === 'horizontal') {
        const s = Math.max(beam.x1, Math.min(slab.x1, slab.x2));
        const e = Math.min(beam.x2, Math.max(slab.x1, slab.x2));
        contactLen = Math.max(0, e - s);
      } else {
        const s = Math.max(beam.y1, Math.min(slab.y1, slab.y2));
        const e = Math.min(beam.y2, Math.max(slab.y1, slab.y2));
        contactLen = Math.max(0, e - s);
      }
      const contactRatio = beam.length > 1e-6 ? contactLen / beam.length : 0;
      if (contactRatio < 0.85) continue; // skip partially overlapping beams

      // Peak load intensity = slab load (kN/m²) × max tributary width = lx/2
      // This is the actual MAXIMUM ordinate of the non-uniform load diagram.
      const wPeak_DL = wDL_service * (lx / 2) * contactRatio; // kN/m
      const wPeak_LL = wLL_service * (lx / 2) * contactRatio; // kN/m

      // Beam self-weight + wall load (always uniform — no profile correction needed)
      const beamSW = (beam.b / 1000) * (beam.h / 1000) * mat.gamma;
      const wallLoad = beam.wallLoad ?? 0;
      const uniformDL_factored = 1.2 * (beamSW + wallLoad);

      // Normalised profile shape (multiplier: 0 → 1 → 1 → 0 or 0 → 1 → 0)
      let shape: Array<{ t: number; m: number }>;
      if (isLongSide) {
        // Trapezoidal: ramp over each end of length (lx/2), plateau in middle
        const a = Math.min(lx / (2 * ly), 0.499); // normalised ramp fraction
        shape = [{ t: 0, m: 0 }, { t: a, m: 1 }, { t: 1 - a, m: 1 }, { t: 1, m: 0 }];
      } else {
        // Triangular: zero at ends, peak at midspan
        shape = [{ t: 0, m: 0 }, { t: 0.5, m: 1 }, { t: 1, m: 0 }];
      }

      elemSlabProfiles.set(elem.id, { wPeak_DL_service: wPeak_DL, wPeak_LL_service: wPeak_LL, uniformDL_factored, shape });

      // IMPORTANT: Override beamDeadLoads / beamLiveLoads so the UDL load case
      // only carries the UNIFORM portion (SW + wall).  The slab contribution is
      // now encoded entirely in the profile (elementLoadProfiles in each load case).
      beamDeadLoads.set(elem.id, uniformDL_factored); // already at 1.2D level
      beamLiveLoads.set(elem.id, 0);                  // LL moved to profile
    }
  }

  /** Build factored profile points for one element-load-case combination. */
  const buildProfile = (
    prof: ElemSlabProfile,
    factorDL: number,
    factorLL: number,
  ): Array<{ t: number; wy: number }> => {
    const scale = -(factorDL * prof.wPeak_DL_service + factorLL * prof.wPeak_LL_service);
    return prof.shape.map(pt => ({ t: pt.t, wy: scale * pt.m }));
  };

  // ── Pattern loading cases — PER FRAME (ACI 318-19 §6.4.3) ───────────────
  // Per-frame approach: alternating live load pattern is applied independently
  // within each frame, not globally across the whole building.
  const patternCases: LoadCase3D[] = [];

  // Base: 1.4D only
  {
    const loads    = new Map<string, { wx: number; wy: number; wz: number }>();
    const profiles = new Map<string, Array<{ t: number; wy: number }>>();
    for (const eid of allBeamElemIds) {
      const wD = beamDeadLoads.get(eid) ?? 0;
      loads.set(eid, { wx: 0, wy: 0, wz: -(1.4 / 1.2) * wD });
      const prof = elemSlabProfiles.get(eid);
      if (prof) profiles.set(eid, buildProfile(prof, 1.4, 0));
    }
    patternCases.push({
      id: 'case_1.4D', name: '1.4D', type: 'dead', elementLoads: loads,
      elementLoadProfiles: profiles.size > 0 ? profiles : undefined,
    });
  }

  // Full load: 1.2D + 1.6L (all spans)
  {
    const loads    = new Map<string, { wx: number; wy: number; wz: number }>();
    const profiles = new Map<string, Array<{ t: number; wy: number }>>();
    for (const eid of allBeamElemIds) {
      const wD = beamDeadLoads.get(eid) ?? 0;
      const wL = beamLiveLoads.get(eid) ?? 0;
      loads.set(eid, { wx: 0, wy: 0, wz: -(wD + wL) });
      const prof = elemSlabProfiles.get(eid);
      if (prof) profiles.set(eid, buildProfile(prof, 1.2, 1.6));
    }
    patternCases.push({
      id: 'case_full', name: '1.2D+1.6L', type: 'dead', elementLoads: loads,
      elementLoadProfiles: profiles.size > 0 ? profiles : undefined,
    });
  }

  // Per-frame alternating live-load patterns
  for (const [frameId, fEids] of frameBeamElemIds) {
    if (fEids.length < 2) continue;
    const nSpans = Math.min(fEids.length, 8); // cap at 2^8 = 256 combinations
    const totalPatterns = Math.pow(2, nSpans);
    for (let mask = 1; mask < totalPatterns - 1; mask++) {
      const loads    = new Map<string, { wx: number; wy: number; wz: number }>();
      const profiles = new Map<string, Array<{ t: number; wy: number }>>();

      // Start with dead-only on all building beams
      for (const eid of allBeamElemIds) {
        const wD = beamDeadLoads.get(eid) ?? 0;
        loads.set(eid, { wx: 0, wy: 0, wz: -wD });
        const prof = elemSlabProfiles.get(eid);
        if (prof) profiles.set(eid, buildProfile(prof, 1.2, 0)); // DL only initially
      }
      // Apply live load to selected spans within this frame
      fEids.forEach((eid, i) => {
        const bitIdx = i < nSpans ? i : i % nSpans;
        const hasLL = (mask >> bitIdx) & 1;
        if (hasLL) {
          const wD = beamDeadLoads.get(eid) ?? 0;
          const wL = beamLiveLoads.get(eid) ?? 0;
          loads.set(eid, { wx: 0, wy: 0, wz: -(wD + wL) });
          const prof = elemSlabProfiles.get(eid);
          if (prof) profiles.set(eid, buildProfile(prof, 1.2, 1.6)); // upgrade to DL+LL
        }
      });
      patternCases.push({
        id: `case_f${frameId}_p${mask}`,
        name: `Frame ${frameId} Pattern ${mask}`,
        type: 'dead',
        elementLoads: loads,
        elementLoadProfiles: profiles.size > 0 ? profiles : undefined,
      });
    }
  }

  // Guard: if no per-frame patterns were generated (only 1 beam per frame), add even/odd
  if (patternCases.length <= 2 && allBeamElemIds.length > 1) {
    const loadsEven    = new Map<string, { wx: number; wy: number; wz: number }>();
    const loadsOdd     = new Map<string, { wx: number; wy: number; wz: number }>();
    const profilesEven = new Map<string, Array<{ t: number; wy: number }>>();
    const profilesOdd  = new Map<string, Array<{ t: number; wy: number }>>();
    allBeamElemIds.forEach((eid, i) => {
      const wD = beamDeadLoads.get(eid) ?? 0;
      const wL = beamLiveLoads.get(eid) ?? 0;
      const llEven = i % 2 === 0;
      const llOdd  = i % 2 === 1;
      loadsEven.set(eid, { wx: 0, wy: 0, wz: -(wD + (llEven ? wL : 0)) });
      loadsOdd .set(eid, { wx: 0, wy: 0, wz: -(wD + (llOdd  ? wL : 0)) });
      const prof = elemSlabProfiles.get(eid);
      if (prof) {
        profilesEven.set(eid, buildProfile(prof, 1.2, llEven ? 1.6 : 0));
        profilesOdd .set(eid, buildProfile(prof, 1.2, llOdd  ? 1.6 : 0));
      }
    });
    patternCases.push({
      id: 'case_even', name: 'Even LL', type: 'dead', elementLoads: loadsEven,
      elementLoadProfiles: profilesEven.size > 0 ? profilesEven : undefined,
    });
    patternCases.push({
      id: 'case_odd', name: 'Odd LL', type: 'dead', elementLoads: loadsOdd,
      elementLoadProfiles: profilesOdd.size > 0 ? profilesOdd : undefined,
    });
  }

  return { model, patternCases, primaryBeamSplitIds };
}

function runPatternEnvelope3D(
  frames: Frame[],
  beams: Beam[],
  columns: Column[],
  mat: MatProps,
  frameEndReleases?: EndReleaseMap,
  beamOnBeamConnections?: BeamOnBeamConnection[],
): {
  beamEnvelope: Map<string, BeamEnvelope3D>;
  colEnvelope: Map<string, ColumnEnvelope3D>;
  primaryBeamSplitIds: Map<string, string>;
} {
  const { model, patternCases, primaryBeamSplitIds } = build3DModelWithPatternLoading(
    frames, beams, columns, mat, frameEndReleases, beamOnBeamConnections,
  );
  const beamEnvelope = new Map<string, BeamEnvelope3D>();
  const colEnvelope  = new Map<string, ColumnEnvelope3D>();

  if (model.elements.length === 0 || patternCases.length === 0) {
    return { beamEnvelope, colEnvelope, primaryBeamSplitIds };
  }

  // Keep value with larger absolute magnitude while preserving sign
  const pickSignedMaxAbs = (current: number, incoming: number) =>
    Math.abs(incoming) > Math.abs(current) ? incoming : current;

  for (const lc of patternCases) {
    const result = analyze3DFrame(model, lc);
    for (const er of result.elements) {

      // ── Column envelope ──────────────────────────────────────────────────
      if (er.elementId.startsWith('col_')) {
        const prev = colEnvelope.get(er.elementId);
        if (!prev) {
          colEnvelope.set(er.elementId, {
            axialMax: er.axial,    // positive = compression
            axialMin: er.axial,
            shearMax: Math.max(Math.abs(er.shearY), Math.abs(er.shearZ)),
            momentYI: er.momentYI,
            momentYJ: er.momentYJ,
            momentYmax: er.momentYmax,
            momentZI: er.momentZI,
            momentZJ: er.momentZJ,
            momentZmax: er.momentZmax,
          });
        } else {
          prev.axialMax = Math.max(prev.axialMax, er.axial);   // max compression
          prev.axialMin = Math.min(prev.axialMin, er.axial);   // min (tension if negative)
          prev.shearMax = Math.max(prev.shearMax, Math.abs(er.shearY), Math.abs(er.shearZ));
          prev.momentYI   = pickSignedMaxAbs(prev.momentYI, er.momentYI);
          prev.momentYJ   = pickSignedMaxAbs(prev.momentYJ, er.momentYJ);
          prev.momentYmax = Math.max(prev.momentYmax, er.momentYmax);
          prev.momentZI   = pickSignedMaxAbs(prev.momentZI, er.momentZI);
          prev.momentZJ   = pickSignedMaxAbs(prev.momentZJ, er.momentZJ);
          prev.momentZmax = Math.max(prev.momentZmax, er.momentZmax);
        }
        continue;
      }

      // ── Beam envelope ────────────────────────────────────────────────────
      if (!er.elementId.startsWith('beam_')) continue;

      const prev = beamEnvelope.get(er.elementId);
      // Convention: negative moment = hogging (top tension), positive = sagging (bottom tension)
      const signedLeft  = er.momentZI <= 0 ? er.momentZI : -er.momentZI;
      const signedRight = er.momentZJ <= 0 ? er.momentZJ : -er.momentZJ;
      if (!prev) {
        beamEnvelope.set(er.elementId, {
          shearYMax: Math.abs(er.shearY),
          shearYI: er.forceI[1],
          shearYJ: er.forceJ[1],
          momentZI: signedLeft,
          momentZJ: signedRight,
          momentZmid: Math.max(0, er.momentZmid),
        });
      } else {
        prev.shearYMax = Math.max(prev.shearYMax, Math.abs(er.shearY));
        prev.shearYI   = pickSignedMaxAbs(prev.shearYI, er.forceI[1]);
        prev.shearYJ   = pickSignedMaxAbs(prev.shearYJ, er.forceJ[1]);
        prev.momentZI  = pickSignedMaxAbs(prev.momentZI,  signedLeft);
        prev.momentZJ  = pickSignedMaxAbs(prev.momentZJ,  signedRight);
        prev.momentZmid = Math.max(prev.momentZmid, Math.max(0, er.momentZmid));
      }
    }
  }

  return { beamEnvelope, colEnvelope, primaryBeamSplitIds };
}

/**
 * Run 3D analysis with pattern loading and return column loads for design.
 * Bug fix: stores both axialMax (compression) and axialMin (tension) envelopes.
 */
export function getColumnLoads3D(
  frames: Frame[],
  beams: Beam[],
  columns: Column[],
  mat: MatProps,
  frameEndReleases?: EndReleaseMap,
  beamOnBeamConnections?: BeamOnBeamConnection[],
): Map<string, ColumnLoads3D> {
  const { colEnvelope } = runPatternEnvelope3D(
    frames, beams, columns, mat, frameEndReleases, beamOnBeamConnections,
  );

  const result = new Map<string, ColumnLoads3D>();
  for (const col of columns) {
    if (col.isRemoved) continue;
    const env = colEnvelope.get(`col_${col.id}`);
    if (env) {
      result.set(col.id, {
        Pu:    Math.max(env.axialMax, 0),   // design compression (≥ 0)
        PuMin: env.axialMin,                // may be negative (tension) — for PM diagram
        Mx: env.momentYmax,
        My: env.momentZmax,
        MxTop: env.momentYJ,
        MxBot: env.momentYI,
        MyTop: env.momentZJ,
        MyBot: env.momentZI,
        Vu: env.shearMax,
      });
    } else {
      result.set(col.id, { Pu: 0, PuMin: 0, Mx: 0, My: 0, MxTop: 0, MxBot: 0, MyTop: 0, MyBot: 0, Vu: 0 });
    }
  }

  return result;
}

/**
 * Run 3D analysis and return beam internal forces grouped by frame.
 * Handles split primary beams (_A/_B) by merging their envelope into one result row.
 */
export function getFrameResults3D(
  frames: Frame[],
  beams: Beam[],
  columns: Column[],
  mat: MatProps,
  frameEndReleases?: EndReleaseMap,
  beamOnBeamConnections?: BeamOnBeamConnection[],
): FrameResult[] {
  const beamsMap = new Map(beams.map(b => [b.id, b]));
  const { beamEnvelope, primaryBeamSplitIds } = runPatternEnvelope3D(
    frames, beams, columns, mat, frameEndReleases, beamOnBeamConnections,
  );

  return frames.map((frame): FrameResult => {
    const frameBeams: FrameResult['beams'] = [];

    for (const beamId of frame.beamIds) {
      const beam = beamsMap.get(beamId);
      if (!beam) continue;

      // Check whether this beam was split into _A/_B sub-elements
      const envA = beamEnvelope.get(`beam_${beamId}_A`);
      const envB = beamEnvelope.get(`beam_${beamId}_B`);
      const env  = beamEnvelope.get(`beam_${beamId}`);

      let finalEnv: BeamEnvelope3D | undefined;
      if (envA && envB) {
        // Merge split sub-elements: take governing envelope from each half
        finalEnv = {
          shearYMax: Math.max(envA.shearYMax, envB.shearYMax),
          shearYI:   envA.shearYI,   // left-end shear from left sub-element
          shearYJ:   envB.shearYJ,   // right-end shear from right sub-element
          momentZI:  envA.momentZI,  // left-end moment from left sub-element
          momentZJ:  envB.momentZJ,  // right-end moment from right sub-element
          // Mid-span sagging: maximum of both sub-element mid moments AND the
          // moment at the shared bearing node (envA.momentZJ = envB.momentZI)
          momentZmid: Math.max(
            envA.momentZmid,
            envB.momentZmid,
            Math.max(0, Math.abs(envA.momentZJ)),
            Math.max(0, Math.abs(envB.momentZI)),
          ),
        };
      } else {
        finalEnv = env;
      }

      frameBeams.push({
        beamId,
        span: beam.length,
        Mleft:  finalEnv?.momentZI  ?? 0,
        Mmid:   finalEnv?.momentZmid ?? 0,
        Mright: finalEnv?.momentZJ  ?? 0,
        Vu:     finalEnv?.shearYMax  ?? 0,
        Rleft:  finalEnv ? Math.abs(finalEnv.shearYI) : 0,
        Rright: finalEnv ? Math.abs(finalEnv.shearYJ) : 0,
      });
    }

    return { frameId: frame.id, beams: frameBeams };
  });
}
