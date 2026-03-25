/**
 * استخراج أحمال الأعمدة من التحليل ثلاثي الأبعاد (3D Frame Analysis)
 * لاستخدامها في التصميم بدلاً من الطريقة التقريبية (2D)
 * 
 * المحاور: للأعمدة الرأسية:
 *   - Local Y = Global X → momentY = Mx (عزم حول المحور العالمي X)
 *   - Local Z = Global Y → momentZ = My (عزم حول المحور العالمي Y)
 *   - nodeI = أسفل العمود (Bot), nodeJ = أعلى العمود (Top)
 */

import type { Beam, Column, Frame, FrameResult, MatProps, BeamOnBeamConnection } from '@/lib/structuralEngine';
import { analyze3DFrame, type Node3D, type Element3D, type Model3D, type LoadCase3D } from '@/lib/solver3D';

export interface ColumnLoads3D {
  Pu: number;
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
  axial: number;
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

function build3DModelWithPatternLoading(
  frames: Frame[],
  beams: Beam[],
  columns: Column[],
  mat: MatProps,
  frameEndReleases?: EndReleaseMap,
  beamOnBeamConnections?: BeamOnBeamConnection[],
): { model: Model3D; patternCases: LoadCase3D[] } {
  const beamsMap = new Map(beams.map(b => [b.id, b]));

  // Build sets for beam-on-beam: secondary beams and their pre-estimated reactions
  const secondaryBeamIds = new Set<string>();
  // Map primaryBeamId → list of point loads (reaction, distance from beam start)
  const primaryBeamPointLoads = new Map<string, Array<{ P_dead: number; P_live: number; a: number }>>();
  if (beamOnBeamConnections) {
    for (const conn of beamOnBeamConnections) {
      for (const secId of conn.secondaryBeamIds) {
        secondaryBeamIds.add(secId);
      }
      // Estimate reaction from secondary beams (simple approach: wL/2 * 1.2 for DL, wL/2 * 1.6 for LL)
      let totalDL = 0;
      let totalLL = 0;
      for (const secId of conn.secondaryBeamIds) {
        const secBeam = beamsMap.get(secId);
        if (!secBeam) continue;
        // Reaction at removed-column end: use actual precomputed reaction if available, else estimate
        // Carried beam: simple-beam reaction at one end = wL/2
        totalDL += secBeam.deadLoad * secBeam.length / 2;
        totalLL += secBeam.liveLoad * secBeam.length / 2;
      }
      const primaryBeam = beamsMap.get(conn.primaryBeamId);
      if (primaryBeam) {
        const existing = primaryBeamPointLoads.get(conn.primaryBeamId) || [];
        // Use the iterative reaction if available (conn.reactionForce > 0), else estimate
        const P_dead = conn.reactionForce > 0 ? conn.reactionForce : 1.2 * totalDL;
        const P_live = conn.reactionForce > 0 ? 0 : 1.6 * totalLL;
        existing.push({ P_dead, P_live, a: conn.distanceOnPrimary });
        primaryBeamPointLoads.set(conn.primaryBeamId, existing);
      }
    }
  }
  const E = 4700 * Math.sqrt(mat.fc) * 1000;
  const G = E / (2 * (1 + 0.2));

  const nodesMap = new Map<string, Node3D>();
  const elements3d: Element3D[] = [];

  // Helper: get or create node by position (ensures multi-story connectivity)
  // Columns at the same (x,y) across stories share nodes at floor levels
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

    // Ground level columns sit on foundations → apply support restraints
    const isGroundLevel = Math.abs(zBot - minZ) < 1;
    let botRestraints: [boolean, boolean, boolean, boolean, boolean, boolean];
    if (isGroundLevel) {
      const isPinned = col.bottomEndCondition === 'P';
      botRestraints = isPinned
        ? [true, true, true, false, false, false]
        : [true, true, true, true, true, true];
    } else {
      // Upper floor column bottom → free node (connected to lower column top via shared node)
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
      // Column self-weight factored with 1.2D
      wLocal: { wx: -1.2 * mat.gamma * (col.b * col.h) / 1e6, wy: 0, wz: 0 },
      stiffnessModifier: 0.70,
    });
  }

  const beamDeadLoads = new Map<string, number>();
  const beamLiveLoads = new Map<string, number>();
  const beamElemIds: string[] = [];
  const processedBeams = new Set<string>();

  for (const frame of frames) {
    for (const beamId of frame.beamIds) {
      if (processedBeams.has(beamId)) continue;
      processedBeams.add(beamId);

      const beam = beamsMap.get(beamId);
      if (!beam) continue;

      const fromCol = columns.find(c => c.id === beam.fromCol);
      const toCol = columns.find(c => c.id === beam.toCol);
      if (!fromCol || !toCol) continue;

      const nodeIId = colTopNodeMap.get(fromCol.id);
      const nodeJId = colTopNodeMap.get(toCol.id);
      if (!nodeIId || !nodeJId) continue;
      if (!nodesMap.has(nodeIId) || !nodesMap.has(nodeJId)) continue;

      const elemId = `beam_${beamId}`;
      // Look up end releases by position key (matching how they're stored in state)
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
      // Skip secondary (carried) beams — they are replaced by point loads on the carrier beam
      if (secondaryBeamIds.has(beamId)) continue;

      // For primary (carrier) beams: look up any point loads from carried beams
      const bobPointLoads = primaryBeamPointLoads.get(beamId);

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
        // Attach point loads from carried beams directly to this element (stored for use in load cases)
        ...(bobPointLoads ? { _bobPointLoads: bobPointLoads } : {}),
      } as Element3D & { _bobPointLoads?: Array<{ P_dead: number; P_live: number; a: number }> });

      beamDeadLoads.set(elemId, 1.2 * beam.deadLoad);
      beamLiveLoads.set(elemId, 1.6 * beam.liveLoad);
      if (bobPointLoads) {
        const totalExtraDL = bobPointLoads.reduce((s, p) => s + p.P_dead, 0);
        const totalExtraLL = bobPointLoads.reduce((s, p) => s + p.P_live, 0);
        beamDeadLoads.set(elemId, (beamDeadLoads.get(elemId) ?? 0) + totalExtraDL);
        beamLiveLoads.set(elemId, (beamLiveLoads.get(elemId) ?? 0) + totalExtraLL);
      }
      beamElemIds.push(elemId);
    }
  }

  const model: Model3D = { nodes: Array.from(nodesMap.values()), elements: elements3d };

  // Pattern loading cases (ACI 318-19 §6.4.3)
  const patternCases: LoadCase3D[] = [];

  // 1.4D
  patternCases.push({
    id: 'case_1.4D', name: '1.4D', type: 'dead',
    elementLoads: new Map(beamElemIds.map(eid => [
      eid, { wx: 0, wy: 0, wz: -(1.4 / 1.2) * beamDeadLoads.get(eid)! }
    ])),
  });

  // Full load
  patternCases.push({
    id: 'case_full', name: '1.2D+1.6L', type: 'dead',
    elementLoads: new Map(beamElemIds.map(eid => [
      eid, { wx: 0, wy: 0, wz: -(beamDeadLoads.get(eid)! + beamLiveLoads.get(eid)!) }
    ])),
  });

  // Even/odd patterns
  patternCases.push({
    id: 'case_even', name: 'Even LL', type: 'dead',
    elementLoads: new Map(beamElemIds.map((eid, i) => [
      eid, { wx: 0, wy: 0, wz: -(beamDeadLoads.get(eid)! + (i % 2 === 0 ? beamLiveLoads.get(eid)! : 0)) }
    ])),
  });
  patternCases.push({
    id: 'case_odd', name: 'Odd LL', type: 'dead',
    elementLoads: new Map(beamElemIds.map((eid, i) => [
      eid, { wx: 0, wy: 0, wz: -(beamDeadLoads.get(eid)! + (i % 2 === 1 ? beamLiveLoads.get(eid)! : 0)) }
    ])),
  });

  // Per-beam patterns
  if (beamElemIds.length > 2) {
    for (let target = 0; target < beamElemIds.length; target++) {
      const loads = new Map<string, { wx: number; wy: number; wz: number }>();
      for (let i = 0; i < beamElemIds.length; i++) {
        const eid = beamElemIds[i];
        const hasLL = (Math.abs(i - target) % 2 === 0);
        loads.set(eid, {
          wx: 0, wy: 0,
          wz: -(beamDeadLoads.get(eid)! + (hasLL ? beamLiveLoads.get(eid)! : 0)),
        });
      }
      patternCases.push({ id: `case_p${target}`, name: `Pattern ${target + 1}`, type: 'dead', elementLoads: loads });
    }
  }

  return { model, patternCases };
}

function runPatternEnvelope3D(
  frames: Frame[],
  beams: Beam[],
  columns: Column[],
  mat: MatProps,
  frameEndReleases?: EndReleaseMap,
  beamOnBeamConnections?: BeamOnBeamConnection[],
): { beamEnvelope: Map<string, BeamEnvelope3D>; colEnvelope: Map<string, ColumnEnvelope3D> } {
  const { model, patternCases } = build3DModelWithPatternLoading(frames, beams, columns, mat, frameEndReleases, beamOnBeamConnections);
  const beamEnvelope = new Map<string, BeamEnvelope3D>();
  const colEnvelope = new Map<string, ColumnEnvelope3D>();

  if (model.elements.length === 0 || patternCases.length === 0) {
    return { beamEnvelope, colEnvelope };
  }

  // Keep the value with the larger absolute magnitude while preserving its sign.
  const pickSignedMaxAbs = (current: number, incoming: number) =>
    Math.abs(incoming) > Math.abs(current) ? incoming : current;

  for (const lc of patternCases) {
    const result = analyze3DFrame(model, lc);
    for (const er of result.elements) {
      if (er.elementId.startsWith('col_')) {
        const prev = colEnvelope.get(er.elementId);
        if (!prev) {
          colEnvelope.set(er.elementId, {
            axial: Math.abs(er.axial),
            shearMax: Math.max(Math.abs(er.shearY), Math.abs(er.shearZ)),
            momentYI: er.momentYI,
            momentYJ: er.momentYJ,
            momentYmax: er.momentYmax,
            momentZI: er.momentZI,
            momentZJ: er.momentZJ,
            momentZmax: er.momentZmax,
          });
        } else {
          prev.axial = Math.max(prev.axial, Math.abs(er.axial));
          prev.shearMax = Math.max(prev.shearMax, Math.abs(er.shearY), Math.abs(er.shearZ));
          prev.momentYI = pickSignedMaxAbs(prev.momentYI, er.momentYI);
          prev.momentYJ = pickSignedMaxAbs(prev.momentYJ, er.momentYJ);
          prev.momentYmax = Math.max(prev.momentYmax, er.momentYmax);
          prev.momentZI = pickSignedMaxAbs(prev.momentZI, er.momentZI);
          prev.momentZJ = pickSignedMaxAbs(prev.momentZJ, er.momentZJ);
          prev.momentZmax = Math.max(prev.momentZmax, er.momentZmax);
        }
        continue;
      }

      if (!er.elementId.startsWith('beam_')) continue;

      const prev = beamEnvelope.get(er.elementId);
      const signedLeft = er.momentZI <= 0 ? er.momentZI : -er.momentZI;
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
        prev.shearYI = pickSignedMaxAbs(prev.shearYI, er.forceI[1]);
        prev.shearYJ = pickSignedMaxAbs(prev.shearYJ, er.forceJ[1]);
        prev.momentZI = pickSignedMaxAbs(prev.momentZI, signedLeft);
        prev.momentZJ = pickSignedMaxAbs(prev.momentZJ, signedRight);
        prev.momentZmid = Math.max(prev.momentZmid, Math.max(0, er.momentZmid));
      }
    }
  }

  return { beamEnvelope, colEnvelope };
}

/**
 * Run 3D analysis with pattern loading and return column loads for design.
 */
export function getColumnLoads3D(
  frames: Frame[],
  beams: Beam[],
  columns: Column[],
  mat: MatProps,
  frameEndReleases?: EndReleaseMap,
  beamOnBeamConnections?: BeamOnBeamConnection[],
): Map<string, ColumnLoads3D> {
  const { colEnvelope } = runPatternEnvelope3D(frames, beams, columns, mat, frameEndReleases, beamOnBeamConnections);

  // Convert to design loads
  const result = new Map<string, ColumnLoads3D>();
  for (const col of columns) {
    if (col.isRemoved) continue;
    const env = colEnvelope.get(`col_${col.id}`);
    if (env) {
      result.set(col.id, {
        Pu: env.axial,
        Mx: env.momentYmax,      // Global X moment
        My: env.momentZmax,      // Global Y moment
        MxTop: env.momentYJ,     // nodeJ = top
        MxBot: env.momentYI,     // nodeI = bottom
        MyTop: env.momentZJ,
        MyBot: env.momentZI,
        Vu: env.shearMax,
      });
    } else {
      result.set(col.id, { Pu: 0, Mx: 0, My: 0, MxTop: 0, MxBot: 0, MyTop: 0, MyBot: 0, Vu: 0 });
    }
  }

  return result;
}

/**
 * Run 3D analysis with pattern loading and return beam internal forces grouped by frame.
 * These results are intended to be the primary analysis/design forces in the app.
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
  const { beamEnvelope } = runPatternEnvelope3D(frames, beams, columns, mat, frameEndReleases, beamOnBeamConnections);

  return frames.map((frame): FrameResult => {
    const frameBeams: FrameResult['beams'] = [];

    for (const beamId of frame.beamIds) {
      const beam = beamsMap.get(beamId);
      if (!beam) continue;

      const env = beamEnvelope.get(`beam_${beamId}`);
      frameBeams.push({
        beamId,
        span: beam.length,
        Mleft: env?.momentZI ?? 0,
        Mmid: env?.momentZmid ?? 0,
        Mright: env?.momentZJ ?? 0,
        Vu: env?.shearYMax ?? 0,
        Rleft: env ? Math.abs(env.shearYI) : 0,
        Rright: env ? Math.abs(env.shearYJ) : 0,
      });
    }

    return { frameId: frame.id, beams: frameBeams };
  });
}
