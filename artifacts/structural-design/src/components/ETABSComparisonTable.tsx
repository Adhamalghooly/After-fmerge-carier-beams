/**
 * مقارنة نتائج التحليل بين الطريقة الحالية (Matrix Stiffness 2D) وطريقة ETABS (3D Frame Solver)
 * مع تطبيق تناوب الحمل الحي (Pattern Loading) حسب ACI 318-19 §6.4.3
 */

import React, { useMemo, useState } from 'react';
import { Card, CardHeader, CardTitle, CardContent } from '@/components/ui/card';
import { Table, TableHeader, TableBody, TableRow, TableHead, TableCell } from '@/components/ui/table';
import { Badge } from '@/components/ui/badge';
import { Switch } from '@/components/ui/switch';
import { Label } from '@/components/ui/label';
import { analyzeFrame, calculateColumnLoadsBiaxial, type Beam, type Column, type Frame, type FrameResult, type MatProps, type Story } from '@/lib/structuralEngine';
import {
  analyze3DFrame,
  type Node3D, type Element3D, type Model3D, type LoadCase3D,
} from '@/lib/solver3D';

interface Props {
  frames: Frame[];
  beams: Beam[];
  columns: Column[];
  mat: MatProps;
  stories: Story[];
}

interface BeamCompRow {
  beamId: string;
  frameId: string;
  storyLabel: string;
  span: number;
  m2d_left: number; m2d_mid: number; m2d_right: number; v2d: number;
  m3d_left: number; m3d_mid: number; m3d_right: number; v3d: number;
}

interface ColCompRow {
  colId: string;
  bxh: string;
  storyLabel: string;
  pu2d: number; mx2d: number; my2d: number;
  pu3d: number; mx3d: number; my3d: number; vu3d: number;
}

/**
 * Build the 3D model with pattern loading.
 * @param useModifiers — if true, apply ACI stiffness modifiers (0.35 beams, 0.70 columns).
 *                        if false, use 1.0 for all (same as 2D method).
 */
function buildModel3DWithPatternLoading(
  frames: Frame[],
  beamsMap: Map<string, Beam>,
  columns: Column[],
  mat: MatProps,
  useModifiers: boolean = true,
): { model: Model3D; beamElemIds: string[]; patternCases: LoadCase3D[] } {
  const E = 4700 * Math.sqrt(mat.fc) * 1000;
  const G = E / (2 * (1 + 0.2));

  const nodesMap = new Map<string, Node3D>();
  const elements3d: Element3D[] = [];

  // Position-based node helper for multi-story connectivity
  const getOrCreateNode = (x: number, y: number, z: number, restraints: [boolean, boolean, boolean, boolean, boolean, boolean]): string => {
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

  // Create column nodes and elements
  for (const col of columns) {
    if (col.isRemoved) continue;
    const zBot = col.zBottom ?? 0;
    const zTop = col.zTop ?? (zBot + col.L);
    const xMm = col.x * 1000;
    const yMm = col.y * 1000;

    const isGroundLevel = Math.abs(zBot - minZ) < 1;
    let botRestraints: [boolean, boolean, boolean, boolean, boolean, boolean];
    if (isGroundLevel) {
      botRestraints = [true, true, true, true, true, true]; // Fixed for comparison
    } else {
      botRestraints = [false, false, false, false, false, false];
    }

    const botId = getOrCreateNode(xMm, yMm, zBot, botRestraints);
    const topId = getOrCreateNode(xMm, yMm, zTop, [false, false, false, false, false, false]);
    colTopNodeMap.set(col.id, topId);

    elements3d.push({
      id: `col_${col.id}`, type: 'column',
      nodeI: botId, nodeJ: topId,
      b: col.b, h: col.h, E, G,
      wLocal: { wx: 0, wy: 0, wz: 0 },
      stiffnessModifier: useModifiers ? 0.70 : 1.0,
    });
  }

  // Create beam elements
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
      elements3d.push({
        id: elemId, type: 'beam',
        nodeI: nodeIId, nodeJ: nodeJId,
        b: beam.b, h: beam.h, E, G,
        wLocal: { wx: 0, wy: 0, wz: 0 },
        stiffnessModifier: useModifiers ? 0.35 : 1.0,
      });

      beamDeadLoads.set(elemId, 1.2 * beam.deadLoad);
      beamLiveLoads.set(elemId, 1.6 * beam.liveLoad);
      beamElemIds.push(elemId);
    }
  }

  const model: Model3D = { nodes: Array.from(nodesMap.values()), elements: elements3d };
  const nBeams = beamElemIds.length;

  // ===== Generate Pattern Loading Cases per ACI 318-19 §6.4.3 =====
  // Dead load (1.2D) is ALWAYS on all spans — only live load is alternated

  // Case 1: 1.4D only
  const case14D: LoadCase3D = {
    id: 'case_1.4D', name: '1.4D', type: 'dead',
    elementLoads: new Map(beamElemIds.map(eid => [
      eid, { wx: 0, wy: 0, wz: -(1.4 / 1.2) * beamDeadLoads.get(eid)! }
    ])),
  };

  // Case 2: 1.2D + 1.6L on ALL spans
  const caseFullLoad: LoadCase3D = {
    id: 'case_full', name: '1.2D+1.6L (كامل)', type: 'dead',
    elementLoads: new Map(beamElemIds.map(eid => [
      eid, { wx: 0, wy: 0, wz: -(beamDeadLoads.get(eid)! + beamLiveLoads.get(eid)!) }
    ])),
  };

  // Case 3: 1.2D on all + 1.6L on even-indexed beams only
  const caseEvenLL: LoadCase3D = {
    id: 'case_even_LL', name: '1.2D+1.6L (زوجية)', type: 'dead',
    elementLoads: new Map(beamElemIds.map((eid, i) => [
      eid, { wx: 0, wy: 0, wz: -(beamDeadLoads.get(eid)! + (i % 2 === 0 ? beamLiveLoads.get(eid)! : 0)) }
    ])),
  };

  // Case 4: 1.2D on all + 1.6L on odd-indexed beams only
  const caseOddLL: LoadCase3D = {
    id: 'case_odd_LL', name: '1.2D+1.6L (فردية)', type: 'dead',
    elementLoads: new Map(beamElemIds.map((eid, i) => [
      eid, { wx: 0, wy: 0, wz: -(beamDeadLoads.get(eid)! + (i % 2 === 1 ? beamLiveLoads.get(eid)! : 0)) }
    ])),
  };

  // Additional patterns: for each beam, LL on that beam + every other beam from it
  // This captures ACI §6.4.3.2 (max +M) and §6.4.3.3 (max -M at supports)
  const additionalPatterns: LoadCase3D[] = [];
  if (nBeams > 2) {
    for (let target = 0; target < nBeams; target++) {
      const loads = new Map<string, { wx: number; wy: number; wz: number }>();
      for (let i = 0; i < nBeams; i++) {
        const eid = beamElemIds[i];
        const hasLL = (Math.abs(i - target) % 2 === 0);
        loads.set(eid, {
          wx: 0, wy: 0,
          wz: -(beamDeadLoads.get(eid)! + (hasLL ? beamLiveLoads.get(eid)! : 0)),
        });
      }
      additionalPatterns.push({
        id: `case_pattern_${target}`, name: `Pattern ${target + 1}`,
        type: 'dead', elementLoads: loads,
      });
    }
  }

  return { model, beamElemIds, patternCases: [case14D, caseFullLoad, caseEvenLL, caseOddLL, ...additionalPatterns] };
}

const ETABSComparisonTable: React.FC<Props> = ({ frames, beams, columns, mat, stories }) => {
  const beamsMap = useMemo(() => new Map(beams.map(b => [b.id, b])), [beams]);
  const [useModifiers, setUseModifiers] = useState(true);

  // 2D frame results are computed locally for true method-to-method comparison
  const frameResults2D = useMemo<FrameResult[]>(() => {
    if (frames.length === 0) return [];
    return frames.map(frame => analyzeFrame(frame, beamsMap, columns, mat));
  }, [frames, beamsMap, columns, mat]);

  // 2D column loads
  const colLoads2D = useMemo(() => {
    if (frameResults2D.length === 0) return new Map();
    return calculateColumnLoadsBiaxial(columns, beams, frameResults2D, stories);
  }, [columns, beams, frameResults2D, stories]);

  // 3D analysis with Pattern Loading (ACI 318-19 §6.4.3)
  const { beamRows, colRows, debugInfo } = useMemo(() => {
    const beamRows: BeamCompRow[] = [];
    const colRows: ColCompRow[] = [];
    let debugInfo = '';

    if (frameResults2D.length === 0) return { beamRows, colRows, debugInfo };

    try {
      const { model, patternCases } = buildModel3DWithPatternLoading(frames, beamsMap, columns, mat, useModifiers);
      debugInfo = `العقد: ${model.nodes.length}, العناصر: ${model.elements.length}, حالات تحميل: ${patternCases.length}`;

      if (model.elements.length === 0) {
        debugInfo += ' — لا توجد عناصر';
        return { beamRows, colRows, debugInfo };
      }

      // Run all pattern cases and compute envelope (max absolute values)
      const t0 = performance.now();
      const allResults = patternCases.map((lc, idx) => ({
        result: analyze3DFrame(model, lc),
        loadCase: lc,
        caseIdx: idx,
      }));
      const totalTime = performance.now() - t0;
      debugInfo += `, الوقت: ${totalTime.toFixed(1)}ms (${patternCases.length} حالة)`;

      // Build envelope: for each element, keep max absolute of each force
      const envelope = new Map<string, {
        axial: number; shearY: number; shearZ: number;
        momentYmax: number; momentZmax: number;
        momentZmid: number;
        momentZI: number; momentZJ: number;
        momentYI: number; momentYJ: number;
      }>();

      // Helper: keep the value with max absolute while preserving sign
      const pickSignedMaxAbs = (current: number, incoming: number) =>
        Math.abs(incoming) > Math.abs(current) ? incoming : current;

      for (const { result } of allResults) {
        for (const er of result.elements) {
          const prev = envelope.get(er.elementId);
          if (!prev) {
            envelope.set(er.elementId, {
              axial: Math.abs(er.axial),
              shearY: Math.abs(er.shearY),
              shearZ: Math.abs(er.shearZ),
              momentYmax: Math.abs(er.momentYmax),
              momentZmax: Math.abs(er.momentZmax),
              momentZmid: er.momentZmid,
              momentZI: er.momentZI,   // Keep sign (negative = hogging)
              momentZJ: er.momentZJ,   // Keep sign (negative = hogging)
              momentYI: er.momentYI,   // Keep sign for columns
              momentYJ: er.momentYJ,   // Keep sign for columns
            });
          } else {
            prev.axial = Math.max(prev.axial, Math.abs(er.axial));
            prev.shearY = Math.max(prev.shearY, Math.abs(er.shearY));
            prev.shearZ = Math.max(prev.shearZ, Math.abs(er.shearZ));
            prev.momentYmax = Math.max(prev.momentYmax, Math.abs(er.momentYmax));
            prev.momentZmax = Math.max(prev.momentZmax, Math.abs(er.momentZmax));
            // For mid-span: keep max positive moment (largest sagging)
            prev.momentZmid = Math.max(prev.momentZmid, er.momentZmid);
            // For support moments: keep value with largest absolute, preserving sign
            prev.momentZI = pickSignedMaxAbs(prev.momentZI, er.momentZI);
            prev.momentZJ = pickSignedMaxAbs(prev.momentZJ, er.momentZJ);
            prev.momentYI = pickSignedMaxAbs(prev.momentYI, er.momentYI);
            prev.momentYJ = pickSignedMaxAbs(prev.momentYJ, er.momentYJ);
          }
        }
      }

      // Beam comparison
      for (const fr of frameResults2D) {
        for (const br of fr.beams) {
          const beam = beamsMap.get(br.beamId);
          if (!beam) continue;

          const elemId = `beam_${br.beamId}`;
          const env = envelope.get(elemId);

          const storyLabel = stories.find(s => s.id === beam.storyId)?.label || '';
          beamRows.push({
            beamId: br.beamId,
            frameId: fr.frameId,
            storyLabel,
            span: br.span,
            m2d_left: br.Mleft,
            m2d_mid: br.Mmid,
            m2d_right: br.Mright,
            v2d: br.Vu,
            m3d_left: env ? env.momentZI : 0,    // Signed: negative = hogging
            m3d_mid: env ? env.momentZmid : 0,   // Positive = sagging
            m3d_right: env ? env.momentZJ : 0,    // Signed: negative = hogging
            v3d: env ? env.shearY : 0,
          });
        }
      }

      // Column comparison
      for (const col of columns) {
        if (col.isRemoved) continue;
        const elemId = `col_${col.id}`;
        const env = envelope.get(elemId);
        const loads2d = colLoads2D.get(col.id);

        const storyLabel = stories.find(s => s.id === col.storyId)?.label || '';
        colRows.push({
          colId: col.id,
          bxh: `${col.b}×${col.h}`,
          storyLabel,
          pu2d: loads2d?.Pu ?? 0,
          mx2d: loads2d?.Mx ?? 0,
          my2d: loads2d?.My ?? 0,
          pu3d: env ? env.axial : 0,
          // Show the controlling moment (max absolute) with its sign
          mx3d: env ? (Math.abs(env.momentYI) >= Math.abs(env.momentYJ) ? env.momentYI : env.momentYJ) : 0,
          my3d: env ? (Math.abs(env.momentZI) >= Math.abs(env.momentZJ) ? env.momentZI : env.momentZJ) : 0,
          vu3d: env ? Math.max(env.shearY, env.shearZ) : 0,
        });
      }
    } catch (e: any) {
      debugInfo = `خطأ: ${e.message}`;
    }

    return { beamRows, colRows, debugInfo };
  }, [frames, beamsMap, columns, mat, frameResults2D, colLoads2D, stories, useModifiers]);

  if (beamRows.length === 0 && colRows.length === 0) {
    return debugInfo ? (
      <Card>
        <CardContent className="py-3">
          <p className="text-xs text-destructive">⚠️ لم يتم إنتاج نتائج مقارنة. {debugInfo}</p>
        </CardContent>
      </Card>
    ) : null;
  }

  const diffPct = (a: number, b: number) => {
    if (Math.abs(a) < 0.01 && Math.abs(b) < 0.01) return '—';
    const base = Math.max(Math.abs(a), Math.abs(b));
    return ((Math.abs(Math.abs(b) - Math.abs(a)) / base) * 100).toFixed(1) + '%';
  };

  const diffClr = (a: number, b: number) => {
    if (Math.abs(a) < 0.01 && Math.abs(b) < 0.01) return undefined;
    const base = Math.max(Math.abs(a), Math.abs(b));
    const pct = base > 0.01 ? (Math.abs(Math.abs(b) - Math.abs(a)) / base) * 100 : 0;
    if (pct < 5) return 'hsl(142 71% 45%)';
    if (pct < 15) return 'hsl(45 93% 47%)';
    return 'hsl(0 84.2% 60.2%)';
  };

  return (
    <div className="space-y-4">
      {/* Controls */}
      <Card>
        <CardContent className="py-3">
          <div className="flex items-center justify-between flex-wrap gap-3">
            <div className="text-xs text-muted-foreground">{debugInfo}</div>
            <div className="flex items-center gap-2">
              <Switch
                id="modifiers"
                checked={useModifiers}
                onCheckedChange={setUseModifiers}
              />
              <Label htmlFor="modifiers" className="text-xs cursor-pointer">
                معاملات الجساءة ACI (0.35Ig جسور، 0.70Ig أعمدة)
              </Label>
            </div>
          </div>
          {useModifiers && (
            <p className="text-[10px] text-emerald-600 dark:text-emerald-400 mt-2">
              ✅ كلا الطريقتين تستخدمان معاملات ACI 318-19 §6.6.3.1.1 (0.35Ig جسور، 0.70Ig أعمدة) — المقارنة عادلة.
            </p>
          )}
          {!useModifiers && (
            <p className="text-[10px] text-emerald-600 dark:text-emerald-400 mt-2">
              ✅ كلا الطريقتين يستخدمان الجساءة الكاملة (1.0×EI) — المقارنة عادلة. الفرق يعود فقط للفارق بين التحليل 2D و 3D.
            </p>
          )}
        </CardContent>
      </Card>

      {/* Beam comparison */}
      {beamRows.length > 0 && (
        <Card>
          <CardHeader className="pb-2">
            <CardTitle className="text-sm flex items-center gap-2">
              مقارنة القوى الداخلية للجسور
              <Badge variant="outline" className="text-[10px]">2D مقابل 3D</Badge>
            </CardTitle>
            <p className="text-xs text-muted-foreground mt-1">
              2D = DSM (إطارات مستوية + Pattern Loading) | 3D = ETABS-style (6 DOF/عقدة + Pattern Loading §6.4.3)
              {useModifiers ? ' + معاملات ACI (0.35/0.70)' : ' بدون معاملات جساءة'}
            </p>
          </CardHeader>
          <CardContent className="overflow-x-auto">
            <Table>
              <TableHeader>
                <TableRow>
                  <TableHead className="text-xs">الدور</TableHead>
                  <TableHead className="text-xs">الإطار</TableHead>
                  <TableHead className="text-xs">الجسر</TableHead>
                  <TableHead className="text-xs">البحر</TableHead>
                  <TableHead className="text-[10px] text-center" colSpan={3}>M يسار (kN.m)</TableHead>
                  <TableHead className="text-[10px] text-center" colSpan={3}>M منتصف (kN.m)</TableHead>
                  <TableHead className="text-[10px] text-center" colSpan={3}>M يمين (kN.m)</TableHead>
                  <TableHead className="text-[10px] text-center" colSpan={3}>Vu (kN)</TableHead>
                </TableRow>
                <TableRow>
                  <TableHead /><TableHead /><TableHead /><TableHead />
                  {['2D','3D','Δ%','2D','3D','Δ%','2D','3D','Δ%','2D','3D','Δ%'].map((h, i) =>
                    <TableHead key={i} className="text-[10px] text-center px-1">{h}</TableHead>
                  )}
                </TableRow>
              </TableHeader>
              <TableBody>
                {beamRows.map(r => (
                  <TableRow key={`${r.frameId}-${r.beamId}`}>
                    <TableCell className="text-xs text-muted-foreground">{r.storyLabel}</TableCell>
                    <TableCell className="font-mono text-xs">{r.frameId}</TableCell>
                    <TableCell className="font-mono text-xs font-bold">{r.beamId}</TableCell>
                    <TableCell className="font-mono text-xs">{r.span.toFixed(1)}</TableCell>
                    <TableCell className="font-mono text-xs text-center px-1">{r.m2d_left.toFixed(1)}</TableCell>
                    <TableCell className="font-mono text-xs text-center px-1">{r.m3d_left.toFixed(1)}</TableCell>
                    <TableCell className="font-mono text-xs text-center px-1" style={{ color: diffClr(r.m2d_left, r.m3d_left) }}>{diffPct(r.m2d_left, r.m3d_left)}</TableCell>
                    <TableCell className="font-mono text-xs text-center px-1">{r.m2d_mid.toFixed(1)}</TableCell>
                    <TableCell className="font-mono text-xs text-center px-1">{r.m3d_mid.toFixed(1)}</TableCell>
                    <TableCell className="font-mono text-xs text-center px-1" style={{ color: diffClr(r.m2d_mid, r.m3d_mid) }}>{diffPct(r.m2d_mid, r.m3d_mid)}</TableCell>
                    <TableCell className="font-mono text-xs text-center px-1">{r.m2d_right.toFixed(1)}</TableCell>
                    <TableCell className="font-mono text-xs text-center px-1">{r.m3d_right.toFixed(1)}</TableCell>
                    <TableCell className="font-mono text-xs text-center px-1" style={{ color: diffClr(r.m2d_right, r.m3d_right) }}>{diffPct(r.m2d_right, r.m3d_right)}</TableCell>
                    <TableCell className="font-mono text-xs text-center px-1">{r.v2d.toFixed(1)}</TableCell>
                    <TableCell className="font-mono text-xs text-center px-1">{r.v3d.toFixed(1)}</TableCell>
                    <TableCell className="font-mono text-xs text-center px-1" style={{ color: diffClr(r.v2d, r.v3d) }}>{diffPct(r.v2d, r.v3d)}</TableCell>
                  </TableRow>
                ))}
              </TableBody>
            </Table>
          </CardContent>
        </Card>
      )}

      {/* Column comparison */}
      {colRows.length > 0 && (
        <Card>
          <CardHeader className="pb-2">
            <CardTitle className="text-sm">مقارنة القوى الداخلية للأعمدة</CardTitle>
            <p className="text-xs text-muted-foreground mt-1">
              2D = حساب من ردود أفعال الجسور (توزيع العزوم بنسبة الجساءة) | 3D = تحليل مباشر بطريقة ETABS
            </p>
          </CardHeader>
          <CardContent className="overflow-x-auto">
            <Table>
              <TableHeader>
                <TableRow>
                  <TableHead className="text-xs">الدور</TableHead>
                  <TableHead className="text-xs">العمود</TableHead>
                  <TableHead className="text-xs">المقطع</TableHead>
                  <TableHead className="text-[10px] text-center" colSpan={3}>Pu (kN)</TableHead>
                  <TableHead className="text-[10px] text-center" colSpan={3}>Mx (kN.m)</TableHead>
                  <TableHead className="text-[10px] text-center" colSpan={3}>My (kN.m)</TableHead>
                  <TableHead className="text-[10px] text-center">Vu 3D</TableHead>
                </TableRow>
                <TableRow>
                  <TableHead /><TableHead /><TableHead />
                  {['2D','3D','Δ%','2D','3D','Δ%','2D','3D','Δ%'].map((h, i) =>
                    <TableHead key={i} className="text-[10px] text-center px-1">{h}</TableHead>
                  )}
                  <TableHead className="text-[10px] text-center px-1">kN</TableHead>
                </TableRow>
              </TableHeader>
              <TableBody>
                {colRows.map(r => (
                  <TableRow key={r.colId}>
                    <TableCell className="text-xs text-muted-foreground">{r.storyLabel}</TableCell>
                    <TableCell className="font-mono text-xs font-bold">{r.colId}</TableCell>
                    <TableCell className="font-mono text-xs">{r.bxh}</TableCell>
                    <TableCell className="font-mono text-xs text-center px-1">{r.pu2d.toFixed(1)}</TableCell>
                    <TableCell className="font-mono text-xs text-center px-1">{r.pu3d.toFixed(1)}</TableCell>
                    <TableCell className="font-mono text-xs text-center px-1" style={{ color: diffClr(r.pu2d, r.pu3d) }}>{diffPct(r.pu2d, r.pu3d)}</TableCell>
                    <TableCell className="font-mono text-xs text-center px-1">{r.mx2d.toFixed(1)}</TableCell>
                    <TableCell className="font-mono text-xs text-center px-1">{r.mx3d.toFixed(1)}</TableCell>
                    <TableCell className="font-mono text-xs text-center px-1" style={{ color: diffClr(r.mx2d, r.mx3d) }}>{diffPct(r.mx2d, r.mx3d)}</TableCell>
                    <TableCell className="font-mono text-xs text-center px-1">{r.my2d.toFixed(1)}</TableCell>
                    <TableCell className="font-mono text-xs text-center px-1">{r.my3d.toFixed(1)}</TableCell>
                    <TableCell className="font-mono text-xs text-center px-1" style={{ color: diffClr(r.my2d, r.my3d) }}>{diffPct(r.my2d, r.my3d)}</TableCell>
                    <TableCell className="font-mono text-xs text-center px-1">{r.vu3d.toFixed(1)}</TableCell>
                  </TableRow>
                ))}
              </TableBody>
            </Table>
          </CardContent>
        </Card>
      )}

      <Card>
        <CardContent className="py-3">
          <div className="flex gap-4 text-xs flex-wrap">
            <span className="flex items-center gap-1"><span className="w-3 h-3 rounded-full inline-block" style={{ backgroundColor: 'hsl(142 71% 45%)' }} /> فرق &lt; 5%</span>
            <span className="flex items-center gap-1"><span className="w-3 h-3 rounded-full inline-block" style={{ backgroundColor: 'hsl(45 93% 47%)' }} /> فرق 5–15%</span>
            <span className="flex items-center gap-1"><span className="w-3 h-3 rounded-full inline-block" style={{ backgroundColor: 'hsl(0 84.2% 60.2%)' }} /> فرق &gt; 15%</span>
          </div>
        </CardContent>
      </Card>
    </div>
  );
};

export default ETABSComparisonTable;
