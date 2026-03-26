/**
 * LoadComparisonPanel
 * ─────────────────────────────────────────────────────────────
 * Two side-by-side comparison tables:
 *
 *  Table 1 – Beam load distribution
 *    Old method (tributary area, ACI)  vs.  FEM Mindlin-Reissner engine
 *
 *  Table 2 – Slab moment resultants
 *    Old method (Marcus/ACI coefficient × q × lx²)  vs.  FEM center Mx/My
 *
 * STRICT: this component is READ-ONLY with respect to the existing engine.
 * It imports from slabFEMEngine (add-on) and structuralEngine (read-only).
 */

import React, { useMemo, useState } from 'react';
import {
  Card, CardHeader, CardTitle, CardContent,
} from '@/components/ui/card';
import {
  Table, TableHeader, TableBody, TableRow, TableHead, TableCell,
} from '@/components/ui/table';
import { Badge } from '@/components/ui/badge';
import { Button } from '@/components/ui/button';
import { Calculator, Info } from 'lucide-react';

import type { Slab, Beam, Column, SlabProps, MatProps } from '@/lib/structuralEngine';
import { designSlab } from '@/lib/structuralEngine';
import {
  getBeamLoadsFromSlab,
  getSlabCenterMoments,
} from '@/slabFEMEngine';
import type { BeamLoadResult, SlabMomentComparison } from '@/slabFEMEngine';

// ─────────────────────────────────────────────────────────────────────────────

interface Props {
  slabs:     Slab[];
  beams:     Beam[];
  columns:   Column[];
  slabProps: SlabProps;
  mat:       MatProps;
  analyzed:  boolean;
  onRunAnalysis: () => void;
}

// ─────────────────────────────────────────────────────────────────────────────
// Helpers
// ─────────────────────────────────────────────────────────────────────────────

function diffBadge(pct: number) {
  const abs = Math.abs(pct);
  if (abs <= 5)  return <Badge className="bg-green-500/15 text-green-700 dark:text-green-400 border-green-400/40 text-[10px]">{pct > 0 ? '+' : ''}{pct.toFixed(1)}%</Badge>;
  if (abs <= 15) return <Badge className="bg-yellow-500/15 text-yellow-700 dark:text-yellow-400 border-yellow-400/40 text-[10px]">{pct > 0 ? '+' : ''}{pct.toFixed(1)}%</Badge>;
  return           <Badge className="bg-red-500/15 text-red-700 dark:text-red-400 border-red-400/40 text-[10px]">{pct > 0 ? '+' : ''}{pct.toFixed(1)}%</Badge>;
}

function pctColor(pct: number) {
  const abs = Math.abs(pct);
  if (abs <= 5)  return 'text-green-600 dark:text-green-400';
  if (abs <= 15) return 'text-yellow-600 dark:text-yellow-400';
  return 'text-red-600 dark:text-red-400';
}

// ─────────────────────────────────────────────────────────────────────────────
// Component
// ─────────────────────────────────────────────────────────────────────────────

const LoadComparisonPanel: React.FC<Props> = ({
  slabs, beams, columns, slabProps, mat, analyzed, onRunAnalysis,
}) => {
  const [computed, setComputed] = useState(false);
  const [computing, setComputing]  = useState(false);

  // ── Compute on demand (FEM can be slow for large models) ─────────────────
  const [beamResults, setBeamResults] = useState<BeamLoadResult[]>([]);
  const [slabResults, setSlabResults] = useState<SlabMomentComparison[]>([]);

  const handleCompute = () => {
    if (slabs.length === 0 || beams.length === 0) return;
    setComputing(true);

    // Defer to next tick so React can render the spinner first
    setTimeout(() => {
      try {
        const femModel = { slabs, beams, columns, slabProps, mat, meshDensity: 4 };

        const br = getBeamLoadsFromSlab({ ...femModel, comparisonMode: true } as never);
        const sr = getSlabCenterMoments(femModel);

        setBeamResults(br);
        setSlabResults(sr);
        setComputed(true);
      } catch (err) {
        console.error('[LoadComparisonPanel] FEM error:', err);
      } finally {
        setComputing(false);
      }
    }, 0);
  };

  // ── Old-method slab moments (Marcus / ACI coefficient × q × lx²) ─────────
  const slabMomentRows = useMemo(() => {
    if (!computed) return [];
    const q_service = (slabProps.thickness / 1000) * mat.gamma + slabProps.finishLoad + slabProps.liveLoad;

    return slabResults.map(sr => {
      const slab = slabs.find(s => s.id === sr.slabId)!;
      if (!slab) return null;

      const ds = designSlab(slab, slabProps, mat, slabs);
      const oldMx = ds.shortCoeff * q_service * sr.lx_m * sr.lx_m;
      const oldMy = ds.isOneWay ? 0 : (ds.longCoeff * q_service * sr.lx_m * sr.lx_m);

      const diffMx = oldMx > 1e-4 ? ((sr.fem.Mx - oldMx) / oldMx) * 100 : 0;
      const diffMy = oldMy > 1e-4 ? ((sr.fem.My - oldMy) / oldMy) * 100 : 0;

      return {
        ...sr,
        shortCoeff: ds.shortCoeff,
        longCoeff:  ds.longCoeff,
        oldMx, oldMy,
        diffMx, diffMy,
      };
    }).filter(Boolean) as (SlabMomentComparison & {
      shortCoeff: number; longCoeff: number;
      oldMx: number; oldMy: number;
      diffMx: number; diffMy: number;
    })[];
  }, [computed, slabResults, slabs, slabProps, mat]);

  // ── Beam comparison rows ──────────────────────────────────────────────────
  const beamRows = useMemo(() => {
    if (!computed) return [];
    return beamResults.map(br => {
      const beam = beams.find(b => b.id === br.beamId);
      if (!beam || !br.oldMethodLoad || !br.femMethodLoad) return null;

      const oldTotal = (br.oldMethodLoad.deadLoad + br.oldMethodLoad.liveLoad) * beam.length;
      const femTotal = trapz(
        br.loads.values.map(p => p.position),
        br.loads.values.map(p => p.w),
      );

      const diff = br.differencePercent ?? (oldTotal > 1e-6 ? ((femTotal - oldTotal) / oldTotal) * 100 : 0);

      return {
        beamId:   br.beamId,
        span:     beam.length,
        oldDL:    br.oldMethodLoad.deadLoad,
        oldLL:    br.oldMethodLoad.liveLoad,
        oldTotal,
        femAvg:   br.femMethodLoad.avgLoad,
        femPeak:  br.femMethodLoad.peakLoad,
        femTotal,
        diff,
      };
    }).filter(Boolean) as {
      beamId: string; span: number;
      oldDL: number; oldLL: number; oldTotal: number;
      femAvg: number; femPeak: number; femTotal: number;
      diff: number;
    }[];
  }, [computed, beamResults, beams]);

  // ─────────────────────────────────────────────────────────────────────────
  // Render
  // ─────────────────────────────────────────────────────────────────────────

  if (!analyzed) {
    return (
      <Card>
        <CardContent className="py-12 text-center">
          <p className="text-muted-foreground mb-4">يرجى تشغيل التحليل الرئيسي أولاً</p>
          <Button onClick={onRunAnalysis} className="min-h-[44px]">
            <Calculator size={16} className="mr-2" />
            تشغيل التحليل
          </Button>
        </CardContent>
      </Card>
    );
  }

  return (
    <div className="space-y-4">

      {/* Header note */}
      <Card className="border-blue-200 dark:border-blue-800">
        <CardContent className="py-3 px-4">
          <div className="flex items-start gap-2 text-xs text-muted-foreground">
            <Info size={13} className="mt-0.5 shrink-0 text-blue-500" />
            <div>
              تقارن هذه الصفحة بين <strong>الطريقة التقليدية (نظام الأحمال المناطقي — Marcus/ACI)</strong> المستخدمة في التطبيق
              و<strong>محرك FEM الجديد (Mindlin-Reissner)</strong> لتوزيع الأحمال وعزوم البلاطات.
              يعتمد FEM على الشبكة الإنشائية ويستخرج الأحمال الموزعة w(x) على كل جسر بدقة أعلى من الطريقة التقليدية.
            </div>
          </div>
        </CardContent>
      </Card>

      {/* Compute trigger */}
      {!computed ? (
        <Card>
          <CardContent className="py-10 text-center">
            <p className="text-sm text-muted-foreground mb-4">
              اضغط لتشغيل محرك FEM وعرض جداول المقارنة
            </p>
            <Button onClick={handleCompute} disabled={computing} className="min-h-[44px]">
              <Calculator size={16} className="mr-2" />
              {computing ? 'جاري الحساب…' : 'تشغيل المقارنة بمحرك FEM'}
            </Button>
            {computing && (
              <p className="text-xs text-muted-foreground mt-3 animate-pulse">
                جاري حل نظام المعادلات… قد يستغرق بضع ثوانٍ
              </p>
            )}
          </CardContent>
        </Card>
      ) : (
        <Button variant="outline" size="sm" onClick={() => { setComputed(false); setBeamResults([]); setSlabResults([]); }}>
          إعادة الحساب
        </Button>
      )}

      {/* ── TABLE 1: Beam Load Comparison ─────────────────────────────────── */}
      {computed && (
        <Card>
          <CardHeader className="pb-2">
            <CardTitle className="text-sm flex items-center gap-2">
              مقارنة توزيع الأحمال على الجسور
              <Badge variant="outline" className="text-[10px]">kN/m · kN</Badge>
            </CardTitle>
            <p className="text-xs text-muted-foreground mt-1">
              الطريقة التقليدية: حمل موزع ثابت (DL + LL) × البحر.
              محرك FEM: حمل موزع متغير w(x) مستخرج من توازن ردود الأفعال العقدية.
            </p>
          </CardHeader>
          <CardContent className="overflow-x-auto">
            {beamRows.length === 0 ? (
              <p className="text-xs text-muted-foreground py-4 text-center">
                لا توجد جسور متصلة ببلاطات — لا يمكن إجراء مقارنة FEM
              </p>
            ) : (
              <Table>
                <TableHeader>
                  <TableRow>
                    <TableHead className="text-xs">الجسر</TableHead>
                    <TableHead className="text-xs">البحر (م)</TableHead>
                    {/* Old method */}
                    <TableHead className="text-xs bg-muted/40">DL قديم (kN/m)</TableHead>
                    <TableHead className="text-xs bg-muted/40">LL قديم (kN/m)</TableHead>
                    <TableHead className="text-xs bg-muted/40 font-semibold">مجموع قديم (kN)</TableHead>
                    {/* FEM */}
                    <TableHead className="text-xs bg-blue-500/10">FEM متوسط (kN/m)</TableHead>
                    <TableHead className="text-xs bg-blue-500/10">FEM ذروة (kN/m)</TableHead>
                    <TableHead className="text-xs bg-blue-500/10 font-semibold">مجموع FEM (kN)</TableHead>
                    {/* Diff */}
                    <TableHead className="text-xs">الفرق</TableHead>
                  </TableRow>
                </TableHeader>
                <TableBody>
                  {beamRows.map(row => (
                    <TableRow key={row.beamId}>
                      <TableCell className="font-mono text-xs font-semibold">{row.beamId}</TableCell>
                      <TableCell className="font-mono text-xs">{row.span.toFixed(2)}</TableCell>
                      {/* Old */}
                      <TableCell className="font-mono text-xs bg-muted/20">{row.oldDL.toFixed(2)}</TableCell>
                      <TableCell className="font-mono text-xs bg-muted/20">{row.oldLL.toFixed(2)}</TableCell>
                      <TableCell className="font-mono text-xs bg-muted/20 font-semibold">{row.oldTotal.toFixed(2)}</TableCell>
                      {/* FEM */}
                      <TableCell className="font-mono text-xs bg-blue-500/5">{row.femAvg.toFixed(2)}</TableCell>
                      <TableCell className="font-mono text-xs bg-blue-500/5">{row.femPeak.toFixed(2)}</TableCell>
                      <TableCell className="font-mono text-xs bg-blue-500/5 font-semibold">{row.femTotal.toFixed(2)}</TableCell>
                      {/* Diff */}
                      <TableCell>{diffBadge(row.diff)}</TableCell>
                    </TableRow>
                  ))}
                </TableBody>
              </Table>
            )}
          </CardContent>
        </Card>
      )}

      {/* ── TABLE 2: Slab Moment Comparison ───────────────────────────────── */}
      {computed && (
        <Card>
          <CardHeader className="pb-2">
            <CardTitle className="text-sm flex items-center gap-2">
              مقارنة عزوم البلاطات عند المنتصف
              <Badge variant="outline" className="text-[10px]">kN·م/م</Badge>
            </CardTitle>
            <p className="text-xs text-muted-foreground mt-1">
              الطريقة التقليدية: معامل Marcus/ACI × q<sub>service</sub> × lx².
              محرك FEM: عزوم Mx/My عند أقرب نقطة غاوس للمنتصف.
              القيم بالخدمة (غير مضاعفة).
            </p>
          </CardHeader>
          <CardContent className="overflow-x-auto">
            {slabMomentRows.length === 0 ? (
              <p className="text-xs text-muted-foreground py-4 text-center">لا توجد بلاطات للمقارنة</p>
            ) : (
              <Table>
                <TableHeader>
                  <TableRow>
                    <TableHead className="text-xs">البلاطة</TableHead>
                    <TableHead className="text-xs">lx × ly (م)</TableHead>
                    <TableHead className="text-xs">β = ly/lx</TableHead>
                    <TableHead className="text-xs">النوع</TableHead>
                    {/* Old */}
                    <TableHead className="text-xs bg-muted/40">α<sub>قصير</sub></TableHead>
                    <TableHead className="text-xs bg-muted/40">Mx قديم (kN·م/م)</TableHead>
                    <TableHead className="text-xs bg-muted/40">My قديم (kN·م/م)</TableHead>
                    {/* FEM */}
                    <TableHead className="text-xs bg-blue-500/10">Mx FEM (kN·م/م)</TableHead>
                    <TableHead className="text-xs bg-blue-500/10">My FEM (kN·م/م)</TableHead>
                    <TableHead className="text-xs bg-blue-500/10">Mxy FEM</TableHead>
                    {/* Diff */}
                    <TableHead className="text-xs">Δ Mx</TableHead>
                    <TableHead className="text-xs">Δ My</TableHead>
                  </TableRow>
                </TableHeader>
                <TableBody>
                  {slabMomentRows.map(row => (
                    <TableRow key={row.slabId}>
                      <TableCell className="font-mono text-xs font-semibold">{row.slabId}</TableCell>
                      <TableCell className="font-mono text-xs">{row.lx_m.toFixed(2)} × {row.ly_m.toFixed(2)}</TableCell>
                      <TableCell className="font-mono text-xs">{row.beta.toFixed(2)}</TableCell>
                      <TableCell className="text-xs">
                        {row.isOneWay
                          ? <Badge variant="outline" className="text-[10px] border-orange-400/50 text-orange-600">أحادية الاتجاه</Badge>
                          : <Badge variant="outline" className="text-[10px] border-blue-400/50 text-blue-600">ثنائية الاتجاه</Badge>}
                      </TableCell>
                      {/* Old */}
                      <TableCell className="font-mono text-xs bg-muted/20">{row.shortCoeff.toFixed(4)}</TableCell>
                      <TableCell className="font-mono text-xs bg-muted/20">{row.oldMx.toFixed(3)}</TableCell>
                      <TableCell className="font-mono text-xs bg-muted/20">{row.oldMy.toFixed(3)}</TableCell>
                      {/* FEM */}
                      <TableCell className="font-mono text-xs bg-blue-500/5 font-semibold">{row.fem.Mx.toFixed(3)}</TableCell>
                      <TableCell className="font-mono text-xs bg-blue-500/5 font-semibold">{row.fem.My.toFixed(3)}</TableCell>
                      <TableCell className="font-mono text-xs bg-blue-500/5 text-muted-foreground">{row.fem.Mxy.toFixed(3)}</TableCell>
                      {/* Diffs */}
                      <TableCell>
                        {row.oldMx > 1e-4
                          ? <span className={`text-xs font-mono ${pctColor(row.diffMx)}`}>{row.diffMx > 0 ? '+' : ''}{row.diffMx.toFixed(1)}%</span>
                          : <span className="text-muted-foreground text-xs">—</span>}
                      </TableCell>
                      <TableCell>
                        {row.oldMy > 1e-4
                          ? <span className={`text-xs font-mono ${pctColor(row.diffMy)}`}>{row.diffMy > 0 ? '+' : ''}{row.diffMy.toFixed(1)}%</span>
                          : <span className="text-muted-foreground text-xs">—</span>}
                      </TableCell>
                    </TableRow>
                  ))}
                </TableBody>
              </Table>
            )}
          </CardContent>

          {/* Legend */}
          <div className="px-4 pb-3 flex flex-wrap gap-3 text-[10px] text-muted-foreground">
            <span className="flex items-center gap-1"><span className="inline-block w-3 h-3 rounded bg-green-500/20 border border-green-500/40" /> فرق ≤ 5% (ممتاز)</span>
            <span className="flex items-center gap-1"><span className="inline-block w-3 h-3 rounded bg-yellow-500/20 border border-yellow-500/40" /> فرق 5–15% (مقبول)</span>
            <span className="flex items-center gap-1"><span className="inline-block w-3 h-3 rounded bg-red-500/20 border border-red-500/40" /> فرق &gt; 15% (مراجعة)</span>
          </div>
        </Card>
      )}
    </div>
  );
};

export default LoadComparisonPanel;

// ─────────────────────────────────────────────────────────────────────────────
// Local helper – trapezoidal integration
// ─────────────────────────────────────────────────────────────────────────────

function trapz(x: number[], y: number[]): number {
  let s = 0;
  for (let i = 0; i < x.length - 1; i++) {
    s += 0.5 * (y[i] + y[i + 1]) * (x[i + 1] - x[i]);
  }
  return s;
}
