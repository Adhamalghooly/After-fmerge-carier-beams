// Quick runner for FEM comparison analysis
// Run with: node --experimental-vm-modules run_analysis.mjs

import { createServer } from 'vite';
import { resolve } from 'path';

const server = await createServer({
  root: resolve(process.cwd()),
  configFile: resolve(process.cwd(), 'vite.config.ts'),
  server: { port: 20599 },
  logLevel: 'silent',
});
await server.listen();

const { getBeamLoadsFromSlab, getSlabCenterMoments } = await server.ssrLoadModule('/src/slabFEMEngine/index.ts');
const { calculateBeamLoads, generateBeams, generateColumns } = await server.ssrLoadModule('/src/lib/structuralEngine.ts');

const slabs = [
  { id: 'S1', x1: 0, y1: 0, x2: 5, y2: 4, storyId: 'st1' },
  { id: 'S2', x1: 5, y1: 0, x2: 9, y2: 4, storyId: 'st1' },
];
const columns   = generateColumns(slabs);
const beams     = generateBeams(slabs, columns);
const slabProps = { thickness: 180, finishLoad: 1.5, liveLoad: 2.0 };
const mat       = { gamma: 25, fc: 25, fy: 420 };

console.log('\n══ الطريقة القديمة (منطقة نفوذ) ══');
for (const beam of beams) {
  const loads = calculateBeamLoads(beam, slabs, slabProps, mat);
  const total = (loads.deadLoad + loads.liveLoad) * beam.length;
  console.log(`${beam.id}  dir=${beam.direction}  L=${beam.length.toFixed(2)}m  DL=${loads.deadLoad.toFixed(3)} LL=${loads.liveLoad.toFixed(3)} kN/m  Total=${total.toFixed(2)} kN`);
}

console.log('\n══ محرك FEM ══');
const femResults = getBeamLoadsFromSlab({ slabs, beams, columns, slabProps, mat, meshDensity: 6, comparisonMode: true });
for (const r of femResults) {
  if (!r.femMethodLoad) continue;
  const beam = beams.find(b => b.id === r.beamId);
  if (!beam) continue;
  const positions = r.loads.values.map(p => p.position);
  const weights   = r.loads.values.map(p => p.w);
  let femTotal = 0;
  for (let i = 0; i < positions.length - 1; i++) {
    femTotal += (weights[i] + weights[i+1]) / 2 * (positions[i+1] - positions[i]);
  }
  const oldTotal = (r.oldMethodLoad.deadLoad + r.oldMethodLoad.liveLoad) * beam.length;
  const diff = oldTotal > 0.01 ? ((femTotal - oldTotal) / oldTotal * 100) : 0;
  const wArr = r.loads.values.map(p => p.w);
  console.log(`${r.beamId}  dir=${beam.direction}  L=${beam.length.toFixed(2)}m  avg=${r.femMethodLoad.avgLoad.toFixed(3)} peak=${r.femMethodLoad.peakLoad.toFixed(3)} kN/m  Total=${femTotal.toFixed(2)} kN  Old=${oldTotal.toFixed(2)} kN  Diff=${diff.toFixed(1)}%  wMin=${Math.min(...wArr).toFixed(3)} wMax=${Math.max(...wArr).toFixed(3)}`);
}

console.log('\n══ عزوم البلاطات ══');
const moments = getSlabCenterMoments({ slabs, beams, columns, slabProps, mat, meshDensity: 6 });
const q_service = (slabProps.thickness / 1000) * mat.gamma + slabProps.finishLoad + slabProps.liveLoad;
for (const m of moments) {
  console.log(`${m.slabId}  lx=${m.lx_m.toFixed(2)}m  ly=${m.ly_m.toFixed(2)}m  β=${m.beta.toFixed(2)}  Mx_FEM=${m.fem.Mx.toFixed(4)}  My_FEM=${m.fem.My.toFixed(4)} kN·m/m  q=${q_service.toFixed(2)} kN/m²`);
}

await server.close();
process.exit(0);
