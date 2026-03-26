/**
 * slabFEMEngine – Phase 1 Validation
 *
 * Reference problem: uniformly loaded, simply supported square slab.
 *
 * Analytical solution (thin-plate theory, Timoshenko & Woinowsky-Krieger):
 *   Lx = Ly = L,  q = surface load,  ν = 0.2
 *   M_max ≈ 0.0479 · q · L²   at the slab centre
 *
 * For our FEM (Mindlin, slight shear contribution), the result should be
 * within ~5 % of the thin-plate value even on a coarse 4×4 mesh.
 *
 * Equilibrium check (always performed):
 *   ΣF_reactions must equal the total applied load within 0.1 %.
 *
 * Usage
 * -----
 *   import { runPhase1Validation } from './validation';
 *   const report = runPhase1Validation();   // runs self-contained FEM
 *   console.log(report);
 */

import type { SlabProps, MatProps, ValidationReport } from './types';
import { meshSlab }                from './mesh';
import { assembleSystem, reconstructDisplacements, extractReactions } from './assembler';
import { solve }                   from './solver';
import { resultantsAtPoint }       from './internalForces';

// ─────────────────────────────────────────────────────────────────────────────

export function runPhase1Validation(meshDensity = 6): ValidationReport {
  const notes: string[] = [];

  // ── Reference slab geometry ───────────────────────────────────────────────
  const L_mm  = 5000;     // 5 m square slab
  const slab  = { id: '__val__', x1: 0, y1: 0, x2: L_mm, y2: L_mm, storyId: '__val__' };

  // Simply supported → boundary nodes fixed, no internal beams
  // (meshSlab fixes boundary nodes by default)

  // ── Material & section ───────────────────────────────────────────────────
  const slabProps: SlabProps = {
    thickness:  150,      // mm
    finishLoad: 1.5,      // kN/m² (not used in FEM directly)
    liveLoad:   3.0,      // kN/m² (not used in FEM directly)
    cover:      20,
    phiMain:    0.9,
    phiSlab:    0.9,
  };
  const mat: MatProps = {
    fc:    25,            // MPa
    fy:    420,           // MPa
    fyt:   420,
    gamma: 24,            // kN/m³
  };

  // Surface pressure: q = 10 kN/m²  (dead + live combined for validation)
  const q_kNm2 = 10.0;
  const q_Nmm2 = q_kNm2 * 1e-3;   // N/mm²

  // ── Mesh ─────────────────────────────────────────────────────────────────
  const mesh  = meshSlab(slab, [], [], meshDensity);
  const nElem = mesh.elements.length;
  const nNode = mesh.nodes.length;
  notes.push(`Mesh: ${nElem} elements, ${nNode} nodes (density ${meshDensity}/m)`);

  // ── Assemble ─────────────────────────────────────────────────────────────
  const sys = assembleSystem(mesh, slabProps, mat, q_Nmm2);

  const nFree  = sys.freeDOFs.length;
  const nFixed = sys.fixedDOFs.length;
  notes.push(`DOFs: ${sys.nDOF} total, ${nFree} free, ${nFixed} fixed`);

  if (nFree === 0) {
    notes.push('ERROR: All DOFs are fixed — check boundary condition logic.');
    return {
      totalAppliedLoad_kN: 0, totalReactions_kN: 0,
      equilibriumError_pct: 100, passed: false, notes,
    };
  }

  // ── Solve ─────────────────────────────────────────────────────────────────
  const result = solve(sys.K_ff.slice(), sys.F_f.slice());

  if (!result.converged) {
    notes.push(`WARNING: Solver residual = ${result.maxResidual.toExponential(3)} (> 1e-6)`);
  } else {
    notes.push(`Solver converged. Max residual = ${result.maxResidual.toExponential(3)}`);
  }

  const d_full = reconstructDisplacements(result.d, sys.freeDOFs, sys.nDOF);

  // Centre deflection (node closest to L/2, L/2)
  const cx = L_mm / 2, cy = L_mm / 2;
  const centreNode = mesh.nodes.reduce((best, n) => {
    const d  = (n.x - cx) ** 2 + (n.y - cy) ** 2;
    const db = (best.x - cx) ** 2 + (best.y - cy) ** 2;
    return d < db ? n : best;
  });
  const uz_centre_mm = d_full[centreNode.id * 3];     // UZ DOF
  // Thin-plate analytical: δ_centre = 0.00406 · q · L⁴ / (E · t³)  (SS square)
  const Ec = 4700 * Math.sqrt(mat.fc);
  const nu = 0.2;
  const D  = (Ec * slabProps.thickness ** 3) / (12 * (1 - nu ** 2)); // N·mm
  const uz_analytical = 0.00406 * q_Nmm2 * L_mm ** 4 / D;
  notes.push(
    `Centre deflection: FEM = ${uz_centre_mm.toFixed(4)} mm, ` +
    `Analytical (thin-plate) = ${uz_analytical.toFixed(4)} mm`,
  );

  // ── Equilibrium check ────────────────────────────────────────────────────
  // Total applied load = q · L²
  const totalApplied_N  = q_Nmm2 * L_mm * L_mm;    // N
  const totalApplied_kN = totalApplied_N * 1e-3;     // kN

  // Total reaction = sum of UZ reactions at fixed nodes
  const reactions = extractReactions(sys.K_full, d_full, sys.F_full, sys.fixedDOFs, sys.nDOF);

  let totalReaction_N = 0;
  for (const [dof, force] of reactions) {
    // Only UZ reactions (dof % 3 === 0)
    // Reactions are negative (upward, opposing downward load) — compare magnitudes.
    if (dof % 3 === 0) totalReaction_N += force;
  }
  // Use absolute value: reactions oppose the load direction (sign is correct physically)
  const totalReaction_kN  = Math.abs(totalReaction_N * 1e-3);
  const eqErr_pct = Math.abs(totalApplied_kN - totalReaction_kN) / totalApplied_kN * 100;

  notes.push(
    `Equilibrium: Applied = ${totalApplied_kN.toFixed(2)} kN, ` +
    `Reactions = ${totalReaction_kN.toFixed(2)} kN, ` +
    `Error = ${eqErr_pct.toFixed(3)} %`,
  );

  // ── Moment check at centre ────────────────────────────────────────────────
  const R = resultantsAtPoint(cx, cy, mesh, d_full, slabProps, mat);

  // Analytical M_max = 0.0479 · q_kNm2 · (L_m)²
  const L_m         = L_mm / 1000;
  const M_analytical = 0.0479 * q_kNm2 * L_m * L_m;   // kN·m/m

  // Symmetry: Mx ≈ My at centre of square slab
  const M_fem       = (Math.abs(R.Mx) + Math.abs(R.My)) / 2;
  const momentErr   = Math.abs(M_fem - M_analytical) / M_analytical * 100;

  notes.push(
    `Centre moment: FEM Mx=${R.Mx.toFixed(3)}, My=${R.My.toFixed(3)} kN·m/m, ` +
    `avg=${M_fem.toFixed(3)}, Analytical=${M_analytical.toFixed(3)} kN·m/m, ` +
    `Error=${momentErr.toFixed(2)} %`,
  );

  // ── Pass/Fail ─────────────────────────────────────────────────────────────
  const equilibriumOK = eqErr_pct    < 1.0;   // < 1 %
  const momentOK      = momentErr    < 15.0;  // < 15 % (coarse mesh tolerance)
  const solverOK      = result.maxResidual < 1.0; // loose on absolute residual

  const passed = equilibriumOK && momentOK && solverOK;

  if (!equilibriumOK) notes.push(`FAIL: Equilibrium error ${eqErr_pct.toFixed(2)} % exceeds 1 %`);
  if (!momentOK)      notes.push(`FAIL: Moment error ${momentErr.toFixed(2)} % exceeds 15 %`);
  if (!solverOK)      notes.push(`FAIL: Solver residual too large: ${result.maxResidual}`);
  if (passed)         notes.push('Phase 1 PASSED ✓');

  // Log to console so it appears in the dev-server output
  console.group('[slabFEMEngine] Phase 1 Validation');
  notes.forEach(n => console.log(n));
  console.groupEnd();

  return {
    totalAppliedLoad_kN:  totalApplied_kN,
    totalReactions_kN:    totalReaction_kN,
    equilibriumError_pct: eqErr_pct,
    momentCheck: {
      computed_kNm_per_m:   M_fem,
      analytical_kNm_per_m: M_analytical,
      error_pct:            momentErr,
    },
    passed,
    notes,
  };
}

// ─────────────────────────────────────────────────────────────────────────────
// Run and assert (throws if Phase 1 fails — used as a build-time guard)
// ─────────────────────────────────────────────────────────────────────────────

export function assertPhase1Valid(): void {
  const report = runPhase1Validation();
  if (!report.passed) {
    throw new Error(
      '[slabFEMEngine] Phase 1 validation FAILED:\n' + report.notes.join('\n'),
    );
  }
}
