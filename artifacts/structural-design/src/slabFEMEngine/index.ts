/**
 * slabFEMEngine – Public API
 * ============================================================
 *
 * This module is a COMPLETELY ISOLATED add-on engine.
 * It does NOT touch any existing analysis logic.
 * Existing code remains unchanged and fully functional.
 *
 * ── Public surface ──────────────────────────────────────────
 *
 *   getBeamLoadsFromSlab(model)   → BeamLoadResult[]
 *     Full FEM-based slab-to-beam load transfer.
 *     Phase 1: mesh + solve + internal forces.
 *     Phase 2: edge force extraction.
 *     Phase 3: beam load distribution w(x), force-conservative.
 *
 *   getSlabCenterMoments(model)   → SlabMomentComparison[]
 *     FEM center Mx/My for each slab (for comparison UI).
 *
 *   runPhase1Validation()         → ValidationReport
 *     Self-contained simply-supported slab test.
 *     Must pass before any production use.
 *
 *   runCase1Regression()          → { passed, equilibriumError_pct, notes }
 *     Phase 2+3 regression: 5×5 m slab, 4 edge beams, q=10 kN/m².
 *     Sum of beam loads must equal total applied load within 5 %.
 *
 *   runCase2Validation()          → Case2Report
 *     NEW: 6×6 m slab with internal beam at mid-span.
 *     Validates non-uniform load distribution and stiffness-based transfer.
 *
 *   runFullValidation()           → FullValidationReport
 *     Runs Phase 1 + Case 1 + Case 2 and returns combined report.
 *
 * ── Integration hook ────────────────────────────────────────
 *
 *   Set  useFEMSlabLoad: true  in the model to activate the engine.
 *   Set  comparisonMode: true  to return both methods side-by-side.
 *
 * ── Unit conventions ────────────────────────────────────────
 *   Internal computation: mm, N, rad.
 *   All OUTPUT values:    kN, m, kN/m, kN·m/m  (engineering units).
 *
 * ── Force conservation ──────────────────────────────────────
 *   Phase 3 normalizes the w(x) profile so that:
 *     ∫ w(x) dx  =  Σ F_i  (exact FEM nodal forces)
 *   The normalization scale factor is within 1–3 % of unity for meshes
 *   with ≥ 4 divisions/m.  A console warning is emitted if it exceeds 5 %.
 */

export type {
  FEMInputModel,
  BeamLoadResult,
  DistributedLoadPoint,
  NodalForce,
  ValidationReport,
  Phase1Result,
  StressResultants,
  ElementForceResult,
} from './types';

import type { FEMInputModel, BeamLoadResult, ValidationReport } from './types';
import { meshSlab, meshSummary } from './mesh';
import { assembleSystem, reconstructDisplacements, extractReactions } from './assembler';
import { solve } from './solver';
import { computeInternalForces } from './internalForces';
import { extractBeamEdgeForces, validatePhase2 } from './edgeForces';
import { mapEdgeForcesToBeams } from './beamMapper';
import {
  runPhase1Validation,
  runCase1Regression,
  runCase2Validation,
  runFullValidation,
  assertPhase1Valid,
} from './validation';

export type { Case2Report, Case2BeamResult, FullValidationReport } from './validation';

export {
  runPhase1Validation,
  runCase1Regression,
  runCase2Validation,
  runFullValidation,
  assertPhase1Valid,
};

// ─────────────────────────────────────────────────────────────────────────────
// Slab moment comparison type (FEM center moments)
// ─────────────────────────────────────────────────────────────────────────────

export interface SlabMomentComparison {
  slabId: string;
  /** Shorter span (m). */
  lx_m: number;
  /** Longer span (m). */
  ly_m: number;
  /** Aspect ratio ly/lx. */
  beta: number;
  isOneWay: boolean;
  /** FEM Mindlin-Reissner center moments (kN·m/m). */
  fem: {
    Mx: number;
    My: number;
    Mxy: number;
  };
}

// ─────────────────────────────────────────────────────────────────────────────
// Main entry point
// ─────────────────────────────────────────────────────────────────────────────

export function getBeamLoadsFromSlab(model: FEMInputModel): BeamLoadResult[] {
  const {
    slabs, beams, columns,
    slabProps, mat,
    meshDensity = 4,
  } = model;

  const comparisonMode = (model as FEMInputModel & { comparisonMode?: boolean })
    .comparisonMode ?? false;

  // Surface pressure: q = ownWeight + finishLoad + liveLoad
  const ownWeight_kNm2 = (slabProps.thickness / 1000) * mat.gamma;
  const q_kNm2 = ownWeight_kNm2 + slabProps.finishLoad + slabProps.liveLoad;
  const q_Nmm2 = q_kNm2 * 1e-3;   // kN/m² → N/mm²

  const allEdgeForces = [];

  // ── Phase 1 + 2: per-slab solve ──────────────────────────────────────────
  for (const slab of slabs) {
    // ─ Meshing ─────────────────────────────────────────────────────────────
    const mesh = meshSlab(slab, beams, columns, meshDensity);
    console.log(`[slabFEMEngine] ${meshSummary(mesh)}`);

    // ─ Assembly ────────────────────────────────────────────────────────────
    const sys = assembleSystem(mesh, slabProps, mat, q_Nmm2);

    if (sys.freeDOFs.length === 0) {
      console.warn(`[slabFEMEngine] Slab ${slab.id}: no free DOFs — check BCs.`);
      continue;
    }

    // ─ Solve ───────────────────────────────────────────────────────────────
    const solveResult = solve(sys.K_ff.slice(), sys.F_f.slice());

    if (!solveResult.converged) {
      console.warn(
        `[slabFEMEngine] Slab ${slab.id}: solver residual = ` +
        `${solveResult.maxResidual.toExponential(3)}`,
      );
    }

    const d_full = reconstructDisplacements(
      solveResult.d, sys.freeDOFs, sys.nDOF,
    );

    // ─ Phase 1 debug log ───────────────────────────────────────────────────
    const reactions = extractReactions(
      sys.K_full, d_full, sys.F_full, sys.fixedDOFs, sys.nDOF,
    );
    const slabArea_mm2 = Math.abs(slab.x2 - slab.x1) * Math.abs(slab.y2 - slab.y1);
    const totalApplied_kN = q_Nmm2 * slabArea_mm2 * 1e-3;

    let totalReaction_N_raw = 0;
    for (const [dof, force] of reactions) {
      if (dof % 3 === 0) totalReaction_N_raw += force;
    }
    const totalReaction_kN = Math.abs(totalReaction_N_raw * 1e-3);
    const eqErr = Math.abs(totalApplied_kN - totalReaction_kN)
                  / Math.max(totalApplied_kN, 1e-6) * 100;

    console.log(
      `[Phase 1] Slab ${slab.id}: ` +
      `Applied=${totalApplied_kN.toFixed(2)} kN, ` +
      `Reactions=${totalReaction_kN.toFixed(2)} kN, ` +
      `Error=${eqErr.toFixed(2)} %`,
    );

    // ─ Phase 2: edge forces (signed nodal reactions w/ junction splitting) ──
    const edgeForces = extractBeamEdgeForces(
      mesh, sys.K_full, d_full, sys.F_full, sys.fixedDOFs, sys.nDOF, beams,
    );

    validatePhase2(edgeForces, totalReaction_kN);

    allEdgeForces.push(...edgeForces);
  }

  // ── Phase 3: map to beams (force-conservative) ────────────────────────────
  const beamLoads = mapEdgeForcesToBeams(allEdgeForces, beams, {
    comparisonMode,
    slabs,
    slabProps,
    mat,
  });

  return beamLoads;
}

// ─────────────────────────────────────────────────────────────────────────────
// Slab center moment extraction (for comparison UI)
// Returns FEM Mindlin-Reissner center Mx/My for each slab.
// The caller (LoadComparisonPanel) pairs these with the old Marcus/ACI method.
// ─────────────────────────────────────────────────────────────────────────────

export function getSlabCenterMoments(model: FEMInputModel): SlabMomentComparison[] {
  const { slabs, beams, columns, slabProps, mat, meshDensity = 4 } = model;

  const ownWeight_kNm2 = (slabProps.thickness / 1000) * mat.gamma;
  const q_kNm2 = ownWeight_kNm2 + slabProps.finishLoad + slabProps.liveLoad;
  const q_Nmm2 = q_kNm2 * 1e-3;

  const results: SlabMomentComparison[] = [];

  for (const slab of slabs) {
    const lx_mm = Math.min(Math.abs(slab.x2 - slab.x1), Math.abs(slab.y2 - slab.y1));
    const ly_mm = Math.max(Math.abs(slab.x2 - slab.x1), Math.abs(slab.y2 - slab.y1));
    const lx_m  = lx_mm / 1000;
    const ly_m  = ly_mm / 1000;
    const beta  = Math.max(lx_m > 0 ? ly_m / lx_m : 1.0, 1.0);
    const isOneWay = beta > 2;

    // ── FEM solve ────────────────────────────────────────────────────────────
    const mesh = meshSlab(slab, beams, columns, meshDensity);
    const sys  = assembleSystem(mesh, slabProps, mat, q_Nmm2);

    let femMx = 0, femMy = 0, femMxy = 0;

    if (sys.freeDOFs.length > 0) {
      const solveResult = solve(sys.K_ff.slice(), sys.F_f.slice());
      const d_full = reconstructDisplacements(solveResult.d, sys.freeDOFs, sys.nDOF);
      const forceResults = computeInternalForces(mesh, d_full, slabProps, mat);

      // Center of slab (mm)
      const cx_mm = (slab.x1 + slab.x2) / 2;
      const cy_mm = (slab.y1 + slab.y2) / 2;

      // Find nearest Gauss-point result to the slab centre
      let minDist = Infinity;
      for (const fr of forceResults) {
        const dist = Math.hypot(fr.x - cx_mm, fr.y - cy_mm);
        if (dist < minDist) {
          minDist = dist;
          femMx  = fr.resultants.Mx;
          femMy  = fr.resultants.My;
          femMxy = fr.resultants.Mxy;
        }
      }
    }

    results.push({
      slabId: slab.id,
      lx_m, ly_m, beta, isOneWay,
      fem: { Mx: Math.abs(femMx), My: Math.abs(femMy), Mxy: Math.abs(femMxy) },
    });
  }

  return results;
}

// ─────────────────────────────────────────────────────────────────────────────
// Convenience: run Phase-1 validation and return the report.
// ─────────────────────────────────────────────────────────────────────────────

export function validatePhase1(meshDensity?: number): ValidationReport {
  return runPhase1Validation(meshDensity);
}
