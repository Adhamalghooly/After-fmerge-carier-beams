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
 *     Phase 3: beam load distribution w(x).
 *
 *   runPhase1Validation()         → ValidationReport
 *     Self-contained simply-supported slab test.
 *     Must pass before any production use.
 *
 * ── Integration hook ────────────────────────────────────────
 *
 *   Set  useFEMSlabLoad: true  in the model to activate the engine.
 *   Set  comparisonMode: true  to return both methods side-by-side.
 *
 * ── Unit conventions ────────────────────────────────────────
 *   Internal computation: mm, N, rad.
 *   All OUTPUT values:    kN, m, kN/m, kN·m/m  (engineering units).
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
// computeInternalForces is available for post-processing (moments/shears) — reserved for Phase 4
void computeInternalForces;
import { mapEdgeForcesToBeams } from './beamMapper';
import { runPhase1Validation } from './validation';

export { runPhase1Validation };

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
    // Reactions oppose the load (negative sign = upward support) → compare magnitudes
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

    // Phase-2 validation
    validatePhase2(edgeForces, totalReaction_kN);

    allEdgeForces.push(...edgeForces);
  }

  // ── Phase 3: map to beams ─────────────────────────────────────────────────
  const beamLoads = mapEdgeForcesToBeams(allEdgeForces, beams, {
    comparisonMode,
    slabs,
    slabProps,
    mat,
  });

  return beamLoads;
}

// ─────────────────────────────────────────────────────────────────────────────
// Convenience: run Phase-1 validation and return the report.
// Call this during development to confirm the engine is correct.
// ─────────────────────────────────────────────────────────────────────────────

export function validatePhase1(meshDensity?: number): ValidationReport {
  return runPhase1Validation(meshDensity);
}
