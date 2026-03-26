/**
 * slabFEMEngine – Phase 1 Types
 *
 * All FEM-specific types for the slab load transfer engine.
 * Re-exports the existing app model types so the rest of the
 * engine only needs to import from this single file.
 */

// ─── Re-export existing app types (read-only – never modified) ───────────────
export type { Slab, Beam, Column, SlabProps, MatProps }
  from '@/lib/structuralEngine';

// ─── Input model ─────────────────────────────────────────────────────────────

import type { Slab, Beam, Column, SlabProps, MatProps }
  from '@/lib/structuralEngine';

export interface FEMInputModel {
  slabs:     Slab[];
  beams:     Beam[];
  columns:   Column[];
  slabProps: SlabProps;
  mat:       MatProps;
  /** Target element divisions per metre in each direction (default 4). */
  meshDensity?: number;
  /**
   * Phase-4 / Phase-5 mode switch.
   * true  → use stress-based edge transfer (σ·n integration).
   * false → use reaction-based extraction (R = K·d − F) — Phase 2 (default).
   * Both produce BeamEdgeForces[] consumed by Phase 3 unchanged.
   */
  useStressBasedTransfer?: boolean;
  /**
   * Phase-5 internal mode (only relevant when useStressBasedTransfer = true).
   * "shear-only" → Phase 4 behaviour: transfer Fz only (no moments).
   * "full"       → Phase 5 behaviour: transfer Fz + Mx + My (default).
   */
  stressMode?: 'shear-only' | 'full';

  // ── Phase 8: True DOF Merging (Constraint Elimination) ───────────────────

  /**
   * Phase-8 mode switch.
   * true  → use true DOF merging (shared nodes get unified DOF indices; no penalty).
   * false → use Phase-7 penalty coupling (default for backward compatibility).
   *
   * When true, the public entry point is getMergedBeamSlabResults().
   * Phase 7 (getCoupledBeamSlabResults) is unaffected by this flag.
   */
  useMergedDOF?: boolean;

  // ── Phase 6: Rotational Coupling ──────────────────────────────────────────

  /**
   * Phase-6 mode switch.
   * true  → apply rotational spring coupling (beam stiffness feedback on slab moments).
   * false → skip coupling (Phase 5 moments used unchanged — default).
   *
   * Requires useStressBasedTransfer = true + stressMode = 'full' to have
   * meaningful Phase-5 moments to correct.  If Phase-5 data is absent,
   * Phase-6 correction will be zero (no-op).
   */
  useRotationalCoupling?: boolean;

  /**
   * Rotational spring tuning factor α in kθ = α · E_c · I / L.
   * Default 1.0.  Range [0.1, 10.0] recommended.
   * Values > 1 increase the coupling effect (approach rigid connection).
   * Values < 1 reduce coupling (approach pinned / semi-rigid).
   */
  couplingAlpha?: number;
}

// ─── Mesh ─────────────────────────────────────────────────────────────────────

/**
 * A single FEM node.
 * The slab lies in the global X-Y plane; Z is vertical (gravity direction).
 * Coordinates are in millimetres (same units as existing models).
 */
export interface FEMNode {
  id: number;
  x:  number;   // mm
  y:  number;   // mm
  /** True → UZ (and rotations) are constrained (column or beam support). */
  isFixed: boolean;
  /** True → a column sits at this node (its reaction goes to the column). */
  atColumn: boolean;
  /** Which beam this node sits on, if any. */
  beamId:  string | null;
  /** Normalised position along that beam [0, 1]. */
  beamPos: number;
}

/**
 * A 4-node bilinear quad element.
 * Nodes are ordered counter-clockwise starting from the bottom-left corner:
 *   n0 (x0,y0) → n1 (x1,y0) → n2 (x1,y1) → n3 (x0,y1)
 */
export interface FEMElement {
  id:      number;
  nodeIds: [number, number, number, number];
  slabId:  string;
}

export interface SlabMesh {
  slabId:   string;
  nodes:    FEMNode[];
  elements: FEMElement[];
  /** Sorted unique X coordinates of the mesh grid lines (mm). */
  xLines: number[];
  /** Sorted unique Y coordinates of the mesh grid lines (mm). */
  yLines: number[];
}

// ─── DOF numbering ───────────────────────────────────────────────────────────

/**
 * Phase-1 element uses 3 DOF per node: [UZ, RX, RY].
 *   UZ  = transverse deflection (mm)
 *   RX  = rotation about X-axis (rad)  ≈ ∂w/∂y  (Kirchhoff limit)
 *   RY  = rotation about Y-axis (rad)  ≈ −∂w/∂x (Kirchhoff limit)
 *
 * Global DOF index for node n:  base = n * 3
 *   DOF_UZ = base + 0
 *   DOF_RX = base + 1
 *   DOF_RY = base + 2
 */
export const DOF_PER_NODE = 3;

// ─── Element stiffness ────────────────────────────────────────────────────────

/** 12 × 12 element stiffness matrix (row-major flat array). */
export type Ke = number[];   // length = 144

// ─── Internal force resultants ────────────────────────────────────────────────

export interface StressResultants {
  /** Bending moment per unit length about Y-axis  (kNm/m). */
  Mx:  number;
  /** Bending moment per unit length about X-axis  (kNm/m). */
  My:  number;
  /** Twisting moment per unit length (kNm/m). */
  Mxy: number;
  /** Transverse shear force per unit length in XZ plane (kN/m). */
  Qx:  number;
  /** Transverse shear force per unit length in YZ plane (kN/m). */
  Qy:  number;
}

/** Resultants at a single evaluation point inside an element. */
export interface ElementForceResult {
  elementId: number;
  slabId:    string;
  /** Gauss-point coordinates in global space (mm). */
  x: number;
  y: number;
  resultants: StressResultants;
}

// ─── Phase-1 solver output ────────────────────────────────────────────────────

export interface Phase1Result {
  mesh:           SlabMesh[];
  /** Full displacement vector (one entry per global DOF). */
  displacements:  number[];
  forceResults:   ElementForceResult[];
  /** Validation report – printed to console and available for UI. */
  validation:     ValidationReport;
}

export interface ValidationReport {
  totalAppliedLoad_kN:   number;
  totalReactions_kN:     number;
  equilibriumError_pct:  number;
  /** Peak |Mx| at slab centre vs. analytical solution (simply supported). */
  momentCheck?: {
    computed_kNm_per_m:   number;
    analytical_kNm_per_m: number;
    error_pct:            number;
  };
  passed: boolean;
  notes:  string[];
}

// ─── Phase-2 / Phase-3 (output stubs – filled in later phases) ───────────────

export interface DistributedLoadPoint {
  /** Distance from beam start node (m). */
  position: number;
  /** Load intensity (kN/m). */
  w: number;
}

export interface NodalForce {
  position: number;   // m from beam start
  Fz:  number;        // kN
  My:  number;        // kNm
}

export interface BeamLoadResult {
  beamId: string;
  loads: {
    type:   'distributed';
    values: DistributedLoadPoint[];
  };
  nodalForces: NodalForce[];
  /** Populated only when comparisonMode = true. */
  oldMethodLoad?:   { deadLoad: number; liveLoad: number };
  femMethodLoad?:   { avgLoad: number; peakLoad: number };
  differencePercent?: number;
}
