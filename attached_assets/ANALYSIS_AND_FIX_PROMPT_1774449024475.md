# Structural Design Suite — Deep Analysis & Fix Prompt
## Comprehensive Bug Report + Corrective Coding Instructions for AI Assistant

---

## 1. PROJECT OVERVIEW

**Stack:** React 18 + TypeScript + Vite, no backend (pure client-side)
**Core Domain:** Reinforced-concrete frame design per ACI 318-19, with ETABS-comparable 3D direct stiffness method (DSM)

### Key Source Files (all in `src/lib/`):

| File | Purpose | Lines |
|---|---|---|
| `structuralEngine.ts` | Main analysis engine: beam loads, beam-on-beam detection, 2D matrix stiffness analysis, flexure/shear/column design | 2982 |
| `solver3D.ts` | Full 3D direct stiffness solver — 6 DOF/node space-frame | ~1464 |
| `analyze3DColumns.ts` | 3D pattern loading, beam/column envelope extraction, public API: `getFrameResults3D()`, `getColumnLoads3D()` | 397 |
| `matrixStiffness.ts` | 2D continuous-beam matrix stiffness (MSM) with point loads, pattern loading | 476 |
| `autoDesigner.ts` | Auto section sizing (auto-design mode) | — |

---

## 2. ARCHITECTURAL OVERVIEW — TWO ANALYSIS PATHS

The app has **two distinct analysis paths** that MUST be understood before any fix:

### PATH A — 3D Analysis (used in production for column design)
```
getColumnLoads3D() / getFrameResults3D()
  → build3DModelWithPatternLoading()      [analyze3DColumns.ts]
    → analyze3DFrame()                    [solver3D.ts]
  → runPatternEnvelope3D()
```

### PATH B — 2D Iterative Beam-on-Beam (used only when removed columns exist)
```
analyzeWithBeamOnBeam()                  [structuralEngine.ts:888]
  → detectBeamOnBeam()                   [structuralEngine.ts:701]
  → analyzeFrame()                       [structuralEngine.ts:764]
    → envelopeAnalysis()                 [matrixStiffness.ts]
  (iterates until convergence)
```

---

## 3. CRITICAL BUG — THE CORE PROBLEM

### Bug Title: **3D Analysis Path Completely Ignores Beam-on-Beam Point Loads**

**Location:** `src/lib/analyze3DColumns.ts`, function `build3DModelWithPatternLoading()` (lines 50–247)

**Severity:** CRITICAL — Produces unconservative (unsafe) results when secondary beams rest on primary beams at removed-column locations

### What the bug is:

When a column is removed (progressive collapse scenario or design scenario with missing column), secondary beams that previously framed into that column now "hang" in the air. In reality, they must bear on an adjacent primary beam — transferring a concentrated point load (reaction force) to that primary beam.

**The 3D model (`build3DModelWithPatternLoading`) does NOT include these point loads.**

It only creates:
1. Column elements (skipping removed columns)
2. Beam elements with distributed gravity loads (dead + live)

But it **never adds the concentrated forces from secondary beams to primary beams**.

The free node at the removed-column location becomes truly free (zero support, zero load transfer), so the secondary beam simply drops its load silently and the primary beam is unaffected. This produces:
- Under-estimated moments and shears in the primary carrier beam
- Under-designed primary beam cross-sections
- Incorrect (non-conservative) column loads for columns adjacent to the removal

### What ETABS does (correct behavior):

ETABS models beam-on-beam connections as:
1. A **moment release (hinge)** at the carried end of the secondary beam — secondary beam can rotate freely at the bearing point
2. A **concentrated vertical load** at the exact bearing location on the primary beam, equal to the reaction from the secondary beam
3. Both loads (secondary beam load + primary beam own loads) are solved **simultaneously in the global stiffness matrix**

The carried beam's reaction becomes a nodal load on the primary beam's intermediate point. Since MSM/DSM does not naturally have nodes at non-column locations, ETABS inserts a **virtual intermediate node** on the primary beam element at the bearing distance.

### How PATH B (2D) already solves this correctly:

The `analyzeWithBeamOnBeam()` function (structuralEngine.ts:888–1009) does the right thing:
1. Detects beam-on-beam connections via `detectBeamOnBeam()`
2. Adds moment hinges at removed-column ends of secondary beams (via `secondaryBeamHinges` Map)
3. Iterates: analyze → extract reactions from secondary beams → apply as point loads on primary beams → re-analyze → repeat until convergence < 1%
4. Uses `MSPointLoad` (`{P, a}`) which the 2D MSM solver handles correctly via `envelopeAnalysis()`

**The fix must replicate this exact logic in the 3D path.**

---

## 4. SECONDARY BUGS

### Bug 4.1 — Primary/Secondary Beam Classification by Span Length is Wrong

**Location:** `src/lib/structuralEngine.ts`, function `detectBeamOnBeam()` (lines 701–758)

**Code:**
```typescript
const hTotalSpan = hBeams.reduce((sum, b) => sum + b.length, 0);
const vTotalSpan = vBeams.reduce((sum, b) => sum + b.length, 0);
const primaryIsHorizontal = hTotalSpan >= vTotalSpan;
```

**Problem:** Comparing total span lengths (sum of all beams in each direction) is a heuristic that can misclassify. For example, if there are 3 short horizontal beams vs 1 long vertical beam, the horizontal direction is incorrectly chosen as primary even though the single vertical beam is much stiffer.

**ETABS approach:** The primary beam is the one with **higher flexural stiffness (EI/L)** at the junction. In practice, this means: the beam with larger cross-section, or in ambiguous cases, the one running in the direction of the longer floor span.

**Correct fix:**
```typescript
// Compute max EI/L for beams in each direction
const hStiffness = hBeams.reduce((max, b) => {
  const I = (b.b / 1000) * Math.pow(b.h / 1000, 3) / 12;
  return Math.max(max, I / b.length);
}, 0);
const vStiffness = vBeams.reduce((max, b) => {
  const I = (b.b / 1000) * Math.pow(b.h / 1000, 3) / 12;
  return Math.max(max, I / b.length);
}, 0);
const primaryIsHorizontal = hStiffness >= vStiffness;
```

### Bug 4.2 — 3D Column Axial Load Takes Absolute Max but Ignores Sign Convention

**Location:** `src/lib/analyze3DColumns.ts`, `runPatternEnvelope3D()` (lines 272–295)

**Code:**
```typescript
prev.axial = Math.max(prev.axial, Math.abs(er.axial));
```

**Problem:** Taking `Math.abs()` before the envelope means we always get the maximum compression. But if a column can experience tension under some pattern (e.g., edge column with eccentric live load), the tension case is silently discarded. For column P-M interaction diagrams, the design must check **both** max compression AND min compression (or max tension) as separate load cases.

**Fix:** Store both `axialMax` and `axialMin` in the envelope:
```typescript
prev.axialMax = Math.max(prev.axialMax, er.axial);   // max compression (positive = compression)
prev.axialMin = Math.min(prev.axialMin, er.axial);   // min / could be tension
```

Then check PM capacity for both `(axialMax, momentAtMaxAxial)` and `(axialMin, momentAtMinAxial)` combinations.

### Bug 4.3 — Beam-on-Beam Point Load Position Calculation Has an Edge Case

**Location:** `src/lib/structuralEngine.ts`, `analyzeWithBeamOnBeam()` (lines 944–953)

**Code:**
```typescript
if (conn.primaryDirection === 'horizontal') {
  distOnPrimary = Math.abs(conn.point.x - primaryBeam.x1);
} else {
  distOnPrimary = Math.abs(conn.point.y - primaryBeam.y1);
}
```

**Problem:** `primaryBeam.x1` and `primaryBeam.y1` are stored coordinates from the beam definition. If the beam was drawn right-to-left or bottom-to-top (i.e., `x1 > x2` or `y1 > y2`), then `x1` is actually the RIGHT end, and `|point.x - x1|` gives the distance from the right end, not the left end. But the `MSPointLoad` `{P, a}` expects `a = distance from left node (nodeI)`.

**Fix:** Always measure from the minimum coordinate (true left/bottom end):
```typescript
if (conn.primaryDirection === 'horizontal') {
  const xMin = Math.min(primaryBeam.x1, primaryBeam.x2);
  distOnPrimary = Math.abs(conn.point.x - xMin);
} else {
  const yMin = Math.min(primaryBeam.y1, primaryBeam.y2);
  distOnPrimary = Math.abs(conn.point.y - yMin);
}
```

### Bug 4.4 — 3D Pattern Loading Uses Even/Odd Beam Index, Not Span Adjacency

**Location:** `src/lib/analyze3DColumns.ts`, `build3DModelWithPatternLoading()` (lines 218–228)

**Code:**
```typescript
patternCases.push({
  id: 'case_even', name: 'Even LL',
  elementLoads: new Map(beamElemIds.map((eid, i) => [
    eid, { wx: 0, wy: 0, wz: -(beamDeadLoads.get(eid)! + (i % 2 === 0 ? beamLiveLoads.get(eid)! : 0)) }
  ])),
});
```

**Problem:** The "even/odd" pattern is based on the iteration index of `beamElemIds`, which is an unordered set derived from `frames[].beamIds`. Beams from different frames (different X-direction frames vs Y-direction frames) get mixed together, so "even/odd" has no geometric meaning.

**ETABS approach:** Pattern loading applies live load on **alternate spans in the same continuous frame**, not alternating across the entire building. Each frame should have its own pattern loading applied independently.

**Fix:** Pattern loading should be done per-frame, and beams within each frame should be indexed by their sequential position in the frame.

### Bug 4.5 — Moment Magnification Uses Default k=1.0 Without Checking Sway vs Non-sway

**Location:** `src/lib/structuralEngine.ts`, `designColumnETABS()` (line 1562)

**Code:**
```typescript
const k = 1.0;
```

**Problem:** ACI 318-19 §6.6.4 specifies different effective length factors:
- Non-sway frames (braced): k ≤ 1.0 (typically 0.65–1.0 from alignment charts)
- Sway frames (unbraced): k > 1.0 (typically 1.2–2.5)

Using k=1.0 for all cases underestimates slenderness for sway frames.

**Fix:** The column design function should accept a `isSway` boolean parameter. For sway frames, use a minimum k of 1.2 (conservative) unless an alignment-chart calculation is performed. For non-sway, k=0.80 is a common conservative default per ACI commentary.

### Bug 4.6 — Deflection Uses Full Service Load but Should Use Sustained Load Only for Long-term

**Location:** `src/lib/structuralEngine.ts`, `calculateDeflection()` (lines 1119–1125)

**Code:**
```typescript
const wSustained = deadLoad * sustainedLoadFraction;
const deltaSustainedImmediate = (K_coeff * wSustained * Math.pow(L, 4)) / (Ec * Ie);
const deltaLongTerm = lambdaDelta * deltaSustainedImmediate;
const deltaTotal = deltaImmediate + deltaLongTerm;
```

**Problem:** `deltaImmediate` includes BOTH dead load AND live load deflection. But `deltaLongTerm` is added on top of that. This double-counts the dead load deflection — immediate dead load deflection is included in `deltaImmediate` AND also magnified in `deltaLongTerm`.

**ACI 318-19 correct approach:**
```
δ_immediate_dead = K × wD × L⁴ / (Ec × Ie)
δ_immediate_live = K × wL × L⁴ / (Ec × Ie)
δ_long_term      = λΔ × δ_immediate_dead × sustainedLoadFraction
δ_total          = δ_immediate_live + δ_immediate_dead + δ_long_term
```

**Fix:** Split `deltaImmediate` into dead and live components, then calculate long-term based only on dead load component.

---

## 5. DETAILED FIX INSTRUCTIONS FOR THE MAIN BUG

### Fix 5.1 — Integrate Beam-on-Beam into the 3D Analysis

**File to modify:** `src/lib/analyze3DColumns.ts`

#### Step 1: Update the function signature of `build3DModelWithPatternLoading`

Add parameter for beam-on-beam connections:

```typescript
function build3DModelWithPatternLoading(
  frames: Frame[],
  beams: Beam[],
  columns: Column[],
  mat: MatProps,
  frameEndReleases?: EndReleaseMap,
  beamOnBeamConnections?: BeamOnBeamConnection[],  // NEW PARAMETER
): { model: Model3D; patternCases: LoadCase3D[] }
```

#### Step 2: After building beam elements, process beam-on-beam connections

Add this block BEFORE constructing `patternCases`, after all beams are added:

```typescript
// ── Beam-on-Beam: Insert intermediate nodes and update element connectivity ──
// For each beam-on-beam connection, we must:
// 1. Split the primary beam into two sub-elements at the bearing point
// 2. Add a hinge (moment release at nodeJ/nodeI) to the secondary beam
// 3. The intermediate node will receive the secondary beam's reaction as a nodal load

const bearingNodeMap = new Map<string, string>(); // connId → intermediate node ID

if (beamOnBeamConnections) {
  for (const conn of beamOnBeamConnections) {
    const primaryBeamElemId = `beam_${conn.primaryBeamId}`;
    const primaryElemIndex = elements3d.findIndex(e => e.id === primaryBeamElemId);
    if (primaryElemIndex < 0) continue;

    const primaryElem = elements3d[primaryElemIndex];
    const nodeI = nodesMap.get(primaryElem.nodeI)!;
    const nodeJ = nodesMap.get(primaryElem.nodeJ)!;

    // Compute bearing point in 3D space (interpolate between nodeI and nodeJ)
    const totalLen = Math.sqrt(
      Math.pow(nodeJ.x - nodeI.x, 2) +
      Math.pow(nodeJ.y - nodeI.y, 2) +
      Math.pow(nodeJ.z - nodeI.z, 2)
    );
    const ratio = totalLen > 0 ? (conn.distanceOnPrimary * 1000) / totalLen : 0.5;
    const bx = nodeI.x + ratio * (nodeJ.x - nodeI.x);
    const by = nodeI.y + ratio * (nodeJ.y - nodeI.y);
    const bz = nodeI.z + ratio * (nodeJ.z - nodeI.z);

    const midNodeId = getOrCreateNode(bx, by, bz, [false, false, false, false, false, false]);
    bearingNodeMap.set(conn.removedColumnId, midNodeId);

    // Split primary beam into [nodeI → midNode] and [midNode → nodeJ]
    const lenI = ratio * totalLen;       // length of left sub-element (mm)
    const lenJ = (1 - ratio) * totalLen; // length of right sub-element (mm)

    const subElemLeft: Element3D = {
      ...primaryElem,
      id: `${primaryBeamElemId}_A`,
      nodeI: primaryElem.nodeI,
      nodeJ: midNodeId,
    };
    const subElemRight: Element3D = {
      ...primaryElem,
      id: `${primaryBeamElemId}_B`,
      nodeI: midNodeId,
      nodeJ: primaryElem.nodeJ,
    };

    // Replace original element with two sub-elements
    elements3d.splice(primaryElemIndex, 1, subElemLeft, subElemRight);

    // Update load maps: distribute original UDL to sub-elements proportionally
    const origDeadLoad = beamDeadLoads.get(primaryBeamElemId);
    const origLiveLoad = beamLiveLoads.get(primaryBeamElemId);
    if (origDeadLoad !== undefined) {
      beamDeadLoads.set(`${primaryBeamElemId}_A`, origDeadLoad);
      beamDeadLoads.set(`${primaryBeamElemId}_B`, origDeadLoad);
      beamDeadLoads.delete(primaryBeamElemId);
    }
    if (origLiveLoad !== undefined) {
      beamLiveLoads.set(`${primaryBeamElemId}_A`, origLiveLoad);
      beamLiveLoads.set(`${primaryBeamElemId}_B`, origLiveLoad);
      beamLiveLoads.delete(primaryBeamElemId);
    }

    // Update beamElemIds list
    const idx = beamElemIds.indexOf(primaryBeamElemId);
    if (idx >= 0) {
      beamElemIds.splice(idx, 1, `${primaryBeamElemId}_A`, `${primaryBeamElemId}_B`);
    }

    // Add moment release to secondary beams at the removed-column end
    for (const secBeamId of conn.secondaryBeamIds) {
      const secElemId = `beam_${secBeamId}`;
      const secElem = elements3d.find(e => e.id === secElemId);
      if (!secElem) continue;
      const secBeam = beams.find(b => b.id === secBeamId);
      if (!secBeam) continue;

      const isAtStart = secBeam.fromCol === conn.removedColumnId;
      // Add moment release (hinge) at the removed-column end
      secElem.releases = secElem.releases ?? {
        nodeI: { ux: false, uy: false, uz: false, mx: false, my: false, mz: false },
        nodeJ: { ux: false, uy: false, uz: false, mx: false, my: false, mz: false },
      };
      if (isAtStart) {
        // nodeI of secondary beam → connects to removed column location
        // The removed column top node → now equals the bearing intermediate node
        secElem.nodeI = midNodeId;
        secElem.releases.nodeI.mz = true; // release moment (hinge)
        secElem.releases.nodeI.my = true; // release biaxial moment too
      } else {
        secElem.nodeJ = midNodeId;
        secElem.releases.nodeJ.mz = true;
        secElem.releases.nodeJ.my = true;
      }
    }
  }
}
```

#### Step 3: Update `runPatternEnvelope3D` and public APIs to pass the connections

In `runPatternEnvelope3D`:
```typescript
function runPatternEnvelope3D(
  frames: Frame[],
  beams: Beam[],
  columns: Column[],
  mat: MatProps,
  frameEndReleases?: EndReleaseMap,
  beamOnBeamConnections?: BeamOnBeamConnection[],  // NEW
): { beamEnvelope: ...; colEnvelope: ... } {
  const { model, patternCases } = build3DModelWithPatternLoading(
    frames, beams, columns, mat, frameEndReleases,
    beamOnBeamConnections,  // pass through
  );
  // ... rest unchanged
}
```

In `getColumnLoads3D` and `getFrameResults3D`:
```typescript
export function getColumnLoads3D(
  frames: Frame[],
  beams: Beam[],
  columns: Column[],
  mat: MatProps,
  frameEndReleases?: EndReleaseMap,
  beamOnBeamConnections?: BeamOnBeamConnection[],  // NEW
): Map<string, ColumnLoads3D> {
  const { colEnvelope } = runPatternEnvelope3D(
    frames, beams, columns, mat, frameEndReleases,
    beamOnBeamConnections
  );
  // ... rest unchanged
}

export function getFrameResults3D(
  frames: Frame[],
  beams: Beam[],
  columns: Column[],
  mat: MatProps,
  frameEndReleases?: EndReleaseMap,
  beamOnBeamConnections?: BeamOnBeamConnection[],  // NEW
): FrameResult[] {
  const { beamEnvelope } = runPatternEnvelope3D(
    frames, beams, columns, mat, frameEndReleases,
    beamOnBeamConnections
  );
  // ... rest unchanged; BUT NOW the envelope contains split sub-element IDs
  // IMPORTANT: must handle `beam_${beamId}_A` and `beam_${beamId}_B` sub-elements
  // when looking up beam_${beamId} in the envelope.
}
```

#### Step 4: Fix the beam envelope lookup for split beams (in `getFrameResults3D`)

After splitting primary beams into `_A` and `_B` sub-elements, the envelope map will have entries like `beam_X_A` and `beam_X_B` instead of `beam_X`. The results lookup must handle this:

```typescript
for (const beamId of frame.beamIds) {
  const beam = beamsMap.get(beamId);
  if (!beam) continue;

  // Check if this beam was split (primary carrier beam)
  const envA = beamEnvelope.get(`beam_${beamId}_A`);
  const envB = beamEnvelope.get(`beam_${beamId}_B`);
  const env = beamEnvelope.get(`beam_${beamId}`);

  let finalEnv: BeamEnvelope3D | undefined;
  if (envA && envB) {
    // Combine split sub-elements: take worst case from each half
    finalEnv = {
      shearYMax: Math.max(envA.shearYMax, envB.shearYMax),
      shearYI: envA.shearYI,                    // left end shear from left sub-element
      shearYJ: envB.shearYJ,                    // right end shear from right sub-element
      momentZI: envA.momentZI,                  // left end moment
      momentZJ: envB.momentZJ,                  // right end moment
      momentZmid: Math.max(envA.momentZmid, envB.momentZmid, Math.abs(envA.momentZJ), Math.abs(envB.momentZI)),
    };
  } else {
    finalEnv = env;
  }
  // ... use finalEnv for frameBeams.push(...)
}
```

#### Step 5: Detect beam-on-beam connections and pass them from the calling code

The caller of `getColumnLoads3D` / `getFrameResults3D` (in `src/pages/Index.tsx` or wherever the analysis is triggered) must also call `detectBeamOnBeam()` and pass the result:

```typescript
import { detectBeamOnBeam } from '@/lib/structuralEngine';
import { getFrameResults3D, getColumnLoads3D } from '@/lib/analyze3DColumns';

const removedCols = columns.filter(c => c.isRemoved).map(c => c.id);
const beamOnBeamConns = detectBeamOnBeam(beams, columns, removedCols);

const frameResults = getFrameResults3D(
  frames, beams, columns, mat,
  frameEndReleases,
  beamOnBeamConns,   // pass connections to 3D path
);

const columnLoads = getColumnLoads3D(
  frames, beams, columns, mat,
  frameEndReleases,
  beamOnBeamConns,
);
```

---

## 6. COMPARISON TABLE: APP vs ETABS

| Feature | Current App (3D Path) | ETABS | Status |
|---|---|---|---|
| Beam-on-beam: carried beam moment release | ✅ In 2D path only | ✅ Always | ❌ Missing in 3D |
| Beam-on-beam: concentrated load on carrier | ✅ In 2D path only | ✅ Always | ❌ Missing in 3D |
| Beam-on-beam iterative convergence | ✅ In 2D path | ✅ Direct (single solve) | Partial |
| ACI stiffness modifiers (0.35 beam, 0.70 col) | ✅ | ✅ | ✅ |
| Pattern loading (ACI 318-19 §6.4.3) | ✅ 2^n combinations | ✅ Per ASCE 7 | ✅ |
| 3D pattern loading per-frame | ❌ Global even/odd index | ✅ Per-frame | Bug 4.4 |
| Primary beam classification (beam-on-beam) | ❌ By span sum | ✅ By EI/L stiffness | Bug 4.1 |
| Column axial envelope (tension case) | ❌ Abs max only | ✅ Both max+min | Bug 4.2 |
| Load position on primary beam (direction sign) | ❌ May be wrong if beam drawn R→L | ✅ Always from nodeI | Bug 4.3 |
| Slenderness: effective length factor k | ❌ Always 1.0 | ✅ Sway/non-sway differentiated | Bug 4.5 |
| Deflection: long-term + immediate separation | ❌ Double-counts dead load | ✅ Separate components | Bug 4.6 |
| 6-DOF space frame with biaxial bending | ✅ | ✅ | ✅ |
| P-M interaction diagram (column design) | ✅ | ✅ | ✅ |
| T-beam effective width (ACI §6.3.2.1) | ✅ | ✅ | ✅ |
| Doubly reinforced beams | ✅ | ✅ | ✅ |
| Shear design (ACI detailed Vc) | ✅ | ✅ | ✅ |

---

## 7. IMPLEMENTATION PRIORITY

1. **CRITICAL (fix first):** Bug in §3 — 3D path ignores beam-on-beam loads. Fix using instructions in §5.
2. **HIGH:** Bug 4.1 — primary/secondary classification by stiffness instead of span sum.
3. **HIGH:** Bug 4.3 — beam point load position coordinate direction issue.
4. **MEDIUM:** Bug 4.2 — column axial envelope tension case.
5. **MEDIUM:** Bug 4.6 — deflection double-counting.
6. **LOW:** Bug 4.4 — 3D pattern loading per-frame.
7. **LOW:** Bug 4.5 — moment magnification k factor.

---

## 8. TESTING VERIFICATION (After Fix)

### Test Case 1: 2×2 Grid with One Corner Column Removed

Setup:
- 4 columns at (0,0), (6,0), (0,6), (6,6) in meters
- Column at (6,0) removed (bottom-right corner)
- Beams: 300×600mm, span 6m each direction
- Dead = 20 kN/m, Live = 15 kN/m (uniform on all beams)
- fc = 25 MPa, fy = 420 MPa

Expected (ETABS-equivalent):
- Secondary beam at Y=0 (horizontal) gets a hinge at x=6m and transfers ~90–120 kN reaction to the vertical beam at x=6m
- Primary vertical beam (x=6, from y=0 to y=6) should show a mid-span point load, producing a moment peak at y≈0 (where secondary loads it)
- If point loads are NOT applied, the vertical beam at x=6 will show only its own distributed load moments — difference should be clearly measurable

### Test Case 2: Convergence Check

Run `analyzeWithBeamOnBeam()` (2D path) and `getFrameResults3D()` (3D path after fix) on the same model.

After fix, the carrier beam moments from both paths should agree within ±5%.

Before fix, the 3D path produces ~0 midspan moment on the carrier beam; the 2D path produces a large concentrated-load moment at the bearing point.

---

## 9. CODE LOCATION REFERENCE

```
artifacts/structural-design-suite/
└── src/
    ├── lib/
    │   ├── structuralEngine.ts    ← analyzeWithBeamOnBeam(), detectBeamOnBeam(), analyzeFrame()
    │   ├── analyze3DColumns.ts    ← build3DModelWithPatternLoading(), getFrameResults3D(), getColumnLoads3D()
    │   ├── solver3D.ts            ← analyze3DFrame() — the core 3D DSM solver
    │   └── matrixStiffness.ts     ← envelopeAnalysis() — 2D MSM with point loads
    ├── pages/
    │   └── Index.tsx              ← Main page: calls analysis functions, passes parameters
    └── building/
        └── buildingModel.ts       ← Frame/Beam/Column data structures
```

---

## 10. IMPORTANT NOTES FOR THE CODING ASSISTANT

1. **Do NOT rewrite `solver3D.ts`** — the 3D DSM solver is correct. The bug is in how the model is built before calling it.

2. **The `analyze3DFrame()` function already supports** intermediate nodes (any topology of nodes and elements). After splitting the primary beam into two elements sharing the intermediate bearing node, the solver will correctly transfer loads through it.

3. **The `Element3D.releases` field** already exists in `solver3D.ts` and supports moment releases. You just need to set `releases.nodeI.mz = true` (and `my = true` for biaxial) for secondary beams.

4. **BeamOnBeamConnection interface** is defined in `structuralEngine.ts` — import it in `analyze3DColumns.ts`:
   ```typescript
   import type { ..., BeamOnBeamConnection } from '@/lib/structuralEngine';
   ```

5. **The `nodesMap` and `getOrCreateNode` helper** are already defined inside `build3DModelWithPatternLoading`. The intermediate bearing node should be created using this same helper to avoid duplicates.

6. **Pattern loading element loads** must be rebuilt after splitting elements. The `beamElemIds` array is used to construct load cases — updating it to include `_A`/`_B` sub-element IDs is essential.

7. **When re-constructing the beam envelope lookup** in `getFrameResults3D`, always check for both the original `beam_${id}` key and the split `beam_${id}_A` / `beam_${id}_B` keys.
8- Plan #1 - Carrier beam: split in analysis, unified display
#1 - Carrier Beam: Split Analysis, Unified Display
What & Why
Currently the carrier beam (الجسر الحامل للجسور / primary beam in a Beam-on-Beam connection) is analyzed as a single element with a concentrated point load applied where the secondary beam connects. The user wants the carrier beam to be split into two sub-beam elements at the connection point during analysis (proper FEM splitting for more accurate force distribution), but have its results merged and displayed as one combined beam in both the analysis tab and design/output tab — with a combined name made up of the two sub-beam names.

Done looks like
During analysis, the carrier beam is internally split into two sub-beam elements at each secondary beam connection point (each segment becoming a separate beam element with the connection node shared)
In the Analysis tab (Frames table), the two sub-beams are merged into one single row showing the combined span, envelope of internal forces (max moment, max shear), and a combined name such as "B3+B4" or "B3/B4" (the two sub-beam IDs joined)
In the Design tab (flexure, shear, deflection tables), the carrier beam appears as one combined row, designed for the governing (envelope) forces from both sub-segments, with the combined name
In all export/output, the carrier beam is treated as one entity with the composite name
All other (non-carrier) beams are unaffected
Out of scope
Splitting carrier beams with more than one secondary beam connection into three or more parts (only two-part splitting is required)
Changing how secondary (carried) beams are analyzed or displayed
Any change to the visual/graphical model view
Tasks
Split carrier beam into two sub-elements during analysis — In analyzeWithBeamOnBeam (structuralEngine.ts), instead of applying a point load on the carrier beam, split the carrier beam at the connection point into two sub-beam elements (Part1: from carrier start to connection point, Part2: from connection point to carrier end). Create temporary sub-beam objects for these two segments, run them through the stiffness analysis as separate elements, and store results under both sub-beam IDs.

Merge and display carrier sub-beams as one row in analysis results — In Index.tsx analysis tab, detect which beams are carrier sub-segments (using the BeamOnBeamConnection data) and merge their two rows into one combined row. The combined row should show: the composite name (e.g., "B3+B4"), the total span (sum of both sub-spans), and the envelope of internal forces (governing Mleft, Mmid, Mright, Vu across both segments).

Merge and display carrier sub-beams as one row in design/output — In Index.tsx design tab and any output/export sections, apply the same merging logic: carrier sub-beams appear as a single combined row designed for the envelope forces from both sub-segments, with the composite name.

Relevant files
artifacts/structural-design-suite/src/lib/structuralEngine.ts:393-401,701-770,885-1020
artifacts/structural-design-suite/src/pages/Index.tsx:349-398,1504-1533,1650-1712
9- Plan #2 - Add delete button to element properties dialog
#2 - Add Delete Button to Element Properties Dialog
What & Why
When the user long-presses on an element (beam, column, or slab) in the modeling tab, a properties sheet appears. The user wants a delete button in this sheet to remove the element directly without closing the dialog and looking for another way to delete it.

Done looks like
A clearly visible "حذف العنصر" (Delete Element) delete button appears inside the ElementPropertiesDialog.
Tapping the delete button removes the element (frame or area) from the model and closes the dialog.
A confirmation step or destructive styling (red button) is used so the user doesn't accidentally delete elements.
Out of scope
Undo/redo support for deletions (not currently in the app).
Deleting nodes (joints) separately.
Tasks
Add onDelete prop to ElementPropertiesDialog — Extend the component's props interface to accept an optional onDelete callback and render a delete button (styled destructively in red) in the dialog footer next to the existing Cancel and Save buttons.
Wire up the delete handler in Index.tsx — Implement handleElemPropsDelete in Index.tsx that dispatches the appropriate action to remove the frame or area from the model state, then closes the dialog. Pass it as onDelete to ElementPropertiesDialog.
Relevant files
artifacts/structural-design-suite/src/components/ElementPropertiesDialog.tsx
artifacts/structural-design-suite/src/pages/Index.tsx
10- 
Plan #3 - Export PDFs: English text & larger fonts
#3 - Export PDFs: English Text & Larger Fonts
What & Why
The PDF export library (jsPDF + helvetica) does not support Arabic characters — Arabic text appears as garbled symbols or boxes in generated PDFs. All output panel export tabs should switch to English labels inside the PDF output, and font sizes for element names and reinforcement quantities should be increased for better readability.

Done looks like
All generated PDFs (structural drawings, construction sheets, BBS, and report) use English labels instead of Arabic for element names, section labels, and reinforcement quantity text
Element IDs (beam IDs, column IDs, slab IDs) in drawings are clearly readable with larger font sizes
Reinforcement quantities (e.g. "3Φ16 Top", "2Φ12 Bot") use larger font sizes and display in English
Arabic text remains in the UI (the ExportPanel interface itself stays in Arabic); only the PDF content switches to English
Out of scope
Embedding an Arabic-capable font (e.g. the .ttf files in assets/fonts) into jsPDF — that is a separate future task
Changing the ExportPanel UI language
Tasks
Translate PDF labels to English — In constructionSheets.ts, replace all Arabic string literals used in drawing labels (beam labels, rebar descriptions like حديد علوي, حديد سفلي, تفريد الحديد, section titles, etc.) with English equivalents (e.g. "Top Steel", "Bot Steel", "Distribution Steel"). Do the same in pdfReport.ts, bbsGenerator.ts, and drawingExporter.ts for any Arabic text embedded in the PDF output.

Increase font sizes for element names — In drawingExporter.ts, increase column ID and beam ID font sizes (currently 6pt) and slab ID font sizes (currently 7pt) to at least 8–9pt. In constructionSheets.ts, increase beam elevation labels (currently 5pt) and cross-section titles (currently 3.8pt) to at least 6–7pt.

Increase font sizes for reinforcement quantities — In constructionSheets.ts, increase rebar quantity text (currently 4–4.5pt) to at least 6pt so quantities like "3Φ16 Top" are clearly legible in the printed PDF.

Relevant files
artifacts/structural-design-suite/src/export/pdfReport.ts
artifacts/structural-design-suite/src/drawings/constructionSheets.ts
artifacts/structural-design-suite/src/rebar/bbsGenerator.ts
artifacts/structural-design-suite/src/components/ExportPanel.tsx