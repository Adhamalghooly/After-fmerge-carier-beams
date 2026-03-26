# Workspace

## Overview

pnpm workspace monorepo using TypeScript. Each package manages its own dependencies.

## Stack

- **Monorepo tool**: pnpm workspaces
- **Node.js version**: 24
- **Package manager**: pnpm
- **TypeScript version**: 5.9
- **API framework**: Express 5
- **Database**: PostgreSQL + Drizzle ORM
- **Validation**: Zod (`zod/v4`), `drizzle-zod`
- **API codegen**: Orval (from OpenAPI spec)
- **Build**: esbuild (CJS bundle)

## Structure

```text
artifacts-monorepo/
├── artifacts/              # Deployable applications
│   └── api-server/         # Express API server
├── lib/                    # Shared libraries
│   ├── api-spec/           # OpenAPI spec + Orval codegen config
│   ├── api-client-react/   # Generated React Query hooks
│   ├── api-zod/            # Generated Zod schemas from OpenAPI
│   └── db/                 # Drizzle ORM schema + DB connection
├── scripts/                # Utility scripts (single workspace package)
│   └── src/                # Individual .ts scripts, run via `pnpm --filter @workspace/scripts run <script>`
├── pnpm-workspace.yaml     # pnpm workspace (artifacts/*, lib/*, lib/integrations/*, scripts)
├── tsconfig.base.json      # Shared TS options (composite, bundler resolution, es2022)
├── tsconfig.json           # Root TS project references
└── package.json            # Root package with hoisted devDeps
```

## TypeScript & Composite Projects

Every package extends `tsconfig.base.json` which sets `composite: true`. The root `tsconfig.json` lists all packages as project references. This means:

- **Always typecheck from the root** — run `pnpm run typecheck` (which runs `tsc --build --emitDeclarationOnly`). This builds the full dependency graph so that cross-package imports resolve correctly. Running `tsc` inside a single package will fail if its dependencies haven't been built yet.
- **`emitDeclarationOnly`** — we only emit `.d.ts` files during typecheck; actual JS bundling is handled by esbuild/tsx/vite...etc, not `tsc`.
- **Project references** — when package A depends on package B, A's `tsconfig.json` must list B in its `references` array. `tsc --build` uses this to determine build order and skip up-to-date packages.

## Root Scripts

- `pnpm run build` — runs `typecheck` first, then recursively runs `build` in all packages that define it
- `pnpm run typecheck` — runs `tsc --build --emitDeclarationOnly` using project references

## Packages

### `artifacts/api-server` (`@workspace/api-server`)

Express 5 API server. Routes live in `src/routes/` and use `@workspace/api-zod` for request and response validation and `@workspace/db` for persistence.

- Entry: `src/index.ts` — reads `PORT`, starts Express
- App setup: `src/app.ts` — mounts CORS, JSON/urlencoded parsing, routes at `/api`
- Routes: `src/routes/index.ts` mounts sub-routers; `src/routes/health.ts` exposes `GET /health` (full path: `/api/health`)
- Depends on: `@workspace/db`, `@workspace/api-zod`
- `pnpm --filter @workspace/api-server run dev` — run the dev server
- `pnpm --filter @workspace/api-server run build` — production esbuild bundle (`dist/index.cjs`)
- Build bundles an allowlist of deps (express, cors, pg, drizzle-orm, zod, etc.) and externalizes the rest

### `lib/db` (`@workspace/db`)

Database layer using Drizzle ORM with PostgreSQL. Exports a Drizzle client instance and schema models.

- `src/index.ts` — creates a `Pool` + Drizzle instance, exports schema
- `src/schema/index.ts` — barrel re-export of all models
- `src/schema/<modelname>.ts` — table definitions with `drizzle-zod` insert schemas (no models definitions exist right now)
- `drizzle.config.ts` — Drizzle Kit config (requires `DATABASE_URL`, automatically provided by Replit)
- Exports: `.` (pool, db, schema), `./schema` (schema only)

Production migrations are handled by Replit when publishing. In development, we just use `pnpm --filter @workspace/db run push`, and we fallback to `pnpm --filter @workspace/db run push-force`.

### `lib/api-spec` (`@workspace/api-spec`)

Owns the OpenAPI 3.1 spec (`openapi.yaml`) and the Orval config (`orval.config.ts`). Running codegen produces output into two sibling packages:

1. `lib/api-client-react/src/generated/` — React Query hooks + fetch client
2. `lib/api-zod/src/generated/` — Zod schemas

Run codegen: `pnpm --filter @workspace/api-spec run codegen`

### `lib/api-zod` (`@workspace/api-zod`)

Generated Zod schemas from the OpenAPI spec (e.g. `HealthCheckResponse`). Used by `api-server` for response validation.

### `lib/api-client-react` (`@workspace/api-client-react`)

Generated React Query hooks and fetch client from the OpenAPI spec (e.g. `useHealthCheck`, `healthCheck`).

### `scripts` (`@workspace/scripts`)

Utility scripts package. Each script is a `.ts` file in `src/` with a corresponding npm script in `package.json`. Run scripts via `pnpm --filter @workspace/scripts run <script>`. Scripts can import any workspace package (e.g., `@workspace/db`) by adding it as a dependency in `scripts/package.json`.

### `artifacts/structural-design` — slabFEMEngine module

Isolated add-on at `src/slabFEMEngine/`. Computes slab-to-beam load transfer via Mindlin-Reissner FEM (ETABS-style). Does NOT modify any existing analysis logic.

**Module files (Phases 1–6, LOCKED):**
- `types.ts` — all type definitions (FEMNode, FEMElement, SlabMesh, etc.)
- `mesh.ts` — structured quad mesh generator (beam lines forced onto grid)
- `mindlinShell.ts` — 4-node Mindlin plate element (2×2 bending, 1×1 shear)
- `assembler.ts` — global K assembly, BC application, reaction extraction
- `solver.ts` — Gaussian elimination with partial pivoting
- `internalForces.ts` — Mx, My, Mxy, Qx, Qy at any point
- `edgeForces.ts` — Phase 2: signed nodal reactions with junction splitting
- `beamMapper.ts` — Phase 3: converts reactions to w(x) BeamLoadResult
- `stressEdgeTransfer.ts` — Phase 4/5: stress-based force + moment transfer
- `rotationalCoupling.ts` — Phase 6: beam–slab rotational spring feedback
- `validation.ts` — Phases 1–5 validation suite (5 cases)
- `index.ts` — public API (getBeamLoadsFromSlab, getCoupledBeamSlabResults, …)

**Phase 7 files (NEW — Full Coupled Beam-Slab FEM):**
- `frameElement.ts` — 3D Euler-Bernoulli frame element (12 DOF, K_local, T, K_global)
- `coupledSystem.ts` — monolithic slab+beam FEM system with penalty coupling
- `phase7Validation.ts` — 4-test validation suite (cantilever, equilibrium, stiffness, load-share)
- `reports/phase7_report.txt` — full engineering report

**Phase 7 public API:**
- `getCoupledBeamSlabResults(model, meshDensity?, penaltyMult?) → CoupledResult[]`
  - Solves slab and beams simultaneously in ONE global stiffness matrix
  - Penalty method enforces UZ/RX/RY compatibility at slab–beam interface
  - Returns beam end forces (local coords), slab deflections, equilibrium check
- `runPhase7Validation() → Phase7Report` — runs all 4 validation tests

**Phase 8 files (true DOF merging — monolithic beam-slab coupling):**
- `mergedDOFSystem.ts` — true DOF merging at slab–beam interface (UZ/RX/RY merged, no penalty); block assembly; exact interface compatibility
- `phase8Validation.ts` — 4-test validation suite
- `reports/phase8_report.txt` — engineering report

**Phase 8 public API:**
- `getMergedBeamSlabResults(model, meshDensity?) → MergedResult[]`
- `runPhase8Validation() → Phase8Report`

**Phase 9 files (sparse matrix solver infrastructure):**
- `sparseMatrix.ts` — CSR format, TripletMatrix COO builder, csrMatVec, permuteCSR, csrStats
- `sparseSolver.ts` — Reverse Cuthill-McKee, sparse Cholesky, PCG with Jacobi preconditioner, unified dispatch
- `sparseAssembler.ts` — sparse assembly (COO→CSR, never builds dense K), BC filtering
- `sparsePhase9.ts` — Phase 9 entry point, drop-in replacement for Phase 8 (flag-gated)
- `phase9Validation.ts` — 4-test validation suite (dense vs sparse agreement, scalability, benchmark, CG convergence)
- `phase9_report.txt` — full engineering report

**Phase 9 public API (in index.ts):**
- `getSparseBeamSlabResults(model, meshDensity?, opts?) → MergedResult & { debug: Phase9DebugInfo }`
- `runPhase9Validation() → Phase9Report`
- All sparse utilities exported: CSRMatrix, TripletMatrix, solveCG, solveCholesky, cuthillMcKee, csrStats

**Solver flags (in types.ts):**
- `useSparseSolver: boolean` — activates Phase 9 path (default false = Phase 8 unchanged)
- `sparseSolverMethod: 'cholesky' | 'cg'` — solver choice (default 'cg')
- `useCuthillMcKee: boolean` — RCM reordering (default true)

**Scalability:**
- Dense (Phase 8): O(n³) solve, O(n²) memory — limited to ~500 free DOF in real-time
- Sparse (Phase 9): O(√κ · nnz) CG, O(nnz) memory — handles 5 000–20 000+ free DOF
- Memory reduction: 40–250× for typical FEM meshes

**Validation status:**
- Phase 1 PASSED: equilibrium 0.0000%, moment error 10.42% (< 15%), deflection error 4.57%
- Phase 2 PASSED: each beam receives exactly 62.50 kN on a 5×5 m / 10 kN/m² test, 0.0000% equilibrium error
- Phase 7: 4 validation tests — cantilever self-test (0.1% tolerance), equilibrium < 2%, stiffness comparison, internal beam load share
- Phase 9: 4 validation tests — dense/sparse agreement < 1%, large-model scalability, performance benchmark, CG convergence < 10⁻¹⁰
- STRICT RULE: Do NOT modify any Phases 1–6 files. Phase 7+ only adds new files.

## Analysis Engine Switching System

**Key files:**
- `src/lib/analysisController.ts` — EngineType, ENGINE_LABELS, adaptLegacyResults(), adaptFEMResults()
- `src/pages/indexReducer.ts` — AppState.selectedEngine, SET_ENGINE action
- `src/pages/Index.tsx` — engine dropdown UI, runAnalysis() dual-path routing

**Architecture:** UI → Controller (runAnalysis) → Engine → Adapter → FrameResult[] → Renderer

**Canonical format:** FrameResult[] (unchanged from legacy) — all downstream rendering code unmodified.

**Engines:**
- `legacy_3d` (default): getFrameResults3D() via 3D stiffness frame analysis
- `fem_coupled`: getCoupledBeamSlabResults() via Phase-7 Coupled Beam–Slab FEM

**FEM Adapter mapping (adaptFEMResults):**
- Accumulates CoupledBeamResult across all slab solves (internal beams get contributions from both adjacent slabs)
- My1/My2 (N·mm ÷ 1e6) → Mleft/Mright (kN·m)
- Max positive central element moment → Mmid (kN·m)
- maxShear_kN envelope → Vu (kN)
- |Vz1|/|Vz2| (N ÷ 1000) → Rleft/Rright (kN)

**Report:** `analysis-engine-switching-report.txt` (full architecture + comparison + observations)
