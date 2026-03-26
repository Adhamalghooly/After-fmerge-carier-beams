/**
 * slabFEMEngine – Mesh Generator (Phase 1)
 *
 * Produces a structured quadrilateral mesh for each rectangular slab.
 *
 * Key constraint: beam lines MUST coincide with element edges so that
 * edge forces can be extracted exactly along the slab-beam interface.
 *
 * Algorithm
 * ---------
 * 1. Collect "required" X and Y grid lines:
 *      - Slab boundary coordinates (x1, x2, y1, y2)
 *      - Any beam that lies ON the slab boundary or passes through it
 * 2. Fill gaps between required lines with additional divisions so that
 *    no segment exceeds (slabSize / meshDensity).
 * 3. Build node grid from the resulting (xLines × yLines) intersections.
 * 4. Mark nodes:
 *      - isFixed = true  → column sits at this point
 *      - beamId          → node lies on a beam line edge
 */

import type { Slab, Beam, Column } from './types';
import type { FEMNode, FEMElement, SlabMesh } from './types';

const EPS = 1e-3; // mm tolerance for coordinate matching

// ─────────────────────────────────────────────────────────────────────────────

export function meshSlab(
  slab:    Slab,
  beams:   Beam[],
  columns: Column[],
  meshDensity: number = 4,   // divisions per metre (default 4 → ~250 mm/div)
): SlabMesh {
  const x1 = Math.min(slab.x1, slab.x2);
  const x2 = Math.max(slab.x1, slab.x2);
  const y1 = Math.min(slab.y1, slab.y2);
  const y2 = Math.max(slab.y1, slab.y2);

  const slabLx = x2 - x1;  // mm
  const slabLy = y2 - y1;  // mm

  // Max segment length based on density (density is divisions/m → mm conversion)
  const maxSeg = Math.max(1000 / meshDensity, 100); // mm (min 100 mm)

  // ── Step 1: Collect required grid lines ──────────────────────────────────

  const xRequired = new Set<number>([x1, x2]);
  const yRequired = new Set<number>([y1, y2]);

  for (const beam of beams) {
    if (!beam.slabs.includes(slab.id)) continue;

    if (beam.direction === 'horizontal') {
      // Horizontal beam → constant Y coordinate
      const by = beam.y1;
      if (by >= y1 - EPS && by <= y2 + EPS) {
        yRequired.add(clamp(by, y1, y2));
      }
    } else {
      // Vertical beam → constant X coordinate
      const bx = beam.x1;
      if (bx >= x1 - EPS && bx <= x2 + EPS) {
        xRequired.add(clamp(bx, x1, x2));
      }
    }
  }

  // ── Step 2: Fill gaps between required lines ─────────────────────────────

  const xLines = fillDivisions(sortSet(xRequired), maxSeg);
  const yLines = fillDivisions(sortSet(yRequired), maxSeg);

  const nx = xLines.length - 1; // number of elements in X
  const ny = yLines.length - 1; // number of elements in Y

  // ── Step 3: Build node grid ───────────────────────────────────────────────

  const nodes: FEMNode[] = [];
  const nodeIndex = (ix: number, iy: number) => iy * (nx + 1) + ix;

  for (let iy = 0; iy <= ny; iy++) {
    for (let ix = 0; ix <= nx; ix++) {
      const x = xLines[ix];
      const y = yLines[iy];

      // ── Classify node ────────────────────────────────────────────────────

      // Is there a column at (x, y)?
      const atColumn = columns.some(
        c => Math.abs(c.x - x) < EPS && Math.abs(c.y - y) < EPS,
      );

      // Which beam (if any) does this node sit on?
      let beamId: string | null = null;
      let beamPos = 0;

      for (const beam of beams) {
        if (!beam.slabs.includes(slab.id)) continue;

        if (beam.direction === 'horizontal') {
          const by = beam.y1;
          if (Math.abs(y - by) < EPS) {
            // Node is on the beam line – check X range
            const bx1 = Math.min(beam.x1, beam.x2);
            const bx2 = Math.max(beam.x1, beam.x2);
            if (x >= bx1 - EPS && x <= bx2 + EPS) {
              beamId  = beam.id;
              beamPos = beam.length > 0 ? (x - bx1) / (bx2 - bx1) : 0;
              break;
            }
          }
        } else {
          const bx = beam.x1;
          if (Math.abs(x - bx) < EPS) {
            const by1 = Math.min(beam.y1, beam.y2);
            const by2 = Math.max(beam.y1, beam.y2);
            if (y >= by1 - EPS && y <= by2 + EPS) {
              beamId  = beam.id;
              beamPos = beam.length > 0 ? (y - by1) / (by2 - by1) : 0;
              break;
            }
          }
        }
      }

      // A node is fixed if it is at a column, on the slab boundary, OR
      // on ANY beam line. Beam-line nodes are modelled as rigid supports:
      // their reaction forces ARE the loads transferred to the beam.
      // This is exact by equilibrium and avoids shear-stress integration issues.
      const onBoundary =
        Math.abs(x - x1) < EPS || Math.abs(x - x2) < EPS ||
        Math.abs(y - y1) < EPS || Math.abs(y - y2) < EPS;

      // A beam-line node is fixed regardless of whether it is on the boundary
      const onBeamLine = beamId !== null;

      nodes.push({
        id:       nodeIndex(ix, iy),
        x,
        y,
        isFixed:  atColumn || onBoundary || onBeamLine,
        atColumn,
        beamId,
        beamPos,
      });
    }
  }

  // ── Step 4: Build elements ────────────────────────────────────────────────

  const elements: FEMElement[] = [];
  let elemId = 0;

  for (let iy = 0; iy < ny; iy++) {
    for (let ix = 0; ix < nx; ix++) {
      // CCW ordering: n0=BL, n1=BR, n2=TR, n3=TL
      const n0 = nodeIndex(ix,     iy    );
      const n1 = nodeIndex(ix + 1, iy    );
      const n2 = nodeIndex(ix + 1, iy + 1);
      const n3 = nodeIndex(ix,     iy + 1);

      elements.push({
        id:      elemId++,
        nodeIds: [n0, n1, n2, n3],
        slabId:  slab.id,
      });
    }
  }

  return { slabId: slab.id, nodes, elements, xLines, yLines };
}

// ─── Helpers ──────────────────────────────────────────────────────────────────

function sortSet(s: Set<number>): number[] {
  return Array.from(s).sort((a, b) => a - b);
}

function clamp(v: number, lo: number, hi: number): number {
  return Math.max(lo, Math.min(hi, v));
}

/**
 * Given a sorted list of "required" positions, insert intermediate divisions
 * so that no gap exceeds `maxSegLen`.
 */
function fillDivisions(required: number[], maxSegLen: number): number[] {
  const result: number[] = [];
  for (let i = 0; i < required.length; i++) {
    result.push(required[i]);
    if (i < required.length - 1) {
      const gap = required[i + 1] - required[i];
      if (gap > maxSegLen + EPS) {
        const n = Math.ceil(gap / maxSegLen);
        for (let k = 1; k < n; k++) {
          result.push(required[i] + (gap * k) / n);
        }
      }
    }
  }
  return result;
}

// ─── Utility: mesh info string ────────────────────────────────────────────────

export function meshSummary(mesh: SlabMesh): string {
  return (
    `Slab ${mesh.slabId}: ` +
    `${mesh.xLines.length - 1} × ${mesh.yLines.length - 1} elements, ` +
    `${mesh.nodes.length} nodes`
  );
}
