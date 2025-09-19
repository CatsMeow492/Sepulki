### Tasks — Catalog-Driven 3D Configurator (Phase 1)

This task list is derived from the Phase 1 requirements and the design. It is organized by phases with clear acceptance ties, and uses checkboxes to track progress.

---

### Milestones

1) Phase 1 Core (MVP, demo-stable)
2) Phase 1.5 Dev Automation (catalog scan in dev)
3) Phase 2 Prod Automation & Enhancements (build-time manifest, substitution logic)

---

### Phase 1 — Core Functionality

- Catalog manifest
  - [x] Implement minimal `GET /api/catalog` route returning `CatalogResponse` with `items: CatalogItem[]` (slugs, `heroGlb`, `urdf`, `linksGlb`).
  - [ ] Add a response validator or type guard to ensure stability before sending to client (log warnings on invalid paths).
  - [ ] Expand static list to include any additional known items/assets (optional, as they are added to `/public/robots`).

- Analyze step (LLM, catalog-constrained)
  - [x] Fetch catalog JSON and embed as `catalogText` in the LLM prompt to constrain recommendations.
  - [x] Persist `analysis` and `userInput` in `localStorage` for flow continuity.
  - [ ] Implement non-blocking UI notification and default fallback when analysis is empty/malformed, per acceptance criteria.

- Configure step (selection + viewer)
  - [x] Heuristic preset selection via `suggestPresetFromAnalysis(userInput, analysis)`.
  - [x] Fetch catalog and choose rendering mode via `selectFromCatalog(catalog, preset)` → hero GLB or URDF; fallback to sample.
  - [x] Unified 3D viewer (`Scene3D`): hero GLB (static) and URDF (articulated) modes.
  - [x] Ground snap, environment lighting, camera/controls configured.
  - [x] `RobotModel` URDF loader with GLB/OBJ mesh support.
  - [x] Joint API exposure, `JointControls`, and `MotionPlayer` integration.
  - [x] ErrorBanner on load failure; placeholder mesh to prevent crashes.
  - [ ] Improve error context to include the failing asset path (per‑mesh failures) and keep rendering.
  - [ ] Add a visible “substitution/fallback in effect” notice when falling back to sample URDF.

- Suggested Components
  - [x] Populate from catalog only; display concrete slugs/paths in the list.
  - [ ] Include a subtle note if a recommended item from LLM is missing and a placeholder is shown (Phase 1 minimal messaging).

- Performance and resilience
  - [ ] Add lightweight performance logging (first content time for hero GLB, average FPS for URDF mode during first 5s).
  - [ ] Size budget check for hero GLBs (log a warning when > 50 MB).
  - [ ] Lazy-load environment HDR only after model begins loading to reduce FCP (if needed after measuring).

- Tests
  - [x] E2E: Configure page renders Canvas and loads sample on “Load Sample” (existing Playwright test covers baseline canvas).
  - [ ] E2E: Preset = `unitree-dog` → hero GLB render path taken when `external` contains a Unitree GLB.
  - [ ] Integration: Analyze → Configure handoff via `localStorage` present.
  - [ ] Unit: `specToUrdf` validation and path assembly.
  - [ ] Unit: `motionPlayer` sampling/looping behavior.
  - [ ] Unit: `selectFromCatalog` ranking and fallback behavior.

---

### Phase 1.5 — Dev Catalog Automation (non‑prod)

- Dev‑only FS scan merged with static list
  - [ ] Implement optional FS scan of `apps/web/public/robots/**` when `NODE_ENV !== 'production'`.
  - [ ] Derive `slug` from directory names; include URDFs and hero GLBs; collect `linksGlb` when present.
  - [ ] Merge with static entries and de‑duplicate by path; return combined manifest.
  - [ ] Add in‑memory cache with short TTL (e.g., 5–10 seconds) to avoid hot‑reload thrash.
  - [ ] Gate behind `CATALOG_SCAN_DEV=true` env to keep default behavior stable.

- Dev safety checks
  - [ ] Skip or cap extremely large GLB files (> 100 MB) to avoid accidental giant assets during dev.
  - [ ] Log per‑asset scan results with counts for debugging.

---

### Phase 2 — Build‑time Manifest & Enhancements

- Build‑time catalog generator
  - [ ] Create `apps/web/scripts/build-catalog.ts` to scan `/public/robots` and emit `apps/web/src/data/catalog.json`.
  - [ ] Update `/api/catalog` to serve the generated JSON (no runtime FS access).
  - [ ] Add a GitHub Action to run the generator and commit updated manifest on asset changes.
  - [ ] Validation step: ensure URDF mesh paths are relative; hero GLBs are absolute under `/robots/`.

- Catalog metadata & selection quality
  - [ ] Extend manifest schema with optional `kind`, `tags[]`, `thumbnail`, and `metrics` (bbox, triangle count).
  - [ ] Add nearest‑match substitution by tag overlap when LLM recommends a missing item; show a clear substitution indicator.
  - [ ] Surface thumbnails and metrics in Suggested Components for better UX.

- Performance & UX
  - [ ] Introduce loader result caching across sessions where feasible.
  - [ ] Split environment map and heavy assets to optimize FCP on slow networks.

- Tests
  - [ ] Unit: manifest generator path inference and schema validation.
  - [ ] E2E: substitution flow visibly indicates alternative selection.

---

### Acceptance Criteria Cross‑check

- [x] Catalog JSON with `slug`, `kind` (future), `tags` (future), `assets` (Phase 1: heroGlb/urdf/linksGlb), `source/license` (future optional).
- [x] Analyze provides catalog to LLM; recommendations reference resolvable catalog items.
- [x] Viewer supports hero GLB and URDF in the same canvas.
- [ ] Per‑asset load failure shows placeholder with failing path and continues rendering.
- [x] Suggested Components draws only from catalog.
- [ ] When an LLM recommendation is missing, show nearest catalog alternative with substitution indicator (Phase 2).
- [x] New assets auto‑included without code changes via planned automation (Phase 1.5/2 roadmap in place).
- [ ] Performance targets measured/logged (≤ 2 s hero GLB FCP, ≥ 30 FPS URDF for ≤ 200k tris).
- [x] Path conventions observed (URDF meshes relative; hero GLBs absolute under `/robots/`).

---

### Definition of Done (Phase 1)

- Core flows stable: Analyze → Configure with catalog‑constrained recommendations.
- Viewer renders hero GLB or URDF with ground snap and lighting, no crashes on asset issues.
- Suggested Components populated from catalog; paths visible.
- Baseline tests in place (unit/integration/E2E) and passing locally.
- Performance logging present; no regressions in demo scenarios.

---

### Risks & Mitigations

- Runtime FS unavailable in prod → Use build‑time generator (Phase 2) and dev‑only scan.
- Large GLBs causing slow FCP → Size budgets, compression, and lazy asset strategies.
- LLM recommending non‑catalog items → Client‑side mapping + substitution logic (Phase 2). 


