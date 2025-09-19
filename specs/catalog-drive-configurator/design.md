### Design — Catalog-Driven 3D Configurator (Phase 1)

#### Objective
Deliver a catalog-driven, demo-stable 3D configurator that: (1) constrains AI recommendations to real catalog items, (2) renders either a static hero GLB or an articulated URDF in a unified viewer, and (3) populates a Suggested Components list from the catalog. Phase 1 prioritizes reliability, graceful fallbacks, and a clean upgrade path to full automation of the catalog manifest.

---

### System Architecture

- **Client (Next.js App Router, React 18, TypeScript)**
  - Pages: `apps/web/src/app/analyze/page.tsx`, `apps/web/src/app/configure/page.tsx`
  - Components: `Scene3D` (unified viewer), `RobotModel` (URDF), `SuggestedComponents`, `JointControls`, `ErrorBanner`
  - Libraries: `specToUrdf` (build URDF from JSON spec), `motionPlayer` (keyframe playback), `presets` (heuristic mapping), `catalog` (fetch/select helpers)
- **API Routes**
  - `GET /api/catalog` → returns catalog manifest (Phase 1: minimal static generator)
  - `POST /api/analyze` → calls OpenAI with user input and the catalog manifest embedded in the prompt
- **Assets**
  - Public robots under `apps/web/public/robots/` (e.g., hero GLBs, URDFs, per-link meshes)

---

### Data Model

- Catalog manifest (runtime response shape)
  - Type: `CatalogResponse` with `items: CatalogItem[]`
  - `CatalogItem` fields (Phase 1):
    - `slug: string`
    - `urdf?: string[]` (absolute URLs under `/robots/…/urdf/*.urdf`)
    - `linksGlb?: string[]` (absolute URLs under `/robots/…/links/*.glb`)
    - `heroGlb?: string[]` (absolute URLs under `/robots/**/*.glb`)
  - Forward‑compatible (Phase 2+ optional): `kind`, `tags[]`, `thumbnail`, `metrics`, `source`, `license`

- Robot specification (viewer input option)
  - `RobotSpec` JSON convertible to URDF via `specToUrdf(spec, baseUrl)`
  - URDF mesh path convention: per Phase 1, mesh paths are relative to the URDF file’s directory; hero GLB is absolute under `/robots/…`

---

### Key Flows

1) Analyze → Configure handoff
   - User lands on Analyze with `?input=<free text>`.
   - Analyze fetches the catalog (`/api/catalog`) and posts `{ userInput, catalogText }` to `/api/analyze`.
   - The response `analysis` is persisted in `localStorage` along with `userInput`.
   - User continues to Configure with `?step=2`.

2) Configure boot
   - Configure loads `analysis` and `userInput` from `localStorage`.
   - Heuristic preset selection via `suggestPresetFromAnalysis(userInput, analysis)` ∈ { `unitree-dog`, `industrial-arm`, `sample-arm` }.
   - Fetch catalog and select assets via `selectFromCatalog(catalog, preset)`:
     - If hero GLB candidates exist: render the first found as a static model (Scene3D `heroUrl` path).
     - Else if URDF exists: render articulated model (Scene3D `urdf` path).
     - Else fallback to the sample arm URDF.

3) Suggested Components
   - `SuggestedComponents` maps preset and catalog items to a minimal BOM list, referencing exact URLs or slugs from the manifest.
   - In Phase 2, this evolves to a tag/score‑based nearest match when exact references are missing.

4) Viewer behavior (Scene3D)
   - Unified Canvas with two modes:
     - Hero GLB mode: auto‑scale to ~2 m height, ground‑snap, environment lighting.
     - URDF mode: `RobotModel` loads URDF with per‑link meshes (OBJ/GLTF/GLB), exposes joint API, supports `MotionPlayer` sampling.
   - Errors surface via `ErrorBanner`; model placeholder is inserted without crashing.

---

### API Design

- `GET /api/catalog`
  - Phase 1: Minimal generator with known slugs and an `external` bucket for common GLBs.
  - Response: `CatalogResponse`
  - Notes: For demo robustness, include multiple hero candidates; the client sorts by relevance.

- `POST /api/analyze`
  - Request: `{ userInput: string, catalogText?: string }`
  - Behavior: Calls OpenAI; the system prompt instructs using only items present in `catalogText`.
  - Response: `{ analysis: string }` (markdown structured per prompt)
  - Resilience: 4xx on missing input; 5xx on upstream errors.

---

### Catalog Automation Plan

- Phase 1 (current):
  - Static builder in `GET /api/catalog` returns known catalog entries (`sample-arm-01`) and an `external` group listing known hero GLBs.

- Phase 1.5 (dev convenience):
  - Add a dev‑only FS scan (Node `fs`/`path`) to enumerate `apps/web/public/robots/**` at request time when `process.env.NODE_ENV !== 'production'`. Merge results into the static list, deduplicated by slug/path.

- Phase 2 (production automation):
  - Build‑time script `apps/web/scripts/build-catalog.ts` scans `public/robots/` and emits `apps/web/src/data/catalog.json` (committed or bundled). The API route serves this JSON directly.
  - Optional GitHub Action to validate new assets (file size, triangle count metadata, license presence) and update the manifest.

Manifest inference rules (for both scan paths):
  - A folder with `urdf/*.urdf` is an articulated item; include relative per‑link mesh references if found.
  - Any `*.glb` directly under a robot folder or under `/robots/external/` is a hero GLB candidate.
  - Derive `slug` from top‑level folder name under `/robots/`.

---

### Viewer & Rendering Behavior

- Grounding and framing
  - Auto compute bounds and lift model so `minY ≈ 0` with a small epsilon to avoid z‑fighting.
  - Use `PerspectiveCamera` + `OrbitControls` with constrained polar range.

- Performance targets
  - First contentful render (hero GLB ≤ 50 MB): < 2 s on the reference laptop.
  - URDF mode target: ≥ 30 FPS with ≤ 200k triangles.
  - Optimizations (non‑breaking, incremental):
    - Prefer binary GLB over glTF+bin.
    - Compress textures; strip animations from hero GLBs.
    - Lazy‑load environment map; cache loader results.

- Robustness
  - Any load error yields an on‑canvas `ErrorBanner` with the failing path and a neutral placeholder node; the scene remains interactive.
  - Scene keeps running even if some per‑link meshes fail; URDF skeleton is still visible/manipulable.

---

### Recommendation Mapping

- Heuristic preset selection in Phase 1 (`presets.ts`) using keyword detection in `(userInput + analysis)`.
- Catalog‑constrained LLM in Analyze step to bias towards resolvable `slug` and exact asset paths.
- Suggested Components populates from the manifest only; if a recommended item is missing, Phase 2 introduces nearest‑match substitution based on tag overlap and shows a substitution indicator.

---

### Error Handling & Fallbacks

- Catalog fetch failure → treat as empty catalog; Configure falls back to sample arm (`/robots/sample-arm-01/urdf/sample.urdf`).
- Asset 404 / parse error → `ErrorBanner` + placeholder, continue rendering.
- Malformed LLM response → fall back to default catalog item (sample arm) and show a non‑blocking notice in Configure.

---

### Acceptance Criteria Mapping (Phase 1)

- Catalog manifest exposed → `GET /api/catalog` returns `slug`, `heroGlb`, `urdf`, `linksGlb`.
- Analyze step supplies catalog to LLM → Analyze fetches catalog and includes it in the prompt body.
- Viewer supports GLB and URDF → `Scene3D` switches between `heroUrl` and `urdf` modes.
- Graceful error fallback → `ErrorBanner` + placeholder; no crashes.
- Suggested Components from catalog → `SuggestedComponents` builds list from manifest.
- New assets auto‑inclusion → Roadmapped: dev FS scan (Phase 1.5) and build‑time generator (Phase 2).
- Performance targets → Documented; enforce via asset curation and lazy loading; measure during demos.
- Path conventions → URDF meshes relative; hero GLBs absolute under `/robots/`.

---

### Phase 1 Implementation Checklist

- API
  - [x] Minimal `GET /api/catalog` static generator
  - [x] `POST /api/analyze` with catalog‑aware system prompt

- UI/Flows
  - [x] Analyze persists `analysis` and `userInput` to `localStorage`
  - [x] Configure loads analysis, selects preset, fetches catalog, chooses hero/URDF
  - [x] Suggested Components lists items from manifest
  - [x] Unified viewer renders GLB/URDF and exposes joint controls + motion playback
  - [x] Error banner + placeholder for asset failures

- Future (Phase 1.5/2)
  - [ ] Dev FS scan for `/robots/**` (non‑prod)
  - [ ] Build‑time catalog generator and JSON manifest
  - [ ] Tagging and nearest‑match substitution with indicator
  - [ ] Optional thumbnails and metrics in manifest

---

### Risks & Mitigations

- Runtime FS access on serverless (production) is restricted
  - Mitigation: build‑time manifest generation (Phase 2), dev‑only FS scan
- Large hero GLBs can delay FCP
  - Mitigation: size budget, compressed assets, preload, and environment map lazy load
- LLM hallucination despite catalog context
  - Mitigation: strict client mapping to catalog; default and substitution behaviors

---

### Testing Strategy (Phase 1)

- Unit
  - `specToUrdf` validation errors and path generation
  - `motionPlayer` sampling/looping
  - Preset heuristic mapping

- Integration
  - Analyze → Configure handoff via `localStorage`
  - `/api/catalog` response shape stable for UI consumers
  - Scene renders hero GLB and URDF; asset failure shows `ErrorBanner`

- E2E (Playwright)
  - Configure page loads sample arm via “Load Sample” and renders Canvas
  - Hero GLB path present renders static model when preset = `unitree-dog`


