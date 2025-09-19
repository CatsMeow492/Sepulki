# Requirements — Catalog-Driven 3D Configurator (Phase 1)

System name: Artifex Catalog-Driven Configurator

## User Stories
- As a prospect, I want to describe my use case in plain language, so that the system recommends concrete products from our catalog.
- As a sales engineer, I want AI recommendations to reference only items we actually have (by slug/path), so that proposals are feasible and demo-ready.
- As a reviewer, I want the 3D viewer to load the selected platform automatically (hero GLB or articulated URDF), so that I can validate fit and motion quickly.
- As a configurator user, I want the Suggested Components list (platform, controller, sensors, end-effector) to come from the catalog, so that I see a realistic, orderable BOM.
- As a content owner, I want new assets added under apps/web/public/robots/ to appear automatically in the catalog, so that expanding the catalog requires no code changes.
- As a demo operator, I want graceful fallbacks for missing/invalid assets, so that the UI never crashes during live demos.

## Acceptance Criteria (EARS)

### Catalog and Selection
- Ubiquitous requirement: The Artifex Catalog-Driven Configurator shall expose a machine-readable catalog manifest (JSON) listing, for each item: `slug`, `kind`, `tags[]`, `assets` (hero GLB path, URDF path(s), per-link GLB path(s)), and `source/license`.
- Event-driven requirement: When the Analyze step completes, the Artifex Catalog-Driven Configurator shall supply the catalog to the LLM and require recommendations to include resolvable catalog references (exact `slug` and/or asset path).
- Optional feature requirement: Where multiple catalog matches exist, the Artifex Catalog-Driven Configurator shall prefer items whose tags best match the inferred use case (e.g., “quadruped”, “patrol”, “industrial-arm”).

### Viewer Behavior
- Ubiquitous requirement: The Artifex Catalog-Driven Configurator shall support hero GLB (static) and URDF (articulated) rendering modes within the same viewer.
- State-driven requirement: While a recommended item includes a hero GLB, the Artifex Catalog-Driven Configurator shall render that GLB as the primary preview on the Configure page.
- State-driven requirement: While a recommended item includes a URDF (and per-link GLBs), the Artifex Catalog-Driven Configurator shall render the articulated model and enable joint controls and motion playback.
- Unwanted-behavior requirement: If an asset path 404s or fails to parse, then the Artifex Catalog-Driven Configurator shall show a visible placeholder with the failing path and continue rendering without crashing.

### Suggested Components and BOM
- Ubiquitous requirement: The Artifex Catalog-Driven Configurator shall populate the Suggested Components list using only items present in the catalog manifest.
- Event-driven requirement: When the LLM returns a Suggested Configuration, the Artifex Catalog-Driven Configurator shall map each recommendation to catalog items (by `slug`/path) and display titles plus resolvable asset paths.
- Unwanted-behavior requirement: If a recommended item is not found in the catalog, then the Artifex Catalog-Driven Configurator shall present the nearest catalog alternative (by tag overlap) and indicate the substitution.

### Catalog Automation
- Event-driven requirement: When new assets are added under apps/web/public/robots/ and the catalog scan is executed, the Artifex Catalog-Driven Configurator shall include those assets in the manifest without code changes.
- Optional feature requirement: Where per-link GLBs and a URDF template are present, the Artifex Catalog-Driven Configurator shall prefer articulated mode for richer interaction.

### Performance and Resilience
- Ubiquitous requirement: The Artifex Catalog-Driven Configurator shall render first content for hero GLB within <2 s for assets ≤50 MB on the reference laptop, and maintain ≥30 FPS for URDF mode with ≤200k triangles.
- Unwanted-behavior requirement: If the LLM response is malformed or empty, then the Artifex Catalog-Driven Configurator shall fall back to a default catalog item and notify the user non-blockingly.

### Constraints & Conventions
- Ubiquitous requirement: The Artifex Catalog-Driven Configurator shall require URDF mesh paths to be relative to the URDF file location and hero GLB paths to be absolute under /robots/.
- Optional feature requirement: Where available, catalog items shall include `thumbnail` and `metrics` (bbox, triangle count) to improve selection and rendering hints.

### References
- EARS guidance: https://alistairmavin.com/ears/
- Acceptance criteria best practices: https://www.altexsoft.com/blog/acceptance-criteria-purposes-formats-and-best-practices/