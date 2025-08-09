# Tasks — 3D Renderer for Robotics Specs (MVP)

Numbered, incremental, code-focused tasks. Each task references the approved requirements and leans on test-first where feasible.

Legend of requirements (short refs):
- R-LR-URDF: Load URDF + glTF and render articulated scene (Loading & Rendering)
- R-LR-PLACEHOLDER: Placeholder + non-blocking error on missing mesh (Loading & Rendering)
- R-LR-CAMERA: Orbit/pan/zoom + default preset (Loading & Rendering)
- R-KM-JOINT-UPDATE: Update transforms ≤33ms, clamp to limits (Kinematics & Motion)
- R-KM-FK: FK driven by spec/UI (Kinematics & Motion)
- R-KM-PLAYBACK: Play/pause/scrub deterministic (Kinematics & Motion)
- R-KM-LIMIT-WARN: Warn/deny on limit exceed (Kinematics & Motion)
- R-INT-UI: Joint sliders, play/pause, speed, reset view (Interaction & UI)
- R-VAL-STRUCTURED: Structured errors for invalid spec/URDF (Validation & Errors)
- R-VAL-CRITICAL: Clear error state + retry without crash (Validation & Errors)
- R-PERF-30FPS: ≥30 FPS on reference device (Performance)
- R-DET-POSE: Deterministic poses without physics (Performance/Determinism)

---

1) Tooling & dependencies (setup)
- [x] Install runtime deps: `npm i -w @artifex/web urdf-loader`
- [x] Install test deps: `npm i -D -w @artifex/web jest @types/jest ts-jest jest-environment-jsdom jest-canvas-mock`
- [x] Install e2e deps: `npm i -D -w @artifex/web @playwright/test`
- [x] Add Jest config (jsdom), setup `jest-canvas-mock`, and a `test` script in `apps/web/package.json`.
- References: R-VAL-STRUCTURED, R-DET-POSE (enables TDD baseline)

2) Types and contracts
- [x] Create `apps/web/src/types/robot.ts` with `RobotSpec`, `JointSpec`, `LinkSpec`, `JointLimit`, `MotionProfile`, etc. (from design)
- [ ] Add ambient type for `urdf-loader` if needed: `apps/web/src/types/urdf-loader.d.ts` declaring `module 'urdf-loader'`.
- References: R-KM-FK, R-KM-PLAYBACK

3) Spec→URDF transformer (unit-tested first)
- [x] Create `apps/web/src/lib/specToUrdf.ts` exporting `specToUrdf(spec: RobotSpec, baseUrl?: string): string`.
- [ ] Unit tests: valid minimal spec → URDF string contains links/joints/limits and mesh refs; invalid refs → throws with structured error type.
- References: R-LR-URDF, R-VAL-STRUCTURED

4) Motion player (deterministic clock)
- [x] Create `apps/web/src/lib/motionPlayer.ts` with APIs: `loadProfile`, `play`, `pause`, `seek`, `getTime`, `sample(t)`.
- [ ] Unit tests: deterministic interpolation (t=0, mid, end), loop behavior, seek boundaries.
- References: R-KM-PLAYBACK, R-DET-POSE

5) RobotModel (URDF ingestion wrapper)
- [x] Create `apps/web/src/components/RobotModel.tsx` that loads URDF via `URDFLoader` and exposes a joint control API via `ref`/callbacks.
- [x] Support: missing mesh → placeholder node + non-blocking error callback; expose `getJoint(name)`, `setJointValue(name, v)`.
- [ ] Smoke test (component): render in jsdom with `jest-canvas-mock` and assert mount + error path.
- References: R-LR-URDF, R-LR-PLACEHOLDER, R-KM-FK

6) Replace Scene3D with prop-driven host
- [x] Refactor `apps/web/src/components/Scene3D.tsx` to accept props: `{ spec?: RobotSpec; urdf?: string | URL; assetBaseUrl?: string; showAxes?: boolean; profile?: MotionProfile; }`.
- [x] Build URDF from `spec` using `specToUrdf` (when provided); then render `<RobotModel>`; keep `OrbitControls`, lights; add reset view button/helper.
- [ ] Ensure deterministic render loop; avoid per-frame allocations; memoize loaders.
- References: R-LR-URDF, R-LR-CAMERA, R-PERF-30FPS, R-DET-POSE

7) JointControls UI
- [x] Create `apps/web/src/components/JointControls.tsx` with sliders: show min/max from limits, clamp on input, visual warn on clamp.
- [ ] Component tests: clamping and value reflection.
- References: R-INT-UI, R-KM-JOINT-UPDATE, R-KM-LIMIT-WARN

8) Basic ErrorBanner
- [x] Create `apps/web/src/components/ErrorBanner.tsx` for non-blocking errors (load/parse issues) with retry.
- [x] Wire into `Scene3D` for URDF load failures.
- References: R-VAL-STRUCTURED, R-VAL-CRITICAL

 9) Configure page wiring
- [x] Update `apps/web/src/app/configure/page.tsx` to pass props into `Scene3D` (SSR disabled import remains).
- [x] Add a source selector (tabs): "Spec JSON" and "URDF"; textarea inputs and a "Build Model" button.
- [x] Render `JointControls` below the canvas; wire to `RobotModel` via callbacks from `Scene3D`.
 - [x] Add simple playback controls (Play/Pause, scrub slider, playback rate) calling `motionPlayer` via `Scene3D`.
- References: R-INT-UI, R-KM-PLAYBACK, R-KM-FK

10) Sample assets and samples
- [x] Add sample folder: `apps/web/public/robots/sample-arm-01/{links,textures,urdf}` (binary `.glb` meshes for 2–3 links, or use URDF primitive shapes for MVP).
- [x] Add `apps/web/public/robots/sample-arm-01/urdf/sample.urdf` (optional); also add a `sampleSpec.json` to be converted at runtime.
- [x] Verify `assetBaseUrl = '/robots/sample-arm-01'` pathing in `specToUrdf` and URDF refs.
- [x] Add basic link meshes (`base.obj`, `link1.obj`) as stand-ins for `.glb`.
- References: R-LR-URDF, R-LR-PLACEHOLDER

11) E2E smoke (Playwright)
- [ ] Add `playwright.config.ts`; write test: open `/configure`, load sample spec via button, expect canvas to appear and no error banner.
- [ ] Interact: move one joint slider, assert UI value changed; click Play then Pause; scrub slider updates displayed time.
- References: R-LR-URDF, R-KM-JOINT-UPDATE, R-KM-PLAYBACK

12) Performance pass
- [ ] Profile with React devtools + browser profiler; ensure ≥30 FPS for sample model; reduce material count, enable shadows cautiously.
- [ ] Throttle UI setState; avoid re-renders on every frame.
- References: R-PERF-30FPS, R-DET-POSE

13) Clean-up and removal of placeholder
- [ ] Remove `apps/web/src/components/RoboticArm.tsx`; update `apps/web/src/components/index.ts` exports.
- [ ] Verify all imports updated; run lints and build.
- References: housekeeping

14) DX polish
- [ ] Add `README` section in `apps/web/README.md` describing how to run the viewer, where to place assets, and how to feed Spec JSON or URDF.
- [ ] Add `npm scripts`: `test`, `test:e2e`, `test:ui`.
- References: onboarding for demos

15) Final verification
- [ ] `npm run lint -w @artifex/web` → 0 errors
- [ ] `npm run build -w @artifex/web` → success
- [ ] `npm run test -w @artifex/web` → unit pass; `npx playwright test -w @artifex/web` → e2e pass
- References: readiness gate for demo

16) Waitlist signup (end-of-workflow CTA)
- [ ] Create `apps/web/src/components/WaitlistSignup.tsx` (email input, inline validation, success/err banner).
- [ ] Create API route `apps/web/src/app/api/waitlist/route.ts` (POST { email, context? }) that integrates with a provider (e.g., Mailchimp) using env vars: `WAITLIST_PROVIDER=mailchimp`, `MAILCHIMP_API_KEY`, `MAILCHIMP_LIST_ID`. In dev/no provider: log and return 200.
- [ ] Add CTA placement at the bottom of `apps/web/src/app/review/page.tsx` (and/or after successful model build in `configure/page.tsx`).
- [ ] Unit test API handler (mock provider fetch) and component (validation); E2E: submit email → see thank-you state.
- [ ] Update `apps/web/README.md` with env var setup and provider notes.
- References: growth funnel; non-functional to core renderer
