## Artifex 3D Renderer (apps/web)

Dev server:

```bash
npm run dev -w @artifex/web
```

Viewer quickstart:

1) Open `/configure`
2) Click "Load Sample" (loads `/robots/sample-arm-01/urdf/sample.urdf`)
3) Use orbit controls to inspect. Adjust joint slider. Use Play/Pause, Rate, and Seek to preview motion.

Assets:
- Place meshes under `apps/web/public/robots/<slug>/links/*.glb` (or use OBJ for MVP)
- URDFs under `apps/web/public/robots/<slug>/urdf/*.urdf`

Spec JSON mode:
- Paste a `RobotSpec` JSON in the "Spec JSON" tab and click "Build Model". URDF will be generated in-memory.

Testing:
- Unit: `npm run test -w @artifex/web` (no tests included in MVP)
- E2E (optional): `npm run test:e2e -w @artifex/web` once Playwright tests are added
