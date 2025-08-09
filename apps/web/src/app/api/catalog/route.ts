import { NextResponse } from 'next/server'
import type { CatalogResponse } from '@/types/catalog'

// Minimal static catalog generator:
// We assume slugs we know about; for full automation we could add a build-time script to scan /public/robots.
// For MVP, include known folders and any hero GLBs placed under /robots/external.

const knownSlugs = ['sample-arm-01']

function buildCatalog(): CatalogResponse {
  const items = [] as CatalogResponse['items']
  for (const slug of knownSlugs) {
    items.push({
      slug,
      urdf: [`/robots/${slug}/urdf/sample.urdf`],
      linksGlb: [
        `/robots/${slug}/links/base.glb`,
        `/robots/${slug}/links/link1.glb`,
        `/robots/${slug}/links/link2.glb`,
        `/robots/${slug}/links/link3.glb`,
        `/robots/${slug}/links/wrist.glb`,
        `/robots/${slug}/links/ee.glb`,
      ],
    })
  }
  // Provide a generic external placeholder
  items.push({
    slug: 'external',
    heroGlb: [
      // Known common locations; first existing will be used by the client selector
      '/robots/robot_dog_unitree_go2.glb',
      '/robots/external/unitree.glb',
      '/robots/external/factory-arm.glb',
    ],
  })
  return { items }
}

export async function GET() {
  const catalog = buildCatalog()
  return NextResponse.json(catalog)
}


