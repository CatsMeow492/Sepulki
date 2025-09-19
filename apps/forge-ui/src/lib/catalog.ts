import type { CatalogItem, CatalogResponse } from '@/types/catalog'

export async function fetchCatalog(): Promise<CatalogItem[]> {
  try {
    const res = await fetch('/api/catalog', { cache: 'no-store' })
    if (!res.ok) return []
    const data = (await res.json()) as CatalogResponse
    return data.items
  } catch {
    return []
  }
}

export function selectFromCatalog(
  catalog: CatalogItem[],
  preset: 'unitree-dog' | 'industrial-arm' | 'sample-arm',
): { heroCandidates?: string[]; urdf?: string } {
  if (preset === 'unitree-dog') {
    const ext = catalog.find((c) => c.slug === 'external')
    const heroes = (ext?.heroGlb || []).filter(Boolean)
    // Prefer 'robot_dog_unitree_go2' or any 'unitree' in name; keep all as fallbacks
    const rank = (u: string) => (u.toLowerCase().includes('robot_dog_unitree_go2') ? 2 : u.toLowerCase().includes('unitree') ? 1 : 0)
    const sorted = heroes.sort((a, b) => rank(b) - rank(a))
    return { heroCandidates: sorted }
  }
  const arm = catalog.find((c) => c.slug === 'sample-arm-01')
  return { urdf: arm?.urdf?.[0] }
}


