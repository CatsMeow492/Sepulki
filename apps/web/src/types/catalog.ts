export interface CatalogItem {
  slug: string
  urdf?: string[] // URLs like /robots/<slug>/urdf/sample.urdf
  linksGlb?: string[] // URLs of per-link GLBs: /robots/<slug>/links/*.glb
  heroGlb?: string[] // URLs of full robot GLBs: /robots/<slug>/*.glb or /robots/external/*.glb
}

export interface CatalogResponse {
  items: CatalogItem[]
}


