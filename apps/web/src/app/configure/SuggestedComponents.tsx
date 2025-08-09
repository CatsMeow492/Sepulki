'use client'

import { useEffect, useState } from 'react'
import { fetchCatalog } from '@/lib/catalog'
import { suggestPresetFromAnalysis } from '@/lib/presets'

type Card = { title: string; subtitle?: string }

export function SuggestedComponents({ analysis, userInput }: { analysis: string; userInput: string }) {
  const [cards, setCards] = useState<Card[]>([])

  useEffect(() => {
    const run = async () => {
      const preset = suggestPresetFromAnalysis(userInput, analysis)
      const catalog = await fetchCatalog()
      const items: Card[] = []
      if (preset === 'unitree-dog') {
        const hero = catalog.find((c) => c.slug === 'external')
        const dog = hero?.heroGlb?.find((u) => /unitree|dog/i.test(u)) || hero?.heroGlb?.[0]
        if (dog) items.push({ title: 'Unitree Robot Dog', subtitle: dog })
        items.push({ title: 'Perception Suite', subtitle: 'Stereo/Depth camera + lidar' })
        items.push({ title: 'Patrol Controller', subtitle: 'Navigation + geofencing' })
      } else {
        const arm = catalog.find((c) => c.slug === 'sample-arm-01')
        if (arm?.urdf?.[0]) items.push({ title: 'Articulated Arm (Sample-01)', subtitle: arm.urdf[0] })
        items.push({ title: 'Controller', subtitle: '6-axes motion + IO' })
        items.push({ title: 'End Effector', subtitle: 'Specify gripper / tool' })
      }
      setCards(items)
    }
    run()
  }, [analysis, userInput])

  return (
    <div className="space-y-4">
      {cards.map((c, i) => (
        <div key={i} className="flex items-start space-x-4">
          <div className="flex-shrink-0">
            <div className="w-10 h-10 bg-gray-100 rounded-lg flex items-center justify-center">⚙️</div>
          </div>
          <div>
            <h3 className="font-medium text-gray-900">{c.title}</h3>
            {c.subtitle && <p className="text-sm text-gray-500 break-all">{c.subtitle}</p>}
          </div>
        </div>
      ))}
    </div>
  )
}


