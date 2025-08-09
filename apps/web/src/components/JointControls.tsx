import { useEffect, useMemo, useState } from 'react'

type JointInfo = {
  name: string
  value: number
  min?: number
  max?: number
}

export function JointControls({
  joints,
  onChange,
}: {
  joints: JointInfo[]
  onChange: (name: string, value: number) => void
}) {
  const [local, setLocal] = useState<Record<string, number>>({})

  useEffect(() => {
    const next: Record<string, number> = {}
    joints.forEach((j) => (next[j.name] = j.value))
    setLocal(next)
  }, [joints])

  return (
    <div className="space-y-3">
      {joints.map((j) => {
        const min = j.min ?? -Math.PI
        const max = j.max ?? Math.PI
        const val = local[j.name] ?? j.value
        return (
          <div key={j.name} className="flex items-center gap-3">
            <label className="w-32 text-sm text-gray-700">{j.name}</label>
            <input
              type="range"
              min={min}
              max={max}
              step={0.01}
              value={val}
              onChange={(e) => {
                const v = Math.max(min, Math.min(max, Number(e.target.value)))
                setLocal((s) => ({ ...s, [j.name]: v }))
                onChange(j.name, v)
              }}
              className="flex-1"
            />
            <span className="w-20 text-right text-xs text-gray-600">{val.toFixed(2)}</span>
          </div>
        )
      })}
    </div>
  )
}


