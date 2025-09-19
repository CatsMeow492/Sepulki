import type { RobotSpec } from '@/types/robot'

export class SpecValidationError extends Error {
  constructor(message: string) {
    super(message)
    this.name = 'SpecValidationError'
  }
}

function xmlEscape(value: string): string {
  return value
    .replace(/&/g, '&amp;')
    .replace(/</g, '&lt;')
    .replace(/>/g, '&gt;')
    .replace(/"/g, '&quot;')
    .replace(/'/g, '&apos;')
}

export function specToUrdf(spec: RobotSpec, baseUrl: string = ''): string {
  if (!spec || !spec.name) throw new SpecValidationError('SPEC_INVALID: missing name')
  if (!Array.isArray(spec.links) || spec.links.length === 0) {
    throw new SpecValidationError('SPEC_INVALID: requires at least one link')
  }
  if (!Array.isArray(spec.joints)) throw new SpecValidationError('SPEC_INVALID: joints must be an array')

  const linkNames = new Set(spec.links.map((l) => l.name))

  for (const j of spec.joints) {
    if (!linkNames.has(j.parent) || !linkNames.has(j.child)) {
      throw new SpecValidationError(`SPEC_INVALID_REFERENCE: joint ${j.name} references unknown link`)
    }
  }

  const linksXml = spec.links
    .map((l) => {
      const visual = l.mesh
        ? `\n    <visual>\n      <geometry>\n        <mesh filename="${xmlEscape(
            `${baseUrl ? baseUrl.replace(/\/$/, '') + '/' : ''}${l.mesh.uri.replace(/^\//, '')}`,
          )}"${l.mesh.scale ? ` scale="${l.mesh.scale.join(' ')}"` : ''}/>\n      </geometry>\n    </visual>`
        : ''
      return `  <link name="${xmlEscape(l.name)}">${visual}\n  </link>`
    })
    .join('\n')

  const jointsXml = spec.joints
    .map((j) => {
      const origin = j.origin
        ? `<origin${j.origin.xyz ? ` xyz="${j.origin.xyz.join(' ')}"` : ''}${
            j.origin.rpy ? ` rpy="${j.origin.rpy.join(' ')}"` : ''
          }/>`
        : ''
      const axis = j.axis ? `<axis xyz="${j.axis.join(' ')}"/>` : ''
      const limit = j.limit
        ? `<limit lower="${j.limit.lower}" upper="${j.limit.upper}"${
            j.limit.velocity != null ? ` velocity="${j.limit.velocity}"` : ''
          }/>`
        : ''
      return `  <joint name="${xmlEscape(j.name)}" type="${j.type}">\n    <parent link="${xmlEscape(
        j.parent,
      )}"/>\n    <child link="${xmlEscape(j.child)}"/>\n    ${origin}\n    ${axis}\n    ${limit}\n  </joint>`
    })
    .join('\n')

  return `<?xml version="1.0"?>\n<robot name="${xmlEscape(spec.name)}">\n${linksXml}\n${jointsXml}\n</robot>`
}


