import type { MotionKeyframe, MotionProfile, JointPose } from '@/types/robot'

function lerp(a: number, b: number, t: number): number {
  return a + (b - a) * t
}

function findFrameInterval(frames: MotionKeyframe[], t: number): [MotionKeyframe, MotionKeyframe] | null {
  if (frames.length === 0) return null
  if (t <= frames[0].t) return [frames[0], frames[0]]
  if (t >= frames[frames.length - 1].t) return [frames[frames.length - 1], frames[frames.length - 1]]
  for (let i = 0; i < frames.length - 1; i++) {
    const a = frames[i]
    const b = frames[i + 1]
    if (t >= a.t && t <= b.t) return [a, b]
  }
  return [frames[frames.length - 1], frames[frames.length - 1]]
}

export function sampleProfile(profile: MotionProfile, t: number): JointPose {
  const duration = profile.duration
  const clamped = Math.max(0, Math.min(duration, t))
  const interval = findFrameInterval(profile.frames, clamped)
  if (!interval) return {}
  const [a, b] = interval
  if (a.t === b.t) return { ...a.pose }
  const localT = (clamped - a.t) / (b.t - a.t)
  const pose: JointPose = {}
  const joints = new Set<string>([...Object.keys(a.pose), ...Object.keys(b.pose)])
  joints.forEach((name) => {
    const va = a.pose[name] ?? 0
    const vb = b.pose[name] ?? va
    pose[name] = lerp(va, vb, localT)
  })
  return pose
}

export class MotionPlayer {
  private profile: MotionProfile | null = null
  private time = 0
  private playing = false
  private rate = 1

  loadProfile(profile: MotionProfile) {
    this.profile = profile
    this.time = 0
    this.playing = false
  }

  setRate(rate: number) {
    this.rate = rate
  }

  play() {
    this.playing = true
  }

  pause() {
    this.playing = false
  }

  seek(t: number) {
    if (!this.profile) return
    this.time = Math.max(0, Math.min(this.profile.duration, t))
  }

  getTime() {
    return this.time
  }

  isPlaying() {
    return this.playing
  }

  tick(deltaSeconds: number) {
    if (!this.profile || !this.playing) return
    this.time += deltaSeconds * this.rate
    if (this.time > this.profile.duration) {
      if (this.profile.loop) {
        this.time = this.time % this.profile.duration
      } else {
        this.time = this.profile.duration
        this.playing = false
      }
    }
    if (this.time < 0) this.time = 0
  }

  sample(): JointPose {
    if (!this.profile) return {}
    return sampleProfile(this.profile, this.time)
  }
}


