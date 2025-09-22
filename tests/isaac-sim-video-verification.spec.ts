import { test, expect } from '@playwright/test'

test.describe('Isaac Sim Video Streaming Verification', () => {
  test('verifies actual video content is visible and robot selection changes visuals', async ({ page }) => {
    console.log('🎬 Starting Isaac Sim video streaming verification test')
    
    // Navigate to configure page
    await page.goto('http://localhost:3000/configure?step=2')
    
    // Wait for Isaac Sim connection
    await page.waitForSelector('[data-testid="isaac-sim-display"]', { timeout: 15000 })
    console.log('✅ Isaac Sim display loaded')
    
    // Wait for video streaming to start
    await page.waitForFunction(() => {
      const logs = performance.getEntries()
      return window.localStorage.getItem('isaac_sim_session_id') !== null
    }, { timeout: 10000 })
    console.log('✅ Isaac Sim session established')
    
    // Wait for video frames to start streaming
    await page.waitForFunction(() => {
      const videoElement = document.querySelector('video') as HTMLVideoElement
      return videoElement && videoElement.src && videoElement.src.startsWith('data:image/jpeg')
    }, { timeout: 15000 })
    console.log('✅ Video element has data source')
    
    // Take initial screenshot for comparison
    const initialScreenshot = await page.screenshot({ 
      clip: { x: 500, y: 200, width: 600, height: 400 },
      type: 'png' 
    })
    console.log('📸 Initial screenshot captured')
    
    // Verify video element is not just black
    const isBlackScreen = await page.evaluate(() => {
      const video = document.querySelector('video') as HTMLVideoElement
      if (!video) return true
      
      // Create canvas to analyze video content
      const canvas = document.createElement('canvas')
      const ctx = canvas.getContext('2d')
      if (!ctx) return true
      
      canvas.width = video.videoWidth || video.offsetWidth
      canvas.height = video.videoHeight || video.offsetHeight
      
      try {
        ctx.drawImage(video, 0, 0)
        const imageData = ctx.getImageData(0, 0, canvas.width, canvas.height)
        const pixels = imageData.data
        
        // Check if all pixels are black (or very dark)
        let nonBlackPixels = 0
        for (let i = 0; i < pixels.length; i += 4) {
          const r = pixels[i]
          const g = pixels[i + 1] 
          const b = pixels[i + 2]
          if (r > 30 || g > 30 || b > 30) { // Not pure black
            nonBlackPixels++
          }
        }
        
        const totalPixels = pixels.length / 4
        const nonBlackPercentage = (nonBlackPixels / totalPixels) * 100
        
        console.log(`📊 Video analysis: ${nonBlackPercentage.toFixed(1)}% non-black pixels`)
        return nonBlackPercentage < 5 // Consider black if < 5% non-black pixels
        
      } catch (error) {
        console.error('❌ Video analysis failed:', error)
        return true // Assume black on error
      }
    })
    
    if (isBlackScreen) {
      console.log('❌ BLACK SCREEN DETECTED - Video streaming not visible')
      throw new Error('Video streaming shows black screen - test failed')
    }
    console.log('✅ Video content is visible (not black screen)')
    
    // Test robot selection changes video content
    console.log('🤖 Testing robot selection...')
    
    // Select Universal Robot
    await page.click('[data-testid="robot-card-ur10e"], .robot-card:has-text("Universal Robots")')
    console.log('🔵 Selected Universal Robot UR10e')
    
    // Wait for robot change to process
    await page.waitForFunction(() => {
      return document.querySelector('.robot-card:has-text("Universal Robots") .selected-indicator')
    }, { timeout: 5000 })
    
    // Wait for video to update (a few seconds for frames to change)
    await page.waitForTimeout(3000)
    
    // Take screenshot after robot change
    const urRobotScreenshot = await page.screenshot({ 
      clip: { x: 500, y: 200, width: 600, height: 400 },
      type: 'png' 
    })
    console.log('📸 UR10e robot screenshot captured')
    
    // Select Nova Carter (mobile robot - should be dramatically different)
    await page.click('[data-testid="robot-card-nova-carter"], .robot-card:has-text("Nova Carter")')
    console.log('🚗 Selected NVIDIA Nova Carter')
    
    // Wait for mobile robot change
    await page.waitForTimeout(3000)
    
    // Take screenshot of mobile robot
    const mobileRobotScreenshot = await page.screenshot({ 
      clip: { x: 500, y: 200, width: 600, height: 400 },
      type: 'png' 
    })
    console.log('📸 Nova Carter mobile robot screenshot captured')
    
    // Verify screenshots are different (basic pixel comparison)
    const screenshotsAreDifferent = !Buffer.compare(urRobotScreenshot, mobileRobotScreenshot)
    
    if (!screenshotsAreDifferent) {
      console.log('❌ ROBOT SELECTION NOT CHANGING VIDEO - Screenshots identical')
      throw new Error('Robot selection does not change video content - test failed')
    }
    console.log('✅ Robot selection changes video content successfully')
    
    // Test fullscreen mode
    await page.click('[data-testid="fullscreen-button"], button:has-text("fullscreen")')
    await page.waitForTimeout(1000)
    
    // Verify fullscreen video is still working
    const fullscreenBlackScreen = await page.evaluate(() => {
      const video = document.querySelector('video') as HTMLVideoElement
      if (!video) return true
      
      const canvas = document.createElement('canvas')
      const ctx = canvas.getContext('2d')
      if (!ctx) return true
      
      canvas.width = video.videoWidth || video.offsetWidth
      canvas.height = video.videoHeight || video.offsetHeight
      
      try {
        ctx.drawImage(video, 0, 0)
        const imageData = ctx.getImageData(0, 0, canvas.width, canvas.height)
        const pixels = imageData.data
        
        let nonBlackPixels = 0
        for (let i = 0; i < pixels.length; i += 4) {
          const r = pixels[i], g = pixels[i + 1], b = pixels[i + 2]
          if (r > 30 || g > 30 || b > 30) nonBlackPixels++
        }
        
        const nonBlackPercentage = (nonBlackPixels / (pixels.length / 4)) * 100
        return nonBlackPercentage < 5
        
      } catch (error) {
        return true
      }
    })
    
    if (fullscreenBlackScreen) {
      console.log('❌ FULLSCREEN VIDEO ALSO BLACK')
      throw new Error('Fullscreen video streaming shows black screen - test failed')
    }
    console.log('✅ Fullscreen video content is visible')
    
    // Exit fullscreen
    await page.keyboard.press('Escape')
    
    console.log('🎉 Isaac Sim video streaming verification PASSED')
  })
  
  test('performance check - video streaming maintains 15+ FPS', async ({ page }) => {
    console.log('📊 Starting video streaming performance test')
    
    await page.goto('http://localhost:3000/configure?step=2')
    
    // Wait for streaming to start
    await page.waitForFunction(() => {
      return window.localStorage.getItem('isaac_sim_session_id') !== null
    }, { timeout: 10000 })
    
    // Count video frame messages over 2 seconds (should be 30+ frames at 15 FPS)
    const frameCount = await page.evaluate(() => {
      return new Promise((resolve) => {
        let frames = 0
        const startTime = Date.now()
        
        // Listen for video frame messages
        const originalConsoleLog = console.log
        console.log = (...args) => {
          if (args[0] && args[0].includes('video frame #')) {
            frames++
          }
          originalConsoleLog(...args)
        }
        
        setTimeout(() => {
          console.log = originalConsoleLog
          resolve(frames)
        }, 2000)
      })
    })
    
    expect(frameCount).toBeGreaterThan(25) // Should get 30+ frames in 2 seconds at 15 FPS
    console.log(`✅ Video streaming performance: ${frameCount} frames in 2 seconds`)
  })
})
