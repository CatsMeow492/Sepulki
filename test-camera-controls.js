#!/usr/bin/env node

const puppeteer = require('puppeteer');

async function testCameraControls() {
  console.log('🎬 Testing Isaac Sim Camera Controls...');
  
  const browser = await puppeteer.launch({ 
    headless: false,
    args: ['--no-sandbox', '--disable-setuid-sandbox']
  });
  
  try {
    const page = await browser.newPage();
    
    // Navigate to configure page
    console.log('📱 Navigating to configure page...');
    await page.goto('http://localhost:3000/configure?step=2', { 
      waitUntil: 'networkidle2',
      timeout: 30000 
    });
    
    // Wait for Isaac Sim display to load
    console.log('⏳ Waiting for Isaac Sim display...');
    await page.waitForSelector('[data-testid="isaac-sim-display"]', { timeout: 15000 });
    console.log('✅ Isaac Sim display loaded');
    
    // Look for camera control toggle button
    console.log('🔍 Looking for camera control toggle...');
    const cameraToggle = await page.$('button[title="Toggle camera controls"]');
    
    if (!cameraToggle) {
      console.log('❌ Camera toggle button not found');
      return;
    }
    
    console.log('✅ Camera toggle button found');
    
    // Click the camera toggle
    console.log('🖱️ Clicking camera toggle...');
    await cameraToggle.click();
    
    // Wait a moment for the controls to appear
    await page.waitForTimeout(1000);
    
    // Check if camera controls are visible
    console.log('👀 Checking if camera controls are visible...');
    const cameraControls = await page.$('.absolute.top-20.right-4.z-30');
    
    if (cameraControls) {
      const isVisible = await cameraControls.isVisible();
      if (isVisible) {
        console.log('✅ Camera controls are visible!');
        
        // Take a screenshot
        await page.screenshot({ 
          path: 'camera-controls-test.png',
          fullPage: true 
        });
        console.log('📸 Screenshot saved as camera-controls-test.png');
        
        // Check for specific control elements
        const positionControls = await page.$('input[type="range"]');
        if (positionControls) {
          console.log('✅ Camera position controls found');
        }
        
        const presetButtons = await page.$('button:has-text("Front")');
        if (presetButtons) {
          console.log('✅ Camera preset buttons found');
        }
        
      } else {
        console.log('❌ Camera controls are not visible');
      }
    } else {
      console.log('❌ Camera controls element not found');
    }
    
    // Test clicking a camera preset
    console.log('🎯 Testing camera preset...');
    const frontButton = await page.$('button:has-text("Front")');
    if (frontButton) {
      await frontButton.click();
      console.log('✅ Front preset clicked');
    }
    
  } catch (error) {
    console.error('❌ Test failed:', error.message);
  } finally {
    await browser.close();
  }
}

// Run the test
testCameraControls().catch(console.error);
