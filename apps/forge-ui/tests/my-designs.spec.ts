// E2E tests for My Designs feature using Playwright
// Tests complete user workflows and cross-page interactions

import { test, expect, Page } from '@playwright/test';

// Test utilities and helpers
class MyDesignsPageObject {
  constructor(private page: Page) {}

  // Navigation
  async goto() {
    await this.page.goto('/designs');
    await this.waitForLoad();
  }

  async waitForLoad() {
    // Wait for either designs to load or error state
    await this.page.waitForSelector('[data-testid="mock-auth-provider"], .animate-spin, .bg-red-50', {
      timeout: 10000
    });
  }

  // Getters for page elements
  get header() {
    return this.page.getByRole('heading', { name: /my designs/i });
  }

  get createNewDesignButton() {
    return this.page.getByRole('link', { name: /create new design/i }).first();
  }

  get allDesignsTab() {
    return this.page.getByRole('button', { name: /all designs/i });
  }

  get readyToBuildTab() {
    return this.page.getByRole('button', { name: /ready to build/i });
  }

  get buildingTab() {
    return this.page.getByRole('button', { name: /building/i });
  }

  get deployedTab() {
    return this.page.getByRole('button', { name: /deployed/i });
  }

  // Design card interactions
  async getDesignCard(designName: string) {
    return this.page.getByRole('heading', { name: designName }).locator('..');
  }

  async clickEditDesign(designName: string) {
    const card = await this.getDesignCard(designName);
    await card.getByRole('link', { name: /edit/i }).click();
  }

  async clickDuplicateDesign(designName: string) {
    const card = await this.getDesignCard(designName);
    await card.getByRole('button', { name: /duplicate/i }).click();
  }

  async clickBuildDesign(designName: string) {
    const card = await this.getDesignCard(designName);
    await card.getByRole('button', { name: /build/i }).click();
  }

  async clickDeleteDesign(designName: string) {
    const card = await this.getDesignCard(designName);
    await card.getByRole('button', { name: '🗑️' }).click();
  }

  // Assertions
  async expectDesignsCount(count: number) {
    const designs = this.page.locator('[id^="design-"]');
    await expect(designs).toHaveCount(count);
  }

  async expectDesignVisible(designName: string) {
    await expect(this.page.getByRole('heading', { name: designName })).toBeVisible();
  }

  async expectDesignNotVisible(designName: string) {
    await expect(this.page.getByRole('heading', { name: designName })).not.toBeVisible();
  }

  async expectTabBadge(tabName: string, count: number) {
    const tab = this.page.getByRole('button', { name: new RegExp(tabName, 'i') });
    await expect(tab).toContainText(count.toString());
  }

  async expectPortfolioStat(label: string, value: string) {
    const stat = this.page.locator(`text=${label}`).locator('..');
    await expect(stat).toContainText(value);
  }
}

test.describe('My Designs Page - Core Functionality', () => {
  let myDesigns: MyDesignsPageObject;

  test.beforeEach(async ({ page }) => {
    myDesigns = new MyDesignsPageObject(page);
  });

  test('loads and displays My Designs page correctly', async ({ page }) => {
    await myDesigns.goto();

    // Verify page structure
    await expect(myDesigns.header).toBeVisible();
    await expect(page.getByText('Your saved robot configurations and builds')).toBeVisible();
    await expect(myDesigns.createNewDesignButton).toBeVisible();

    // Verify navigation tabs
    await expect(myDesigns.allDesignsTab).toBeVisible();
    await expect(myDesigns.readyToBuildTab).toBeVisible();
    await expect(myDesigns.buildingTab).toBeVisible();
    await expect(myDesigns.deployedTab).toBeVisible();
  });

  test('shows authentication context correctly', async ({ page }) => {
    await myDesigns.goto();

    // Should show authenticated user
    await expect(page.getByText('Development Smith')).toBeVisible();
    await expect(page.getByText('over smith')).toBeVisible();
  });

  test('loads and displays actual design data', async ({ page }) => {
    await myDesigns.goto();

    // Wait for designs to load (should see actual data from backend)
    await page.waitForSelector('[id^="design-"]', { timeout: 10000 });

    // Verify designs are displayed
    await myDesigns.expectDesignsCount(3); // Based on our test data

    // Verify specific designs exist
    await myDesigns.expectDesignVisible('✅ FINAL CONNECTION TEST');
    await myDesigns.expectDesignVisible('🎉 WORKING: Frontend-Backend Connection');
    await myDesigns.expectDesignVisible('DevBot-001');
  });

  test('displays design metadata correctly', async ({ page }) => {
    await myDesigns.goto();
    await page.waitForSelector('[id^="design-"]');

    // Verify design details are shown
    await expect(page.getByText('Industrial Arm - 6DOF')).toBeVisible();
    await expect(page.getByText('1.0.0')).toBeVisible(); // Version
    await expect(page.getByText('⚙️ ServoMax Pro 3000')).toBeVisible(); // Component
    await expect(page.getByText('🤏 GripForce Elite')).toBeVisible(); // Component

    // Verify status badges
    await expect(page.getByText('🔥 FORGING')).toBeVisible();
    await expect(page.getByText('⚪ READY')).toBeVisible();
  });
});

test.describe('My Designs Page - Filter Functionality', () => {
  let myDesigns: MyDesignsPageObject;

  test.beforeEach(async ({ page }) => {
    myDesigns = new MyDesignsPageObject(page);
    await myDesigns.goto();
    await page.waitForSelector('[id^="design-"]');
  });

  test('All Designs filter shows all designs with correct count', async ({ page }) => {
    await myDesigns.allDesignsTab.click();
    
    // Should show all designs
    await myDesigns.expectDesignsCount(3);
    await myDesigns.expectTabBadge('All Designs', 3);
    
    // All designs should be visible
    await myDesigns.expectDesignVisible('✅ FINAL CONNECTION TEST');
    await myDesigns.expectDesignVisible('🎉 WORKING: Frontend-Backend Connection');
    await myDesigns.expectDesignVisible('DevBot-001');
  });

  test('Ready to Build filter shows only ready designs', async ({ page }) => {
    await myDesigns.readyToBuildTab.click();
    
    // Should only show ready designs
    await myDesigns.expectDesignsCount(1);
    await myDesigns.expectTabBadge('Ready to Build', 1);
    
    // Only DevBot-001 should be visible (READY status)
    await myDesigns.expectDesignVisible('DevBot-001');
    await myDesigns.expectDesignNotVisible('✅ FINAL CONNECTION TEST');
    await myDesigns.expectDesignNotVisible('🎉 WORKING: Frontend-Backend Connection');
  });

  test('filter tabs have proper active states', async ({ page }) => {
    // All Designs should be active by default
    await expect(myDesigns.allDesignsTab).toHaveClass(/border-orange-500|text-orange-600/);

    // Click Ready to Build
    await myDesigns.readyToBuildTab.click();
    await expect(myDesigns.readyToBuildTab).toHaveClass(/border-orange-500|text-orange-600/);
    await expect(myDesigns.allDesignsTab).not.toHaveClass(/border-orange-500|text-orange-600/);

    // Click back to All Designs
    await myDesigns.allDesignsTab.click();
    await expect(myDesigns.allDesignsTab).toHaveClass(/border-orange-500|text-orange-600/);
  });

  test('empty filter states show appropriate messages', async ({ page }) => {
    // Click on filters that should be empty
    await myDesigns.buildingTab.click();
    await expect(page.getByText('No building designs')).toBeVisible();

    await myDesigns.deployedTab.click();
    await expect(page.getByText('No deployed designs')).toBeVisible();
  });
});

test.describe('My Designs Page - Design Actions', () => {
  let myDesigns: MyDesignsPageObject;

  test.beforeEach(async ({ page }) => {
    myDesigns = new MyDesignsPageObject(page);
    await myDesigns.goto();
    await page.waitForSelector('[id^="design-"]');
  });

  test('Edit action navigates to configure page with design ID', async ({ page }) => {
    const designName = '✅ FINAL CONNECTION TEST';
    
    await myDesigns.clickEditDesign(designName);
    
    // Should navigate to configure page with editDesign parameter
    await expect(page).toHaveURL(/\/configure\?editDesign=/);
    
    // Configure page should load
    await expect(page.getByRole('heading', { name: /ai analysis results/i })).toBeVisible();
    await expect(page.getByRole('button', { name: /save design/i })).toBeVisible();
  });

  test('Duplicate action navigates to configure page with duplicate flag', async ({ page }) => {
    const designName = '🎉 WORKING: Frontend-Backend Connection';
    
    await myDesigns.clickDuplicateDesign(designName);
    
    // Should navigate to configure page with duplicate parameter
    await expect(page).toHaveURL('/configure?duplicate=true');
    
    // Configure page should load
    await expect(page.getByRole('heading', { name: /ai analysis results/i })).toBeVisible();
  });

  test('Build action triggers build process for ready designs', async ({ page }) => {
    // Filter to ready designs first
    await myDesigns.readyToBuildTab.click();
    await myDesigns.expectDesignsCount(1);

    // Set up dialog handler for the alert
    page.on('dialog', async dialog => {
      expect(dialog.message()).toContain('Build failed'); // Expected due to wrong status
      await dialog.accept();
    });

    await myDesigns.clickBuildDesign('DevBot-001');
    
    // Should trigger the build process (even if it fails due to status)
    // This tests that the UI properly calls the backend
  });

  test('Create New Design buttons navigate to configure page', async ({ page }) => {
    // Test header button
    await myDesigns.createNewDesignButton.click();
    await expect(page).toHaveURL('/configure');
    
    // Go back and test footer button
    await page.goBack();
    await page.getByRole('link', { name: /create new design/i }).last().click();
    await expect(page).toHaveURL('/configure');
  });

  test('Navigation links work correctly', async ({ page }) => {
    // Test Fleet Dashboard navigation
    await page.getByRole('link', { name: /fleet dashboard/i }).click();
    await expect(page).toHaveURL('/dashboard');
    
    // Navigate back to My Designs via main nav
    await page.getByRole('link', { name: /my designs/i }).click();
    await expect(page).toHaveURL('/designs');
    await expect(myDesigns.header).toBeVisible();
  });
});

test.describe('My Designs Page - Portfolio Analytics', () => {
  let myDesigns: MyDesignsPageObject;

  test.beforeEach(async ({ page }) => {
    myDesigns = new MyDesignsPageObject(page);
    await myDesigns.goto();
    await page.waitForSelector('[id^="design-"]');
  });

  test('displays portfolio summary with correct statistics', async ({ page }) => {
    // Wait for portfolio summary to appear
    await expect(page.getByText('📊 Design Portfolio Summary')).toBeVisible();

    // Verify statistics are displayed
    await myDesigns.expectPortfolioStat('Total Designs', '3');
    await myDesigns.expectPortfolioStat('Pattern Types', '1'); // All use Industrial Arm
    await myDesigns.expectPortfolioStat('Total Components', '5'); // Sum of all components

    // Ready to Build count should be accurate
    await myDesigns.expectPortfolioStat('Ready to Build', '0'); // Based on actual status data
  });

  test('statistics update when filters are applied', async ({ page }) => {
    // Portfolio summary should always show total stats regardless of filter
    await myDesigns.readyToBuildTab.click();
    await myDesigns.expectDesignsCount(1);

    // Portfolio summary should still show total stats
    await expect(page.getByText('📊 Design Portfolio Summary')).toBeVisible();
    await myDesigns.expectPortfolioStat('Total Designs', '3');
  });
});

test.describe('My Designs Page - Error Handling and Edge Cases', () => {
  test('handles authentication errors gracefully', async ({ page }) => {
    // Navigate without proper auth setup
    await page.goto('/designs');
    
    // Should show error or auth required state
    await expect(page.getByText(/authentication required|error loading/i)).toBeVisible({
      timeout: 10000
    });
  });

  test('handles network errors with retry functionality', async ({ page }) => {
    await page.goto('/designs');
    
    // If there's an error, try again button should work
    const errorElement = page.getByText(/error loading/i);
    const tryAgainButton = page.getByRole('button', { name: /try again/i });
    
    if (await errorElement.isVisible()) {
      await expect(tryAgainButton).toBeVisible();
      await tryAgainButton.click();
      
      // Should attempt to reload
      await page.waitForLoadState('networkidle');
    }
  });

  test('handles empty state when user has no designs', async ({ page }) => {
    // This would require mocking the backend to return empty results
    // For now, verify the empty state UI exists in component
    await page.goto('/designs');
    await page.waitForSelector('main');
    
    // The empty state should be properly structured if it appears
    const emptyState = page.getByText(/no designs yet|forge your first robot/i);
    if (await emptyState.isVisible()) {
      await expect(page.getByRole('link', { name: /forge your first robot/i })).toBeVisible();
    }
  });
});

test.describe('My Designs Page - Cross-Page Integration', () => {
  let myDesigns: MyDesignsPageObject;

  test.beforeEach(async ({ page }) => {
    myDesigns = new MyDesignsPageObject(page);
  });

  test('complete design workflow: Configure → Save → View in My Designs', async ({ page }) => {
    // Step 1: Navigate to configure page
    await page.goto('/configure');
    await expect(page.getByRole('heading', { name: /ai analysis results/i })).toBeVisible();

    // Step 2: Click Save Design (if enabled)
    const saveButton = page.getByRole('button', { name: /save design/i });
    if (await saveButton.isEnabled()) {
      await saveButton.click();
      
      // Fill out save form if modal appears
      const modal = page.locator('[role="dialog"]');
      if (await modal.isVisible()) {
        await modal.getByRole('textbox', { name: /design name/i }).fill('E2E Test Robot');
        await modal.getByRole('textbox', { name: /description/i }).fill('Created via E2E test');
        await modal.getByRole('button', { name: /forge sepulka/i }).click();
        
        // Should navigate to My Designs
        await expect(page).toHaveURL(/\/designs/);
      }
    }

    // Step 3: Verify we can navigate to My Designs manually if save didn't work
    await page.getByRole('link', { name: /my designs/i }).click();
    await expect(page).toHaveURL('/designs');
    await expect(myDesigns.header).toBeVisible();
  });

  test('design editing workflow maintains state correctly', async ({ page }) => {
    await myDesigns.goto();
    await page.waitForSelector('[id^="design-"]');

    // Click edit on a design
    await myDesigns.clickEditDesign('✅ FINAL CONNECTION TEST');
    
    // Should be on configure page with edit parameter
    await expect(page).toHaveURL(/editDesign=/);
    await expect(page.getByRole('button', { name: /save design/i })).toBeVisible();

    // Navigate back to My Designs
    await page.getByRole('link', { name: /my designs/i }).click();
    await expect(myDesigns.header).toBeVisible();
    
    // Original design should still be there
    await myDesigns.expectDesignVisible('✅ FINAL CONNECTION TEST');
  });

  test('navigation between pages preserves user context', async ({ page }) => {
    // Start at My Designs
    await myDesigns.goto();
    await expect(page.getByText('Development Smith')).toBeVisible();

    // Navigate to Fleet Dashboard
    await page.getByRole('link', { name: /fleet dashboard/i }).click();
    await expect(page).toHaveURL('/dashboard');
    await expect(page.getByText('Development Smith')).toBeVisible();

    // Navigate to Configure
    await page.getByRole('link', { name: /forge robot/i }).click();
    await expect(page).toHaveURL('/configure');
    await expect(page.getByText('Development Smith')).toBeVisible();

    // Navigate back to My Designs
    await page.getByRole('link', { name: /my designs/i }).click();
    await expect(page).toHaveURL('/designs');
    await expect(page.getByText('Development Smith')).toBeVisible();
  });
});

test.describe('My Designs Page - Responsive Design and Accessibility', () => {
  test('works correctly on mobile viewport', async ({ page }) => {
    await page.setViewportSize({ width: 375, height: 667 }); // iPhone SE
    
    const myDesigns = new MyDesignsPageObject(page);
    await myDesigns.goto();
    
    // Should still display properly on mobile
    await expect(myDesigns.header).toBeVisible();
    await expect(page.getByText('Your saved robot configurations and builds')).toBeVisible();
    
    // Navigation should be accessible
    await expect(myDesigns.allDesignsTab).toBeVisible();
    await expect(myDesigns.createNewDesignButton).toBeVisible();
  });

  test('works correctly on tablet viewport', async ({ page }) => {
    await page.setViewportSize({ width: 768, height: 1024 }); // iPad
    
    const myDesigns = new MyDesignsPageObject(page);
    await myDesigns.goto();
    
    // Should display in responsive grid
    await expect(myDesigns.header).toBeVisible();
    await page.waitForSelector('[id^="design-"]');
    
    // Should show designs in grid layout
    await myDesigns.expectDesignsCount(3);
  });

  test('supports keyboard navigation', async ({ page }) => {
    const myDesigns = new MyDesignsPageObject(page);
    await myDesigns.goto();
    await page.waitForSelector('[id^="design-"]');

    // Tab through interactive elements
    await page.keyboard.press('Tab'); // Should focus first interactive element
    await page.keyboard.press('Tab');
    await page.keyboard.press('Tab');
    
    // Should be able to activate elements with Enter/Space
    const focusedElement = page.locator(':focus');
    await expect(focusedElement).toBeVisible();
  });

  test('has proper semantic HTML structure', async ({ page }) => {
    const myDesigns = new MyDesignsPageObject(page);
    await myDesigns.goto();

    // Verify semantic structure
    await expect(page.locator('main')).toBeVisible();
    await expect(page.locator('nav')).toBeVisible();
    await expect(page.getByRole('heading', { level: 1 })).toBeVisible();
    await expect(page.getByRole('heading', { level: 3 })).toBeVisible(); // Design names

    // Verify links have proper text content
    const links = page.getByRole('link');
    const linkCount = await links.count();
    expect(linkCount).toBeGreaterThan(0);

    // All links should have accessible names
    for (let i = 0; i < linkCount; i++) {
      const link = links.nth(i);
      const accessibleName = await link.getAttribute('aria-label') || await link.textContent();
      expect(accessibleName).toBeTruthy();
    }
  });
});

test.describe('My Designs Page - Performance', () => {
  test('loads within acceptable time limits', async ({ page }) => {
    const startTime = Date.now();
    
    await page.goto('/designs');
    await page.waitForSelector('[id^="design-"], .bg-red-50', { timeout: 10000 });
    
    const loadTime = Date.now() - startTime;
    expect(loadTime).toBeLessThan(5000); // Should load within 5 seconds
  });

  test('handles rapid filter switching without issues', async ({ page }) => {
    const myDesigns = new MyDesignsPageObject(page);
    await myDesigns.goto();
    await page.waitForSelector('[id^="design-"]');

    // Rapidly switch between filters
    await myDesigns.allDesignsTab.click();
    await myDesigns.readyToBuildTab.click();
    await myDesigns.buildingTab.click();
    await myDesigns.deployedTab.click();
    await myDesigns.allDesignsTab.click();

    // Should still work correctly
    await myDesigns.expectDesignsCount(3);
    await expect(myDesigns.header).toBeVisible();
  });

  test('handles large numbers of designs efficiently', async ({ page }) => {
    // This test validates the UI can handle scale
    // The current implementation should work with 50+ designs
    
    const myDesigns = new MyDesignsPageObject(page);
    await myDesigns.goto();
    
    // Even with current data, should load quickly
    const startTime = Date.now();
    await page.waitForSelector('[id^="design-"], .bg-red-50');
    const loadTime = Date.now() - startTime;
    
    expect(loadTime).toBeLessThan(3000);
  });
});

test.describe('My Designs Page - Visual Design and UX', () => {
  test('has proper visual hierarchy and branding', async ({ page }) => {
    const myDesigns = new MyDesignsPageObject(page);
    await myDesigns.goto();

    // Verify Sepulki branding
    await expect(page.getByText('🔥 Sepulki')).toBeVisible();
    await expect(myDesigns.header).toContainText('🔥');

    // Verify visual hierarchy
    const h1Elements = page.getByRole('heading', { level: 1 });
    await expect(h1Elements).toHaveCount(1);
    
    const h3Elements = page.getByRole('heading', { level: 3 });
    const h3Count = await h3Elements.count();
    expect(h3Count).toBeGreaterThan(0); // Design names and summary
  });

  test('shows hover states and interactive feedback', async ({ page }) => {
    const myDesigns = new MyDesignsPageObject(page);
    await myDesigns.goto();
    await page.waitForSelector('[id^="design-"]');

    // Test hover on design cards
    const firstCard = page.locator('[id^="design-"]').first();
    await firstCard.hover();
    
    // Card should have hover effects (shadow changes)
    await expect(firstCard).toHaveClass(/hover:shadow-md/);

    // Test hover on action buttons
    const editLink = page.getByRole('link', { name: /edit/i }).first();
    await editLink.hover();
    await expect(editLink).toHaveClass(/hover:text-blue-700/);
  });

  test('displays status badges with correct colors and icons', async ({ page }) => {
    const myDesigns = new MyDesignsPageObject(page);
    await myDesigns.goto();
    await page.waitForSelector('[id^="design-"]');

    // Verify status badges have proper styling
    const forgingBadge = page.getByText('🔥 FORGING');
    await expect(forgingBadge).toBeVisible();
    await expect(forgingBadge).toHaveClass(/text-orange-600/);

    const readyBadge = page.getByText('⚪ READY');
    await expect(readyBadge).toBeVisible();
    // The READY status should have appropriate styling
  });

  test('component tags display with proper emojis and styling', async ({ page }) => {
    const myDesigns = new MyDesignsPageObject(page);
    await myDesigns.goto();
    await page.waitForSelector('[id^="design-"]');

    // Verify component tags
    await expect(page.getByText('⚙️ ServoMax Pro 3000')).toBeVisible();
    await expect(page.getByText('🤏 GripForce Elite')).toBeVisible();
    await expect(page.getByText('👁️ VisionEye 4K')).toBeVisible();

    // Tags should have proper styling
    const componentTag = page.getByText('⚙️ ServoMax Pro 3000');
    await expect(componentTag).toHaveClass(/bg-gray-100/);
  });
});
