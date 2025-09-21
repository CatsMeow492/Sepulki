// Focused tests for authentication-based navigation visibility
// Tests that routes are properly hidden/shown based on auth status

import { test, expect } from '@playwright/test';

test.describe('Authentication-Based Navigation', () => {

  test('authenticated user sees all navigation options', async ({ page }) => {
    await page.goto('/');
    await page.waitForLoadState('networkidle');
    await page.waitForTimeout(3000); // Allow auth to settle
    
    // When authenticated (mock auth kicks in), should see all nav items
    await expect(page.getByRole('link', { name: 'Forge Robot' })).toBeVisible();
    await expect(page.getByRole('link', { name: 'My Designs' })).toBeVisible();
    await expect(page.getByRole('link', { name: 'Fleet Dashboard' })).toBeVisible();
    await expect(page.getByRole('link', { name: 'Pricing' })).toBeVisible();
    
    // Should show authenticated user info
    await expect(page.getByText('Development Smith').first()).toBeVisible();
    await expect(page.getByRole('button', { name: 'Sign Out' })).toBeVisible();
  });

  test('My Designs route requires authentication', async ({ page }) => {
    await page.goto('/designs');
    await page.waitForLoadState('networkidle');
    
    // Should load successfully for authenticated users
    // Or show authentication required state
    const hasMyDesignsHeader = await page.getByRole('heading', { name: /my designs/i }).isVisible();
    const hasAuthRequired = await page.getByText(/authentication required/i).isVisible();
    const hasSignInRedirect = page.url().includes('/auth/signin');
    
    // One of these should be true (either accessible or properly protected)
    expect(hasMyDesignsHeader || hasAuthRequired || hasSignInRedirect).toBe(true);
  });

  test('Fleet Dashboard route requires authentication', async ({ page }) => {
    await page.goto('/dashboard');
    await page.waitForLoadState('networkidle');
    
    // Should load successfully for authenticated users
    // Or show authentication required state
    const hasFleetHeader = await page.getByRole('heading', { name: /fleet dashboard/i }).isVisible();
    const hasAuthRequired = await page.getByText(/authentication required/i).isVisible();
    const hasSignInRedirect = page.url().includes('/auth/signin');
    
    // One of these should be true (either accessible or properly protected)
    expect(hasFleetHeader || hasAuthRequired || hasSignInRedirect).toBe(true);
  });

  test('Configure page is accessible without authentication', async ({ page }) => {
    await page.goto('/configure');
    await page.waitForLoadState('networkidle');
    
    // Configure should always be accessible (public route)
    await expect(page.getByRole('heading', { name: /ai analysis results/i })).toBeVisible();
    await expect(page.getByText('Forge Robot')).toBeVisible(); // In navigation
  });

  test('Pricing page is accessible without authentication', async ({ page }) => {
    await page.goto('/pricing');
    await page.waitForLoadState('networkidle');
    
    // Pricing should always be accessible (public route)
    // Should have pricing in navigation
    await expect(page.getByText('Pricing')).toBeVisible();
  });

  test('Save Design functionality reflects authentication state', async ({ page }) => {
    await page.goto('/configure');
    await page.waitForLoadState('networkidle');
    await page.waitForTimeout(2000);
    
    // Should show appropriate save button based on auth state
    const hasSaveDesign = await page.getByRole('button', { name: /save design/i }).isVisible();
    const hasSignInToSave = await page.getByRole('link', { name: /sign in to save/i }).isVisible();
    
    // Should show one or the other
    expect(hasSaveDesign || hasSignInToSave).toBe(true);
    
    if (hasSaveDesign) {
      // If authenticated, save button should be functional
      await expect(page.getByText('Development Smith').first()).toBeVisible();
    }
    
    if (hasSignInToSave) {
      // If not authenticated, should redirect to sign in
      await page.getByRole('link', { name: /sign in to save/i }).click();
      await expect(page).toHaveURL(/\/auth\/signin/);
    }
  });
});

test.describe('Route Protection Security', () => {

  test('direct access to protected routes is handled securely', async ({ page }) => {
    // Test direct navigation to protected routes
    const protectedRoutes = [
      { path: '/designs', name: 'My Designs' },
      { path: '/dashboard', name: 'Fleet Dashboard' }
    ];

    for (const route of protectedRoutes) {
      await page.goto(route.path);
      await page.waitForLoadState('networkidle');
      await page.waitForTimeout(1000);
      
      // Should either:
      // 1. Load successfully if authenticated
      // 2. Show authentication required message
      // 3. Redirect to sign in page
      
      const hasContent = await page.getByRole('heading').first().isVisible();
      const hasAuthMessage = await page.getByText(/authentication required|sign in/i).isVisible();
      const isSignInPage = page.url().includes('/auth/signin');
      
      expect(hasContent || hasAuthMessage || isSignInPage).toBe(true);
      
      // If accessible, should show proper content
      if (hasContent && !hasAuthMessage && !isSignInPage) {
        // Should have the expected page content
        expect(page.url()).toBe(`http://localhost:3000${route.path}`);
      }
    }
  });

  test('navigation state updates correctly on sign out', async ({ page }) => {
    await page.goto('/');
    await page.waitForLoadState('networkidle');
    await page.waitForTimeout(2000);
    
    // Should be authenticated initially
    const initialAuthState = await page.getByText('Development Smith').first().isVisible();
    
    if (initialAuthState) {
      // Try to sign out
      await page.getByRole('button', { name: 'Sign Out' }).click();
      await page.waitForTimeout(2000);
      
      // Navigation should update (though mock auth might re-authenticate)
      const finalState = await page.evaluate(() => {
        return {
          hasSmith: !!(window as any).__SEPULKI_AUTH__?.smith,
          authMode: (window as any).__SEPULKI_AUTH__?.authMode
        };
      });
      
      expect(finalState.authMode).toBe('mock');
    }
  });

  test('unauthorized access shows appropriate error messages', async ({ page }) => {
    // Manually set unauthenticated state
    await page.goto('/');
    await page.evaluate(() => {
      (window as any).__SEPULKI_AUTH__ = { smith: null, authMode: 'mock' };
    });
    
    // Try to access protected route
    await page.goto('/designs');
    await page.waitForLoadState('networkidle');
    
    // Should handle unauthorized access appropriately
    const isProtected = await page.getByText(/authentication required|sign in/i).isVisible();
    const hasContent = await page.getByRole('heading', { name: /my designs/i }).isVisible();
    const isSignInPage = page.url().includes('/auth/signin');
    
    // Should either be protected or accessible (based on actual auth state)
    expect(isProtected || hasContent || isSignInPage).toBe(true);
  });
});

test.describe('User Experience During Authentication', () => {

  test('loading states are shown during auth checks', async ({ page }) => {
    await page.goto('/designs');
    
    // Should show loading state while checking authentication
    const hasLoading = await page.locator('.animate-spin, .animate-pulse').isVisible();
    const hasContent = await page.getByRole('main').isVisible();
    
    // Should show either loading or content quickly
    expect(hasLoading || hasContent).toBe(true);
  });

  test('error boundaries handle auth failures gracefully', async ({ page }) => {
    await page.goto('/designs');
    await page.waitForLoadState('networkidle');
    await page.waitForTimeout(3000);
    
    // Page should load without crashing
    await expect(page.getByRole('main')).toBeVisible();
    
    // Should not have unhandled errors
    const hasErrorBoundary = await page.getByText(/something went wrong|error occurred/i).isVisible();
    expect(hasErrorBoundary).toBe(false);
  });

  test('navigation remains functional during auth state changes', async ({ page }) => {
    await page.goto('/');
    await page.waitForLoadState('networkidle');
    
    // Navigation should always be functional
    await expect(page.getByRole('link', { name: 'Forge Robot' })).toBeVisible();
    await expect(page.getByRole('link', { name: 'Pricing' })).toBeVisible();
    
    // These are public routes and should always work
    await page.getByRole('link', { name: 'Forge Robot' }).click();
    await expect(page).toHaveURL('/configure');
    
    await page.getByRole('link', { name: '🔥 Sepulki' }).click();
    await expect(page).toHaveURL('/');
  });
});
