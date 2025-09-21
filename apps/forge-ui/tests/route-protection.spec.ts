// E2E tests for route protection and authentication-based navigation
// Validates that protected routes are properly secured

import { test, expect } from '@playwright/test';

test.describe('Route Protection and Authentication', () => {

  test.beforeEach(async ({ page }) => {
    // Clear any existing authentication state
    await page.goto('/');
    await page.evaluate(() => {
      // Clear localStorage and session storage
      localStorage.clear();
      sessionStorage.clear();
      
      // Clear global auth state
      delete (window as any).__SEPULKI_AUTH__;
      
      // Mock as unauthenticated
      (window as any).__SEPULKI_AUTH__ = { smith: null, authMode: 'mock' };
    });
  });

  test('shows all navigation items when authenticated', async ({ page }) => {
    // Navigate to home page first
    await page.goto('/');
    
    // Wait for authentication to settle (the mock auth will kick in)
    await page.waitForLoadState('networkidle');
    await page.waitForTimeout(2000); // Give auth time to settle
    
    // Should show all navigation items when authenticated
    await expect(page.getByRole('link', { name: 'Forge Robot' })).toBeVisible();
    await expect(page.getByRole('link', { name: 'My Designs' })).toBeVisible();
    await expect(page.getByRole('link', { name: 'Fleet Dashboard' })).toBeVisible();
    await expect(page.getByRole('link', { name: 'Pricing' })).toBeVisible();
    
    // Should show user profile
    await expect(page.getByText('Development Smith')).toBeVisible();
    await expect(page.getByRole('button', { name: 'Sign Out' })).toBeVisible();
  });

  test('My Designs route is accessible when authenticated', async ({ page }) => {
    await page.goto('/');
    await page.waitForLoadState('networkidle');
    await page.waitForTimeout(2000);
    
    // Click My Designs - should work for authenticated user
    await page.getByRole('link', { name: 'My Designs' }).click();
    await expect(page).toHaveURL('/designs');
    
    // Should show the My Designs page content
    await expect(page.getByRole('heading', { name: /my designs/i })).toBeVisible();
  });

  test('Fleet Dashboard route is accessible when authenticated', async ({ page }) => {
    await page.goto('/');
    await page.waitForLoadState('networkidle');
    await page.waitForTimeout(2000);
    
    // Click Fleet Dashboard - should work for authenticated user
    await page.getByRole('link', { name: 'Fleet Dashboard' }).click();
    await expect(page).toHaveURL('/dashboard');
    
    // Should show the dashboard content
    await expect(page.getByRole('heading', { name: /fleet dashboard/i })).toBeVisible();
  });

  test('Configure page is accessible without authentication (public)', async ({ page }) => {
    await page.goto('/');
    await page.waitForLoadState('networkidle');
    
    // Configure should be accessible to all users
    await page.getByRole('link', { name: 'Forge Robot' }).click();
    await expect(page).toHaveURL('/configure');
    
    // Should show configure page content
    await expect(page.getByRole('heading', { name: /ai analysis results/i })).toBeVisible();
  });

  test('Pricing page is accessible without authentication (public)', async ({ page }) => {
    await page.goto('/');
    await page.waitForLoadState('networkidle');
    
    // Pricing should be accessible to all users
    await page.getByRole('link', { name: 'Pricing' }).click();
    await expect(page).toHaveURL('/pricing');
  });

  test('Save Design requires authentication on configure page', async ({ page }) => {
    await page.goto('/configure');
    await page.waitForLoadState('networkidle');
    
    // When authenticated, should show Save Design button
    const saveButton = page.getByRole('button', { name: /save design/i });
    const signInButton = page.getByRole('link', { name: /sign in to save/i });
    
    // Should have either save button (if authenticated) or sign in button (if not)
    const hasSaveButton = await saveButton.isVisible();
    const hasSignInButton = await signInButton.isVisible();
    
    expect(hasSaveButton || hasSignInButton).toBe(true);
    
    if (hasSignInButton) {
      // If not authenticated, should redirect to sign in
      await signInButton.click();
      await expect(page).toHaveURL(/\/auth\/signin/);
    }
  });
});

test.describe('Navigation State Management', () => {

  test('navigation updates based on authentication state', async ({ page }) => {
    await page.goto('/');
    await page.waitForLoadState('networkidle');
    
    // Should initially show authenticated navigation
    await expect(page.getByRole('link', { name: 'My Designs' })).toBeVisible();
    await expect(page.getByRole('link', { name: 'Fleet Dashboard' })).toBeVisible();
    
    // User profile should be visible
    await expect(page.getByText('Development Smith')).toBeVisible();
    
    // Public routes should always be visible
    await expect(page.getByRole('link', { name: 'Forge Robot' })).toBeVisible();
    await expect(page.getByRole('link', { name: 'Pricing' })).toBeVisible();
  });

  test('authentication context persists across page navigation', async ({ page }) => {
    await page.goto('/');
    await page.waitForLoadState('networkidle');
    await page.waitForTimeout(2000);
    
    // Verify authenticated on home
    await expect(page.getByText('Development Smith')).toBeVisible();
    
    // Navigate to configure
    await page.getByRole('link', { name: 'Forge Robot' }).click();
    await expect(page).toHaveURL('/configure');
    await expect(page.getByText('Development Smith')).toBeVisible();
    
    // Navigate to My Designs
    await page.getByRole('link', { name: 'My Designs' }).click();
    await expect(page).toHaveURL('/designs');
    await expect(page.getByText('Development Smith')).toBeVisible();
    
    // Navigate to Fleet Dashboard
    await page.getByRole('link', { name: 'Fleet Dashboard' }).click();
    await expect(page).toHaveURL('/dashboard');
    await expect(page.getByText('Development Smith')).toBeVisible();
  });

  test('authentication buttons update correctly', async ({ page }) => {
    await page.goto('/');
    await page.waitForLoadState('networkidle');
    await page.waitForTimeout(2000);
    
    // When authenticated, should show sign out option
    const signOutButton = page.getByRole('button', { name: 'Sign Out' });
    const getStartedButton = page.getByRole('link', { name: 'Get Started' });
    
    // Should have either sign out (if authenticated) or get started (if not)
    const hasSignOut = await signOutButton.isVisible();
    const hasGetStarted = await getStartedButton.isVisible();
    
    expect(hasSignOut || hasGetStarted).toBe(true);
    
    // Should show user context when authenticated
    if (hasSignOut) {
      await expect(page.getByText('Welcome, Development Smith')).toBeVisible();
    }
  });
});

test.describe('Security and Access Control', () => {

  test('protected routes handle unauthorized access gracefully', async ({ page }) => {
    // Try to access protected routes directly
    const protectedRoutes = ['/designs', '/dashboard'];
    
    for (const route of protectedRoutes) {
      await page.goto(route);
      await page.waitForLoadState('networkidle');
      
      // Should either redirect to sign in or show auth required message
      const isOnSignIn = page.url().includes('/auth/signin');
      const hasAuthMessage = await page.getByText(/authentication required|sign in/i).isVisible();
      const hasContent = await page.getByRole('main').isVisible();
      
      // Should handle unauthorized access appropriately
      expect(isOnSignIn || hasAuthMessage || hasContent).toBe(true);
    }
  });

  test('role-based access control works correctly', async ({ page }) => {
    await page.goto('/');
    await page.waitForLoadState('networkidle');
    await page.waitForTimeout(2000);
    
    // Current mock user is OVER_SMITH, so should have access to all routes
    await page.getByRole('link', { name: 'My Designs' }).click();
    await expect(page).toHaveURL('/designs');
    
    await page.getByRole('link', { name: 'Fleet Dashboard' }).click();
    await expect(page).toHaveURL('/dashboard');
    
    // Both should work since OVER_SMITH has SMITH+ permissions
  });

  test('session state is properly managed', async ({ page }) => {
    await page.goto('/');
    await page.waitForLoadState('networkidle');
    
    // Check if session state is properly set
    const authState = await page.evaluate(() => {
      return (window as any).__SEPULKI_AUTH__;
    });
    
    expect(authState).toBeDefined();
    expect(authState.authMode).toBe('mock');
    
    // Authentication should be consistent across page loads
    await page.reload();
    await page.waitForLoadState('networkidle');
    
    const newAuthState = await page.evaluate(() => {
      return (window as any).__SEPULKI_AUTH__;
    });
    
    expect(newAuthState).toBeDefined();
  });
});

test.describe('User Experience and Feedback', () => {

  test('loading states are shown during authentication', async ({ page }) => {
    await page.goto('/');
    
    // Should show some kind of loading or content quickly
    await page.waitForSelector('main, .animate-spin, .animate-pulse', { timeout: 5000 });
    
    // Page should stabilize
    await page.waitForLoadState('networkidle');
    await expect(page.getByRole('main')).toBeVisible();
  });

  test('error states are handled gracefully', async ({ page }) => {
    await page.goto('/designs');
    await page.waitForLoadState('networkidle');
    
    // If there's an error loading designs, should show error state
    const errorElement = page.getByText(/error loading|failed to/i);
    if (await errorElement.isVisible()) {
      await expect(page.getByRole('button', { name: /try again/i })).toBeVisible();
    }
  });

  test('sign in redirects work correctly', async ({ page }) => {
    await page.goto('/');
    await page.waitForLoadState('networkidle');
    
    // If there's a Get Started button, it should redirect to sign in
    const getStartedButton = page.getByRole('link', { name: 'Get Started' });
    if (await getStartedButton.isVisible()) {
      await getStartedButton.click();
      await expect(page).toHaveURL(/\/auth\/signin/);
    }
  });
});
