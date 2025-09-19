import { test, expect } from '@playwright/test'

test.describe('Configure viewer smoke', () => {
  test('shows canvas on configure page', async ({ page }) => {
    await page.goto('/configure')
    await expect(page.locator('canvas')).toBeVisible()
  })
})


