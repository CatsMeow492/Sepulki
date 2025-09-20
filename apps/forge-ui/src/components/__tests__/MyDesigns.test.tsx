// Unit tests for My Designs page component
// Tests component rendering, data fetching, filtering, and user interactions

import React from 'react';
import { screen, waitFor, fireEvent } from '@testing-library/react';
import userEvent from '@testing-library/user-event';
import { 
  renderWithAuth, 
  mockGraphQLResponses, 
  createTestSepulka,
  setupTestEnvironment,
  cleanupTestEnvironment,
  mockSmith
} from '@/test-utils';

// Import the component to test
import MyDesignsPage from '@/app/designs/page';

// Mock the GraphQL functions
jest.mock('@/lib/graphql', () => ({
  getMySepulkas: jest.fn(),
  castIngot: jest.fn(),
  deleteSepulka: jest.fn(),
}));

import { getMySepulkas, castIngot, deleteSepulka } from '@/lib/graphql';

describe('My Designs Page', () => {
  let testEnv: ReturnType<typeof setupTestEnvironment>;

  beforeEach(() => {
    testEnv = setupTestEnvironment();
    jest.clearAllMocks();
  });

  afterEach(() => {
    cleanupTestEnvironment();
  });

  describe('Component Rendering', () => {
    test('renders page header and navigation correctly', async () => {
      (getMySepulkas as jest.Mock).mockResolvedValue([]);

      renderWithAuth(<MyDesignsPage />);

      expect(screen.getByRole('heading', { name: /my designs/i })).toBeInTheDocument();
      expect(screen.getByText('Your saved robot configurations and builds')).toBeInTheDocument();
      expect(screen.getByRole('link', { name: /create new design/i })).toBeInTheDocument();
    });

    test('shows authentication required when not logged in', async () => {
      renderWithAuth(<MyDesignsPage />, { smith: null });

      expect(screen.getByRole('heading', { name: /authentication required/i })).toBeInTheDocument();
      expect(screen.getByText('Please sign in to view your robot designs')).toBeInTheDocument();
      expect(screen.getByRole('link', { name: /sign in/i })).toBeInTheDocument();
    });

    test('shows loading state initially', async () => {
      (getMySepulkas as jest.Mock).mockImplementation(() => new Promise(() => {})); // Never resolves

      renderWithAuth(<MyDesignsPage />);

      expect(screen.getByText('Loading your designs...')).toBeInTheDocument();
      expect(screen.getByRole('status', { hidden: true })).toBeInTheDocument(); // Loading spinner
    });
  });

  describe('Data Fetching and Display', () => {
    test('fetches and displays user designs correctly', async () => {
      const testDesigns = [
        createTestSepulka({ 
          name: 'Test Robot 1', 
          status: 'FORGING',
          alloys: [{ name: 'Servo Motor', type: 'ACTUATOR' }]
        }),
        createTestSepulka({ 
          name: 'Test Robot 2', 
          status: 'READY',
          alloys: [
            { name: 'Servo Motor', type: 'ACTUATOR' },
            { name: 'Gripper', type: 'END_EFFECTOR' }
          ]
        })
      ];

      (getMySepulkas as jest.Mock).mockResolvedValue(testDesigns);

      renderWithAuth(<MyDesignsPage />);

      await waitFor(() => {
        expect(screen.getByText('Test Robot 1')).toBeInTheDocument();
        expect(screen.getByText('Test Robot 2')).toBeInTheDocument();
      });

      // Verify design details are displayed
      expect(screen.getByText('🔥 FORGING')).toBeInTheDocument();
      expect(screen.getByText('⚪ READY')).toBeInTheDocument();
      expect(screen.getByText('Industrial Arm - 6DOF')).toBeInTheDocument();
      expect(screen.getAllByText('1.0.0')).toHaveLength(2); // Version numbers
    });

    test('shows error state when data fetching fails', async () => {
      (getMySepulkas as jest.Mock).mockRejectedValue(new Error('Network error'));

      renderWithAuth(<MyDesignsPage />);

      await waitFor(() => {
        expect(screen.getByText('Error loading designs')).toBeInTheDocument();
        expect(screen.getByText('Network error')).toBeInTheDocument();
      });

      expect(screen.getByRole('button', { name: /try again/i })).toBeInTheDocument();
    });

    test('shows empty state when user has no designs', async () => {
      (getMySepulkas as jest.Mock).mockResolvedValue([]);

      renderWithAuth(<MyDesignsPage />);

      await waitFor(() => {
        expect(screen.getByText('No designs yet')).toBeInTheDocument();
        expect(screen.getByText('Start by creating your first robot design')).toBeInTheDocument();
      });

      expect(screen.getByRole('link', { name: /forge your first robot/i })).toBeInTheDocument();
    });
  });

  describe('Filter Functionality', () => {
    const testDesigns = [
      createTestSepulka({ name: 'Forging Robot', status: 'FORGING' }),
      createTestSepulka({ name: 'Ready Robot', status: 'READY' }),
      createTestSepulka({ name: 'Building Robot', status: 'CASTING' }),
      createTestSepulka({ name: 'Deployed Robot', status: 'DEPLOYED' }),
    ];

    beforeEach(() => {
      (getMySepulkas as jest.Mock).mockResolvedValue(testDesigns);
    });

    test('shows all designs by default', async () => {
      renderWithAuth(<MyDesignsPage />);

      await waitFor(() => {
        expect(screen.getByText('All Designs')).toBeInTheDocument();
        expect(screen.getByText('4')).toBeInTheDocument(); // Count badge
      });

      expect(screen.getByText('Forging Robot')).toBeInTheDocument();
      expect(screen.getByText('Ready Robot')).toBeInTheDocument();
      expect(screen.getByText('Building Robot')).toBeInTheDocument();
      expect(screen.getByText('Deployed Robot')).toBeInTheDocument();
    });

    test('filters to ready designs when Ready to Build tab is clicked', async () => {
      const user = userEvent.setup();
      renderWithAuth(<MyDesignsPage />);

      await waitFor(() => {
        expect(screen.getByText('Ready Robot')).toBeInTheDocument();
      });

      await user.click(screen.getByRole('button', { name: /ready to build/i }));

      // Only ready design should be visible
      expect(screen.getByText('Ready Robot')).toBeInTheDocument();
      expect(screen.queryByText('Forging Robot')).not.toBeInTheDocument();
      expect(screen.queryByText('Building Robot')).not.toBeInTheDocument();
      expect(screen.queryByText('Deployed Robot')).not.toBeInTheDocument();
    });

    test('shows empty state for filters with no results', async () => {
      const user = userEvent.setup();
      renderWithAuth(<MyDesignsPage />);

      await waitFor(() => {
        expect(screen.getByText('Ready Robot')).toBeInTheDocument();
      });

      await user.click(screen.getByRole('button', { name: /building/i }));

      expect(screen.getByText('No building designs')).toBeInTheDocument();
    });
  });

  describe('Design Actions', () => {
    const testDesign = createTestSepulka({ 
      name: 'Actionable Robot',
      status: 'READY' 
    });

    beforeEach(() => {
      (getMySepulkas as jest.Mock).mockResolvedValue([testDesign]);
    });

    test('build action calls castIngot mutation', async () => {
      (castIngot as jest.Mock).mockResolvedValue({
        ingot: { id: 'test-ingot-001' },
        errors: []
      });

      const user = userEvent.setup();
      renderWithAuth(<MyDesignsPage />);

      await waitFor(() => {
        expect(screen.getByText('Actionable Robot')).toBeInTheDocument();
      });

      const buildButton = screen.getByRole('button', { name: /build/i });
      await user.click(buildButton);

      await waitFor(() => {
        expect(castIngot).toHaveBeenCalledWith(testDesign.id);
      });
    });

    test('build action shows error when mutation fails', async () => {
      (castIngot as jest.Mock).mockResolvedValue({
        ingot: null,
        errors: [{ code: 'VALIDATION_ERROR', message: 'Invalid status' }]
      });

      // Mock window.alert
      const alertSpy = jest.spyOn(window, 'alert').mockImplementation(() => {});

      const user = userEvent.setup();
      renderWithAuth(<MyDesignsPage />);

      await waitFor(() => {
        expect(screen.getByText('Actionable Robot')).toBeInTheDocument();
      });

      const buildButton = screen.getByRole('button', { name: /build/i });
      await user.click(buildButton);

      await waitFor(() => {
        expect(alertSpy).toHaveBeenCalledWith('Build failed: Invalid status');
      });

      alertSpy.mockRestore();
    });

    test('delete action calls deleteSepulka mutation with confirmation', async () => {
      (deleteSepulka as jest.Mock).mockResolvedValue({ success: true });

      // Mock window.confirm
      const confirmSpy = jest.spyOn(window, 'confirm').mockReturnValue(true);
      const alertSpy = jest.spyOn(window, 'alert').mockImplementation(() => {});

      const user = userEvent.setup();
      renderWithAuth(<MyDesignsPage />);

      await waitFor(() => {
        expect(screen.getByText('Actionable Robot')).toBeInTheDocument();
      });

      const deleteButton = screen.getByRole('button', { name: '🗑️' });
      await user.click(deleteButton);

      await waitFor(() => {
        expect(confirmSpy).toHaveBeenCalledWith(
          expect.stringContaining('Are you sure you want to delete "Actionable Robot"')
        );
        expect(deleteSepulka).toHaveBeenCalledWith(testDesign.id);
        expect(alertSpy).toHaveBeenCalledWith(
          expect.stringContaining('"Actionable Robot" has been deleted successfully')
        );
      });

      confirmSpy.mockRestore();
      alertSpy.mockRestore();
    });

    test('delete action cancelled when user rejects confirmation', async () => {
      // Mock window.confirm to return false
      const confirmSpy = jest.spyOn(window, 'confirm').mockReturnValue(false);

      const user = userEvent.setup();
      renderWithAuth(<MyDesignsPage />);

      await waitFor(() => {
        expect(screen.getByText('Actionable Robot')).toBeInTheDocument();
      });

      const deleteButton = screen.getByRole('button', { name: '🗑️' });
      await user.click(deleteButton);

      expect(confirmSpy).toHaveBeenCalled();
      expect(deleteSepulka).not.toHaveBeenCalled();

      confirmSpy.mockRestore();
    });

    test('duplicate action sets localStorage and navigates', async () => {
      const user = userEvent.setup();
      renderWithAuth(<MyDesignsPage />);

      await waitFor(() => {
        expect(screen.getByText('Actionable Robot')).toBeInTheDocument();
      });

      // Mock window.location.href for navigation
      delete (window as any).location;
      (window as any).location = { href: '' };

      const duplicateButton = screen.getByRole('button', { name: /duplicate/i });
      await user.click(duplicateButton);

      expect(testEnv.localStorage.setItem).toHaveBeenCalledWith(
        'duplicateDesign',
        expect.stringContaining('Actionable Robot (Copy)')
      );
      expect(window.location.href).toBe('/configure?duplicate=true');
    });
  });

  describe('Portfolio Analytics', () => {
    test('calculates and displays portfolio statistics correctly', async () => {
      const testDesigns = [
        createTestSepulka({ status: 'FORGING', alloys: [{ name: 'Motor', type: 'ACTUATOR' }] }),
        createTestSepulka({ status: 'READY', alloys: [{ name: 'Gripper', type: 'END_EFFECTOR' }] }),
        createTestSepulka({ 
          status: 'DEPLOYED', 
          pattern: { name: 'Quadruped', category: 'QUADRUPED' },
          alloys: [
            { name: 'Motor', type: 'ACTUATOR' },
            { name: 'Sensor', type: 'SENSOR' }
          ]
        }),
      ];

      (getMySepulkas as jest.Mock).mockResolvedValue(testDesigns);

      renderWithAuth(<MyDesignsPage />);

      await waitFor(() => {
        expect(screen.getByText('📊 Design Portfolio Summary')).toBeInTheDocument();
      });

      // Verify statistics
      expect(screen.getByText('3')).toBeInTheDocument(); // Total designs
      expect(screen.getByText('Total Designs')).toBeInTheDocument();
      expect(screen.getByText('2')).toBeInTheDocument(); // Pattern types (Industrial + Quadruped)
      expect(screen.getByText('Pattern Types')).toBeInTheDocument();
      expect(screen.getByText('4')).toBeInTheDocument(); // Total components
      expect(screen.getByText('Total Components')).toBeInTheDocument();
    });

    test('hides analytics when no designs exist', async () => {
      (getMySepulkas as jest.Mock).mockResolvedValue([]);

      renderWithAuth(<MyDesignsPage />);

      await waitFor(() => {
        expect(screen.getByText('No designs yet')).toBeInTheDocument();
      });

      expect(screen.queryByText('Design Portfolio Summary')).not.toBeInTheDocument();
    });
  });

  describe('Error Handling and Edge Cases', () => {
    test('handles network errors gracefully', async () => {
      (getMySepulkas as jest.Mock).mockRejectedValue(new Error('Network error'));

      renderWithAuth(<MyDesignsPage />);

      await waitFor(() => {
        expect(screen.getByText('Error loading designs')).toBeInTheDocument();
        expect(screen.getByText('Network error')).toBeInTheDocument();
      });

      const tryAgainButton = screen.getByRole('button', { name: /try again/i });
      expect(tryAgainButton).toBeInTheDocument();

      // Test retry functionality
      (getMySepulkas as jest.Mock).mockResolvedValue([createTestSepulka()]);
      
      await userEvent.setup().click(tryAgainButton);

      await waitFor(() => {
        expect(screen.queryByText('Error loading designs')).not.toBeInTheDocument();
        expect(screen.getByText('Test Robot Design')).toBeInTheDocument();
      });
    });

    test('handles designs with missing or partial data', async () => {
      const incompleteDesign = createTestSepulka({
        name: 'Incomplete Design',
        description: undefined,
        pattern: null,
        alloys: []
      });

      (getMySepulkas as jest.Mock).mockResolvedValue([incompleteDesign]);

      renderWithAuth(<MyDesignsPage />);

      await waitFor(() => {
        expect(screen.getByText('Incomplete Design')).toBeInTheDocument();
      });

      // Should still render without errors
      expect(screen.getByText('0 selected')).toBeInTheDocument(); // No components
      expect(screen.queryByText('Industrial Arm')).not.toBeInTheDocument(); // No pattern
    });

    test('handles very long design names and descriptions', async () => {
      const longNameDesign = createTestSepulka({
        name: 'This is a very long robot design name that might cause layout issues if not handled properly',
        description: 'This is an extremely long description that goes on and on and might cause text overflow issues in the UI if not properly handled with CSS text truncation or other responsive design techniques'
      });

      (getMySepulkas as jest.Mock).mockResolvedValue([longNameDesign]);

      renderWithAuth(<MyDesignsPage />);

      await waitFor(() => {
        expect(screen.getByText(longNameDesign.name)).toBeInTheDocument();
      });

      // Verify the long content is displayed (CSS should handle truncation)
      expect(screen.getByText(longNameDesign.description!)).toBeInTheDocument();
    });
  });

  describe('User Interactions and State Management', () => {
    test('manages loading states during actions correctly', async () => {
      const testDesign = createTestSepulka({ status: 'READY' });
      (getMySepulkas as jest.Mock).mockResolvedValue([testDesign]);

      // Mock a slow castIngot call
      (castIngot as jest.Mock).mockImplementation(() => 
        new Promise(resolve => setTimeout(resolve, 1000))
      );

      const user = userEvent.setup();
      renderWithAuth(<MyDesignsPage />);

      await waitFor(() => {
        expect(screen.getByText('Test Robot Design')).toBeInTheDocument();
      });

      const buildButton = screen.getByRole('button', { name: /build/i });
      await user.click(buildButton);

      // Should show loading state
      expect(screen.getByText('⏳ Build')).toBeInTheDocument();
      expect(buildButton).toBeDisabled();
    });

    test('calls getMySepulkas with correct smith ID', async () => {
      (getMySepulkas as jest.Mock).mockResolvedValue([]);

      renderWithAuth(<MyDesignsPage />);

      await waitFor(() => {
        expect(getMySepulkas).toHaveBeenCalledWith(mockSmith.id);
      });
    });

    test('refreshes data after successful build', async () => {
      const testDesign = createTestSepulka({ status: 'READY' });
      (getMySepulkas as jest.Mock).mockResolvedValue([testDesign]);
      (castIngot as jest.Mock).mockResolvedValue({
        ingot: { id: 'test-ingot' },
        errors: []
      });

      // Mock window.alert
      const alertSpy = jest.spyOn(window, 'alert').mockImplementation(() => {});

      const user = userEvent.setup();
      renderWithAuth(<MyDesignsPage />);

      await waitFor(() => {
        expect(screen.getByText('Test Robot Design')).toBeInTheDocument();
      });

      const buildButton = screen.getByRole('button', { name: /build/i });
      await user.click(buildButton);

      await waitFor(() => {
        expect(getMySepulkas).toHaveBeenCalledTimes(2); // Initial load + refresh after build
      });

      alertSpy.mockRestore();
    });
  });

  describe('Component Integration', () => {
    test('integrates with AuthProvider correctly', async () => {
      (getMySepulkas as jest.Mock).mockResolvedValue([]);

      renderWithAuth(<MyDesignsPage />);

      // Should show authenticated user's name in UI
      expect(screen.getByText('Development Smith')).toBeInTheDocument();
      expect(screen.getByText('over smith')).toBeInTheDocument();
    });

    test('handles route parameters for new design highlighting', async () => {
      const testDesign = createTestSepulka({ name: 'New Design' });
      (getMySepulkas as jest.Mock).mockResolvedValue([testDesign]);

      // Mock URLSearchParams to simulate ?newDesign=123 query parameter
      const mockURLSearchParams = {
        get: jest.fn().mockReturnValue(testDesign.id)
      };
      (global as any).URLSearchParams = jest.fn().mockImplementation(() => mockURLSearchParams);

      renderWithAuth(<MyDesignsPage />);

      await waitFor(() => {
        expect(screen.getByText('New Design')).toBeInTheDocument();
      });

      // Verify URLSearchParams was used to check for newDesign parameter
      expect(mockURLSearchParams.get).toHaveBeenCalledWith('newDesign');
    });
  });

  describe('Accessibility and UX', () => {
    test('has proper ARIA labels and roles', async () => {
      const testDesigns = [createTestSepulka()];
      (getMySepulkas as jest.Mock).mockResolvedValue(testDesigns);

      renderWithAuth(<MyDesignsPage />);

      await waitFor(() => {
        expect(screen.getByRole('main')).toBeInTheDocument();
        expect(screen.getByRole('navigation')).toBeInTheDocument();
        expect(screen.getAllByRole('button')).toHaveLength(7); // Filter tabs + action buttons
        expect(screen.getAllByRole('link')).toHaveLength(7); // Navigation + action links
      });
    });

    test('supports keyboard navigation for actions', async () => {
      const testDesign = createTestSepulka({ status: 'READY' });
      (getMySepulkas as jest.Mock).mockResolvedValue([testDesign]);

      renderWithAuth(<MyDesignsPage />);

      await waitFor(() => {
        expect(screen.getByText('Test Robot Design')).toBeInTheDocument();
      });

      // All action buttons should be focusable
      const editLink = screen.getByRole('link', { name: /edit/i });
      const duplicateButton = screen.getByRole('button', { name: /duplicate/i });
      const buildButton = screen.getByRole('button', { name: /build/i });
      const deleteButton = screen.getByRole('button', { name: '🗑️' });

      expect(editLink).toBeInTheDocument();
      expect(duplicateButton).toBeInTheDocument();
      expect(buildButton).toBeInTheDocument();
      expect(deleteButton).toBeInTheDocument();

      // Should all be keyboard accessible
      editLink.focus();
      expect(document.activeElement).toBe(editLink);
    });
  });
});
