// Testing utilities for Sepulki Forge UI
// Provides helpers for unit tests, integration tests, and test data

import React from 'react';
import { render, RenderOptions } from '@testing-library/react';
import { AuthProvider } from '@/components/AuthProvider';

// Mock Smith for testing
export const mockSmith = {
  id: 'test-smith-001',
  name: 'Test Smith',
  email: 'test@sepulki.com', 
  image: '',
  role: 'OVER_SMITH' as const,
};

// Mock auth context
export const MockAuthProvider = ({ children, smith = mockSmith }: { 
  children: React.ReactNode;
  smith?: typeof mockSmith | null;
}) => {
  // Mock the auth provider with test data
  const mockAuthContext = {
    smith,
    loading: false,
    signOut: jest.fn(),
    authMode: 'mock' as const,
  };

  // Override the useAuth hook for testing
  React.useEffect(() => {
    if (typeof window !== 'undefined') {
      (window as any).__SEPULKI_AUTH__ = { smith, authMode: 'mock' };
    }
  }, [smith]);

  return React.createElement(
    'div',
    { 'data-testid': 'mock-auth-provider' },
    children
  );
};

// Custom render function with providers
export const renderWithAuth = (
  ui: React.ReactElement,
  options: RenderOptions & { smith?: typeof mockSmith | null } = {}
) => {
  const { smith = mockSmith, ...renderOptions } = options;

  const Wrapper = ({ children }: { children: React.ReactNode }) =>
    React.createElement(MockAuthProvider, { smith }, children);

  return render(ui, { wrapper: Wrapper, ...renderOptions });
};

// Mock GraphQL responses
export const mockGraphQLResponses = {
  getMySepulkas: {
    success: {
      data: {
        sepulkas: [
          {
            id: 'test-sepulka-001',
            name: 'Test Robot Design',
            description: 'A test robot design',
            version: '1.0.0',
            status: 'FORGING',
            pattern: {
              name: 'Industrial Arm - 6DOF',
              category: 'INDUSTRIAL_ARM'
            },
            alloys: [
              { name: 'Test Servo', type: 'ACTUATOR' },
              { name: 'Test Gripper', type: 'END_EFFECTOR' }
            ],
            createdAt: '2025-09-20T10:00:00Z',
            updatedAt: '2025-09-20T10:00:00Z'
          },
          {
            id: 'test-sepulka-002',
            name: 'Ready Robot Design',
            description: 'A robot ready for building',
            version: '1.0.0',
            status: 'READY',
            pattern: {
              name: 'Industrial Arm - 6DOF',
              category: 'INDUSTRIAL_ARM'
            },
            alloys: [
              { name: 'Test Servo', type: 'ACTUATOR' }
            ],
            createdAt: '2025-09-19T15:30:00Z',
            updatedAt: '2025-09-19T15:30:00Z'
          }
        ]
      }
    },
    error: {
      errors: [{ message: 'Authentication required' }]
    },
    empty: {
      data: { sepulkas: [] }
    }
  },
  
  castIngot: {
    success: {
      data: {
        castIngot: {
          ingot: {
            id: 'test-ingot-001',
            sepulkaId: 'test-sepulka-002',
            version: '1.0.0',
            buildHash: 'build_123456_abc123',
            status: 'BUILDING',
            artifacts: [],
            createdAt: '2025-09-20T12:00:00Z'
          },
          errors: []
        }
      }
    },
    error: {
      data: {
        castIngot: {
          ingot: null,
          errors: [{ code: 'VALIDATION_ERROR', message: 'Sepulka must be in CAST_READY status' }]
        }
      }
    }
  },

  deleteSepulka: {
    success: {
      data: {
        deleteSepulka: {
          success: true,
          errors: []
        }
      }
    },
    error: {
      data: {
        deleteSepulka: {
          success: false,
          errors: [{ code: 'NOT_FOUND', message: 'Sepulka not found' }]
        }
      }
    }
  }
};

// Test data helpers
export const createTestSepulka = (overrides = {}) => ({
  id: 'test-sepulka-' + Math.random().toString(36).substr(2, 9),
  name: 'Test Robot Design',
  description: 'A test robot design',
  version: '1.0.0',
  status: 'FORGING',
  pattern: {
    name: 'Industrial Arm - 6DOF',
    category: 'INDUSTRIAL_ARM'
  },
  alloys: [
    { name: 'Test Servo', type: 'ACTUATOR' },
    { name: 'Test Gripper', type: 'END_EFFECTOR' }
  ],
  createdAt: new Date().toISOString(),
  updatedAt: new Date().toISOString(),
  ...overrides
});

// Mock GraphQL client for testing
export const createMockGraphQLClient = (responses: any = {}) => ({
  request: jest.fn().mockImplementation(({ query, variables }) => {
    const queryName = query.match(/(?:query|mutation)\s+(\w+)/)?.[1];
    
    if (responses[queryName]) {
      return Promise.resolve(responses[queryName]);
    }
    
    // Default successful response
    return Promise.resolve({ data: {} });
  })
});

// Async component testing helper
export const waitForAsyncComponent = async (container: HTMLElement, timeout = 5000) => {
  const start = Date.now();
  while (Date.now() - start < timeout) {
    if (container.querySelector('[data-testid]')) {
      break;
    }
    await new Promise(resolve => setTimeout(resolve, 50));
  }
};

// Mock environment setup
export const setupTestEnvironment = () => {
  // Mock localStorage
  const localStorageMock = {
    getItem: jest.fn(),
    setItem: jest.fn(),
    removeItem: jest.fn(),
    clear: jest.fn(),
  };

  // Only define localStorage if it doesn't exist
  if (!window.localStorage) {
    Object.defineProperty(window, 'localStorage', {
      value: localStorageMock,
      writable: true,
    });
  }

  // Mock fetch for GraphQL requests
  const fetchMock = jest.fn();
  global.fetch = fetchMock;

  return {
    localStorage: localStorageMock,
    fetch: fetchMock,
  };
};

// Clean up after tests
export const cleanupTestEnvironment = () => {
  jest.clearAllMocks();
  jest.restoreAllMocks();
  
  // Clean up global auth state
  if (typeof window !== 'undefined') {
    delete (window as any).__SEPULKI_AUTH__;
  }
};

export * from '@testing-library/react';
export { default as userEvent } from '@testing-library/user-event';
