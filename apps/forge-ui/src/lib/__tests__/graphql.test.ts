// Integration tests for GraphQL client and My Designs queries/mutations
// Tests API integration, authentication, and error handling

import { 
  getMySepulkas, 
  castIngot, 
  deleteSepulka, 
  forgeSepulka 
} from '../graphql';
import { setupTestEnvironment, cleanupTestEnvironment, mockSmith } from '@/test-utils';

// Mock fetch for GraphQL requests
const mockFetch = jest.fn();
global.fetch = mockFetch as any;

describe('GraphQL Client', () => {
  let testEnv: ReturnType<typeof setupTestEnvironment>;

  beforeEach(() => {
    testEnv = setupTestEnvironment();
    mockFetch.mockClear();
  });

  afterEach(() => {
    cleanupTestEnvironment();
  });

  describe('Authentication and Token Generation', () => {
    test('getMySepulkas includes auth token when smith is available', async () => {
      // Set up global auth context
      (window as any).__SEPULKI_AUTH__ = { smith: mockSmith, authMode: 'mock' };

      // Mock successful response
      testEnv.fetch.mockResolvedValue({
        ok: true,
        json: () => Promise.resolve({ data: { sepulkas: [] } })
      } as any);

      await getMySepulkas('test-smith-id');

      expect(testEnv.fetch).toHaveBeenCalledWith(
        expect.any(String),
        expect.objectContaining({
          headers: expect.objectContaining({
            'Authorization': expect.stringMatching(/Bearer .+/)
          })
        })
      );
    });

    test('getMySepulkas works without token when no smith available', async () => {
      // No auth context
      delete (window as any).__SEPULKI_AUTH__;

      testEnv.fetch.mockResolvedValue({
        ok: true,
        json: () => Promise.resolve({ data: { sepulkas: [] } })
      } as any);

      await getMySepulkas('test-smith-id');

      expect(testEnv.fetch).toHaveBeenCalledWith(
        expect.any(String),
        expect.objectContaining({
          headers: expect.not.objectContaining({
            'Authorization': expect.anything()
          })
        })
      );
    });
  });

  describe('getMySepulkas Query', () => {
    test('makes correct GraphQL request for user designs', async () => {
      const mockResponse = {
        data: {
          sepulkas: [
            {
              id: 'test-1',
              name: 'Test Design',
              status: 'FORGING',
              pattern: { name: 'Industrial Arm', category: 'INDUSTRIAL_ARM' },
              alloys: [{ name: 'Motor', type: 'ACTUATOR' }],
              createdAt: '2025-09-20T10:00:00Z'
            }
          ]
        }
      };

      mockFetch.mockResolvedValue({
        ok: true,
        json: () => Promise.resolve(mockResponse)
      });

      const result = await getMySepulkas('test-smith-id');

      expect(mockFetch).toHaveBeenCalledWith(
        expect.any(String),
        expect.objectContaining({
          method: 'POST',
          body: expect.stringContaining('GetMySepulkas'),
        })
      );

      expect(result).toEqual(mockResponse.data.sepulkas);
    });

    test('handles GraphQL errors correctly', async () => {
      const mockErrorResponse = {
        errors: [{ message: 'Authentication required' }]
      };

      mockFetch.mockResolvedValue({
        ok: true,
        json: () => Promise.resolve(mockErrorResponse)
      });

      await expect(getMySepulkas('test-smith-id')).rejects.toThrow('Authentication required');
    });

    test('handles network errors correctly', async () => {
      mockFetch.mockRejectedValue(new Error('Network error'));

      await expect(getMySepulkas('test-smith-id')).rejects.toThrow('Network error');
    });

    test('handles HTTP error responses', async () => {
      mockFetch.mockResolvedValue({
        ok: false,
        status: 401,
        statusText: 'Unauthorized'
      });

      await expect(getMySepulkas('test-smith-id')).rejects.toThrow('HTTP 401: Unauthorized');
    });
  });

  describe('castIngot Mutation', () => {
    test('makes correct GraphQL request for building design', async () => {
      const mockResponse = {
        data: {
          castIngot: {
            ingot: {
              id: 'test-ingot-001',
              sepulkaId: 'test-sepulka-001',
              version: '1.0.0',
              buildHash: 'build_123_abc',
              status: 'BUILDING',
              artifacts: [],
              createdAt: '2025-09-20T12:00:00Z'
            },
            errors: []
          }
        }
      };

      mockFetch.mockResolvedValue({
        ok: true,
        json: () => Promise.resolve(mockResponse)
      });

      const result = await castIngot('test-sepulka-001');

      expect(mockFetch).toHaveBeenCalledWith(
        expect.any(String),
        expect.objectContaining({
          method: 'POST',
          body: expect.stringContaining('CastIngot'),
        })
      );

      expect(result).toEqual(mockResponse.data.castIngot);
    });

    test('handles build errors from backend', async () => {
      const mockErrorResponse = {
        data: {
          castIngot: {
            ingot: null,
            errors: [{ code: 'VALIDATION_ERROR', message: 'Sepulka must be in CAST_READY status' }]
          }
        }
      };

      mockFetch.mockResolvedValue({
        ok: true,
        json: () => Promise.resolve(mockErrorResponse)
      });

      const result = await castIngot('test-sepulka-001');

      expect(result.errors).toHaveLength(1);
      expect(result.errors![0].message).toBe('Sepulka must be in CAST_READY status');
      expect(result.ingot).toBeNull();
    });
  });

  describe('deleteSepulka Mutation', () => {
    test('makes correct GraphQL request for deleting design', async () => {
      const mockResponse = {
        data: {
          deleteSepulka: {
            success: true,
            errors: []
          }
        }
      };

      mockFetch.mockResolvedValue({
        ok: true,
        json: () => Promise.resolve(mockResponse)
      });

      const result = await deleteSepulka('test-sepulka-001');

      expect(mockFetch).toHaveBeenCalledWith(
        expect.any(String),
        expect.objectContaining({
          method: 'POST',
          body: expect.stringContaining('DeleteSepulka'),
        })
      );

      expect(result.success).toBe(true);
    });

    test('handles delete errors from backend', async () => {
      const mockErrorResponse = {
        errors: [{ message: 'Sepulka not found' }]
      };

      mockFetch.mockResolvedValue({
        ok: true,
        json: () => Promise.resolve(mockErrorResponse)
      });

      await expect(deleteSepulka('non-existent-id')).rejects.toThrow('Sepulka not found');
    });
  });

  describe('Request Formatting and Error Handling', () => {
    test('includes proper headers in all requests', async () => {
      (window as any).__SEPULKI_AUTH__ = { smith: mockSmith, authMode: 'mock' };

      mockFetch.mockResolvedValue({
        ok: true,
        json: () => Promise.resolve({ data: { sepulkas: [] } })
      });

      await getMySepulkas('test-smith-id');

      expect(mockFetch).toHaveBeenCalledWith(
        expect.any(String),
        expect.objectContaining({
          method: 'POST',
          headers: expect.objectContaining({
            'Content-Type': 'application/json',
            'Authorization': expect.stringMatching(/Bearer .+/)
          }),
          credentials: 'include'
        })
      );
    });

    test('handles malformed JSON responses', async () => {
      mockFetch.mockResolvedValue({
        ok: true,
        json: () => Promise.reject(new Error('Invalid JSON'))
      });

      await expect(getMySepulkas('test-smith-id')).rejects.toThrow('Invalid JSON');
    });

    test('handles timeout scenarios', async () => {
      // Simulate a timeout
      mockFetch.mockImplementation(() => 
        new Promise((_, reject) => 
          setTimeout(() => reject(new Error('Request timeout')), 100)
        )
      );

      await expect(getMySepulkas('test-smith-id')).rejects.toThrow('Request timeout');
    });
  });
});
