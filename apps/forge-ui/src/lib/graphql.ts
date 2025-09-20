// GraphQL client utilities for Sepulki frontend
// Handles authentication and error management

interface GraphQLResponse<T = any> {
  data?: T;
  errors?: Array<{
    message: string;
    code?: string;
    field?: string;
  }>;
}

interface GraphQLRequest {
  query: string;
  variables?: Record<string, any>;
}

class GraphQLClient {
  private endpoint: string;
  private getAuthToken: () => string | null;

  constructor(endpoint: string, getAuthToken: () => string | null) {
    this.endpoint = endpoint;
    this.getAuthToken = getAuthToken;
  }

  async request<T = any>(request: GraphQLRequest): Promise<GraphQLResponse<T>> {
    const token = this.getAuthToken();
    
    const response = await fetch(this.endpoint, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
        ...(token && { 'Authorization': `Bearer ${token}` }),
      },
      credentials: 'include',
      body: JSON.stringify(request),
    });

    if (!response.ok) {
      throw new Error(`HTTP ${response.status}: ${response.statusText}`);
    }

    return response.json();
  }
}

// Mock JWT token generator for development
function generateMockJWT(smith: any): string {
  // Create a simple mock JWT for development - backend will validate this
  const header = { alg: 'HS256', typ: 'JWT' };
  const payload = {
    sub: smith.id,
    email: smith.email,
    name: smith.name,
    role: smith.role,
    sessionId: 'mock-session-001',
    iat: Math.floor(Date.now() / 1000),
    exp: Math.floor(Date.now() / 1000) + (24 * 60 * 60) // 24 hours
  };

  // Simple base64 encoding for development (NOT secure for production)
  const encodedHeader = btoa(JSON.stringify(header));
  const encodedPayload = btoa(JSON.stringify(payload));
  const signature = 'mock-signature-for-development';

  return `${encodedHeader}.${encodedPayload}.${signature}`;
}

// Get current smith from AuthProvider context
function getCurrentSmith() {
  // Access the auth context - this is a simple approach for development
  if (typeof window === 'undefined') return null;
  
  const authData = (window as any).__SEPULKI_AUTH__;
  return authData?.smith || null;
}

// Create client instance
const graphqlClient = new GraphQLClient(
  process.env.NEXT_PUBLIC_GRAPHQL_ENDPOINT || 'http://localhost:4000/graphql',
  () => {
    const smith = getCurrentSmith();
    if (smith) {
      return generateMockJWT(smith);
    }
    return null;
  }
);

// GraphQL mutations for robot design
export const FORGE_SEPULKA_MUTATION = `
  mutation ForgeSepulka($input: ForgeInput!) {
    forgeSepulka(input: $input) {
      sepulka {
        id
        name
        description
        version
        status
        pattern {
          id
          name
          category
        }
        alloys {
          id
          name
          type
          specifications
        }
        parameters
        createdAt
        createdBy {
          id
          name
          email
        }
      }
      errors {
        code
        message
        field
      }
    }
  }
`;

export const GET_MY_SEPULKAS_QUERY = `
  query GetMySepulkas($smithId: ID) {
    sepulkas(filter: { createdBy: $smithId }, limit: 50) {
      id
      name
      description
      version
      status
      pattern {
        name
        category
      }
      alloys {
        name
        type
      }
      createdAt
      updatedAt
    }
  }
`;

export const GET_PATTERNS_QUERY = `
  query GetPatterns {
    patterns(limit: 100) {
      id
      name
      description
      category
      parameters
      defaults
    }
  }
`;

export const GET_ALLOYS_QUERY = `
  query GetAlloys {
    alloys(limit: 100) {
      id
      name
      description
      type
      specifications
      version
      tags
    }
  }
`;

// Type-safe mutation functions
export interface ForgeInput {
  name: string;
  description?: string;
  patternId?: string;
  alloyIds: string[];
  parameters?: Record<string, any>;
}

export interface ForgeSepulkaResponse {
  sepulka?: {
    id: string;
    name: string;
    description?: string;
    version: string;
    status: string;
    pattern?: {
      id: string;
      name: string;
      category: string;
    };
    alloys: Array<{
      id: string;
      name: string;
      type: string;
      specifications: any;
    }>;
    parameters: Record<string, any>;
    createdAt: string;
    createdBy: {
      id: string;
      name: string;
      email: string;
    };
  };
  errors?: Array<{
    code: string;
    message: string;
    field?: string;
  }>;
}

export async function forgeSepulka(input: ForgeInput): Promise<ForgeSepulkaResponse> {
  const response = await graphqlClient.request<{ forgeSepulka: ForgeSepulkaResponse }>({
    query: FORGE_SEPULKA_MUTATION,
    variables: { input },
  });

  if (response.errors) {
    throw new Error(`GraphQL Error: ${response.errors[0].message}`);
  }

  return response.data!.forgeSepulka;
}

export async function getMySepulkas(smithId?: string) {
  const response = await graphqlClient.request({
    query: GET_MY_SEPULKAS_QUERY,
    variables: smithId ? { smithId } : {},
  });

  if (response.errors) {
    throw new Error(`GraphQL Error: ${response.errors[0].message}`);
  }

  return response.data.sepulkas;
}

export async function getPatterns() {
  const response = await graphqlClient.request({
    query: GET_PATTERNS_QUERY,
  });

  if (response.errors) {
    throw new Error(`GraphQL Error: ${response.errors[0].message}`);
  }

  return response.data.patterns;
}

export async function getAlloys() {
  const response = await graphqlClient.request({
    query: GET_ALLOYS_QUERY,
  });

  if (response.errors) {
    throw new Error(`GraphQL Error: ${response.errors[0].message}`);
  }

  return response.data.alloys;
}

// Additional mutations for design management
export const CAST_INGOT_MUTATION = `
  mutation CastIngot($sepulkaId: ID!) {
    castIngot(sepulkaId: $sepulkaId) {
      ingot {
        id
        sepulkaId
        version
        buildHash
        status
        artifacts {
          type
          path
          checksum
        }
        createdAt
      }
      errors {
        code
        message
      }
    }
  }
`;

export const DELETE_SEPULKA_MUTATION = `
  mutation DeleteSepulka($id: ID!) {
    deleteSepulka(id: $id) {
      success
      errors {
        code
        message
      }
    }
  }
`;

// Type-safe mutation functions for design management
export interface CastIngotResponse {
  ingot?: {
    id: string;
    sepulkaId: string;
    version: string;
    buildHash: string;
    status: string;
    artifacts: Array<{
      type: string;
      path: string;
      checksum: string;
    }>;
    createdAt: string;
  };
  errors?: Array<{
    code: string;
    message: string;
  }>;
}

export async function castIngot(sepulkaId: string): Promise<CastIngotResponse> {
  const response = await graphqlClient.request<{ castIngot: CastIngotResponse }>({
    query: CAST_INGOT_MUTATION,
    variables: { sepulkaId },
  });

  if (response.errors) {
    throw new Error(`GraphQL Error: ${response.errors[0].message}`);
  }

  return response.data!.castIngot;
}

export async function deleteSepulka(id: string): Promise<{ success: boolean }> {
  const response = await graphqlClient.request<{ deleteSepulka: { success: boolean } }>({
    query: DELETE_SEPULKA_MUTATION,
    variables: { id },
  });

  if (response.errors) {
    throw new Error(`GraphQL Error: ${response.errors[0].message}`);
  }

  return response.data!.deleteSepulka;
}

export { graphqlClient };
