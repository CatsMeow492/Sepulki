// Environment Configuration for Sepulki
// Handles automatic switching between development and production environments

export interface EnvironmentConfig {
  NODE_ENV: 'development' | 'production' | 'test'
  isDevelopment: boolean
  isProduction: boolean
  isTest: boolean
  
  // API Configuration
  graphqlEndpoint: string
  anvilSimEndpoint: string

  // Authentication
  useRealAuth: boolean
  authProviders: string[]
  
  // Storage
  minioEndpoint?: string
  useS3: boolean
  
  // Email
  emailService: 'local' | 'production'
  
  // Deployment platform detection
  deploymentPlatform: 'local' | 'vercel' | 'railway' | 'docker' | 'kubernetes' | 'other'
}

// Detect deployment platform
function detectDeploymentPlatform(): EnvironmentConfig['deploymentPlatform'] {
  if (process.env.VERCEL) return 'vercel'
  if (process.env.RAILWAY_STATIC_URL) return 'railway'
  if (process.env.KUBERNETES_SERVICE_HOST) return 'kubernetes'
  if (process.env.DOCKER_CONTAINER) return 'docker'
  if (process.env.NODE_ENV === 'production') return 'other'
  return 'local'
}

// Create environment-aware configuration
function createEnvironmentConfig(): EnvironmentConfig {
  const NODE_ENV = (process.env.NODE_ENV as EnvironmentConfig['NODE_ENV']) || 'development'
  const isDevelopment = NODE_ENV === 'development'
  const isProduction = NODE_ENV === 'production'
  const isTest = NODE_ENV === 'test'
  const deploymentPlatform = detectDeploymentPlatform()

  // Determine if we should use real authentication
  const useRealAuth = isProduction || !!process.env.GITHUB_CLIENT_ID || !!process.env.GOOGLE_CLIENT_ID
  
  // Auto-detect GraphQL endpoint
  const graphqlEndpoint = process.env.NEXT_PUBLIC_GRAPHQL_ENDPOINT ||
    (isProduction
      ? (process.env.VERCEL_URL
          ? `https://${process.env.VERCEL_URL}/api/graphql`
          : '/api/graphql')
      : 'http://localhost:4000/graphql')

  // Anvil Sim service endpoint (defaults to Brev instance if available)
  const anvilSimEndpoint = process.env.NEXT_PUBLIC_ANVIL_SIM_ENDPOINT ||
    process.env.ANVIL_SIM_ENDPOINT ||
    'http://localhost:8002'

  // Determine auth providers available
  const authProviders: string[] = []
  if (process.env.GITHUB_CLIENT_ID) authProviders.push('github')
  if (process.env.GOOGLE_CLIENT_ID) authProviders.push('google')
  if (process.env.LOCAL_OAUTH_CLIENT_ID) authProviders.push('local-oauth')
  if (isDevelopment && authProviders.length === 0) authProviders.push('mock')

  return {
    NODE_ENV,
    isDevelopment,
    isProduction, 
    isTest,
    deploymentPlatform,
    
    // API Configuration
    graphqlEndpoint,
    anvilSimEndpoint,
    
    // Authentication
    useRealAuth,
    authProviders,
    
    // Storage
    minioEndpoint: process.env.NEXT_PUBLIC_MINIO_ENDPOINT,
    useS3: isProduction && !process.env.NEXT_PUBLIC_MINIO_ENDPOINT,
    
    // Email
    emailService: isDevelopment ? 'local' : 'production',
  }
}

// Export singleton configuration
export const env = createEnvironmentConfig()

// Helper functions
export const isLocal = () => env.isDevelopment
export const isProduction = () => env.isProduction
export const shouldUseMockAuth = () => env.isDevelopment && env.authProviders.includes('mock')
export const shouldUseRealAuth = () => env.useRealAuth

// Development helpers
if (env.isDevelopment) {
  console.log('🔧 Sepulki Environment Configuration:', {
    platform: env.deploymentPlatform,
    auth: env.authProviders,
    graphql: env.graphqlEndpoint,
    anvilSim: env.anvilSimEndpoint,
    mockAuth: shouldUseMockAuth(),
  })
}
