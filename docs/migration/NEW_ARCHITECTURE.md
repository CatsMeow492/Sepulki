# Sepulki Platform Architecture

## Brand-Aligned Service Architecture

Based on the metallurgy/smithing brand guidelines, here's the planned service architecture for the robotics-as-a-service platform:

### Core Services

#### Frontend Layer
- **forge-ui** (Next.js/React)
  - 3D Robot Design Studio (Isaac Sim integration)
  - Fleet Management Dashboard  
  - Task Assignment Interface
  - User Authentication & Profiles
  - Real-time telemetry display (Bellows integration)

#### Backend Services
- **hammer-orchestrator** (Node.js/GraphQL)
  - Fleet coordination and load balancing
  - Task assignment and routing
  - Safety monitoring and killswitches
  - Primary GraphQL API gateway

- **foundry-pipeline** (Node.js/Docker)
  - Build and compilation pipeline
  - Spec → URDF → Container conversion
  - Version management and signing
  - Artifact publishing to Vault

- **anvil-sim** (Python/Isaac Sim)
  - Physics simulation and validation
  - Collision detection and stress testing
  - Latency and performance modeling
  - 3D scene management

- **vault-registry** (Node.js/PostgreSQL)
  - Component library (Alloys) management
  - Pattern templates storage
  - Artifact and build storage
  - Version control and metadata

- **bellows-telemetry** (Node.js/InfluxDB)
  - Time-series telemetry ingestion
  - Real-time metrics streaming
  - Health monitoring and alerts
  - Performance analytics

- **tongs-policy** (Node.js/OPA)
  - Safety constraint enforcement
  - Compliance policy engine
  - Real-time safety guards
  - Geo-fencing and operational limits

- **choreo-dispatch** (Python/OR-Tools)
  - Task planning and optimization
  - Route solving and scheduling
  - Resource allocation
  - Mission orchestration

#### Data Layer
- **PostgreSQL**: Core data (sepulkas, alloys, patterns, fleets, tasks)
- **InfluxDB**: Time-series telemetry (bellows streams)
- **Redis**: Caching and real-time state
- **MinIO/S3**: Asset storage (3D models, builds)

### GraphQL Schema Structure

```graphql
# Core Types
type Sepulka {
  id: ID!
  name: String!
  version: String!
  pattern: Pattern
  alloys: [Alloy!]!
  ingots: [Ingot!]!
  status: SepulkaStatus!
}

type Alloy {
  id: ID!
  name: String!
  type: AlloyType!
  specifications: JSON
  meshAssets: [String!]
  compatibility: [AlloyCompatibility!]
}

type Fleet {
  id: ID!
  name: String!
  locus: Locus!
  robots: [Robot!]!
  activeTask: Task
  status: FleetStatus!
}

type Task {
  id: ID!
  name: String!
  type: TaskType!
  parameters: JSON
  assignedRobots: [Robot!]
  runs: [Run!]!
  status: TaskStatus!
}

# Mutations
type Mutation {
  forgeSepulka(input: ForgeInput!): ForgeSepulkaPayload!
  castIngot(sepulkaId: ID!): CastIngotPayload!
  temperIngot(ingotId: ID!): TemperIngotPayload!
  quenchToFleet(ingotId: ID!, fleetId: ID!): QuenchPayload!
  dispatchTask(fleetId: ID!, taskInput: TaskInput!): DispatchPayload!
}
```

### Migration Strategy

#### Phase 1: Foundation (Week 1-2)
1. Set up new repository structure
2. Migrate existing forge-ui components
3. Create basic GraphQL schema and hammer-orchestrator
4. Set up development infrastructure

#### Phase 2: Core Services (Week 3-4)
5. Implement foundry-pipeline for build management
6. Create vault-registry for component catalog
7. Set up bellows-telemetry for basic metrics
8. Implement user authentication

#### Phase 3: Advanced Features (Week 5-6)
9. Integrate Isaac Sim through anvil-sim service
10. Implement tongs-policy for safety constraints  
11. Create choreo-dispatch for task planning
12. Add fleet management capabilities

#### Phase 4: Integration & Polish (Week 7-8)
13. End-to-end testing and integration
14. Performance optimization
15. Documentation and deployment guides
16. Production readiness

### Repository Structure

```
sepulki/
├── apps/
│   └── forge-ui/                 # Frontend React app
├── services/
│   ├── hammer-orchestrator/      # GraphQL API gateway
│   ├── foundry-pipeline/         # Build & CI/CD
│   ├── anvil-sim/               # Isaac Sim integration
│   ├── vault-registry/          # Component catalog
│   ├── bellows-telemetry/       # Metrics & monitoring
│   ├── tongs-policy/            # Safety & compliance
│   └── choreo-dispatch/         # Task planning
├── packages/
│   ├── shared-types/            # Common TypeScript types
│   ├── graphql-schema/          # Shared GraphQL definitions
│   └── sepulki-sdk/             # Client SDK
├── infrastructure/
│   ├── docker-compose.yml       # Local development
│   ├── kubernetes/              # Production deployment
│   └── terraform/               # Infrastructure as code
└── docs/
    ├── api/                     # API documentation
    └── guides/                  # User guides
```

This architecture maintains the excellent 3D capabilities you've built while scaling to a full robotics-as-a-service platform with proper microservices, GraphQL API, and the cohesive metallurgy branding.
