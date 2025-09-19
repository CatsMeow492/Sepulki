# 🔥 Sepulki - Robotics as a Service Platform

A comprehensive robotics-as-a-service platform with metallurgy-themed branding, enabling users to design, deploy, and manage robot fleets through an intuitive 3D interface powered by Isaac Sim.

## 🏗️ Architecture Overview

Sepulki follows a microservices architecture with a metallurgy/smithing theme:

### Core Services

- **🔨 Hammer Orchestrator** - GraphQL API gateway and fleet coordination
- **⚒️ Forge UI** - 3D robot design studio and fleet management dashboard  
- **🏭 Foundry Pipeline** - Build and deployment automation
- **🔧 Anvil Sim** - Physics simulation and validation (Isaac Sim integration)
- **📦 Vault Registry** - Component library and artifact storage
- **📊 Bellows Telemetry** - Real-time metrics and monitoring
- **⚖️ Tongs Policy** - Safety constraints and compliance engine  
- **🎯 Choreo Dispatch** - Task planning and optimization

### Technology Stack

- **Frontend**: Next.js, React, TypeScript, React Three Fiber
- **Backend**: Node.js, GraphQL (Apollo Server), PostgreSQL, Redis
- **Simulation**: Isaac Sim, Python
- **Infrastructure**: Docker, Kubernetes, MinIO
- **Monitoring**: InfluxDB, Grafana

## 🚀 Quick Start

### Prerequisites

- Node.js 18.18.0+
- Docker & Docker Compose
- Git

### Development Setup

1. **Clone the repository**
   ```bash
   git clone https://github.com/CatsMeow492/Sepulki.git
   cd Sepulki
   ```

2. **Install dependencies**
   ```bash
   npm install
   ```

3. **Start infrastructure services**
   ```bash
   npm run docker:up
   ```

4. **Run the development servers**
   ```bash
   npm run dev
   ```

5. **Access the applications**
   - **Forge UI**: http://localhost:3000
   - **GraphQL Playground**: http://localhost:4000/graphql  
   - **MinIO Console**: http://localhost:9001

### Migration from Artifex

If migrating from the previous Artifex project:

```bash
npm run migrate
```

This will:
- Rename and restructure directories
- Update package configurations  
- Set up the new service architecture
- Preserve existing 3D components

## 📁 Project Structure

```
sepulki/
├── apps/
│   └── forge-ui/                 # 3D Robot Design Studio
├── services/
│   ├── hammer-orchestrator/      # GraphQL API Gateway
│   ├── foundry-pipeline/         # Build & CI/CD
│   ├── anvil-sim/               # Isaac Sim Integration
│   ├── vault-registry/          # Component Catalog
│   ├── bellows-telemetry/       # Metrics & Monitoring  
│   ├── tongs-policy/            # Safety & Compliance
│   └── choreo-dispatch/         # Task Planning
├── packages/
│   ├── shared-types/            # Common TypeScript Types
│   ├── graphql-schema/          # Shared GraphQL Schema
│   └── sepulki-sdk/             # Client SDK
├── infrastructure/
│   ├── sql/                     # Database Schemas
│   ├── kubernetes/              # K8s Manifests  
│   └── terraform/               # Infrastructure as Code
└── docs/                        # Documentation
```

## 🎨 Brand Guidelines

Sepulki uses a consistent metallurgy/smithing theme throughout:

### Core Concepts
- **Sepulka** - Robot design (singular unit)
- **Alloy** - Component (hardware/software module) 
- **Pattern** - Design template
- **Ingot** - Compiled build artifact
- **Fleet** - Group of robots
- **Locus** - Physical location
- **Smith** - User/designer
- **Edict** - Policy rule

### Operations
- **Forge** - Create/design
- **Cast** - Compile/build
- **Temper** - Optimize  
- **Quench** - Deploy
- **Recall** - Rollback

## 🔧 Development

### Available Scripts

- `npm run dev` - Start all development servers
- `npm run build` - Build all packages and services
- `npm run test` - Run all tests
- `npm run lint` - Lint all code
- `npm run docker:up` - Start infrastructure
- `npm run docker:down` - Stop infrastructure

### Service Development

Each service is independently developed:

```bash
# Work on the orchestrator  
cd services/hammer-orchestrator
npm run dev

# Work on the UI
cd apps/forge-ui  
npm run dev
```

### Database Management

```bash
# View database logs
npm run docker:logs db

# Connect to database
docker exec -it sepulki_postgres_1 psql -U smith -d sepulki
```

## 🧪 Testing

```bash
# Run all tests
npm test

# Run E2E tests
npm run test:e2e

# Test specific service
npm test --workspace @sepulki/hammer-orchestrator
```

## 📊 GraphQL API

The Hammer Orchestrator exposes a comprehensive GraphQL API:

### Key Queries
```graphql
query GetFleets {
  fleets {
    id
    name
    status
    robots {
      id
      name
      status
      batteryLevel
    }
  }
}
```

### Key Mutations  
```graphql
mutation ForgeSepulka($input: ForgeInput!) {
  forgeSepulka(input: $input) {
    sepulka {
      id
      name
      status
    }
    errors {
      code
      message
    }
  }
}
```

## 🚢 Deployment

### Local Development
Uses Docker Compose for local infrastructure

### Production
Kubernetes manifests provided in `infrastructure/kubernetes/`

```bash
kubectl apply -f infrastructure/kubernetes/
```

## 🤝 Contributing

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/amazing-feature`)
3. Commit changes (`git commit -m 'Add amazing feature'`)
4. Push to branch (`git push origin feature/amazing-feature`)
5. Open a Pull Request

## 📄 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## 🔗 Links

- **Documentation**: [docs/](./docs/)
- **Architecture**: [docs/migration/NEW_ARCHITECTURE.md](./docs/migration/NEW_ARCHITECTURE.md)
- **API Reference**: http://localhost:4000/graphql
- **Issues**: https://github.com/CatsMeow492/Sepulki/issues

---

Made with ⚒️ by the Sepulki team