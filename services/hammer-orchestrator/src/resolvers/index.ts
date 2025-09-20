import { DateTimeResolver, JSONResolver } from 'graphql-scalars';
import { sepulkaResolvers } from './sepulka';
import { alloyResolvers } from './alloy';
import { fleetResolvers } from './fleet';
import { taskResolvers } from './task';
import { authResolvers } from './auth';
import { telemetryResolvers } from './telemetry';
import type { Resolvers } from './types';

export const resolvers: Resolvers = {
  // Scalar types
  DateTime: DateTimeResolver,
  JSON: JSONResolver,

  // Queries
  Query: {
    ...sepulkaResolvers.Query,
    ...alloyResolvers.Query,
    ...fleetResolvers.Query,
    ...taskResolvers.Query,
    ...telemetryResolvers.Query,
    
    // Stub implementation for edicts (TODO: Implement properly)
    edicts: async () => [],
  },

  // Mutations  
  Mutation: {
    ...sepulkaResolvers.Mutation,
    ...fleetResolvers.Mutation,
    ...taskResolvers.Mutation,
    ...authResolvers.Mutation,
    
    // Stub implementations for missing mutations (TODO: Implement properly)
    recallFleet: async () => { throw new Error('recallFleet not yet implemented') },
    addEdict: async () => { throw new Error('addEdict not yet implemented') },
    updateEdict: async () => { throw new Error('updateEdict not yet implemented') },
    deactivateEdict: async () => { throw new Error('deactivateEdict not yet implemented') },
    logout: async () => { throw new Error('logout not yet implemented') },
  },

  // Subscriptions
  Subscription: {
    ...telemetryResolvers.Subscription,
    
    // Stub implementations for missing subscriptions (TODO: Implement properly)
    taskUpdates: async () => { throw new Error('taskUpdates subscription not yet implemented') },
    robotStatus: async () => { throw new Error('robotStatus subscription not yet implemented') },
    policyBreaches: async () => { throw new Error('policyBreaches subscription not yet implemented') },
  },

  // Type resolvers
  Sepulka: sepulkaResolvers.Sepulka,
  Ingot: sepulkaResolvers.Ingot,
  Alloy: alloyResolvers.Alloy,
  Fleet: fleetResolvers.Fleet,
  Robot: fleetResolvers.Robot,
  Task: taskResolvers.Task,
  Run: taskResolvers.Run,
  Smith: authResolvers.Smith,
};
