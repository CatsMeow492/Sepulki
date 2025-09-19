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
  },

  // Mutations  
  Mutation: {
    ...sepulkaResolvers.Mutation,
    ...fleetResolvers.Mutation,
    ...taskResolvers.Mutation,
    ...authResolvers.Mutation,
  },

  // Subscriptions
  Subscription: {
    ...telemetryResolvers.Subscription,
    ...fleetResolvers.Subscription,
    ...taskResolvers.Subscription,
  },

  // Type resolvers
  Sepulka: sepulkaResolvers.Sepulka,
  Alloy: alloyResolvers.Alloy,
  Fleet: fleetResolvers.Fleet,
  Robot: fleetResolvers.Robot,
  Task: taskResolvers.Task,
  Run: taskResolvers.Run,
  Smith: authResolvers.Smith,
};
