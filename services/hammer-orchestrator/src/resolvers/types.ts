import type { GraphQLResolveInfo } from 'graphql';
import type { Context } from '../context';

export type Resolver<TResult, TParent = {}, TArgs = {}> = (
  parent: TParent,
  args: TArgs,
  context: Context,
  info: GraphQLResolveInfo
) => Promise<TResult> | TResult;

export interface Resolvers {
  DateTime: any;
  JSON: any;
  
  Query: {
    sepulkas: Resolver<any[], any, { filter?: any; limit?: number; offset?: number }>;
    sepulka: Resolver<any | null, any, { id: string }>;
    alloys: Resolver<any[], any, { filter?: any; limit?: number; offset?: number }>;
    alloy: Resolver<any | null, any, { id: string }>;
    patterns: Resolver<any[], any, { filter?: any; limit?: number; offset?: number }>;
    pattern: Resolver<any | null, any, { id: string }>;
    fleets: Resolver<any[], any, { filter?: any; limit?: number; offset?: number }>;
    fleet: Resolver<any | null, any, { id: string }>;
    robots: Resolver<any[], any, { filter?: any; limit?: number; offset?: number }>;
    robot: Resolver<any | null, any, { id: string }>;
    tasks: Resolver<any[], any, { filter?: any; limit?: number; offset?: number }>;
    task: Resolver<any | null, any, { id: string }>;
    runs: Resolver<any[], any, { filter?: any; limit?: number; offset?: number }>;
    bellows: Resolver<any, any, { fleetId?: string; timeRange?: any }>;
    edicts: Resolver<any[], any, { filter?: any; limit?: number; offset?: number }>;
  };
  
  Mutation: {
    forgeSepulka: Resolver<any, any, { input: any }>;
    deleteSepulka: Resolver<any, any, { id: string }>;
    castIngot: Resolver<any, any, { sepulkaId: string }>;
    temperIngot: Resolver<any, any, { ingotId: string; input: any }>;
    quenchToFleet: Resolver<any, any, { ingotId: string; fleetId: string; rolloutPercent?: number }>;
    recallFleet: Resolver<any, any, { fleetId: string; toVersion: string }>;
    dispatchTask: Resolver<any, any, { input: any }>;
    cancelTask: Resolver<any, any, { taskId: string }>;
    updateRobotStatus: Resolver<any, any, { robotId: string; status: string }>;
    emergencyStop: Resolver<any, any, { fleetId: string }>;
    addEdict: Resolver<any, any, { input: any }>;
    updateEdict: Resolver<any, any, { id: string; input: any }>;
    deactivateEdict: Resolver<any, any, { id: string }>;
    login: Resolver<any, any, { credentials: any }>;
    refreshToken: Resolver<any, any, { refreshToken: string }>;
    logout: Resolver<any, any, {}>;
  };
  
  Subscription: {
    bellowsStream: Resolver<any>;
    taskUpdates: Resolver<any>;
    robotStatus: Resolver<any>;
    policyBreaches: Resolver<any>;
  };
  
  Sepulka: {
    createdAt: Resolver<any>;
    updatedAt: Resolver<any>;
    pattern: Resolver<any>;
    alloys: Resolver<any[]>;
    ingots: Resolver<any[]>;
    createdBy: Resolver<any>;
  };
  
  Ingot: {
    sepulkaId: Resolver<any>;
    buildHash: Resolver<any>;
    createdAt: Resolver<any>;
    artifacts: Resolver<any[]>;
  };
  
  Alloy: any;
  Fleet: any;
  Robot: any;
  Task: any;
  Run: any;
  Smith: any;
}
