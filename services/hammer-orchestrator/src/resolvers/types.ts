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
    sepulkas: Resolver<any[]>;
    sepulka: Resolver<any | null>;
    alloys: Resolver<any[]>;
    alloy: Resolver<any | null>;
    patterns: Resolver<any[]>;
    pattern: Resolver<any | null>;
    fleets: Resolver<any[]>;
    fleet: Resolver<any | null>;
    robots: Resolver<any[]>;
    robot: Resolver<any | null>;
    tasks: Resolver<any[]>;
    task: Resolver<any | null>;
    runs: Resolver<any[]>;
    bellows: Resolver<any>;
    edicts: Resolver<any[]>;
  };
  
  Mutation: {
    forgeSepulka: Resolver<any>;
    castIngot: Resolver<any>;
    temperIngot: Resolver<any>;
    quenchToFleet: Resolver<any>;
    recallFleet: Resolver<any>;
    dispatchTask: Resolver<any>;
    cancelTask: Resolver<any>;
    updateRobotStatus: Resolver<any>;
    emergencyStop: Resolver<any>;
    addEdict: Resolver<any>;
    updateEdict: Resolver<any>;
    deactivateEdict: Resolver<any>;
  };
  
  Subscription: {
    bellowsStream: Resolver<any>;
    taskUpdates: Resolver<any>;
    robotStatus: Resolver<any>;
    policyBreaches: Resolver<any>;
  };
  
  Sepulka: {
    pattern: Resolver<any>;
    alloys: Resolver<any[]>;
    ingots: Resolver<any[]>;
    createdBy: Resolver<any>;
  };
  
  Alloy: any;
  Fleet: any;
  Robot: any;
  Task: any;
  Run: any;
  Smith: any;
}
