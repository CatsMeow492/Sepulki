import type { Context } from '../context';
import { requirePermission } from '../context';
import { NotFoundError, ServiceError, ValidationError } from '../errors';
import { Permission, RobotStatus } from '@sepulki/shared-types';

export const fleetResolvers = {
  Query: {
    async fleets(parent: any, args: any, context: Context) {
      await requirePermission(context, Permission.VIEW_FLEET);
      
      const { filter, limit = 50, offset = 0 } = args;
      let query = 'SELECT * FROM fleets WHERE 1=1';
      const params: any[] = [];
      let paramIndex = 1;

      // Apply filters
      if (filter?.status) {
        query += ` AND status = $${paramIndex}`;
        params.push(filter.status);
        paramIndex++;
      }

      if (filter?.locusId) {
        query += ` AND locus_id = $${paramIndex}`;
        params.push(filter.locusId);
        paramIndex++;
      }

      // Apply pagination
      query += ` ORDER BY name ASC LIMIT $${paramIndex} OFFSET $${paramIndex + 1}`;
      params.push(limit, offset);

      try {
        const result = await context.db.query(query, params);
        return result.rows;
      } catch (error) {
        throw new ServiceError('database', `Failed to fetch fleets: ${error}`);
      }
    },

    async fleet(parent: any, { id }: { id: string }, context: Context) {
      await requirePermission(context, Permission.VIEW_FLEET);
      
      const fleet = await context.dataloaders.fleet.load(id);
      if (!fleet) {
        throw new NotFoundError('Fleet', id);
      }
      return fleet;
    },

    async robots(parent: any, args: any, context: Context) {
      await requirePermission(context, Permission.VIEW_ROBOTS);
      
      const { fleetId, status, limit = 50, offset = 0 } = args;
      let query = 'SELECT * FROM robots WHERE 1=1';
      const params: any[] = [];
      let paramIndex = 1;

      if (fleetId) {
        query += ` AND fleet_id = $${paramIndex}`;
        params.push(fleetId);
        paramIndex++;
      }

      if (status) {
        query += ` AND status = $${paramIndex}`;
        params.push(status);
        paramIndex++;
      }

      query += ` ORDER BY name ASC LIMIT $${paramIndex} OFFSET $${paramIndex + 1}`;
      params.push(limit, offset);

      try {
        const result = await context.db.query(query, params);
        return result.rows;
      } catch (error) {
        throw new ServiceError('database', `Failed to fetch robots: ${error}`);
      }
    },

    async robot(parent: any, { id }: { id: string }, context: Context) {
      await requirePermission(context, Permission.VIEW_ROBOTS);
      
      const robot = await context.dataloaders.robot.load(id);
      if (!robot) {
        throw new NotFoundError('Robot', id);
      }
      return robot;
    }
  },

  Mutation: {
    async updateRobotStatus(parent: any, { robotId, status }: any, context: Context) {
      await requirePermission(context, Permission.MANAGE_ROBOTS);
      
      if (!Object.values(RobotStatus).includes(status)) {
        throw new ValidationError(`Invalid robot status: ${status}`, 'status');
      }

      try {
        const result = await context.db.query(
          `UPDATE robots 
           SET status = $1, updated_at = NOW() 
           WHERE id = $2 
           RETURNING *`,
          [status, robotId]
        );

        if (result.rows.length === 0) {
          throw new NotFoundError('Robot', robotId);
        }

        const robot = result.rows[0];

        // Clear cache
        context.dataloaders.robot.clear(robotId);

        // Publish status change
        await context.redis.publish('robot:status', JSON.stringify({
          robotId,
          status,
          timestamp: new Date().toISOString()
        }));

        return robot;
      } catch (error) {
        throw new ServiceError('database', `Failed to update robot status: ${error}`);
      }
    },

    async emergencyStop(parent: any, { fleetId }: { fleetId: string }, context: Context) {
      await requirePermission(context, Permission.EMERGENCY_STOP);

      try {
        // Update all robots in fleet to MAINTENANCE status
        const result = await context.db.query(
          `UPDATE robots 
           SET status = 'MAINTENANCE', updated_at = NOW() 
           WHERE fleet_id = $1 
           RETURNING *`,
          [fleetId]
        );

        // Update fleet status
        await context.db.query(
          `UPDATE fleets 
           SET status = 'MAINTENANCE', updated_at = NOW() 
           WHERE id = $1`,
          [fleetId]
        );

        // Cancel all active tasks for this fleet
        await context.db.query(
          `UPDATE tasks 
           SET status = 'CANCELLED', updated_at = NOW() 
           WHERE id IN (
             SELECT DISTINCT t.id 
             FROM tasks t 
             JOIN task_robots tr ON t.id = tr.task_id 
             JOIN robots r ON tr.robot_id = r.id 
             WHERE r.fleet_id = $1 AND t.status IN ('PENDING', 'ASSIGNED', 'IN_PROGRESS')
           )`,
          [fleetId]
        );

        // Clear caches
        context.dataloaders.fleet.clear(fleetId);
        result.rows.forEach(robot => {
          context.dataloaders.robot.clear(robot.id);
        });

        // Publish emergency stop event
        await context.redis.publish('fleet:emergency_stop', JSON.stringify({
          fleetId,
          timestamp: new Date().toISOString(),
          affectedRobots: result.rows.length
        }));

        // Get updated fleet
        const fleet = await context.dataloaders.fleet.load(fleetId);
        return fleet;
      } catch (error) {
        throw new ServiceError('database', `Failed to execute emergency stop: ${error}`);
      }
    }
  },

  Subscription: {
    robotStatus: {
      subscribe: async (parent: any, { robotId }: any, context: Context) => {
        await requirePermission(context, Permission.VIEW_ROBOTS);
        
        // TODO: Implement subscription with Redis pub/sub
        throw new ServiceError('subscriptions', 'Real-time subscriptions not yet implemented');
      }
    }
  },

  Fleet: {
    async locus(parent: any, args: any, context: Context) {
      if (!parent.locus_id) return null;
      
      const result = await context.db.query(
        'SELECT * FROM loci WHERE id = $1',
        [parent.locus_id]
      );
      return result.rows[0] || null;
    },

    async robots(parent: any, args: any, context: Context) {
      return await context.dataloaders.robotsByFleet.load(parent.id);
    },

    async activeTask(parent: any, args: any, context: Context) {
      if (!parent.active_task_id) return null;
      return await context.dataloaders.task.load(parent.active_task_id);
    },

    async constraints(parent: any, args: any, context: Context) {
      if (!parent.constraint_ids || parent.constraint_ids.length === 0) return [];
      
      const result = await context.db.query(
        'SELECT * FROM edicts WHERE id = ANY($1)',
        [parent.constraint_ids]
      );
      return result.rows;
    },

    async telemetry(parent: any, args: any, context: Context) {
      // TODO: Implement telemetry stream from Bellows service
      return {
        fleetId: parent.id,
        metrics: [],
        events: [],
        realTime: false
      };
    }
  },

  Robot: {
    async currentIngot(parent: any, args: any, context: Context) {
      if (!parent.current_ingot_id) return null;
      
      const result = await context.db.query(
        'SELECT * FROM ingots WHERE id = $1',
        [parent.current_ingot_id]
      );
      return result.rows[0] || null;
    },

    async pose(parent: any, args: any, context: Context) {
      // TODO: Get latest pose from telemetry service
      return parent.last_pose || null;
    }
  }
};
