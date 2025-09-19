import type { Context } from '../context';
import { requirePermission } from '../context';
import { NotFoundError, ServiceError, ValidationError } from '../errors';
import { Permission, TaskStatus, TaskPriority } from '@sepulki/shared-types';

export const taskResolvers = {
  Query: {
    async tasks(parent: any, args: any, context: Context) {
      await requirePermission(context, Permission.VIEW_TASKS);
      
      const { filter, limit = 50, offset = 0 } = args;
      let query = 'SELECT * FROM tasks WHERE 1=1';
      const params: any[] = [];
      let paramIndex = 1;

      // Apply filters
      if (filter?.status) {
        query += ` AND status = $${paramIndex}`;
        params.push(filter.status);
        paramIndex++;
      }

      if (filter?.type) {
        query += ` AND type = $${paramIndex}`;
        params.push(filter.type);
        paramIndex++;
      }

      if (filter?.priority) {
        query += ` AND priority = $${paramIndex}`;
        params.push(filter.priority);
        paramIndex++;
      }

      if (filter?.createdBy) {
        query += ` AND created_by = $${paramIndex}`;
        params.push(filter.createdBy);
        paramIndex++;
      }

      // Apply pagination
      query += ` ORDER BY created_at DESC LIMIT $${paramIndex} OFFSET $${paramIndex + 1}`;
      params.push(limit, offset);

      try {
        const result = await context.db.query(query, params);
        return result.rows;
      } catch (error) {
        throw new ServiceError('database', `Failed to fetch tasks: ${error}`);
      }
    },

    async task(parent: any, { id }: { id: string }, context: Context) {
      await requirePermission(context, Permission.VIEW_TASKS);
      
      const task = await context.dataloaders.task.load(id);
      if (!task) {
        throw new NotFoundError('Task', id);
      }
      return task;
    },

    async runs(parent: any, args: any, context: Context) {
      await requirePermission(context, Permission.VIEW_TASKS);
      
      const { taskId, robotId, limit = 50, offset = 0 } = args;
      let query = 'SELECT * FROM runs WHERE 1=1';
      const params: any[] = [];
      let paramIndex = 1;

      if (taskId) {
        query += ` AND task_id = $${paramIndex}`;
        params.push(taskId);
        paramIndex++;
      }

      if (robotId) {
        query += ` AND robot_id = $${paramIndex}`;
        params.push(robotId);
        paramIndex++;
      }

      query += ` ORDER BY started_at DESC LIMIT $${paramIndex} OFFSET $${paramIndex + 1}`;
      params.push(limit, offset);

      try {
        const result = await context.db.query(query, params);
        return result.rows;
      } catch (error) {
        throw new ServiceError('database', `Failed to fetch runs: ${error}`);
      }
    }
  },

  Mutation: {
    async dispatchTask(parent: any, { fleetId, input }: any, context: Context) {
      const { smith } = await requirePermission(context, Permission.CREATE_TASK);

      const { name, description, type, parameters, priority = TaskPriority.NORMAL, scheduledAt } = input;

      // Validate input
      if (!name || name.trim().length === 0) {
        throw new ValidationError('Task name is required', 'name');
      }

      if (!type) {
        throw new ValidationError('Task type is required', 'type');
      }

      try {
        // Start transaction
        const client = await context.db.connect();
        await client.query('BEGIN');

        try {
          // Create task
          const taskQuery = `
            INSERT INTO tasks (name, description, type, parameters, priority, status, scheduled_at, created_by)
            VALUES ($1, $2, $3, $4, $5, $6, $7, $8)
            RETURNING *
          `;
          const taskResult = await client.query(taskQuery, [
            name,
            description,
            type,
            parameters || {},
            priority,
            TaskStatus.PENDING,
            scheduledAt || new Date(),
            smith.id
          ]);

          const task = taskResult.rows[0];

          // Get available robots from fleet
          const robotsQuery = `
            SELECT * FROM robots 
            WHERE fleet_id = $1 AND status IN ('IDLE', 'WORKING') 
            ORDER BY CASE WHEN status = 'IDLE' THEN 0 ELSE 1 END, last_seen DESC
          `;
          const robotsResult = await client.query(robotsQuery, [fleetId]);

          if (robotsResult.rows.length === 0) {
            throw new ValidationError('No available robots in fleet', 'fleetId');
          }

          // For now, assign to the first available robot
          // TODO: Implement smart assignment via Choreo dispatch service
          const assignedRobot = robotsResult.rows[0];

          // Create task assignment
          await client.query(
            'INSERT INTO task_robots (task_id, robot_id, assigned_at) VALUES ($1, $2, NOW())',
            [task.id, assignedRobot.id]
          );

          // Update task status
          await client.query(
            'UPDATE tasks SET status = $1, updated_at = NOW() WHERE id = $2',
            [TaskStatus.ASSIGNED, task.id]
          );

          // Update robot status if idle
          if (assignedRobot.status === 'IDLE') {
            await client.query(
              'UPDATE robots SET status = $1, updated_at = NOW() WHERE id = $2',
              ['WORKING', assignedRobot.id]
            );
          }

          await client.query('COMMIT');
          client.release();

          // Clear caches
          context.dataloaders.task.clear(task.id);
          context.dataloaders.robot.clear(assignedRobot.id);

          // Publish task assignment
          await context.redis.publish('task:assigned', JSON.stringify({
            taskId: task.id,
            robotId: assignedRobot.id,
            fleetId: fleetId,
            assignedBy: smith.id,
            timestamp: new Date().toISOString()
          }));

          return {
            task: { ...task, status: TaskStatus.ASSIGNED },
            assignments: [{
              taskId: task.id,
              robotId: assignedRobot.id,
              assignedAt: new Date(),
              estimatedDuration: 0, // TODO: Calculate from task parameters
              confidence: 1.0
            }],
            errors: []
          };
        } catch (error) {
          await client.query('ROLLBACK');
          client.release();
          throw error;
        }
      } catch (error) {
        throw new ServiceError('database', `Failed to dispatch task: ${error}`);
      }
    },

    async cancelTask(parent: any, { taskId }: { taskId: string }, context: Context) {
      await requirePermission(context, Permission.CANCEL_TASK);

      try {
        // Update task status
        const result = await context.db.query(
          `UPDATE tasks 
           SET status = $1, updated_at = NOW() 
           WHERE id = $2 AND status IN ('PENDING', 'ASSIGNED', 'IN_PROGRESS')
           RETURNING *`,
          [TaskStatus.CANCELLED, taskId]
        );

        if (result.rows.length === 0) {
          throw new NotFoundError('Task', taskId);
        }

        const task = result.rows[0];

        // Cancel any active runs
        await context.db.query(
          `UPDATE runs 
           SET status = 'CANCELLED', completed_at = NOW() 
           WHERE task_id = $1 AND status IN ('PENDING', 'RUNNING')`,
          [taskId]
        );

        // Free up assigned robots
        await context.db.query(
          `UPDATE robots 
           SET status = 'IDLE', updated_at = NOW() 
           WHERE id IN (
             SELECT robot_id FROM task_robots WHERE task_id = $1
           ) AND status = 'WORKING'`,
          [taskId]
        );

        // Clear cache
        context.dataloaders.task.clear(taskId);

        // Publish cancellation
        await context.redis.publish('task:cancelled', JSON.stringify({
          taskId,
          timestamp: new Date().toISOString()
        }));

        return task;
      } catch (error) {
        throw new ServiceError('database', `Failed to cancel task: ${error}`);
      }
    }
  },

  Subscription: {
    taskUpdates: {
      subscribe: async (parent: any, { fleetId }: any, context: Context) => {
        await requirePermission(context, Permission.VIEW_TASKS);
        
        // TODO: Implement subscription with Redis pub/sub
        throw new ServiceError('subscriptions', 'Real-time subscriptions not yet implemented');
      }
    }
  },

  Task: {
    async assignedRobots(parent: any, args: any, context: Context) {
      const result = await context.db.query(
        `SELECT r.* FROM robots r 
         JOIN task_robots tr ON r.id = tr.robot_id 
         WHERE tr.task_id = $1`,
        [parent.id]
      );
      return result.rows;
    },

    async runs(parent: any, args: any, context: Context) {
      const result = await context.db.query(
        'SELECT * FROM runs WHERE task_id = $1 ORDER BY started_at DESC',
        [parent.id]
      );
      return result.rows;
    },

    async createdBy(parent: any, args: any, context: Context) {
      return await context.dataloaders.smith.load(parent.created_by);
    }
  },

  Run: {
    async metrics(parent: any, args: any, context: Context) {
      // TODO: Fetch detailed metrics from telemetry service
      return parent.metrics || null;
    }
  }
};
