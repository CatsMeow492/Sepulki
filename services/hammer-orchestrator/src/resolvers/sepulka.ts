import type { Context } from '../context';
import { requirePermission } from '../context';
import { NotFoundError, ValidationError, ServiceError } from '../errors';
import { Permission } from '@sepulki/shared-types';

export const sepulkaResolvers = {
  Query: {
    async sepulkas(parent: any, args: any, context: Context) {
      await requirePermission(context, Permission.VIEW_CATALOG);
      
      const { filter, limit = 50, offset = 0 } = args;
      let query = 'SELECT * FROM sepulkas WHERE 1=1';
      const params: any[] = [];
      let paramIndex = 1;

      // Apply filters
      if (filter?.status) {
        query += ` AND status = $${paramIndex}`;
        params.push(filter.status);
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
        throw new ServiceError('database', `Failed to fetch sepulkas: ${error}`);
      }
    },

    async sepulka(parent: any, { id }: { id: string }, context: Context) {
      await requirePermission(context, Permission.VIEW_CATALOG);
      
      const sepulka = await context.dataloaders.sepulka.load(id);
      if (!sepulka) {
        throw new NotFoundError('Sepulka', id);
      }
      return sepulka;
    },
  },

  Mutation: {
    async forgeSepulka(parent: any, { input }: any, context: Context) {
      const { smith } = await requirePermission(context, Permission.FORGE_SEPULKA);

      const { name, description, patternId, alloyIds, parameters } = input;

      // Validate input
      if (!name || name.trim().length === 0) {
        throw new ValidationError('Sepulka name is required', 'name');
      }

      if (!alloyIds || alloyIds.length === 0) {
        throw new ValidationError('At least one alloy is required', 'alloyIds');
      }

      try {
        // Start transaction
        const client = await context.db.connect();
        await client.query('BEGIN');

        try {
          // Create sepulka
          const sepulkaQuery = `
            INSERT INTO sepulkas (name, description, pattern_id, status, created_by, parameters)
            VALUES ($1, $2, $3, $4, $5, $6)
            RETURNING *
          `;
          const sepulkaResult = await client.query(sepulkaQuery, [
            name,
            description,
            patternId,
            'FORGING',
            smith.id,
            parameters || {}
          ]);

          const sepulka = sepulkaResult.rows[0];

          // Associate alloys
          for (const alloyId of alloyIds) {
            await client.query(
              'INSERT INTO sepulka_alloys (sepulka_id, alloy_id) VALUES ($1, $2)',
              [sepulka.id, alloyId]
            );
          }

          await client.query('COMMIT');
          client.release();

          // Clear cache
          context.dataloaders.sepulka.clear(sepulka.id);

          return {
            sepulka,
            errors: []
          };
        } catch (error) {
          await client.query('ROLLBACK');
          client.release();
          throw error;
        }
      } catch (error) {
        throw new ServiceError('database', `Failed to forge sepulka: ${error}`);
      }
    },

    async castIngot(parent: any, { sepulkaId }: { sepulkaId: string }, context: Context) {
      const { smith } = await requirePermission(context, Permission.CAST_INGOT);

      // Verify sepulka exists and is ready
      const sepulka = await context.dataloaders.sepulka.load(sepulkaId);
      if (!sepulka) {
        throw new NotFoundError('Sepulka', sepulkaId);
      }

      if (sepulka.status !== 'CAST_READY') {
        throw new ValidationError(`Sepulka must be in CAST_READY status, currently: ${sepulka.status}`);
      }

      try {
        // Update sepulka status
        await context.db.query(
          'UPDATE sepulkas SET status = $1, updated_at = NOW() WHERE id = $2',
          ['CASTING', sepulkaId]
        );

        // Create ingot record
        const ingotQuery = `
          INSERT INTO ingots (sepulka_id, status, build_hash, created_by)
          VALUES ($1, $2, $3, $4)
          RETURNING *
        `;
        const buildHash = `build_${Date.now()}_${Math.random().toString(36).substr(2, 9)}`;
        const ingotResult = await context.db.query(ingotQuery, [
          sepulkaId,
          'BUILDING',
          buildHash,
          smith.id
        ]);

        const ingot = ingotResult.rows[0];

        // Trigger foundry build process (async)
        await context.redis.publish('foundry:build', JSON.stringify({
          ingotId: ingot.id,
          sepulkaId: sepulkaId,
          buildHash: buildHash,
          requestedBy: smith.id
        }));

        // Clear cache
        context.dataloaders.sepulka.clear(sepulkaId);

        return {
          ingot,
          errors: []
        };
      } catch (error) {
        throw new ServiceError('foundry', `Failed to cast ingot: ${error}`);
      }
    },

    async temperIngot(parent: any, { ingotId, input }: any, context: Context) {
      const { smith } = await requirePermission(context, Permission.TEMPER_INGOT);

      // TODO: Implement tempering logic
      // This would integrate with the optimization service
      
      throw new ServiceError('temper', 'Tempering service not yet implemented');
    },

    async quenchToFleet(parent: any, { ingotId, fleetId, rolloutPercent = 100 }: any, context: Context) {
      const { smith } = await requirePermission(context, Permission.QUENCH_TO_FLEET);

      // TODO: Implement deployment logic
      // This would coordinate with the deployment service
      
      throw new ServiceError('deployment', 'Deployment service not yet implemented');
    }
  },

  Sepulka: {
    async pattern(parent: any, args: any, context: Context) {
      if (!parent.pattern_id) return null;
      return await context.dataloaders.pattern.load(parent.pattern_id);
    },

    async alloys(parent: any, args: any, context: Context) {
      return await context.dataloaders.alloysBySepulka.load(parent.id);
    },

    async ingots(parent: any, args: any, context: Context) {
      const result = await context.db.query(
        'SELECT * FROM ingots WHERE sepulka_id = $1 ORDER BY created_at DESC',
        [parent.id]
      );
      return result.rows;
    },

    async createdBy(parent: any, args: any, context: Context) {
      return await context.dataloaders.smith.load(parent.created_by);
    }
  }
};
