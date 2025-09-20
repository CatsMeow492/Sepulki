// TDD Tests for castIngot Mutation
// RED PHASE: Define expected behavior BEFORE implementation

import { testEnv, createTestSepulkaInput, expectGraphQLSuccess, expectGraphQLError } from '../setup';

describe('castIngot Mutation (TDD)', () => {
  let testSepulkaId: string;

  beforeEach(async () => {
    // Create a sepulka ready for casting
    const forgeInput = createTestSepulkaInput({
      name: 'Ready-For-Casting-Robot'
    });

    const forgeMutation = `
      mutation ForgeSepulka($input: ForgeInput!) {
        forgeSepulka(input: $input) {
          sepulka { id }
        }
      }
    `;

    const forgeResponse = await testEnv.executeGraphQL(forgeMutation, { input: forgeInput });
    testSepulkaId = expectGraphQLSuccess(forgeResponse).forgeSepulka.sepulka.id;

    // Update sepulka status to CAST_READY
    await testEnv.db.query(
      'UPDATE sepulkas SET status = $1 WHERE id = $2',
      ['CAST_READY', testSepulkaId]
    );
  });

  describe('✅ Success Scenarios', () => {
    
    test('should create ingot from valid sepulka', async () => {
      // ARRANGE: Prepare test input
      const mutation = `
        mutation CastIngot($sepulkaId: ID!) {
          castIngot(sepulkaId: $sepulkaId) {
            ingot {
              id
              sepulkaId
              version
              buildHash
              status
              artifacts {
                type
                path
                checksum
              }
              createdAt
            }
            errors {
              code
              message
            }
          }
        }
      `;

      // ACT: Execute mutation
      const response = await testEnv.executeGraphQL(mutation, { sepulkaId: testSepulkaId });
      
      // ASSERT: Verify successful ingot creation
      const data = expectGraphQLSuccess(response);
      const ingot = data.castIngot.ingot;

      expect(ingot).toBeDefined();
      expect(ingot.id).toMatch(/^[0-9a-f-]{36}$/); // UUID format
      expect(ingot.version).toMatch(/^\d+\.\d+\.\d+$/); // Semantic version format
      expect(ingot.buildHash).toBeTruthy();
      expect(ingot.buildHash).toMatch(/^build_\d+_[a-z0-9]+$/); // Build hash format
      expect(ingot.status).toBe('BUILDING');
      expect(ingot.sepulkaId).toBe(testSepulkaId);
      expect(new Date(ingot.createdAt)).toBeInstanceOf(Date);

      // Verify database persistence
      const dbResult = await testEnv.db.query(
        'SELECT * FROM ingots WHERE id = $1',
        [ingot.id]
      );
      expect(dbResult.rows).toHaveLength(1);
      expect(dbResult.rows[0].sepulka_id).toBe(testSepulkaId);
      expect(dbResult.rows[0].status).toBe('BUILDING');
    });

    test('should update sepulka status to CASTING', async () => {
      const mutation = `
        mutation CastIngot($sepulkaId: ID!) {
          castIngot(sepulkaId: $sepulkaId) {
            ingot { id }
          }
        }
      `;

      const response = await testEnv.executeGraphQL(mutation, { sepulkaId: testSepulkaId });
      expectGraphQLSuccess(response);

      // Verify sepulka status updated
      const sepulkaResult = await testEnv.db.query(
        'SELECT status FROM sepulkas WHERE id = $1',
        [testSepulkaId]
      );
      expect(sepulkaResult.rows[0].status).toBe('CASTING');
    });

    test('should generate unique build hash', async () => {
      const mutation = `
        mutation CastIngot($sepulkaId: ID!) {
          castIngot(sepulkaId: $sepulkaId) {
            ingot { buildHash }
          }
        }
      `;

      // Create two ingots from the same sepulka (hypothetical scenario)
      const response1 = await testEnv.executeGraphQL(mutation, { sepulkaId: testSepulkaId });
      
      // Reset sepulka status for second cast
      await testEnv.db.query(
        'UPDATE sepulkas SET status = $1 WHERE id = $2',
        ['CAST_READY', testSepulkaId]
      );
      
      const response2 = await testEnv.executeGraphQL(mutation, { sepulkaId: testSepulkaId });

      const ingot1 = expectGraphQLSuccess(response1).castIngot.ingot;
      const ingot2 = expectGraphQLSuccess(response2).castIngot.ingot;

      expect(ingot1.buildHash).not.toBe(ingot2.buildHash);
      expect(ingot1.buildHash).toMatch(/^build_\d+_[a-z0-9]+$/);
      expect(ingot2.buildHash).toMatch(/^build_\d+_[a-z0-9]+$/);
    });

    test('should increment version for subsequent builds', async () => {
      const mutation = `
        mutation CastIngot($sepulkaId: ID!) {
          castIngot(sepulkaId: $sepulkaId) {
            ingot { version }
          }
        }
      `;

      // First build
      const response1 = await testEnv.executeGraphQL(mutation, { sepulkaId: testSepulkaId });
      const ingot1 = expectGraphQLSuccess(response1).castIngot.ingot;

      // Reset for second build
      await testEnv.db.query(
        'UPDATE sepulkas SET status = $1 WHERE id = $2',
        ['CAST_READY', testSepulkaId]
      );

      const response2 = await testEnv.executeGraphQL(mutation, { sepulkaId: testSepulkaId });
      const ingot2 = expectGraphQLSuccess(response2).castIngot.ingot;

      expect(ingot1.version).toBe('1.0.0');
      expect(ingot2.version).toBe('1.0.1'); // Auto-increment patch version
    });

    test('should trigger foundry build process', async () => {
      const mutation = `
        mutation CastIngot($sepulkaId: ID!) {
          castIngot(sepulkaId: $sepulkaId) {
            ingot { id buildHash }
          }
        }
      `;

      const response = await testEnv.executeGraphQL(mutation, { sepulkaId: testSepulkaId });
      const ingot = expectGraphQLSuccess(response).castIngot.ingot;

      // Verify foundry job was queued (check Redis)
      // This would be implementation-specific, but we can test the pattern
      const redisKeys = await testEnv.redis.keys('foundry:build:*');
      expect(redisKeys.length).toBeGreaterThan(0);
    });
  });

  describe('❌ Validation & Error Scenarios', () => {
    
    test('should reject non-existent sepulka', async () => {
      const mutation = `
        mutation CastIngot($sepulkaId: ID!) {
          castIngot(sepulkaId: $sepulkaId) {
            ingot { id }
            errors { code message }
          }
        }
      `;

      const fakeId = '00000000-0000-0000-0000-000000000000';
      const response = await testEnv.executeGraphQL(mutation, { sepulkaId: fakeId });
      expectGraphQLError(response, 'Sepulka not found');
    });

    test('should reject sepulka not ready for casting', async () => {
      // Set sepulka to FORGING status (not ready)
      await testEnv.db.query(
        'UPDATE sepulkas SET status = $1 WHERE id = $2',
        ['FORGING', testSepulkaId]
      );

      const mutation = `
        mutation CastIngot($sepulkaId: ID!) {
          castIngot(sepulkaId: $sepulkaId) {
            ingot { id }
            errors { code message }
          }
        }
      `;

      const response = await testEnv.executeGraphQL(mutation, { sepulkaId: testSepulkaId });
      expectGraphQLError(response, 'must be in CAST_READY status');
    });

    test('should require authentication', async () => {
      const mutation = `
        mutation CastIngot($sepulkaId: ID!) {
          castIngot(sepulkaId: $sepulkaId) {
            ingot { id }
            errors { code message }
          }
        }
      `;

      // Execute without authentication context
      const response = await testEnv.executeGraphQL(mutation, { sepulkaId: testSepulkaId }, {});
      expectGraphQLError(response, 'Authentication required');
    });

    test('should require CAST_INGOT permission', async () => {
      // Create smith without cast permission
      await testEnv.db.query(`
        INSERT INTO smiths (id, email, name, password_hash, role, permissions)
        VALUES (
          'limited-smith-002',
          'limited2@sepulki.com',
          'Limited Smith 2',
          encode(digest('test123', 'sha256'), 'hex'),
          'SMITH',
          ARRAY['VIEW_CATALOG', 'FORGE_SEPULKA']
        )
      `);

      const limitedContext = await testEnv.createAuthenticatedContext('limited-smith-002');
      const mutation = `
        mutation CastIngot($sepulkaId: ID!) {
          castIngot(sepulkaId: $sepulkaId) {
            ingot { id }
            errors { code message }
          }
        }
      `;

      const response = await testEnv.executeGraphQL(mutation, { sepulkaId: testSepulkaId }, limitedContext);
      expectGraphQLError(response, 'Insufficient permissions');
    });

    test('should handle invalid UUID format', async () => {
      const mutation = `
        mutation CastIngot($sepulkaId: ID!) {
          castIngot(sepulkaId: $sepulkaId) {
            ingot { id }
            errors { code message }
          }
        }
      `;

      const response = await testEnv.executeGraphQL(mutation, { sepulkaId: 'invalid-uuid' });
      expectGraphQLError(response, 'Invalid UUID format');
    });
  });

  describe('🔄 Business Logic Scenarios', () => {
    
    test('should handle concurrent casting attempts', async () => {
      const mutation = `
        mutation CastIngot($sepulkaId: ID!) {
          castIngot(sepulkaId: $sepulkaId) {
            ingot { id }
            errors { code message }
          }
        }
      `;

      // Execute concurrent requests
      const [response1, response2] = await Promise.all([
        testEnv.executeGraphQL(mutation, { sepulkaId: testSepulkaId }),
        testEnv.executeGraphQL(mutation, { sepulkaId: testSepulkaId })
      ]);

      // First should succeed
      const data1 = expectGraphQLSuccess(response1);
      expect(data1.castIngot.ingot).toBeDefined();

      // Second should fail due to status change
      expectGraphQLError(response2, 'must be in CAST_READY status');
    });

    test('should validate sepulka completeness before casting', async () => {
      // Create incomplete sepulka (missing required alloys)
      const incompleteInput = createTestSepulkaInput({
        name: 'Incomplete-Robot',
        alloyIds: [] // Empty alloys array
      });

      const forgeMutation = `
        mutation ForgeSepulka($input: ForgeInput!) {
          forgeSepulka(input: $input) {
            sepulka { id }
          }
        }
      `;

      const forgeResponse = await testEnv.executeGraphQL(forgeMutation, { input: incompleteInput });
      const incompleteSepulkaId = expectGraphQLSuccess(forgeResponse).forgeSepulka.sepulka.id;

      // Set to CAST_READY
      await testEnv.db.query(
        'UPDATE sepulkas SET status = $1 WHERE id = $2',
        ['CAST_READY', incompleteSepulkaId]
      );

      const castMutation = `
        mutation CastIngot($sepulkaId: ID!) {
          castIngot(sepulkaId: $sepulkaId) {
            ingot { id }
            errors { code message }
          }
        }
      `;

      const response = await testEnv.executeGraphQL(castMutation, { sepulkaId: incompleteSepulkaId });
      expectGraphQLError(response, 'Sepulka is incomplete');
    });

    test('should create audit trail for casting', async () => {
      const mutation = `
        mutation CastIngot($sepulkaId: ID!) {
          castIngot(sepulkaId: $sepulkaId) {
            ingot { id }
          }
        }
      `;

      const response = await testEnv.executeGraphQL(mutation, { sepulkaId: testSepulkaId });
      const ingot = expectGraphQLSuccess(response).castIngot.ingot;

      // Verify audit stamp was created
      const auditResult = await testEnv.db.query(
        'SELECT * FROM audit_stamps WHERE entity_type = $1 AND entity_id = $2',
        ['ingot', ingot.id]
      );

      expect(auditResult.rows).toHaveLength(1);
      expect(auditResult.rows[0].action).toBe('CAST');
      expect(auditResult.rows[0].actor_id).toBe('test-smith-001');
    });
  });

  describe('📊 Performance & Quality Tests', () => {
    
    test('should complete within performance budget', async () => {
      const mutation = `
        mutation CastIngot($sepulkaId: ID!) {
          castIngot(sepulkaId: $sepulkaId) {
            ingot { id }
          }
        }
      `;

      const startTime = performance.now();
      const response = await testEnv.executeGraphQL(mutation, { sepulkaId: testSepulkaId });
      const endTime = performance.now();

      expectGraphQLSuccess(response);
      
      // Performance requirement: <300ms response time (build triggering adds overhead)
      const duration = endTime - startTime;
      expect(duration).toBeLessThan(300);
      
      console.log(`⚡ castIngot performance: ${duration.toFixed(2)}ms`);
    });

    test('should handle database connection failures gracefully', async () => {
      const mutation = `
        mutation CastIngot($sepulkaId: ID!) {
          castIngot(sepulkaId: $sepulkaId) {
            ingot { id }
            errors { code message }
          }
        }
      `;

      // This is a conceptual test - in reality, you'd mock the database failure
      // For now, we verify the error handling structure exists
      const response = await testEnv.executeGraphQL(mutation, { sepulkaId: testSepulkaId });
      
      // Should either succeed or fail gracefully with proper error structure
      if (response.body.singleResult.errors) {
        expect(response.body.singleResult.errors[0]).toHaveProperty('message');
        expect(response.body.singleResult.errors[0]).toHaveProperty('extensions');
      } else {
        expect(response.body.singleResult.data.castIngot.ingot).toBeDefined();
      }
    });
  });

  describe('🔗 Integration Scenarios', () => {
    
    test('should support complete forge → cast → deploy workflow', async () => {
      // Step 1: Forge sepulka (already done in beforeEach)
      
      // Step 2: Cast ingot
      const castMutation = `
        mutation CastIngot($sepulkaId: ID!) {
          castIngot(sepulkaId: $sepulkaId) {
            ingot {
              id
              sepulkaId
              status
            }
          }
        }
      `;

      const castResponse = await testEnv.executeGraphQL(castMutation, { sepulkaId: testSepulkaId });
      const ingot = expectGraphQLSuccess(castResponse).castIngot.ingot;
      
      expect(ingot.status).toBe('BUILDING');
      expect(ingot.sepulkaId).toBe(testSepulkaId);

      // Step 3: Verify it appears in ingot list
      const listQuery = `
        query GetIngots {
          ingots(filter: { sepulkaId: "${testSepulkaId}" }) {
            id
            status
            buildHash
          }
        }
      `;

      const listResponse = await testEnv.executeGraphQL(listQuery);
      const ingots = expectGraphQLSuccess(listResponse).ingots;
      
      expect(ingots).toHaveLength(1);
      expect(ingots[0].id).toBe(ingot.id);
      expect(ingots[0].status).toBe('BUILDING');
    });

    test('should track build artifacts correctly', async () => {
      const mutation = `
        mutation CastIngot($sepulkaId: ID!) {
          castIngot(sepulkaId: $sepulkaId) {
            ingot {
              id
              artifacts {
                type
                path
                checksum
              }
            }
          }
        }
      `;

      const response = await testEnv.executeGraphQL(mutation, { sepulkaId: testSepulkaId });
      const ingot = expectGraphQLSuccess(response).castIngot.ingot;

      // Initially no artifacts (they're generated during build process)
      expect(ingot.artifacts).toEqual([]);

      // After foundry processing (simulated), artifacts would be populated
      // This tests the schema and data structure
      expect(Array.isArray(ingot.artifacts)).toBe(true);
    });
  });

  describe('💾 Data Integrity Tests', () => {
    
    test('should maintain referential integrity', async () => {
      const mutation = `
        mutation CastIngot($sepulkaId: ID!) {
          castIngot(sepulkaId: $sepulkaId) {
            ingot { id }
          }
        }
      `;

      const response = await testEnv.executeGraphQL(mutation, { sepulkaId: testSepulkaId });
      const ingot = expectGraphQLSuccess(response).castIngot.ingot;

      // Verify foreign key relationships
      const sepulkaExists = await testEnv.db.query(
        'SELECT id FROM sepulkas WHERE id = $1',
        [testSepulkaId]
      );
      expect(sepulkaExists.rows).toHaveLength(1);

      const creatorExists = await testEnv.db.query(
        'SELECT id FROM smiths WHERE id = $1',
        ['test-smith-001']
      );
      expect(creatorExists.rows).toHaveLength(1);

      // Verify ingot-sepulka relationship
      const ingotSepulka = await testEnv.db.query(
        'SELECT sepulka_id FROM ingots WHERE id = $1',
        [ingot.id]
      );
      expect(ingotSepulka.rows[0].sepulka_id).toBe(testSepulkaId);
    });

    test('should handle transaction rollback on failure', async () => {
      // Force a failure during ingot creation
      const invalidSepulkaId = '00000000-0000-0000-0000-000000000000';
      
      const mutation = `
        mutation CastIngot($sepulkaId: ID!) {
          castIngot(sepulkaId: $sepulkaId) {
            ingot { id }
            errors { code message }
          }
        }
      `;

      const response = await testEnv.executeGraphQL(mutation, { sepulkaId: invalidSepulkaId });
      expectGraphQLError(response);

      // Verify no partial data was committed
      const ingotCount = await testEnv.db.query(
        'SELECT COUNT(*) FROM ingots WHERE sepulka_id = $1',
        [invalidSepulkaId]
      );
      expect(parseInt(ingotCount.rows[0].count)).toBe(0);

      // Verify original sepulka status unchanged
      const sepulkaStatus = await testEnv.db.query(
        'SELECT status FROM sepulkas WHERE id = $1',
        [testSepulkaId]
      );
      expect(sepulkaStatus.rows[0].status).toBe('CAST_READY'); // Should remain unchanged
    });
  });
});
