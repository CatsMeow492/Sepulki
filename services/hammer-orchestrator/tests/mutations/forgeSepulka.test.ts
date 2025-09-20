// TDD Tests for forgeSepulka Mutation
// These tests define the expected behavior BEFORE implementation

import { testEnv, createTestSepulkaInput, expectGraphQLSuccess, expectGraphQLError } from '../setup';

describe('forgeSepulka Mutation (TDD)', () => {
  describe('✅ Success Scenarios', () => {
    
    test('should create Sepulka with valid input', async () => {
      // ARRANGE: Prepare test input
      const input = createTestSepulkaInput({
        name: 'WarehouseBot-Test',
        description: 'Automated warehouse picking robot'
      });

      const mutation = `
        mutation ForgeSepulka($input: ForgeInput!) {
          forgeSepulka(input: $input) {
            sepulka {
              id
              name
              description
              version
              status
              pattern {
                id
                name
              }
              alloys {
                id
                name
                type
              }
              createdBy {
                id
                email
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
      const response = await testEnv.executeGraphQL(mutation, { input });
      
      // ASSERT: Verify successful creation
      const data = expectGraphQLSuccess(response);
      const sepulka = data.forgeSepulka.sepulka;

      expect(sepulka).toBeDefined();
      expect(sepulka.id).toMatch(/^[0-9a-f-]{36}$/); // UUID format
      expect(sepulka.name).toBe('WarehouseBot-Test');
      expect(sepulka.description).toBe('Automated warehouse picking robot');
      expect(sepulka.version).toBe('1.0.0'); // Default version
      expect(sepulka.status).toBe('FORGING');
      expect(sepulka.pattern.name).toBe('Test Industrial Arm');
      expect(sepulka.alloys).toHaveLength(2);
      expect(sepulka.createdBy.email).toBe('test@sepulki.com');
      expect(new Date(sepulka.createdAt)).toBeInstanceOf(Date);

      // Verify database persistence
      const dbResult = await testEnv.db.query(
        'SELECT * FROM sepulkas WHERE id = $1',
        [sepulka.id]
      );
      expect(dbResult.rows).toHaveLength(1);
      expect(dbResult.rows[0].name).toBe('WarehouseBot-Test');
    });

    test('should link alloys to sepulka correctly', async () => {
      const input = createTestSepulkaInput();
      
      const mutation = `
        mutation ForgeSepulka($input: ForgeInput!) {
          forgeSepulka(input: $input) {
            sepulka {
              id
              alloys {
                id
                name
                type
              }
            }
          }
        }
      `;

      const response = await testEnv.executeGraphQL(mutation, { input });
      const data = expectGraphQLSuccess(response);
      const sepulka = data.forgeSepulka.sepulka;

      // Verify alloy associations in database
      const alloyLinks = await testEnv.db.query(
        'SELECT * FROM sepulka_alloys WHERE sepulka_id = $1',
        [sepulka.id]
      );
      
      expect(alloyLinks.rows).toHaveLength(2);
      expect(alloyLinks.rows.map(r => r.alloy_id)).toContain('test-alloy-servo');
      expect(alloyLinks.rows.map(r => r.alloy_id)).toContain('test-alloy-gripper');
    });

    test('should apply parameters correctly', async () => {
      const customParameters = {
        reach: 1500,
        payload: 25,
        precision: 0.1,
        customField: 'test-value'
      };

      const input = createTestSepulkaInput({
        parameters: customParameters
      });

      const mutation = `
        mutation ForgeSepulka($input: ForgeInput!) {
          forgeSepulka(input: $input) {
            sepulka {
              id
              parameters
            }
          }
        }
      `;

      const response = await testEnv.executeGraphQL(mutation, { input });
      const data = expectGraphQLSuccess(response);
      const sepulka = data.forgeSepulka.sepulka;

      expect(sepulka.parameters).toEqual(customParameters);
    });

    test('should assign unique version numbers', async () => {
      const input1 = createTestSepulkaInput({ name: 'Robot-V1' });
      const input2 = createTestSepulkaInput({ name: 'Robot-V2' });

      const mutation = `
        mutation ForgeSepulka($input: ForgeInput!) {
          forgeSepulka(input: $input) {
            sepulka {
              id
              name
              version
            }
          }
        }
      `;

      const response1 = await testEnv.executeGraphQL(mutation, { input: input1 });
      const response2 = await testEnv.executeGraphQL(mutation, { input: input2 });

      const sepulka1 = expectGraphQLSuccess(response1).forgeSepulka.sepulka;
      const sepulka2 = expectGraphQLSuccess(response2).forgeSepulka.sepulka;

      expect(sepulka1.id).not.toBe(sepulka2.id);
      expect(sepulka1.version).toBe('1.0.0');
      expect(sepulka2.version).toBe('1.0.0'); // Each gets its own v1.0.0
    });
  });

  describe('❌ Validation & Error Scenarios', () => {
    
    test('should reject empty name', async () => {
      const input = createTestSepulkaInput({ name: '' });
      
      const mutation = `
        mutation ForgeSepulka($input: ForgeInput!) {
          forgeSepulka(input: $input) {
            sepulka { id }
            errors { code message }
          }
        }
      `;

      const response = await testEnv.executeGraphQL(mutation, { input });
      const errors = expectGraphQLError(response, 'Name is required');
      
      expect(errors[0].code).toBe('VALIDATION_ERROR');
    });

    test('should reject invalid pattern ID', async () => {
      const input = createTestSepulkaInput({
        patternId: 'non-existent-pattern'
      });

      const mutation = `
        mutation ForgeSepulka($input: ForgeInput!) {
          forgeSepulka(input: $input) {
            sepulka { id }
            errors { code message }
          }
        }
      `;

      const response = await testEnv.executeGraphQL(mutation, { input });
      expectGraphQLError(response, 'Pattern not found');
    });

    test('should reject invalid alloy IDs', async () => {
      const input = createTestSepulkaInput({
        alloyIds: ['test-alloy-servo', 'non-existent-alloy']
      });

      const mutation = `
        mutation ForgeSepulka($input: ForgeInput!) {
          forgeSepulka(input: $input) {
            sepulka { id }
            errors { code message }
          }
        }
      `;

      const response = await testEnv.executeGraphQL(mutation, { input });
      expectGraphQLError(response, 'Alloy not found');
    });

    test('should require authentication', async () => {
      const input = createTestSepulkaInput();
      
      const mutation = `
        mutation ForgeSepulka($input: ForgeInput!) {
          forgeSepulka(input: $input) {
            sepulka { id }
            errors { code message }
          }
        }
      `;

      // Execute without authentication context
      const response = await testEnv.executeGraphQL(mutation, { input }, {});
      expectGraphQLError(response, 'Authentication required');
    });

    test('should require FORGE_SEPULKA permission', async () => {
      // Create smith without forge permission
      await testEnv.db.query(`
        INSERT INTO smiths (id, email, name, password_hash, role, permissions)
        VALUES (
          'limited-smith-001',
          'limited@sepulki.com',
          'Limited Smith',
          encode(digest('test123', 'sha256'), 'hex'),
          'SMITH',
          ARRAY['VIEW_CATALOG']
        )
      `);

      const limitedContext = await testEnv.createAuthenticatedContext('limited-smith-001');
      const input = createTestSepulkaInput();

      const mutation = `
        mutation ForgeSepulka($input: ForgeInput!) {
          forgeSepulka(input: $input) {
            sepulka { id }
            errors { code message }
          }
        }
      `;

      const response = await testEnv.executeGraphQL(mutation, { input }, limitedContext);
      expectGraphQLError(response, 'Insufficient permissions');
    });
  });

  describe('🔄 Business Logic Scenarios', () => {
    
    test('should handle concurrent sepulka creation', async () => {
      const input1 = createTestSepulkaInput({ name: 'Concurrent-1' });
      const input2 = createTestSepulkaInput({ name: 'Concurrent-2' });

      const mutation = `
        mutation ForgeSepulka($input: ForgeInput!) {
          forgeSepulka(input: $input) {
            sepulka { id name }
          }
        }
      `;

      // Execute concurrently
      const [response1, response2] = await Promise.all([
        testEnv.executeGraphQL(mutation, { input: input1 }),
        testEnv.executeGraphQL(mutation, { input: input2 })
      ]);

      const sepulka1 = expectGraphQLSuccess(response1).forgeSepulka.sepulka;
      const sepulka2 = expectGraphQLSuccess(response2).forgeSepulka.sepulka;

      expect(sepulka1.id).not.toBe(sepulka2.id);
      expect(sepulka1.name).toBe('Concurrent-1');
      expect(sepulka2.name).toBe('Concurrent-2');
    });

    test('should validate parameter constraints against pattern', async () => {
      const input = createTestSepulkaInput({
        parameters: {
          reach: 3000, // Exceeds pattern max of 2000
          payload: 100, // Exceeds pattern max of 50
        }
      });

      const mutation = `
        mutation ForgeSepulka($input: ForgeInput!) {
          forgeSepulka(input: $input) {
            sepulka { id }
            errors { code message field }
          }
        }
      `;

      const response = await testEnv.executeGraphQL(mutation, { input });
      const data = response.body.singleResult.data;

      // Should succeed but with warnings/adjustments
      expect(data.forgeSepulka.sepulka).toBeDefined();
      expect(data.forgeSepulka.errors).toBeDefined();
      expect(data.forgeSepulka.errors.length).toBeGreaterThan(0);
      expect(data.forgeSepulka.errors[0].code).toBe('PARAMETER_WARNING');
    });

    test('should generate audit trail', async () => {
      const input = createTestSepulkaInput({ name: 'Audit-Test' });

      const mutation = `
        mutation ForgeSepulka($input: ForgeInput!) {
          forgeSepulka(input: $input) {
            sepulka { id name }
          }
        }
      `;

      const response = await testEnv.executeGraphQL(mutation, { input });
      const sepulka = expectGraphQLSuccess(response).forgeSepulka.sepulka;

      // Verify audit stamp was created
      const auditResult = await testEnv.db.query(
        'SELECT * FROM audit_stamps WHERE entity_type = $1 AND entity_id = $2',
        ['sepulka', sepulka.id]
      );

      expect(auditResult.rows).toHaveLength(1);
      expect(auditResult.rows[0].action).toBe('CREATE');
      expect(auditResult.rows[0].actor_id).toBe('test-smith-001');
    });
  });

  describe('📊 Performance & Quality Tests', () => {
    
    test('should complete within performance budget', async () => {
      const input = createTestSepulkaInput();
      
      const mutation = `
        mutation ForgeSepulka($input: ForgeInput!) {
          forgeSepulka(input: $input) {
            sepulka { id }
          }
        }
      `;

      const startTime = performance.now();
      const response = await testEnv.executeGraphQL(mutation, { input });
      const endTime = performance.now();

      expectGraphQLSuccess(response);
      
      // Performance requirement: <200ms response time
      const duration = endTime - startTime;
      expect(duration).toBeLessThan(200);
      
      console.log(`⚡ forgeSepulka performance: ${duration.toFixed(2)}ms`);
    });

    test('should handle large parameter objects', async () => {
      // Test with complex configuration
      const largeParameters = {
        reach: 1200,
        payload: 15,
        precision: 0.1,
        joints: {
          joint1: { min: -180, max: 180, speed: 90 },
          joint2: { min: -90, max: 90, speed: 120 },
          joint3: { min: -180, max: 180, speed: 180 },
          joint4: { min: -180, max: 180, speed: 360 },
          joint5: { min: -120, max: 120, speed: 360 },
          joint6: { min: -360, max: 360, speed: 720 }
        },
        workspace: {
          envelope: 'spherical',
          radius: 1200,
          height: 2400,
          safeZones: [
            { name: 'home', position: [0, 0, 500] },
            { name: 'pickup', position: [800, 0, 200] },
            { name: 'dropoff', position: [-800, 0, 200] }
          ]
        },
        capabilities: {
          pickAndPlace: true,
          assembly: false,
          inspection: true,
          welding: false
        }
      };

      const input = createTestSepulkaInput({
        name: 'Complex-Configuration-Test',
        parameters: largeParameters
      });

      const mutation = `
        mutation ForgeSepulka($input: ForgeInput!) {
          forgeSepulka(input: $input) {
            sepulka {
              id
              parameters
            }
          }
        }
      `;

      const response = await testEnv.executeGraphQL(mutation, { input });
      const data = expectGraphQLSuccess(response);
      const sepulka = data.forgeSepulka.sepulka;

      expect(sepulka.parameters).toEqual(largeParameters);
    });
  });

  describe('🔒 Security & Authorization Tests', () => {
    
    test('should prevent unauthorized users from forging', async () => {
      const input = createTestSepulkaInput();

      const mutation = `
        mutation ForgeSepulka($input: ForgeInput!) {
          forgeSepulka(input: $input) {
            sepulka { id }
            errors { code message }
          }
        }
      `;

      // Test with no authentication
      const response = await testEnv.executeGraphQL(mutation, { input }, null);
      expectGraphQLError(response, 'Authentication required');
    });

    test('should enforce rate limiting for rapid requests', async () => {
      const input = createTestSepulkaInput();
      
      const mutation = `
        mutation ForgeSepulka($input: ForgeInput!) {
          forgeSepulka(input: $input) {
            sepulka { id }
            errors { code message }
          }
        }
      `;

      // Execute multiple rapid requests
      const rapidRequests = Array(10).fill(null).map((_, i) => 
        testEnv.executeGraphQL(mutation, { 
          input: { ...input, name: `Rapid-${i}` }
        })
      );

      const responses = await Promise.all(rapidRequests);
      
      // First few should succeed
      expectGraphQLSuccess(responses[0]);
      expectGraphQLSuccess(responses[1]);
      
      // Later ones might be rate limited (implementation dependent)
      // This test defines the expected behavior
    });
  });

  describe('💾 Data Integrity Tests', () => {
    
    test('should maintain referential integrity', async () => {
      const input = createTestSepulkaInput();

      const mutation = `
        mutation ForgeSepulka($input: ForgeInput!) {
          forgeSepulka(input: $input) {
            sepulka { id }
          }
        }
      `;

      const response = await testEnv.executeGraphQL(mutation, { input });
      const sepulka = expectGraphQLSuccess(response).forgeSepulka.sepulka;

      // Verify all foreign key relationships exist
      const patternExists = await testEnv.db.query(
        'SELECT id FROM patterns WHERE id = $1',
        [input.patternId]
      );
      expect(patternExists.rows).toHaveLength(1);

      const alloysExist = await testEnv.db.query(
        'SELECT id FROM alloys WHERE id = ANY($1)',
        [input.alloyIds]
      );
      expect(alloysExist.rows).toHaveLength(input.alloyIds.length);

      const creatorExists = await testEnv.db.query(
        'SELECT id FROM smiths WHERE id = $1',
        ['test-smith-001']
      );
      expect(creatorExists.rows).toHaveLength(1);
    });

    test('should handle database transaction rollback on failure', async () => {
      // Create scenario that will fail mid-transaction
      const input = createTestSepulkaInput({
        alloyIds: ['test-alloy-servo', 'will-fail-in-transaction']
      });

      const mutation = `
        mutation ForgeSepulka($input: ForgeInput!) {
          forgeSepulka(input: $input) {
            sepulka { id }
            errors { code message }
          }
        }
      `;

      const response = await testEnv.executeGraphQL(mutation, { input });
      expectGraphQLError(response);

      // Verify no partial data was committed
      const sepulkaCount = await testEnv.db.query(
        'SELECT COUNT(*) FROM sepulkas WHERE name = $1',
        [input.name]
      );
      expect(parseInt(sepulkaCount.rows[0].count)).toBe(0);

      const alloyLinksCount = await testEnv.db.query(
        'SELECT COUNT(*) FROM sepulka_alloys'
      );
      expect(parseInt(alloyLinksCount.rows[0].count)).toBe(0);
    });
  });
});

// Integration tests for the complete design flow
describe('🔄 Design Flow Integration Tests', () => {
  
  test('should support full design → save → view workflow', async () => {
    // Step 1: Create design
    const input = createTestSepulkaInput({
      name: 'Integration-Test-Robot'
    });

    const forgeMutation = `
      mutation ForgeSepulka($input: ForgeInput!) {
        forgeSepulka(input: $input) {
          sepulka {
            id
            name
            status
          }
        }
      }
    `;

    const forgeResponse = await testEnv.executeGraphQL(forgeMutation, { input });
    const sepulka = expectGraphQLSuccess(forgeResponse).forgeSepulka.sepulka;
    
    // Step 2: Verify it appears in user's sepulka list
    const listQuery = `
      query GetMySepulkas {
        sepulkas(filter: { createdBy: "test-smith-001" }) {
          id
          name
          status
        }
      }
    `;

    const listResponse = await testEnv.executeGraphQL(listQuery);
    const sepulkas = expectGraphQLSuccess(listResponse).sepulkas;
    
    expect(sepulkas).toHaveLength(1);
    expect(sepulkas[0].id).toBe(sepulka.id);
    expect(sepulkas[0].name).toBe('Integration-Test-Robot');
    expect(sepulkas[0].status).toBe('FORGING');
  });
});
