-- Sepulki Development Seed Data
-- Comprehensive test data for local development and testing

-- Insert development smiths
INSERT INTO smiths (email, name, password_hash, role, permissions, preferences) VALUES 
(
  'dev@sepulki.com',
  'Development Smith',
  encode(digest('dev123', 'sha256'), 'hex'),
  'OVER_SMITH',
  ARRAY[
    'FORGE_SEPULKA', 'EDIT_SEPULKA', 'DELETE_SEPULKA',
    'CAST_INGOT', 'TEMPER_INGOT',
    'QUENCH_TO_FLEET', 'RECALL_FLEET', 'EMERGENCY_STOP',
    'VIEW_FLEET', 'MANAGE_FLEET', 'VIEW_ROBOTS', 'MANAGE_ROBOTS',
    'CREATE_TASK', 'ASSIGN_TASK', 'CANCEL_TASK', 'VIEW_TASKS',
    'VIEW_CATALOG', 'MANAGE_ALLOYS', 'MANAGE_PATTERNS',
    'VIEW_EDICTS', 'MANAGE_EDICTS',
    'VIEW_BELLOWS', 'EXPORT_TELEMETRY'
  ],
  '{"theme": "dark", "language": "en", "timezone": "UTC", "notifications": {"email": true, "push": false}, "dashboard": {"defaultView": "overview", "widgets": ["fleets", "tasks", "telemetry"]}}'
),
(
  'demo@sepulki.com',
  'Demo Smith',
  encode(digest('demo123', 'sha256'), 'hex'), 
  'SMITH',
  ARRAY[
    'FORGE_SEPULKA', 'EDIT_SEPULKA',
    'CAST_INGOT',
    'VIEW_FLEET', 'VIEW_ROBOTS',
    'CREATE_TASK', 'VIEW_TASKS',
    'VIEW_CATALOG',
    'VIEW_BELLOWS'
  ],
  '{"theme": "light", "language": "en", "timezone": "America/New_York", "notifications": {"email": true, "push": true}, "dashboard": {"defaultView": "fleets", "widgets": ["robots", "tasks"]}}'
),
(
  'test@sepulki.com',
  'Test Over-Smith',
  encode(digest('test123', 'sha256'), 'hex'),
  'OVER_SMITH', 
  ARRAY[
    'FORGE_SEPULKA', 'EDIT_SEPULKA', 'DELETE_SEPULKA',
    'CAST_INGOT', 'TEMPER_INGOT',
    'QUENCH_TO_FLEET', 'RECALL_FLEET',
    'VIEW_FLEET', 'MANAGE_FLEET', 'VIEW_ROBOTS', 'MANAGE_ROBOTS',
    'CREATE_TASK', 'ASSIGN_TASK', 'CANCEL_TASK', 'VIEW_TASKS',
    'VIEW_CATALOG', 'MANAGE_ALLOYS', 'MANAGE_PATTERNS',
    'VIEW_EDICTS',
    'VIEW_BELLOWS', 'EXPORT_TELEMETRY'
  ],
  '{}'
);

-- Insert development loci
INSERT INTO loci (name, description, coordinates, safety_zones) VALUES 
(
  'Development Lab',
  'Local development and testing facility',
  '{"latitude": 37.7749, "longitude": -122.4194, "altitude": 10}',
  '[{"id": "dev-safe-zone", "name": "Safe Testing Area", "type": "RESTRICTED", "boundaries": [{"type": "circle", "coordinates": [[37.7749, -122.4194]], "radius": 100}], "restrictions": ["max_speed_1ms"]}]'
),
(
  'Demo Warehouse', 
  'Simulated warehouse environment for demonstrations',
  '{"latitude": 40.7128, "longitude": -74.0060, "altitude": 5}',
  '[{"id": "warehouse-zone", "name": "Warehouse Floor", "type": "RESTRICTED", "boundaries": [{"type": "rectangle", "coordinates": [[40.7128, -74.0060], [40.7130, -74.0058]]}], "restrictions": ["supervised_operation"]}]'
),
(
  'Factory Simulation',
  'Virtual factory floor for testing assembly operations', 
  '{"latitude": 42.3601, "longitude": -71.0589, "altitude": 15}',
  '[]'
);

-- Insert development fleets
INSERT INTO fleets (name, description, locus_id, status) VALUES 
(
  'Dev Fleet Alpha',
  'Primary development and testing fleet',
  (SELECT id FROM loci WHERE name = 'Development Lab'),
  'ACTIVE'
),
(
  'Demo Warehouse Bots',
  'Demonstration robots for warehouse operations',
  (SELECT id FROM loci WHERE name = 'Demo Warehouse'),
  'IDLE'
),
(
  'Factory Test Units',
  'Assembly line testing robots',
  (SELECT id FROM loci WHERE name = 'Factory Simulation'),
  'MAINTENANCE'
);

-- Insert test sepulkas (robot designs)
INSERT INTO sepulkas (name, description, version, pattern_id, status, parameters, created_by) VALUES 
(
  'DevBot-001',
  'Basic development robot for testing',
  '1.0.0',
  (SELECT id FROM patterns WHERE name = 'Industrial Arm - 6DOF'),
  'READY',
  '{"reach": 1500, "payload": 15, "precision": 0.3}',
  (SELECT id FROM smiths WHERE email = 'dev@sepulki.com')
),
(
  'WarehouseWorker-Demo',
  'Demonstration warehouse automation robot',
  '2.1.0', 
  (SELECT id FROM patterns WHERE name = 'Mobile Base - Differential Drive'),
  'CAST_READY',
  '{"max_speed": 2.5, "payload": 150, "battery_capacity": 120}',
  (SELECT id FROM smiths WHERE email = 'demo@sepulki.com')
),
(
  'AssemblyBot-Test',
  'Test robot for assembly operations',
  '1.5.0',
  (SELECT id FROM patterns WHERE name = 'Industrial Arm - 6DOF'),
  'FORGING',
  '{"reach": 800, "payload": 5, "precision": 0.1}',
  (SELECT id FROM smiths WHERE email = 'test@sepulki.com')
);

-- Associate alloys with sepulkas
INSERT INTO sepulka_alloys (sepulka_id, alloy_id, configuration) VALUES 
(
  (SELECT id FROM sepulkas WHERE name = 'DevBot-001'),
  (SELECT id FROM alloys WHERE name = 'ServoMax Pro 3000'),
  '{"mount_position": "joint1", "calibration": {"offset": 0.05, "scale": 1.0}}'
),
(
  (SELECT id FROM sepulkas WHERE name = 'DevBot-001'),
  (SELECT id FROM alloys WHERE name = 'VisionEye 4K'),
  '{"mount_position": "end_effector", "settings": {"resolution": "1080p", "fps": 30}}'
),
(
  (SELECT id FROM sepulkas WHERE name = 'WarehouseWorker-Demo'),
  (SELECT id FROM alloys WHERE name = 'PowerCore 500W'),
  '{"mount_position": "chassis", "settings": {"voltage": 24, "backup_enabled": true}}'
),
(
  (SELECT id FROM sepulkas WHERE name = 'AssemblyBot-Test'),
  (SELECT id FROM alloys WHERE name = 'GripForce Elite'),
  '{"mount_position": "end_effector", "settings": {"force_limit": 50, "precision_mode": true}}'
);

-- Create test ingots (build artifacts)
INSERT INTO ingots (sepulka_id, version, build_hash, status, artifacts, tempered, created_by) VALUES 
(
  (SELECT id FROM sepulkas WHERE name = 'DevBot-001'),
  '1.0.0',
  'build_dev001_abc123',
  'READY',
  '[
    {"type": "URDF", "path": "/builds/devbot-001/robot.urdf", "size": 15424, "checksum": "sha256:abc123"},
    {"type": "CONTAINER", "path": "/builds/devbot-001/robot.tar", "size": 524288000, "checksum": "sha256:def456"},
    {"type": "CONFIG", "path": "/builds/devbot-001/config.json", "size": 2048, "checksum": "sha256:789xyz"}
  ]',
  false,
  (SELECT id FROM smiths WHERE email = 'dev@sepulki.com')
),
(
  (SELECT id FROM sepulkas WHERE name = 'WarehouseWorker-Demo'),
  '2.1.0',
  'build_warehouse_xyz789',
  'TEMPERED',
  '[
    {"type": "URDF", "path": "/builds/warehouse-demo/robot.urdf", "size": 18632, "checksum": "sha256:warehouse123"},
    {"type": "CONTAINER", "path": "/builds/warehouse-demo/robot.tar", "size": 612345600, "checksum": "sha256:warehouse456"}
  ]',
  true,
  (SELECT id FROM smiths WHERE email = 'demo@sepulki.com')
);

-- Create test robots
INSERT INTO robots (name, sepulka_id, fleet_id, current_ingot_id, status, battery_level, health_score, last_pose) VALUES 
(
  'DevBot-Alpha',
  (SELECT id FROM sepulkas WHERE name = 'DevBot-001'),
  (SELECT id FROM fleets WHERE name = 'Dev Fleet Alpha'),
  (SELECT id FROM ingots WHERE build_hash = 'build_dev001_abc123'),
  'WORKING',
  87.5,
  95.2,
  '{"position": {"x": 1.2, "y": 0.8, "z": 0.5}, "orientation": {"x": 0, "y": 0, "z": 0, "w": 1}, "jointPositions": {"joint1": 0.5, "joint2": -0.3}, "timestamp": "2024-01-15T10:30:00Z"}'
),
(
  'DevBot-Beta',
  (SELECT id FROM sepulkas WHERE name = 'DevBot-001'),
  (SELECT id FROM fleets WHERE name = 'Dev Fleet Alpha'),
  (SELECT id FROM ingots WHERE build_hash = 'build_dev001_abc123'),
  'IDLE',
  92.1,
  98.7,
  '{"position": {"x": 0.0, "y": 0.0, "z": 0.0}, "orientation": {"x": 0, "y": 0, "z": 0, "w": 1}, "jointPositions": {"joint1": 0.0, "joint2": 0.0}, "timestamp": "2024-01-15T10:25:00Z"}'
),
(
  'WarehouseBot-Demo-01',
  (SELECT id FROM sepulkas WHERE name = 'WarehouseWorker-Demo'),
  (SELECT id FROM fleets WHERE name = 'Demo Warehouse Bots'),
  (SELECT id FROM ingots WHERE build_hash = 'build_warehouse_xyz789'),
  'CHARGING',
  34.8,
  78.3,
  '{"position": {"x": 5.2, "y": 3.1, "z": 0.0}, "orientation": {"x": 0, "y": 0, "z": 0.707, "w": 0.707}, "jointPositions": {}, "timestamp": "2024-01-15T10:20:00Z"}'
),
(
  'WarehouseBot-Demo-02',
  (SELECT id FROM sepulkas WHERE name = 'WarehouseWorker-Demo'),
  (SELECT id FROM fleets WHERE name = 'Demo Warehouse Bots'),
  (SELECT id FROM ingots WHERE build_hash = 'build_warehouse_xyz789'),
  'WORKING',
  76.4,
  89.1,
  '{"position": {"x": 8.7, "y": 1.5, "z": 0.0}, "orientation": {"x": 0, "y": 0, "z": -0.707, "w": 0.707}, "jointPositions": {}, "timestamp": "2024-01-15T10:32:00Z"}'
);

-- Create test tasks
INSERT INTO tasks (name, description, type, parameters, status, priority, scheduled_at, created_by) VALUES 
(
  'Pick and Place Demo',
  'Demonstration of basic pick and place operations for warehouse tour',
  'PICK_AND_PLACE',
  '{
    "source": {"position": [2.0, 1.5, 0.8], "approach": "top"},
    "destination": {"position": [5.0, 3.2, 0.8], "placement": "place"},
    "object": {"type": "box", "dimensions": [0.2, 0.15, 0.1], "weight": 0.8},
    "constraints": {"maxForce": 50, "speed": 0.5, "precision": 0.01}
  }',
  'IN_PROGRESS',
  'NORMAL',
  NOW() - INTERVAL '5 minutes',
  (SELECT id FROM smiths WHERE email = 'demo@sepulki.com')
),
(
  'Assembly Testing',
  'Test assembly operations with precision components',
  'ASSEMBLY',
  '{
    "parts": [
      {"id": "part_a", "type": "circuit_board", "position": [1.0, 1.0, 0.9]},
      {"id": "part_b", "type": "connector", "position": [1.1, 1.0, 0.9]}
    ],
    "sequence": [
      {"order": 1, "action": "pick", "partId": "part_a", "parameters": {"force": 10}},
      {"order": 2, "action": "place", "partId": "part_a", "parameters": {"precision": 0.001}}
    ],
    "tolerances": {"position": 0.001, "force": 5}
  }',
  'PENDING',
  'HIGH',
  NOW() + INTERVAL '1 hour',
  (SELECT id FROM smiths WHERE email = 'test@sepulki.com')
),
(
  'Quality Inspection',
  'Automated visual inspection of manufactured parts',
  'INSPECTION',
  '{
    "target": {"type": "manufactured_part", "position": [3.0, 2.0, 1.0], "features": ["dimensions", "surface_quality"]},
    "criteria": {
      "dimensions": {"min": 0.95, "max": 1.05, "tolerance": 0.02},
      "quality": {"defects": ["scratches", "dents"], "threshold": 0.95}
    },
    "methods": [
      {"type": "visual", "sensor": "VisionEye 4K", "parameters": {"resolution": "4K", "lighting": "full_spectrum"}}
    ]
  }',
  'COMPLETED',
  'NORMAL',
  NOW() - INTERVAL '2 hours',
  (SELECT id FROM smiths WHERE email = 'dev@sepulki.com')
),
(
  'Fleet Patrol',
  'Routine patrol and monitoring of facility perimeter',
  'PATROL',
  '{
    "route": [
      {"position": [0.0, 0.0, 0.0], "duration": 30},
      {"position": [10.0, 0.0, 0.0], "duration": 60},
      {"position": [10.0, 10.0, 0.0], "duration": 60},
      {"position": [0.0, 10.0, 0.0], "duration": 60},
      {"position": [0.0, 0.0, 0.0], "duration": 30}
    ],
    "sensors": ["camera", "lidar", "proximity"],
    "alerts": ["unauthorized_access", "obstacle_detection", "equipment_malfunction"]
  }',
  'ASSIGNED',
  'LOW',
  NOW() + INTERVAL '30 minutes',
  (SELECT id FROM smiths WHERE email = 'demo@sepulki.com')
);

-- Associate tasks with robots
INSERT INTO task_robots (task_id, robot_id, assigned_at, estimated_duration, confidence) VALUES 
(
  (SELECT id FROM tasks WHERE name = 'Pick and Place Demo'),
  (SELECT id FROM robots WHERE name = 'WarehouseBot-Demo-02'),
  NOW() - INTERVAL '5 minutes',
  1800, -- 30 minutes
  0.95
),
(
  (SELECT id FROM tasks WHERE name = 'Assembly Testing'),
  (SELECT id FROM robots WHERE name = 'DevBot-Alpha'),
  NOW() - INTERVAL '10 minutes',
  3600, -- 1 hour
  0.87
),
(
  (SELECT id FROM tasks WHERE name = 'Fleet Patrol'),
  (SELECT id FROM robots WHERE name = 'WarehouseBot-Demo-01'),
  NOW() + INTERVAL '25 minutes',
  7200, -- 2 hours
  0.92
);

-- Create test runs
INSERT INTO runs (task_id, robot_id, status, started_at, completed_at, metrics, logs) VALUES 
(
  (SELECT id FROM tasks WHERE name = 'Quality Inspection'),
  (SELECT id FROM robots WHERE name = 'DevBot-Beta'),
  'COMPLETED',
  NOW() - INTERVAL '2 hours 15 minutes',
  NOW() - INTERVAL '2 hours',
  '{"duration": 900, "energyConsumed": 12.5, "accuracy": 0.98, "errors": 0, "warnings": 1, "performance": {"cycle_time": 45, "precision": 0.001}}',
  ARRAY[
    '2024-01-15T08:45:00Z - INFO: Starting quality inspection task',
    '2024-01-15T08:45:15Z - INFO: Calibrating vision system',
    '2024-01-15T08:45:30Z - INFO: Beginning inspection sequence',
    '2024-01-15T08:55:45Z - WARN: Minor surface irregularity detected (within tolerance)',
    '2024-01-15T09:00:00Z - INFO: Inspection completed successfully'
  ]
),
(
  (SELECT id FROM tasks WHERE name = 'Pick and Place Demo'),
  (SELECT id FROM robots WHERE name = 'WarehouseBot-Demo-02'),
  'RUNNING',
  NOW() - INTERVAL '5 minutes',
  NULL,
  '{"duration": 300, "energyConsumed": 8.2, "accuracy": 0.96, "errors": 0, "warnings": 0, "performance": {"cycle_time": 35, "precision": 0.005}}',
  ARRAY[
    '2024-01-15T10:25:00Z - INFO: Starting pick and place operation',
    '2024-01-15T10:25:12Z - INFO: Approaching source position',
    '2024-01-15T10:26:05Z - INFO: Object grasped successfully', 
    '2024-01-15T10:27:22Z - INFO: Moving to destination',
    '2024-01-15T10:28:45Z - INFO: Object placed successfully'
  ]
);

-- Create safety edicts
INSERT INTO edicts (name, description, type, rules, severity, applies_to, created_by) VALUES 
(
  'Maximum Speed Limit',
  'Enforce maximum speed limits in restricted areas',
  'SAFETY',
  '{
    "condition": {"type": "LOCATION", "operator": "and", "rules": [{"field": "zone_type", "operator": "eq", "value": "RESTRICTED"}]},
    "actions": [{"type": "THROTTLE", "parameters": {"max_speed": 1.0}, "immediate": true}],
    "exceptions": []
  }',
  'CRITICAL',
  '{"locations": ["development_lab", "demo_warehouse"]}',
  (SELECT id FROM smiths WHERE email = 'admin@sepulki.com')
),
(
  'Battery Level Warning',
  'Alert when robot battery drops below safe operating level',
  'OPERATIONAL',
  '{
    "condition": {"type": "ROBOT_STATE", "operator": "and", "rules": [{"field": "battery_level", "operator": "lt", "value": 20}]},
    "actions": [{"type": "WARN", "parameters": {"message": "Low battery - return to charging station"}, "immediate": true}, {"type": "ALERT", "parameters": {"severity": "warning"}, "immediate": false}],
    "exceptions": []
  }',
  'WARNING',
  '{"fleets": [], "robots": []}',
  (SELECT id FROM smiths WHERE email = 'admin@sepulki.com')
),
(
  'Force Limit Safety',
  'Emergency stop if force sensors detect excessive force',
  'SAFETY',
  '{
    "condition": {"type": "PERFORMANCE", "operator": "and", "rules": [{"field": "applied_force", "operator": "gt", "value": 100}]},
    "actions": [{"type": "EMERGENCY_STOP", "parameters": {"reason": "Force limit exceeded"}, "immediate": true}],
    "exceptions": []
  }',
  'CRITICAL',
  '{"taskTypes": ["ASSEMBLY", "PICK_AND_PLACE"]}',
  (SELECT id FROM smiths WHERE email = 'test@sepulki.com')
);

-- Update fleet active tasks
UPDATE fleets SET active_task_id = (
  SELECT id FROM tasks WHERE name = 'Pick and Place Demo'
) WHERE name = 'Demo Warehouse Bots';

UPDATE fleets SET active_task_id = (
  SELECT id FROM tasks WHERE name = 'Assembly Testing'
) WHERE name = 'Dev Fleet Alpha';

-- Create additional alloys for more comprehensive testing
INSERT INTO alloys (name, description, type, specifications, tags, version) VALUES
(
  'TurboMove 5000',
  'High-speed linear actuator for rapid positioning',
  'ACTUATOR',
  '{"type": "linear", "stroke": 500, "force": 2000, "speed": 2.5, "precision": 0.05, "voltage": 48, "protocol": "EtherCAT"}',
  ARRAY['linear', 'high-speed', 'positioning'],
  '4.2.1'
),
(
  'SmartSense Pro',
  'Multi-modal sensor array with edge AI processing',
  'SENSOR',
  '{"type": "MULTI", "sensors": ["IMU", "CAMERA", "LIDAR", "FORCE"], "ai_processing": true, "edge_compute": true, "protocols": ["ROS2", "CAN"]}',
  ARRAY['multi-sensor', 'ai', 'edge-compute'],
  '2.8.3'
),
(
  'FlexGrip Universal', 
  'Universal gripper with adaptive finger configuration',
  'END_EFFECTOR',
  '{"type": "ADAPTIVE", "payload": 25, "finger_count": 4, "stroke": 120, "force_range": [0.1, 300], "material_detection": true}',
  ARRAY['adaptive', 'universal', 'smart-grip'],
  '3.1.5'
),
(
  'MegaBase Chassis',
  'Heavy-duty mobile platform for industrial applications',
  'CHASSIS',
  '{"type": "wheeled", "drive": "omnidirectional", "payload": 500, "dimensions": [1.2, 0.8, 0.3], "ground_clearance": 0.1, "max_speed": 3.0}',
  ARRAY['heavy-duty', 'omnidirectional', 'industrial'],
  '1.9.2'
);

-- Add more patterns for comprehensive testing
INSERT INTO patterns (name, description, category, parameters, defaults) VALUES 
(
  'Humanoid Assistant',
  'Bipedal humanoid robot for human-robot collaboration',
  'HUMANOID',
  '{"height": {"type": "number", "min": 1.4, "max": 1.9}, "dexterity": {"type": "number", "min": 5, "max": 15}, "mobility": {"type": "string", "options": ["walking", "wheeled_base", "hybrid"]}}',
  '{"height": 1.65, "dexterity": 10, "mobility": "walking"}'
),
(
  'Inspection Drone',
  'Aerial drone for facility monitoring and inspection',
  'DRONE',
  '{"flight_time": {"type": "number", "min": 20, "max": 180}, "camera_gimbal": {"type": "boolean"}, "payload": {"type": "number", "min": 0.5, "max": 5.0}}',
  '{"flight_time": 45, "camera_gimbal": true, "payload": 2.0}'
),
(
  'Precision Assembly Unit',
  'High-precision robot for electronic component assembly',
  'INDUSTRIAL_ARM',
  '{"reach": {"type": "number", "min": 300, "max": 1200}, "precision": {"type": "number", "min": 0.001, "max": 0.1}, "force_control": {"type": "boolean"}}',
  '{"reach": 600, "precision": 0.01, "force_control": true}'
);

COMMIT;

-- Display summary
SELECT 
  'Database seeded successfully!' as message,
  (SELECT COUNT(*) FROM smiths) as smiths_count,
  (SELECT COUNT(*) FROM sepulkas) as sepulkas_count,
  (SELECT COUNT(*) FROM robots) as robots_count,
  (SELECT COUNT(*) FROM tasks) as tasks_count,
  (SELECT COUNT(*) FROM alloys) as alloys_count,
  (SELECT COUNT(*) FROM patterns) as patterns_count;
