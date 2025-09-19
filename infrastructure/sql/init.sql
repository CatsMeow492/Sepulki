-- Sepulki Database Schema
-- Robotics-as-a-Service Platform with Metallurgy Theme

-- Create extensions
CREATE EXTENSION IF NOT EXISTS "uuid-ossp";
CREATE EXTENSION IF NOT EXISTS "pgcrypto";

-- Create enum types
CREATE TYPE sepulka_status AS ENUM (
  'FORGING',
  'CAST_READY', 
  'CASTING',
  'CAST_FAILED',
  'READY'
);

CREATE TYPE alloy_type AS ENUM (
  'ACTUATOR',
  'SENSOR',
  'CONTROLLER', 
  'END_EFFECTOR',
  'CHASSIS',
  'POWER',
  'COMMUNICATION'
);

CREATE TYPE pattern_category AS ENUM (
  'INDUSTRIAL_ARM',
  'MOBILE_ROBOT',
  'HUMANOID',
  'QUADRUPED', 
  'DRONE',
  'CUSTOM'
);

CREATE TYPE ingot_status AS ENUM (
  'BUILDING',
  'BUILD_FAILED',
  'READY',
  'TEMPERING',
  'TEMPER_FAILED', 
  'TEMPERED',
  'DEPLOYED'
);

CREATE TYPE fleet_status AS ENUM (
  'IDLE',
  'ACTIVE',
  'MAINTENANCE',
  'ERROR',
  'OFFLINE'
);

CREATE TYPE robot_status AS ENUM (
  'IDLE',
  'WORKING',
  'CHARGING',
  'MAINTENANCE',
  'ERROR',
  'OFFLINE'
);

CREATE TYPE task_status AS ENUM (
  'PENDING',
  'ASSIGNED',
  'IN_PROGRESS',
  'COMPLETED',
  'FAILED',
  'CANCELLED'
);

CREATE TYPE task_priority AS ENUM (
  'LOW',
  'NORMAL',
  'HIGH',
  'URGENT'
);

CREATE TYPE task_type AS ENUM (
  'PICK_AND_PLACE',
  'ASSEMBLY',
  'INSPECTION',
  'TRANSPORT',
  'MAINTENANCE', 
  'PATROL',
  'CUSTOM'
);

CREATE TYPE run_status AS ENUM (
  'PENDING',
  'RUNNING',
  'COMPLETED',
  'FAILED',
  'CANCELLED'
);

CREATE TYPE smith_role AS ENUM (
  'SMITH',
  'OVER_SMITH',
  'ADMIN'
);

CREATE TYPE edict_type AS ENUM (
  'SAFETY',
  'PERFORMANCE',
  'COMPLIANCE',
  'OPERATIONAL'
);

CREATE TYPE edict_severity AS ENUM (
  'INFO',
  'WARNING',
  'CRITICAL'
);

-- Core tables

-- Smiths (Users)
CREATE TABLE smiths (
  id UUID PRIMARY KEY DEFAULT uuid_generate_v4(),
  email VARCHAR(255) UNIQUE NOT NULL,
  name VARCHAR(255) NOT NULL,
  password_hash VARCHAR(255) NOT NULL,
  role smith_role NOT NULL DEFAULT 'SMITH',
  permissions TEXT[] DEFAULT ARRAY[]::TEXT[],
  is_active BOOLEAN DEFAULT TRUE,
  preferences JSONB DEFAULT '{}',
  last_login_at TIMESTAMP WITH TIME ZONE,
  created_at TIMESTAMP WITH TIME ZONE DEFAULT NOW(),
  updated_at TIMESTAMP WITH TIME ZONE DEFAULT NOW()
);

-- Patterns (Design Templates)
CREATE TABLE patterns (
  id UUID PRIMARY KEY DEFAULT uuid_generate_v4(),
  name VARCHAR(255) NOT NULL,
  description TEXT,
  category pattern_category NOT NULL,
  parameters JSONB DEFAULT '{}',
  defaults JSONB DEFAULT '{}',
  template TEXT,
  tags TEXT[] DEFAULT ARRAY[]::TEXT[],
  created_at TIMESTAMP WITH TIME ZONE DEFAULT NOW(),
  updated_at TIMESTAMP WITH TIME ZONE DEFAULT NOW()
);

-- Alloys (Components)
CREATE TABLE alloys (
  id UUID PRIMARY KEY DEFAULT uuid_generate_v4(),
  name VARCHAR(255) NOT NULL,
  description TEXT,
  type alloy_type NOT NULL,
  specifications JSONB DEFAULT '{}',
  mesh_assets TEXT[] DEFAULT ARRAY[]::TEXT[],
  urdf_template TEXT,
  compatibility JSONB DEFAULT '[]',
  tags TEXT[] DEFAULT ARRAY[]::TEXT[],
  version VARCHAR(50) NOT NULL DEFAULT '1.0.0',
  created_at TIMESTAMP WITH TIME ZONE DEFAULT NOW(),
  updated_at TIMESTAMP WITH TIME ZONE DEFAULT NOW()
);

-- Sepulkas (Robot Designs)
CREATE TABLE sepulkas (
  id UUID PRIMARY KEY DEFAULT uuid_generate_v4(),
  name VARCHAR(255) NOT NULL,
  description TEXT,
  version VARCHAR(50) NOT NULL DEFAULT '1.0.0',
  pattern_id UUID REFERENCES patterns(id),
  status sepulka_status NOT NULL DEFAULT 'FORGING',
  parameters JSONB DEFAULT '{}',
  created_by UUID NOT NULL REFERENCES smiths(id),
  created_at TIMESTAMP WITH TIME ZONE DEFAULT NOW(),
  updated_at TIMESTAMP WITH TIME ZONE DEFAULT NOW()
);

-- Sepulka-Alloy associations
CREATE TABLE sepulka_alloys (
  sepulka_id UUID REFERENCES sepulkas(id) ON DELETE CASCADE,
  alloy_id UUID REFERENCES alloys(id) ON DELETE CASCADE,
  configuration JSONB DEFAULT '{}',
  added_at TIMESTAMP WITH TIME ZONE DEFAULT NOW(),
  PRIMARY KEY (sepulka_id, alloy_id)
);

-- Ingots (Build Artifacts)
CREATE TABLE ingots (
  id UUID PRIMARY KEY DEFAULT uuid_generate_v4(),
  sepulka_id UUID NOT NULL REFERENCES sepulkas(id),
  version VARCHAR(50) NOT NULL,
  build_hash VARCHAR(255) UNIQUE NOT NULL,
  status ingot_status NOT NULL DEFAULT 'BUILDING',
  artifacts JSONB DEFAULT '[]',
  build_logs TEXT[] DEFAULT ARRAY[]::TEXT[],
  tempered BOOLEAN DEFAULT FALSE,
  temper_results JSONB,
  created_by UUID NOT NULL REFERENCES smiths(id),
  created_at TIMESTAMP WITH TIME ZONE DEFAULT NOW(),
  updated_at TIMESTAMP WITH TIME ZONE DEFAULT NOW()
);

-- Loci (Locations)
CREATE TABLE loci (
  id UUID PRIMARY KEY DEFAULT uuid_generate_v4(),
  name VARCHAR(255) NOT NULL,
  description TEXT,
  coordinates JSONB, -- {lat, lng, alt}
  constraints UUID[], -- References to edicts
  safety_zones JSONB DEFAULT '[]',
  created_at TIMESTAMP WITH TIME ZONE DEFAULT NOW(),
  updated_at TIMESTAMP WITH TIME ZONE DEFAULT NOW()
);

-- Fleets
CREATE TABLE fleets (
  id UUID PRIMARY KEY DEFAULT uuid_generate_v4(),
  name VARCHAR(255) NOT NULL,
  description TEXT,
  locus_id UUID REFERENCES loci(id),
  status fleet_status NOT NULL DEFAULT 'IDLE',
  constraint_ids UUID[] DEFAULT ARRAY[]::UUID[],
  active_task_id UUID,
  created_at TIMESTAMP WITH TIME ZONE DEFAULT NOW(),
  updated_at TIMESTAMP WITH TIME ZONE DEFAULT NOW()
);

-- Robots
CREATE TABLE robots (
  id UUID PRIMARY KEY DEFAULT uuid_generate_v4(),
  name VARCHAR(255) NOT NULL,
  sepulka_id UUID NOT NULL REFERENCES sepulkas(id),
  fleet_id UUID NOT NULL REFERENCES fleets(id),
  current_ingot_id UUID REFERENCES ingots(id),
  status robot_status NOT NULL DEFAULT 'IDLE',
  last_seen TIMESTAMP WITH TIME ZONE,
  last_pose JSONB, -- {position, orientation, jointPositions, timestamp}
  battery_level FLOAT CHECK (battery_level >= 0 AND battery_level <= 100),
  health_score FLOAT CHECK (health_score >= 0 AND health_score <= 100),
  created_at TIMESTAMP WITH TIME ZONE DEFAULT NOW(),
  updated_at TIMESTAMP WITH TIME ZONE DEFAULT NOW()
);

-- Tasks
CREATE TABLE tasks (
  id UUID PRIMARY KEY DEFAULT uuid_generate_v4(),
  name VARCHAR(255) NOT NULL,
  description TEXT,
  type task_type NOT NULL,
  parameters JSONB DEFAULT '{}',
  status task_status NOT NULL DEFAULT 'PENDING',
  priority task_priority NOT NULL DEFAULT 'NORMAL',
  scheduled_at TIMESTAMP WITH TIME ZONE,
  created_by UUID NOT NULL REFERENCES smiths(id),
  created_at TIMESTAMP WITH TIME ZONE DEFAULT NOW(),
  updated_at TIMESTAMP WITH TIME ZONE DEFAULT NOW()
);

-- Task-Robot assignments
CREATE TABLE task_robots (
  task_id UUID REFERENCES tasks(id) ON DELETE CASCADE,
  robot_id UUID REFERENCES robots(id) ON DELETE CASCADE,
  assigned_at TIMESTAMP WITH TIME ZONE DEFAULT NOW(),
  estimated_duration INTEGER, -- seconds
  confidence FLOAT DEFAULT 1.0,
  PRIMARY KEY (task_id, robot_id)
);

-- Runs (Task Executions)
CREATE TABLE runs (
  id UUID PRIMARY KEY DEFAULT uuid_generate_v4(),
  task_id UUID NOT NULL REFERENCES tasks(id),
  robot_id UUID NOT NULL REFERENCES robots(id),
  status run_status NOT NULL DEFAULT 'PENDING',
  started_at TIMESTAMP WITH TIME ZONE,
  completed_at TIMESTAMP WITH TIME ZONE,
  metrics JSONB,
  logs TEXT[] DEFAULT ARRAY[]::TEXT[],
  created_at TIMESTAMP WITH TIME ZONE DEFAULT NOW(),
  updated_at TIMESTAMP WITH TIME ZONE DEFAULT NOW()
);

-- Edicts (Policies)
CREATE TABLE edicts (
  id UUID PRIMARY KEY DEFAULT uuid_generate_v4(),
  name VARCHAR(255) NOT NULL,
  description TEXT,
  type edict_type NOT NULL,
  rules JSONB NOT NULL,
  severity edict_severity NOT NULL DEFAULT 'WARNING',
  active BOOLEAN DEFAULT TRUE,
  applies_to JSONB DEFAULT '{}', -- {fleets, robots, locations, taskTypes}
  created_by UUID NOT NULL REFERENCES smiths(id),
  created_at TIMESTAMP WITH TIME ZONE DEFAULT NOW(),
  updated_at TIMESTAMP WITH TIME ZONE DEFAULT NOW()
);

-- Policy Violations
CREATE TABLE policy_violations (
  id UUID PRIMARY KEY DEFAULT uuid_generate_v4(),
  edict_id UUID NOT NULL REFERENCES edicts(id),
  robot_id UUID REFERENCES robots(id),
  fleet_id UUID REFERENCES fleets(id), 
  task_id UUID REFERENCES tasks(id),
  severity edict_severity NOT NULL,
  message TEXT NOT NULL,
  data JSONB DEFAULT '{}',
  resolved BOOLEAN DEFAULT FALSE,
  resolved_by UUID REFERENCES smiths(id),
  resolved_at TIMESTAMP WITH TIME ZONE,
  created_at TIMESTAMP WITH TIME ZONE DEFAULT NOW()
);

-- Audit Log
CREATE TABLE audit_stamps (
  id UUID PRIMARY KEY DEFAULT uuid_generate_v4(),
  entity_type VARCHAR(100) NOT NULL,
  entity_id UUID NOT NULL,
  action VARCHAR(100) NOT NULL,
  actor_id UUID REFERENCES smiths(id),
  changes JSONB,
  timestamp TIMESTAMP WITH TIME ZONE DEFAULT NOW(),
  ip_address INET,
  user_agent TEXT
);

-- Create indexes for performance
CREATE INDEX idx_sepulkas_status ON sepulkas(status);
CREATE INDEX idx_sepulkas_created_by ON sepulkas(created_by);
CREATE INDEX idx_sepulkas_created_at ON sepulkas(created_at);

CREATE INDEX idx_alloys_type ON alloys(type);
CREATE INDEX idx_alloys_tags ON alloys USING GIN(tags);

CREATE INDEX idx_ingots_sepulka_id ON ingots(sepulka_id);
CREATE INDEX idx_ingots_status ON ingots(status);
CREATE INDEX idx_ingots_build_hash ON ingots(build_hash);

CREATE INDEX idx_robots_fleet_id ON robots(fleet_id);
CREATE INDEX idx_robots_status ON robots(status);
CREATE INDEX idx_robots_last_seen ON robots(last_seen);

CREATE INDEX idx_tasks_status ON tasks(status);
CREATE INDEX idx_tasks_priority ON tasks(priority);
CREATE INDEX idx_tasks_type ON tasks(type);
CREATE INDEX idx_tasks_created_by ON tasks(created_by);
CREATE INDEX idx_tasks_scheduled_at ON tasks(scheduled_at);

CREATE INDEX idx_runs_task_id ON runs(task_id);
CREATE INDEX idx_runs_robot_id ON runs(robot_id);
CREATE INDEX idx_runs_status ON runs(status);
CREATE INDEX idx_runs_started_at ON runs(started_at);

CREATE INDEX idx_policy_violations_edict_id ON policy_violations(edict_id);
CREATE INDEX idx_policy_violations_resolved ON policy_violations(resolved);
CREATE INDEX idx_policy_violations_created_at ON policy_violations(created_at);

CREATE INDEX idx_audit_stamps_entity ON audit_stamps(entity_type, entity_id);
CREATE INDEX idx_audit_stamps_timestamp ON audit_stamps(timestamp);

-- Create triggers for updated_at
CREATE OR REPLACE FUNCTION update_updated_at_column()
RETURNS TRIGGER AS $$
BEGIN
    NEW.updated_at = NOW();
    RETURN NEW;
END;
$$ language 'plpgsql';

CREATE TRIGGER update_smiths_updated_at BEFORE UPDATE ON smiths 
  FOR EACH ROW EXECUTE FUNCTION update_updated_at_column();
CREATE TRIGGER update_patterns_updated_at BEFORE UPDATE ON patterns 
  FOR EACH ROW EXECUTE FUNCTION update_updated_at_column();
CREATE TRIGGER update_alloys_updated_at BEFORE UPDATE ON alloys 
  FOR EACH ROW EXECUTE FUNCTION update_updated_at_column();
CREATE TRIGGER update_sepulkas_updated_at BEFORE UPDATE ON sepulkas 
  FOR EACH ROW EXECUTE FUNCTION update_updated_at_column();
CREATE TRIGGER update_ingots_updated_at BEFORE UPDATE ON ingots 
  FOR EACH ROW EXECUTE FUNCTION update_updated_at_column();
CREATE TRIGGER update_loci_updated_at BEFORE UPDATE ON loci 
  FOR EACH ROW EXECUTE FUNCTION update_updated_at_column();
CREATE TRIGGER update_fleets_updated_at BEFORE UPDATE ON fleets 
  FOR EACH ROW EXECUTE FUNCTION update_updated_at_column();
CREATE TRIGGER update_robots_updated_at BEFORE UPDATE ON robots 
  FOR EACH ROW EXECUTE FUNCTION update_updated_at_column();
CREATE TRIGGER update_tasks_updated_at BEFORE UPDATE ON tasks 
  FOR EACH ROW EXECUTE FUNCTION update_updated_at_column();
CREATE TRIGGER update_runs_updated_at BEFORE UPDATE ON runs 
  FOR EACH ROW EXECUTE FUNCTION update_updated_at_column();
CREATE TRIGGER update_edicts_updated_at BEFORE UPDATE ON edicts 
  FOR EACH ROW EXECUTE FUNCTION update_updated_at_column();

-- Seed data

-- Create default admin user
INSERT INTO smiths (email, name, password_hash, role, permissions) VALUES (
  'admin@sepulki.com',
  'System Administrator', 
  encode(digest('admin123', 'sha256'), 'hex'), -- Change this in production!
  'ADMIN',
  ARRAY[
    'FORGE_SEPULKA', 'EDIT_SEPULKA', 'DELETE_SEPULKA',
    'CAST_INGOT', 'TEMPER_INGOT',
    'QUENCH_TO_FLEET', 'RECALL_FLEET', 'EMERGENCY_STOP',
    'VIEW_FLEET', 'MANAGE_FLEET', 'VIEW_ROBOTS', 'MANAGE_ROBOTS',
    'CREATE_TASK', 'ASSIGN_TASK', 'CANCEL_TASK', 'VIEW_TASKS',
    'VIEW_CATALOG', 'MANAGE_ALLOYS', 'MANAGE_PATTERNS',
    'VIEW_EDICTS', 'MANAGE_EDICTS',
    'VIEW_BELLOWS', 'EXPORT_TELEMETRY',
    'MANAGE_SMITHS', 'SYSTEM_CONFIG', 'AUDIT_LOGS'
  ]
);

-- Create sample patterns
INSERT INTO patterns (name, description, category, parameters, defaults) VALUES 
(
  'Industrial Arm - 6DOF',
  'Standard 6-degree-of-freedom industrial robotic arm for pick-and-place operations',
  'INDUSTRIAL_ARM',
  '{"reach": {"type": "number", "min": 500, "max": 2000}, "payload": {"type": "number", "min": 1, "max": 50}, "precision": {"type": "number", "min": 0.1, "max": 5.0}}',
  '{"reach": 1000, "payload": 10, "precision": 0.5}'
),
(
  'Mobile Base - Differential Drive',
  'Differential drive mobile platform for warehouse and logistics applications',
  'MOBILE_ROBOT', 
  '{"max_speed": {"type": "number", "min": 0.5, "max": 5.0}, "payload": {"type": "number", "min": 10, "max": 500}, "battery_capacity": {"type": "number", "min": 50, "max": 200}}',
  '{"max_speed": 2.0, "payload": 100, "battery_capacity": 100}'
),
(
  'Quadruped - General Purpose',
  'Four-legged robot for patrol, inspection, and rough terrain navigation',
  'QUADRUPED',
  '{"max_speed": {"type": "number", "min": 1.0, "max": 8.0}, "step_height": {"type": "number", "min": 0.1, "max": 0.5}, "battery_life": {"type": "number", "min": 60, "max": 480}}',
  '{"max_speed": 3.5, "step_height": 0.2, "battery_life": 120}'
);

-- Create sample alloys  
INSERT INTO alloys (name, description, type, specifications, tags, version) VALUES
(
  'ServoMax Pro 3000',
  'High-torque servo actuator with encoder feedback',
  'ACTUATOR',
  '{"torque": 30, "speed": 180, "precision": 0.1, "voltage": 24, "current": 5.0, "protocol": "CAN"}',
  ARRAY['industrial', 'high-torque', 'servo'],
  '2.1.0'
),
(
  'VisionEye 4K',
  'Industrial machine vision camera with AI processing',
  'SENSOR', 
  '{"type": "CAMERA", "resolution": "4K", "fps": 60, "interface": "GigE", "ai_processing": true}',
  ARRAY['vision', '4k', 'ai', 'industrial'],
  '1.3.2'
),
(
  'GripForce Elite',
  'Adaptive parallel gripper with force feedback',
  'END_EFFECTOR',
  '{"type": "GRIPPER", "payload": 15, "stroke": 100, "force_range": [1, 200], "precision": 0.1}',
  ARRAY['gripper', 'adaptive', 'force-feedback'],
  '1.8.0'
),
(
  'PowerCore 500W',
  'Industrial power supply unit with backup capabilities',
  'POWER',
  '{"output_power": 500, "voltage_range": [12, 48], "efficiency": 0.95, "backup_time": 30}',
  ARRAY['power-supply', 'backup', 'industrial'],
  '3.0.1'
);

-- Create sample locus
INSERT INTO loci (name, description, coordinates) VALUES (
  'Factory Floor A',
  'Main production floor with assembly lines',
  '{"latitude": 37.7749, "longitude": -122.4194, "altitude": 10}'
);

-- Create sample fleet
INSERT INTO fleets (name, description, locus_id, status) VALUES (
  'Assembly Line Alpha',
  'Primary assembly line robots',
  (SELECT id FROM loci WHERE name = 'Factory Floor A'),
  'IDLE'
);

COMMIT;
