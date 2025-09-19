import DataLoader from 'dataloader';
import type { Pool } from 'pg';
import type { 
  Sepulka, 
  Alloy, 
  Pattern, 
  Fleet, 
  Robot, 
  Task, 
  Smith 
} from '@sepulki/shared-types';

export function setupDataLoaders(db: Pool) {
  
  // Sepulka loaders
  const sepulkaLoader = new DataLoader<string, Sepulka | null>(async (ids) => {
    const query = 'SELECT * FROM sepulkas WHERE id = ANY($1)';
    const result = await db.query(query, [ids]);
    
    const sepulkaMap = new Map(result.rows.map(row => [row.id, row]));
    return ids.map(id => sepulkaMap.get(id) || null);
  });

  // Alloy loaders
  const alloyLoader = new DataLoader<string, Alloy | null>(async (ids) => {
    const query = 'SELECT * FROM alloys WHERE id = ANY($1)';
    const result = await db.query(query, [ids]);
    
    const alloyMap = new Map(result.rows.map(row => [row.id, row]));
    return ids.map(id => alloyMap.get(id) || null);
  });

  const alloysBySepulkaLoader = new DataLoader<string, Alloy[]>(async (sepulkaIds) => {
    const query = `
      SELECT a.*, sa.sepulka_id 
      FROM alloys a 
      JOIN sepulka_alloys sa ON a.id = sa.alloy_id 
      WHERE sa.sepulka_id = ANY($1)
    `;
    const result = await db.query(query, [sepulkaIds]);
    
    const alloyMap = new Map<string, Alloy[]>();
    sepulkaIds.forEach(id => alloyMap.set(id, []));
    
    result.rows.forEach(row => {
      const sepulkaAlloys = alloyMap.get(row.sepulka_id) || [];
      sepulkaAlloys.push(row);
      alloyMap.set(row.sepulka_id, sepulkaAlloys);
    });
    
    return sepulkaIds.map(id => alloyMap.get(id) || []);
  });

  // Pattern loader
  const patternLoader = new DataLoader<string, Pattern | null>(async (ids) => {
    const query = 'SELECT * FROM patterns WHERE id = ANY($1)';
    const result = await db.query(query, [ids]);
    
    const patternMap = new Map(result.rows.map(row => [row.id, row]));
    return ids.map(id => patternMap.get(id) || null);
  });

  // Fleet loaders
  const fleetLoader = new DataLoader<string, Fleet | null>(async (ids) => {
    const query = 'SELECT * FROM fleets WHERE id = ANY($1)';
    const result = await db.query(query, [ids]);
    
    const fleetMap = new Map(result.rows.map(row => [row.id, row]));
    return ids.map(id => fleetMap.get(id) || null);
  });

  const robotsByFleetLoader = new DataLoader<string, Robot[]>(async (fleetIds) => {
    const query = 'SELECT * FROM robots WHERE fleet_id = ANY($1)';
    const result = await db.query(query, [fleetIds]);
    
    const robotMap = new Map<string, Robot[]>();
    fleetIds.forEach(id => robotMap.set(id, []));
    
    result.rows.forEach(row => {
      const fleetRobots = robotMap.get(row.fleet_id) || [];
      fleetRobots.push(row);
      robotMap.set(row.fleet_id, fleetRobots);
    });
    
    return fleetIds.map(id => robotMap.get(id) || []);
  });

  // Robot loader
  const robotLoader = new DataLoader<string, Robot | null>(async (ids) => {
    const query = 'SELECT * FROM robots WHERE id = ANY($1)';
    const result = await db.query(query, [ids]);
    
    const robotMap = new Map(result.rows.map(row => [row.id, row]));
    return ids.map(id => robotMap.get(id) || null);
  });

  // Task loaders
  const taskLoader = new DataLoader<string, Task | null>(async (ids) => {
    const query = 'SELECT * FROM tasks WHERE id = ANY($1)';
    const result = await db.query(query, [ids]);
    
    const taskMap = new Map(result.rows.map(row => [row.id, row]));
    return ids.map(id => taskMap.get(id) || null);
  });

  const tasksByFleetLoader = new DataLoader<string, Task[]>(async (fleetIds) => {
    const query = `
      SELECT t.*, tr.fleet_id 
      FROM tasks t 
      JOIN task_robots tr ON t.id = tr.task_id 
      JOIN robots r ON tr.robot_id = r.id 
      WHERE r.fleet_id = ANY($1)
    `;
    const result = await db.query(query, [fleetIds]);
    
    const taskMap = new Map<string, Task[]>();
    fleetIds.forEach(id => taskMap.set(id, []));
    
    result.rows.forEach(row => {
      const fleetTasks = taskMap.get(row.fleet_id) || [];
      fleetTasks.push(row);
      taskMap.set(row.fleet_id, fleetTasks);
    });
    
    return fleetIds.map(id => taskMap.get(id) || []);
  });

  // Smith loader
  const smithLoader = new DataLoader<string, Smith | null>(async (ids) => {
    const query = 'SELECT * FROM smiths WHERE id = ANY($1)';
    const result = await db.query(query, [ids]);
    
    const smithMap = new Map(result.rows.map(row => [row.id, row]));
    return ids.map(id => smithMap.get(id) || null);
  });

  return {
    sepulka: sepulkaLoader,
    alloy: alloyLoader,
    alloysBySepulka: alloysBySepulkaLoader,
    pattern: patternLoader,
    fleet: fleetLoader,
    robotsByFleet: robotsByFleetLoader,
    robot: robotLoader,
    task: taskLoader,
    tasksByFleet: tasksByFleetLoader,
    smith: smithLoader,
  };
}
