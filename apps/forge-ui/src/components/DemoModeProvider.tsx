'use client';

import { createContext, useContext, useState, useEffect } from 'react';

interface DemoData {
  sepulkas: Array<{
    id: string;
    name: string;
    description?: string;
    status: string;
    pattern: { name: string };
    alloys: Array<{ name: string; type: string }>;
    createdAt: string;
    updatedAt: string;
  }>;
  ingots: Array<{
    id: string;
    sepulkaId: string;
    version: string;
    buildHash: string;
    status: string;
    artifacts: Array<{ type: string; path: string; checksum: string }>;
    createdAt: string;
  }>;
}

interface DemoContextType {
  demoMode: boolean;
  demoData: DemoData;
  updateDesignStatus: (id: string, status: string) => void;
  addIngot: (sepulkaId: string, ingot: any) => void;
  deleteDesign: (id: string) => void;
  refreshData: () => void;
}

const DemoContext = createContext<DemoContextType | null>(null);

export function useDemoMode() {
  const context = useContext(DemoContext);
  if (!context) {
    throw new Error('useDemoMode must be used within a DemoModeProvider');
  }
  return context;
}

// Rich demo data showcasing different design states
const initialDemoData: DemoData = {
  sepulkas: [
    {
      id: 'demo-sepulka-001',
      name: 'Warehouse Picker Pro',
      description: 'Advanced warehouse automation with vision guidance and 50kg payload capacity',
      status: 'CAST_READY',
      pattern: { name: 'Industrial Arm - 6DOF' },
      alloys: [
        { name: 'ServoMax Pro 3000', type: 'ACTUATOR' },
        { name: 'GripForce Elite', type: 'END_EFFECTOR' },
        { name: 'VisionEye 4K', type: 'SENSOR' },
        { name: 'MotionController X1', type: 'CONTROLLER' }
      ],
      createdAt: new Date(Date.now() - 2 * 24 * 60 * 60 * 1000).toISOString(),
      updatedAt: new Date(Date.now() - 1 * 24 * 60 * 60 * 1000).toISOString(),
    },
    {
      id: 'demo-sepulka-002', 
      name: 'Assembly Line Assistant',
      description: 'Precision assembly robot for electronics manufacturing',
      status: 'FORGING',
      pattern: { name: 'Precision Assembler - 4DOF' },
      alloys: [
        { name: 'PrecisionDrive Mini', type: 'ACTUATOR' },
        { name: 'MicroGripper v2', type: 'END_EFFECTOR' },
        { name: 'ForceGuard Pro', type: 'SENSOR' }
      ],
      createdAt: new Date(Date.now() - 5 * 60 * 60 * 1000).toISOString(),
      updatedAt: new Date(Date.now() - 3 * 60 * 60 * 1000).toISOString(),
    },
    {
      id: 'demo-sepulka-003',
      name: 'Quality Inspector Bot', 
      description: 'Automated visual inspection with AI-powered defect detection',
      status: 'DEPLOYED',
      pattern: { name: 'Inspection Platform - 3DOF' },
      alloys: [
        { name: 'StealthMove Linear', type: 'ACTUATOR' },
        { name: 'VisionEye 8K Pro', type: 'SENSOR' },
        { name: 'AI EdgeBox v3', type: 'CONTROLLER' }
      ],
      createdAt: new Date(Date.now() - 7 * 24 * 60 * 60 * 1000).toISOString(),
      updatedAt: new Date(Date.now() - 2 * 24 * 60 * 60 * 1000).toISOString(),
    }
  ],
  ingots: [
    {
      id: 'demo-ingot-001',
      sepulkaId: 'demo-sepulka-003',
      version: '1.0.0',
      buildHash: 'build_1732195200_abc123def',
      status: 'DEPLOYED',
      artifacts: [
        { type: 'URDF', path: '/artifacts/quality-inspector-v1.urdf', checksum: 'sha256:abc123...' },
        { type: 'PACKAGE', path: '/artifacts/quality-inspector-v1.tar.gz', checksum: 'sha256:def456...' }
      ],
      createdAt: new Date(Date.now() - 6 * 24 * 60 * 60 * 1000).toISOString(),
    }
  ]
};

export function DemoModeProvider({ children }: { children: React.ReactNode }) {
  const [demoMode, setDemoMode] = useState(false);
  const [demoData, setDemoData] = useState<DemoData>(initialDemoData);

  useEffect(() => {
    // Enable demo mode when backend is unavailable
    const checkBackend = async () => {
      try {
        const response = await fetch('http://localhost:4000/graphql', {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify({ query: '{ __typename }' }),
        });
        
        if (!response.ok) {
          setDemoMode(true);
          console.log('📺 Demo mode activated - using local data');
        }
      } catch (error) {
        setDemoMode(true);
        console.log('📺 Demo mode activated - backend unavailable');
      }
    };

    checkBackend();
  }, []);

  const updateDesignStatus = (id: string, status: string) => {
    setDemoData(prev => ({
      ...prev,
      sepulkas: prev.sepulkas.map(s => 
        s.id === id ? { ...s, status, updatedAt: new Date().toISOString() } : s
      )
    }));
  };

  const addIngot = (sepulkaId: string, ingot: any) => {
    setDemoData(prev => ({
      ...prev,
      ingots: [...prev.ingots, {
        ...ingot,
        sepulkaId,
        createdAt: new Date().toISOString()
      }]
    }));
  };

  const deleteDesign = (id: string) => {
    setDemoData(prev => ({
      ...prev,
      sepulkas: prev.sepulkas.filter(s => s.id !== id),
      ingots: prev.ingots.filter(i => i.sepulkaId !== id)
    }));
  };

  const refreshData = () => {
    // Force re-render
    setDemoData(prev => ({ ...prev }));
  };

  return (
    <DemoContext.Provider value={{
      demoMode,
      demoData,
      updateDesignStatus,
      addIngot,
      deleteDesign,
      refreshData
    }}>
      {children}
    </DemoContext.Provider>
  );
}
