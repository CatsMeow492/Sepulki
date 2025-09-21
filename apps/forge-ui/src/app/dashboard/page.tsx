'use client'

import { useAuth } from '@/components/AuthProvider'
import { RouteGuard } from '@/components/RouteGuard'
import { useState, useEffect } from 'react'

// Mock data that would come from GraphQL API
const mockFleetData = {
  fleets: [
    {
      id: '1',
      name: 'Dev Fleet Alpha',
      status: 'ACTIVE',
      robotCount: 2,
      activeTask: 'Assembly Testing',
      location: 'Development Lab'
    },
    {
      id: '2', 
      name: 'Demo Warehouse Bots',
      status: 'IDLE',
      robotCount: 2,
      activeTask: 'Pick and Place Demo',
      location: 'Demo Warehouse'
    },
    {
      id: '3',
      name: 'Factory Test Units', 
      status: 'MAINTENANCE',
      robotCount: 0,
      activeTask: null,
      location: 'Factory Simulation'
    }
  ],
  robots: [
    { id: '1', name: 'DevBot-Alpha', status: 'WORKING', battery: 87.5, fleet: 'Dev Fleet Alpha' },
    { id: '2', name: 'DevBot-Beta', status: 'IDLE', battery: 92.1, fleet: 'Dev Fleet Alpha' },
    { id: '3', name: 'WarehouseBot-Demo-01', status: 'CHARGING', battery: 34.8, fleet: 'Demo Warehouse Bots' },
    { id: '4', name: 'WarehouseBot-Demo-02', status: 'WORKING', battery: 76.4, fleet: 'Demo Warehouse Bots' }
  ],
  tasks: [
    { id: '1', name: 'Pick and Place Demo', status: 'IN_PROGRESS', priority: 'NORMAL', robot: 'WarehouseBot-Demo-02' },
    { id: '2', name: 'Assembly Testing', status: 'PENDING', priority: 'HIGH', robot: 'DevBot-Alpha' },
    { id: '3', name: 'Quality Inspection', status: 'COMPLETED', priority: 'NORMAL', robot: 'DevBot-Beta' },
    { id: '4', name: 'Fleet Patrol', status: 'ASSIGNED', priority: 'LOW', robot: 'WarehouseBot-Demo-01' }
  ]
}

function DashboardPageContent() {
  const { smith } = useAuth()
  const [data, setData] = useState(mockFleetData)
  const [selectedFleet, setSelectedFleet] = useState<string | null>(null)

  const getStatusColor = (status: string) => {
    switch (status) {
      case 'ACTIVE': case 'WORKING': case 'IN_PROGRESS': 
        return 'text-green-600 bg-green-100'
      case 'IDLE': case 'PENDING':
        return 'text-blue-600 bg-blue-100'
      case 'CHARGING':
        return 'text-yellow-600 bg-yellow-100'
      case 'MAINTENANCE': case 'ASSIGNED':
        return 'text-orange-600 bg-orange-100'
      case 'OFFLINE': case 'COMPLETED':
        return 'text-gray-600 bg-gray-100'
      default:
        return 'text-gray-600 bg-gray-100'
    }
  }

  const getPriorityColor = (priority: string) => {
    switch (priority) {
      case 'URGENT': return 'text-red-600 bg-red-100'
      case 'HIGH': return 'text-orange-600 bg-orange-100'
      case 'NORMAL': return 'text-blue-600 bg-blue-100'
      case 'LOW': return 'text-gray-600 bg-gray-100'
      default: return 'text-gray-600 bg-gray-100'
    }
  }

  return (
    <div className="max-w-7xl mx-auto px-4 py-8">
      {/* Welcome Header */}
      <div className="mb-8">
        <h1 className="text-3xl font-bold text-gray-900 flex items-center">
          ⚒️ Fleet Dashboard
          {smith && (
            <span className="ml-4 text-lg font-normal text-gray-600">
              Welcome, {smith.name}
            </span>
          )}
        </h1>
        <p className="text-gray-600 mt-2">Monitor and manage your robot fleets</p>
      </div>

      {/* Stats Overview */}
      <div className="grid grid-cols-1 md:grid-cols-4 gap-6 mb-8">
        <div className="bg-white p-6 rounded-lg shadow-sm border">
          <div className="flex items-center">
            <div className="text-2xl mr-3">🏭</div>
            <div>
              <p className="text-sm font-medium text-gray-500">Active Fleets</p>
              <p className="text-2xl font-bold text-gray-900">
                {data.fleets.filter(f => f.status === 'ACTIVE').length}
              </p>
            </div>
          </div>
        </div>

        <div className="bg-white p-6 rounded-lg shadow-sm border">
          <div className="flex items-center">
            <div className="text-2xl mr-3">🤖</div>
            <div>
              <p className="text-sm font-medium text-gray-500">Working Robots</p>
              <p className="text-2xl font-bold text-gray-900">
                {data.robots.filter(r => r.status === 'WORKING').length}
              </p>
            </div>
          </div>
        </div>

        <div className="bg-white p-6 rounded-lg shadow-sm border">
          <div className="flex items-center">
            <div className="text-2xl mr-3">⚡</div>
            <div>
              <p className="text-sm font-medium text-gray-500">Avg Battery</p>
              <p className="text-2xl font-bold text-gray-900">
                {Math.round(data.robots.reduce((acc, r) => acc + r.battery, 0) / data.robots.length)}%
              </p>
            </div>
          </div>
        </div>

        <div className="bg-white p-6 rounded-lg shadow-sm border">
          <div className="flex items-center">
            <div className="text-2xl mr-3">📋</div>
            <div>
              <p className="text-sm font-medium text-gray-500">Active Tasks</p>
              <p className="text-2xl font-bold text-gray-900">
                {data.tasks.filter(t => ['IN_PROGRESS', 'ASSIGNED', 'PENDING'].includes(t.status)).length}
              </p>
            </div>
          </div>
        </div>
      </div>

      <div className="grid grid-cols-1 lg:grid-cols-2 gap-8">
        {/* Fleet Status */}
        <div className="bg-white rounded-lg shadow-sm border">
          <div className="p-6">
            <h2 className="text-xl font-bold text-gray-900 mb-4 flex items-center">
              🏭 Fleet Status
            </h2>
            <div className="space-y-4">
              {data.fleets.map((fleet) => (
                <div 
                  key={fleet.id} 
                  className="p-4 border rounded-lg hover:bg-gray-50 cursor-pointer"
                  onClick={() => setSelectedFleet(fleet.id)}
                >
                  <div className="flex items-center justify-between">
                    <div>
                      <h3 className="font-medium text-gray-900">{fleet.name}</h3>
                      <p className="text-sm text-gray-500">{fleet.location}</p>
                    </div>
                    <div className="text-right">
                      <span className={`inline-flex px-2 py-1 text-xs font-medium rounded-full ${getStatusColor(fleet.status)}`}>
                        {fleet.status}
                      </span>
                      <p className="text-sm text-gray-500 mt-1">
                        {fleet.robotCount} robots
                      </p>
                    </div>
                  </div>
                  {fleet.activeTask && (
                    <div className="mt-2 text-sm text-blue-600">
                      Current: {fleet.activeTask}
                    </div>
                  )}
                </div>
              ))}
            </div>
          </div>
        </div>

        {/* Robot Status */}
        <div className="bg-white rounded-lg shadow-sm border">
          <div className="p-6">
            <h2 className="text-xl font-bold text-gray-900 mb-4 flex items-center">
              🤖 Robot Status
            </h2>
            <div className="space-y-4">
              {data.robots.map((robot) => (
                <div key={robot.id} className="flex items-center justify-between p-3 border rounded-lg">
                  <div>
                    <h3 className="font-medium text-gray-900">{robot.name}</h3>
                    <p className="text-sm text-gray-500">{robot.fleet}</p>
                  </div>
                  <div className="text-right">
                    <span className={`inline-flex px-2 py-1 text-xs font-medium rounded-full ${getStatusColor(robot.status)}`}>
                      {robot.status}
                    </span>
                    <div className="mt-1 flex items-center">
                      <div className="w-16 bg-gray-200 rounded-full h-2 mr-2">
                        <div 
                          className={`h-2 rounded-full ${
                            robot.battery > 50 ? 'bg-green-500' : 
                            robot.battery > 20 ? 'bg-yellow-500' : 'bg-red-500'
                          }`}
                          style={{ width: `${robot.battery}%` }}
                        ></div>
                      </div>
                      <span className="text-xs text-gray-500">{robot.battery}%</span>
                    </div>
                  </div>
                </div>
              ))}
            </div>
          </div>
        </div>

        {/* Active Tasks */}
        <div className="bg-white rounded-lg shadow-sm border">
          <div className="p-6">
            <h2 className="text-xl font-bold text-gray-900 mb-4 flex items-center">
              📋 Active Tasks
            </h2>
            <div className="space-y-4">
              {data.tasks.map((task) => (
                <div key={task.id} className="p-4 border rounded-lg">
                  <div className="flex items-center justify-between">
                    <div>
                      <h3 className="font-medium text-gray-900">{task.name}</h3>
                      <p className="text-sm text-gray-500">Assigned to: {task.robot}</p>
                    </div>
                    <div className="text-right">
                      <span className={`inline-flex px-2 py-1 text-xs font-medium rounded-full ${getStatusColor(task.status)}`}>
                        {task.status}
                      </span>
                      <div className="mt-1">
                        <span className={`inline-flex px-2 py-1 text-xs font-medium rounded-full ${getPriorityColor(task.priority)}`}>
                          {task.priority}
                        </span>
                      </div>
                    </div>
                  </div>
                </div>
              ))}
            </div>
          </div>
        </div>

        {/* System Health */}
        <div className="bg-white rounded-lg shadow-sm border">
          <div className="p-6">
            <h2 className="text-xl font-bold text-gray-900 mb-4 flex items-center">
              🏥 System Health
            </h2>
            <div className="space-y-3">
              <div className="flex items-center justify-between">
                <span className="text-gray-700">Database Connection</span>
                <span className="inline-flex px-2 py-1 text-xs font-medium rounded-full text-green-600 bg-green-100">
                  HEALTHY
                </span>
              </div>
              <div className="flex items-center justify-between">
                <span className="text-gray-700">GraphQL API</span>
                <span className="inline-flex px-2 py-1 text-xs font-medium rounded-full text-green-600 bg-green-100">
                  RESPONDING
                </span>
              </div>
              <div className="flex items-center justify-between">
                <span className="text-gray-700">Asset Storage</span>
                <span className="inline-flex px-2 py-1 text-xs font-medium rounded-full text-green-600 bg-green-100">
                  CONNECTED
                </span>
              </div>
              <div className="flex items-center justify-between">
                <span className="text-gray-700">Telemetry Stream</span>
                <span className="inline-flex px-2 py-1 text-xs font-medium rounded-full text-yellow-600 bg-yellow-100">
                  SIMULATED
                </span>
              </div>
            </div>
            
            <div className="mt-6 pt-4 border-t">
              <p className="text-sm text-gray-500 mb-2">Development Mode Active</p>
              <div className="text-xs text-blue-600 bg-blue-50 p-3 rounded">
                <p>🧪 Using mock authentication and simulated telemetry</p>
                <p>💾 Database: 4 smiths, 3 sepulkas, 4 robots, 4 tasks</p>
                <p>🔗 GraphQL: http://localhost:4000/graphql</p>
              </div>
            </div>
          </div>
        </div>
      </div>
    </div>
  )
}

export default function DashboardPage() {
  return (
    <RouteGuard requiresAuth={true} minRole="SMITH">
      <DashboardPageContent />
    </RouteGuard>
  );
}
