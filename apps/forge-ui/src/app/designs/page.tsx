'use client';

import { useAuth } from '@/components/AuthProvider';
import { useState, useEffect } from 'react';
import { getMySepulkas, castIngot, deleteSepulka } from '@/lib/graphql';
import Link from 'next/link';

interface Sepulka {
  id: string;
  name: string;
  description?: string;
  version: string;
  status: string;
  pattern?: {
    name: string;
    category: string;
  };
  alloys: Array<{
    name: string;
    type: string;
  }>;
  createdAt: string;
  updatedAt: string;
}

export default function MyDesignsPage() {
  const { smith } = useAuth();
  const [designs, setDesigns] = useState<Sepulka[]>([]);
  const [loading, setLoading] = useState(true);
  const [error, setError] = useState<string | null>(null);
  const [filter, setFilter] = useState<'all' | 'ready' | 'building' | 'deployed'>('all');
  const [actionLoading, setActionLoading] = useState<string | null>(null);

  // Check for new design highlight
  useEffect(() => {
    const urlParams = new URLSearchParams(window.location.search);
    const newDesignId = urlParams.get('newDesign');
    if (newDesignId) {
      // Highlight the new design briefly
      setTimeout(() => {
        const element = document.getElementById(`design-${newDesignId}`);
        if (element) {
          element.scrollIntoView({ behavior: 'smooth' });
          element.classList.add('ring-2', 'ring-orange-500', 'ring-opacity-50');
          setTimeout(() => {
            element.classList.remove('ring-2', 'ring-orange-500', 'ring-opacity-50');
          }, 3000);
        }
      }, 500);
    }
  }, [designs]);

  useEffect(() => {
    if (smith) {
      // Small delay to ensure global auth context is set
      setTimeout(() => {
        loadDesigns();
      }, 100);
    }
  }, [smith]);

  const loadDesigns = async () => {
    try {
      setLoading(true);
      setError(null);
      const userDesigns = await getMySepulkas(smith?.id);
      setDesigns(userDesigns || []);
    } catch (err) {
      console.error('Failed to load designs:', err);
      setError(err instanceof Error ? err.message : 'Failed to load designs');
    } finally {
      setLoading(false);
    }
  };

  const handleBuildDesign = async (design: Sepulka) => {
    try {
      setActionLoading(design.id);
      const result = await castIngot(design.id);
      
      if (result.errors && result.errors.length > 0) {
        alert(`Build failed: ${result.errors[0].message}`);
        return;
      }

      alert(`🔨 Build started! Ingot ${result.ingot?.id} is being created.`);
      
      // Refresh designs to show updated status
      await loadDesigns();
    } catch (error) {
      console.error('Build failed:', error);
      alert(`Build failed: ${error instanceof Error ? error.message : 'Unknown error'}`);
    } finally {
      setActionLoading(null);
    }
  };

  const handleDeleteDesign = async (design: Sepulka) => {
    if (!confirm(`Are you sure you want to delete "${design.name}"? This action cannot be undone.`)) {
      return;
    }

    try {
      setActionLoading(design.id);
      await deleteSepulka(design.id);
      
      // Remove from local state immediately for better UX
      setDesigns(prev => prev.filter(d => d.id !== design.id));
      
      // Show success message
      alert(`✅ "${design.name}" has been deleted successfully.`);
    } catch (error) {
      console.error('Delete failed:', error);
      alert(`Delete failed: ${error instanceof Error ? error.message : 'Unknown error'}`);
      // Refresh to restore the item if delete failed
      await loadDesigns();
    } finally {
      setActionLoading(null);
    }
  };

  const getStatusColor = (status: string) => {
    switch (status) {
      case 'FORGING':
        return 'text-orange-600 bg-orange-100';
      case 'CAST_READY':
        return 'text-blue-600 bg-blue-100';
      case 'CASTING':
        return 'text-yellow-600 bg-yellow-100';
      case 'INGOT_READY':
        return 'text-green-600 bg-green-100';
      case 'DEPLOYED':
        return 'text-purple-600 bg-purple-100';
      default:
        return 'text-gray-600 bg-gray-100';
    }
  };

  const getStatusIcon = (status: string) => {
    switch (status) {
      case 'FORGING':
        return '🔥';
      case 'CAST_READY':
        return '⚡';
      case 'CASTING':
        return '🔨';
      case 'INGOT_READY':
        return '📦';
      case 'DEPLOYED':
        return '🚀';
      default:
        return '⚪';
    }
  };

  const filteredDesigns = designs.filter(design => {
    if (filter === 'all') return true;
    if (filter === 'ready') return design.status === 'CAST_READY' || design.status === 'READY';
    if (filter === 'building') return design.status === 'CASTING';
    if (filter === 'deployed') return design.status === 'DEPLOYED';
    return true;
  });

  if (!smith) {
    return (
      <div className="max-w-7xl mx-auto px-4 py-8">
        <div className="text-center py-12">
          <h1 className="text-2xl font-bold text-gray-900 mb-4">Authentication Required</h1>
          <p className="text-gray-600 mb-8">Please sign in to view your robot designs</p>
          <Link
            href="/auth/signin"
            className="bg-orange-600 text-white px-6 py-3 rounded-lg hover:bg-orange-700"
          >
            Sign In
          </Link>
        </div>
      </div>
    );
  }

  return (
    <div className="max-w-7xl mx-auto px-4 py-8">
      {/* Header */}
      <div className="flex justify-between items-center mb-8">
        <div>
          <h1 className="text-3xl font-bold text-gray-900 flex items-center">
            🔥 My Designs
          </h1>
          <p className="text-gray-600 mt-2">Your saved robot configurations and builds</p>
        </div>
        <Link
          href="/configure"
          className="bg-orange-600 text-white px-6 py-3 rounded-lg hover:bg-orange-700 focus:outline-none focus:ring-2 focus:ring-offset-2 focus:ring-orange-500 flex items-center"
        >
          ➕ Create New Design
        </Link>
      </div>

      {/* Filter Tabs */}
      <div className="border-b border-gray-200 mb-6">
        <nav className="-mb-px flex space-x-8">
          {[
            { key: 'all', label: 'All Designs', count: designs.length },
            { key: 'ready', label: 'Ready to Build', count: designs.filter(d => d.status === 'CAST_READY' || d.status === 'READY').length },
            { key: 'building', label: 'Building', count: designs.filter(d => d.status === 'CASTING').length },
            { key: 'deployed', label: 'Deployed', count: designs.filter(d => d.status === 'DEPLOYED').length },
          ].map((tab) => (
            <button
              key={tab.key}
              onClick={() => setFilter(tab.key as any)}
              className={`${
                filter === tab.key
                  ? 'border-orange-500 text-orange-600'
                  : 'border-transparent text-gray-500 hover:text-gray-700 hover:border-gray-300'
              } whitespace-nowrap py-2 px-1 border-b-2 font-medium text-sm flex items-center`}
            >
              {tab.label}
              {tab.count > 0 && (
                <span className={`ml-2 py-0.5 px-2 text-xs rounded-full ${
                  filter === tab.key ? 'bg-orange-100 text-orange-800' : 'bg-gray-100 text-gray-600'
                }`}>
                  {tab.count}
                </span>
              )}
            </button>
          ))}
        </nav>
      </div>

      {/* Content */}
      {loading ? (
        <div className="text-center py-12">
          <div className="animate-spin rounded-full h-8 w-8 border-2 border-orange-600 border-t-transparent mx-auto mb-4"></div>
          <p className="text-gray-600">Loading your designs...</p>
        </div>
      ) : error ? (
        <div className="bg-red-50 border border-red-200 rounded-lg p-6 mb-6">
          <div className="flex">
            <div className="flex-shrink-0">
              <svg className="h-5 w-5 text-red-400" viewBox="0 0 20 20" fill="currentColor">
                <path fillRule="evenodd" d="M10 18a8 8 0 100-16 8 8 0 000 16zM8.707 7.293a1 1 0 00-1.414 1.414L8.586 10l-1.293 1.293a1 1 0 101.414 1.414L10 11.414l1.293 1.293a1 1 0 001.414-1.414L11.414 10l1.293-1.293a1 1 0 00-1.414-1.414L10 8.586 8.707 7.293z" clipRule="evenodd" />
              </svg>
            </div>
            <div className="ml-3">
              <h3 className="text-sm font-medium text-red-800">Error loading designs</h3>
              <p className="text-sm text-red-700 mt-1">{error}</p>
              <button
                onClick={loadDesigns}
                className="mt-2 text-sm bg-red-100 text-red-800 px-3 py-1 rounded hover:bg-red-200"
              >
                Try Again
              </button>
            </div>
          </div>
        </div>
      ) : filteredDesigns.length === 0 ? (
        <div className="text-center py-12">
          <div className="text-6xl mb-4">🔥</div>
          <h3 className="text-lg font-medium text-gray-900 mb-2">
            {filter === 'all' ? 'No designs yet' : `No ${filter} designs`}
          </h3>
          <p className="text-gray-600 mb-8">
            {filter === 'all' 
              ? "Start by creating your first robot design"
              : `You don't have any designs in ${filter} status`
            }
          </p>
          <Link
            href="/configure"
            className="bg-orange-600 text-white px-6 py-3 rounded-lg hover:bg-orange-700"
          >
            🔥 Forge Your First Robot
          </Link>
        </div>
      ) : (
        <div className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-3 gap-6">
          {filteredDesigns.map((design) => (
            <div 
              key={design.id} 
              id={`design-${design.id}`}
              className="bg-white rounded-lg shadow-sm border hover:shadow-md transition-all duration-200"
            >
              <div className="p-6">
                {/* Header */}
                <div className="flex items-start justify-between mb-4">
                  <div className="flex-1">
                    <h3 className="text-lg font-semibold text-gray-900 mb-1">
                      {design.name}
                    </h3>
                    {design.description && (
                      <p className="text-sm text-gray-600 mb-2 line-clamp-2">
                        {design.description}
                      </p>
                    )}
                  </div>
                  <div className="ml-3 flex-shrink-0">
                    <span className={`inline-flex items-center px-2.5 py-0.5 rounded-full text-xs font-medium ${getStatusColor(design.status)}`}>
                      {getStatusIcon(design.status)} {design.status}
                    </span>
                  </div>
                </div>

                {/* Design Details */}
                <div className="space-y-3 mb-6">
                  {design.pattern && (
                    <div className="flex items-center text-sm">
                      <span className="text-gray-500 w-20">Pattern:</span>
                      <span className="text-gray-900 font-medium">{design.pattern.name}</span>
                    </div>
                  )}
                  
                  <div className="flex items-center text-sm">
                    <span className="text-gray-500 w-20">Version:</span>
                    <span className="text-gray-900 font-mono">{design.version}</span>
                  </div>

                  <div className="flex items-center text-sm">
                    <span className="text-gray-500 w-20">Components:</span>
                    <span className="text-gray-900">{design.alloys.length} selected</span>
                  </div>

                  <div className="flex items-center text-sm">
                    <span className="text-gray-500 w-20">Created:</span>
                    <span className="text-gray-900">
                      {new Date(design.createdAt).toLocaleDateString()}
                    </span>
                  </div>
                </div>

                {/* Component Tags */}
                <div className="mb-6">
                  <div className="flex flex-wrap gap-1">
                    {design.alloys.slice(0, 3).map((alloy, index) => (
                      <span
                        key={index}
                        className="inline-flex items-center px-2 py-1 rounded-md text-xs font-medium bg-gray-100 text-gray-800"
                      >
                        {alloy.type === 'ACTUATOR' ? '⚙️' : 
                         alloy.type === 'END_EFFECTOR' ? '🤏' :
                         alloy.type === 'SENSOR' ? '👁️' : '🔧'} {alloy.name}
                      </span>
                    ))}
                    {design.alloys.length > 3 && (
                      <span className="inline-flex items-center px-2 py-1 rounded-md text-xs font-medium bg-gray-100 text-gray-600">
                        +{design.alloys.length - 3} more
                      </span>
                    )}
                  </div>
                </div>

                {/* Actions */}
                <div className="flex items-center justify-between pt-4 border-t">
                  <div className="flex space-x-2">
                    <Link
                      href={`/configure?editDesign=${design.id}`}
                      className="text-blue-600 hover:text-blue-700 text-sm font-medium"
                    >
                      📝 Edit
                    </Link>
                    <button
                      className="text-gray-600 hover:text-gray-700 text-sm font-medium"
                      onClick={() => {
                        // Create a duplicate by redirecting to configure with preset data
                        const duplicateName = `${design.name} (Copy)`;
                        localStorage.setItem('duplicateDesign', JSON.stringify({
                          name: duplicateName,
                          description: design.description,
                          patternId: design.pattern?.name,
                          alloys: design.alloys
                        }));
                        window.location.href = '/configure?duplicate=true';
                      }}
                    >
                      📋 Duplicate
                    </button>
                    {(design.status === 'CAST_READY' || design.status === 'READY') && (
                      <button
                        className="text-green-600 hover:text-green-700 text-sm font-medium disabled:opacity-50"
                        onClick={() => handleBuildDesign(design)}
                        disabled={actionLoading === design.id}
                      >
                        {actionLoading === design.id ? '⏳' : '🔨'} Build
                      </button>
                    )}
                  </div>
                  <button
                    className="text-red-600 hover:text-red-700 text-sm disabled:opacity-50"
                    onClick={() => handleDeleteDesign(design)}
                    disabled={actionLoading === design.id}
                  >
                    {actionLoading === design.id ? '⏳' : '🗑️'}
                  </button>
                </div>
              </div>
            </div>
          ))}
        </div>
      )}

      {/* Summary Stats */}
      {!loading && !error && designs.length > 0 && (
        <div className="mt-12 bg-gray-50 rounded-lg p-6">
          <h3 className="text-lg font-semibold text-gray-900 mb-4">📊 Design Portfolio Summary</h3>
          <div className="grid grid-cols-2 md:grid-cols-4 gap-4">
            <div className="text-center">
              <div className="text-2xl font-bold text-orange-600">{designs.length}</div>
              <div className="text-sm text-gray-600">Total Designs</div>
            </div>
            <div className="text-center">
              <div className="text-2xl font-bold text-blue-600">
                {designs.filter(d => d.status === 'CAST_READY').length}
              </div>
              <div className="text-sm text-gray-600">Ready to Build</div>
            </div>
            <div className="text-center">
              <div className="text-2xl font-bold text-green-600">
                {new Set(designs.map(d => d.pattern?.category)).size}
              </div>
              <div className="text-sm text-gray-600">Pattern Types</div>
            </div>
            <div className="text-center">
              <div className="text-2xl font-bold text-purple-600">
                {designs.reduce((acc, d) => acc + d.alloys.length, 0)}
              </div>
              <div className="text-sm text-gray-600">Total Components</div>
            </div>
          </div>
        </div>
      )}

      {/* Quick Actions */}
      <div className="mt-8 flex justify-center">
        <div className="flex space-x-4">
          <Link
            href="/configure"
            className="bg-orange-600 text-white px-6 py-3 rounded-lg hover:bg-orange-700 flex items-center"
          >
            🔥 Create New Design
          </Link>
          <Link
            href="/dashboard" 
            className="bg-gray-600 text-white px-6 py-3 rounded-lg hover:bg-gray-700 flex items-center"
          >
            ⚒️ Fleet Dashboard
          </Link>
        </div>
      </div>
    </div>
  );
}
