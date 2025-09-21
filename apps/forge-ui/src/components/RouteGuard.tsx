'use client';

import { useAuth } from './AuthProvider';
import { useRouter } from 'next/navigation';
import { useEffect } from 'react';
import Link from 'next/link';

interface RouteGuardProps {
  children: React.ReactNode;
  requiresAuth?: boolean;
  minRole?: 'SMITH' | 'OVER_SMITH' | 'ADMIN';
  fallback?: React.ReactNode;
}

export function RouteGuard({ 
  children, 
  requiresAuth = false, 
  minRole,
  fallback 
}: RouteGuardProps) {
  const { smith, loading } = useAuth();
  const router = useRouter();

  useEffect(() => {
    // Don't redirect during loading
    if (loading) return;

    // If authentication is required but user is not logged in
    if (requiresAuth && !smith) {
      router.push('/auth/signin');
      return;
    }

    // If role is required but user doesn't have sufficient role
    if (minRole && smith) {
      const roleHierarchy = { 'SMITH': 1, 'OVER_SMITH': 2, 'ADMIN': 3 };
      const userLevel = roleHierarchy[smith.role] || 0;
      const requiredLevel = roleHierarchy[minRole] || 0;
      
      if (userLevel < requiredLevel) {
        router.push('/'); // Redirect to home if insufficient permissions
        return;
      }
    }
  }, [smith, loading, requiresAuth, minRole, router]);

  // Show loading state during authentication check
  if (loading) {
    return (
      <div className="max-w-7xl mx-auto px-4 py-8">
        <div className="text-center py-12">
          <div className="animate-spin rounded-full h-8 w-8 border-2 border-orange-600 border-t-transparent mx-auto mb-4"></div>
          <p className="text-gray-600">Loading...</p>
        </div>
      </div>
    );
  }

  // Show authentication required state
  if (requiresAuth && !smith) {
    if (fallback) {
      return <>{fallback}</>;
    }

    return (
      <div className="max-w-7xl mx-auto px-4 py-8">
        <div className="text-center py-12">
          <div className="text-6xl mb-4">🔒</div>
          <h1 className="text-2xl font-bold text-gray-900 mb-4">Authentication Required</h1>
          <p className="text-gray-600 mb-8">Please sign in to access this page</p>
          <Link
            href="/auth/signin"
            className="bg-orange-600 text-white px-6 py-3 rounded-lg hover:bg-orange-700 focus:outline-none focus:ring-2 focus:ring-offset-2 focus:ring-orange-500"
          >
            Sign In
          </Link>
        </div>
      </div>
    );
  }

  // Show insufficient permissions state
  if (minRole && smith) {
    const roleHierarchy = { 'SMITH': 1, 'OVER_SMITH': 2, 'ADMIN': 3 };
    const userLevel = roleHierarchy[smith.role] || 0;
    const requiredLevel = roleHierarchy[minRole] || 0;
    
    if (userLevel < requiredLevel) {
      return (
        <div className="max-w-7xl mx-auto px-4 py-8">
          <div className="text-center py-12">
            <div className="text-6xl mb-4">⚠️</div>
            <h1 className="text-2xl font-bold text-gray-900 mb-4">Insufficient Permissions</h1>
            <p className="text-gray-600 mb-8">
              This page requires {minRole.replace('_', ' ').toLowerCase()} level access or higher
            </p>
            <Link
              href="/"
              className="bg-gray-600 text-white px-6 py-3 rounded-lg hover:bg-gray-700"
            >
              Go Home
            </Link>
          </div>
        </div>
      );
    }
  }

  // User is authenticated and has sufficient permissions
  return <>{children}</>;
}
