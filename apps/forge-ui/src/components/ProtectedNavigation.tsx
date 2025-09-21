'use client';

import Link from 'next/link';
import { useAuth } from './AuthProvider';

interface NavigationItem {
  href: string;
  label: string;
  requiresAuth: boolean;
  minRole?: 'SMITH' | 'OVER_SMITH' | 'ADMIN';
}

const navigationItems: NavigationItem[] = [
  {
    href: '/configure',
    label: 'Forge Robot',
    requiresAuth: false, // Basic robot configuration available to all
  },
  {
    href: '/designs',
    label: 'My Designs',
    requiresAuth: true, // Requires authentication to view saved designs
    minRole: 'SMITH',
  },
  {
    href: '/dashboard',
    label: 'Fleet Dashboard',
    requiresAuth: true, // Requires authentication to view fleet data
    minRole: 'SMITH',
  },
  {
    href: '/pricing',
    label: 'Pricing',
    requiresAuth: false, // Public pricing information
  },
];

export function ProtectedNavigation() {
  const { smith, loading } = useAuth();

  // Don't render navigation during loading
  if (loading) {
    return (
      <div className="hidden sm:ml-6 sm:flex sm:space-x-8">
        <div className="animate-pulse bg-gray-200 h-6 w-24 rounded"></div>
        <div className="animate-pulse bg-gray-200 h-6 w-20 rounded"></div>
      </div>
    );
  }

  const visibleItems = navigationItems.filter(item => {
    // Public routes are always visible
    if (!item.requiresAuth) {
      return true;
    }

    // Protected routes require authentication
    if (!smith) {
      return false;
    }

    // Check role requirements if specified
    if (item.minRole) {
      const roleHierarchy = { 'SMITH': 1, 'OVER_SMITH': 2, 'ADMIN': 3 };
      const userLevel = roleHierarchy[smith.role] || 0;
      const requiredLevel = roleHierarchy[item.minRole] || 0;
      
      return userLevel >= requiredLevel;
    }

    return true;
  });

  return (
    <div className="hidden sm:ml-6 sm:flex sm:space-x-8">
      {visibleItems.map((item) => (
        <Link
          key={item.href}
          href={item.href}
          className="text-gray-900 inline-flex items-center px-1 pt-1 border-b-2 border-transparent hover:border-orange-500"
        >
          {item.label}
        </Link>
      ))}
    </div>
  );
}

export function AuthenticationButton() {
  const { smith, loading, signOut } = useAuth();

  if (loading) {
    return (
      <div className="animate-pulse bg-gray-200 h-10 w-24 rounded"></div>
    );
  }

  if (smith) {
    return (
      <div className="flex items-center space-x-3">
        <div className="text-sm text-gray-700">
          Welcome, <span className="font-medium">{smith.name}</span>
        </div>
        <button
          onClick={signOut}
          className="bg-gray-600 text-white px-4 py-2 rounded-md text-sm font-medium hover:bg-gray-700 focus:outline-none focus:ring-2 focus:ring-offset-2 focus:ring-gray-500"
        >
          Sign Out
        </button>
      </div>
    );
  }

  return (
    <Link
      href="/auth/signin"
      className="bg-orange-600 text-white px-4 py-2 rounded-md text-sm font-medium hover:bg-orange-700 focus:outline-none focus:ring-2 focus:ring-offset-2 focus:ring-orange-500"
    >
      Get Started
    </Link>
  );
}
