// Jest setup for React Testing Library and custom matchers
import '@testing-library/jest-dom';
import React from 'react';

// Mock Next.js router
jest.mock('next/navigation', () => ({
  useRouter: () => ({
    push: jest.fn(),
    replace: jest.fn(),
    back: jest.fn(),
    forward: jest.fn(),
    refresh: jest.fn(),
    pathname: '/',
    query: {},
    asPath: '/',
  }),
  useSearchParams: () => ({
    get: jest.fn(),
  }),
  usePathname: () => '/',
}));

// Mock Next.js dynamic imports
jest.mock('next/dynamic', () => (fn: any) => {
  const dynamicModule = fn();
  return dynamicModule.default || dynamicModule;
});

// Mock Next.js Link component
jest.mock('next/link', () => {
  return ({ children, href, ...props }: any) => {
    return React.createElement('a', { href, ...props }, children);
  };
});

// Global test setup
beforeEach(() => {
  // Clear localStorage
  Object.defineProperty(window, 'localStorage', {
    value: {
      getItem: jest.fn(),
      setItem: jest.fn(),
      removeItem: jest.fn(),
      clear: jest.fn(),
    },
    writable: true,
  });

  // Mock window.alert and window.confirm
  window.alert = jest.fn();
  window.confirm = jest.fn();

  // Mock URLSearchParams
  global.URLSearchParams = jest.fn().mockImplementation(() => ({
    get: jest.fn(),
    set: jest.fn(),
    toString: jest.fn(),
  }));
});

afterEach(() => {
  jest.clearAllMocks();
});


