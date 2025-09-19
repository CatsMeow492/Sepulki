// Sepulki Shared Types
// Common TypeScript interfaces aligned with GraphQL schema

export * from './sepulka';
export * from './alloy';
export * from './fleet';
export * from './task';
export * from './auth';
export * from './telemetry';
export * from './policy';

// Base types
export interface BaseEntity {
  id: string;
  createdAt: Date;
  updatedAt: Date;
}

export interface Coordinates {
  latitude: number;
  longitude: number;
  altitude?: number;
}

export interface TimeRange {
  start: Date;
  end: Date;
}

export interface Error {
  code: string;
  message: string;
  field?: string;
}

// Pagination
export interface PaginationArgs {
  limit?: number;
  offset?: number;
}

export interface PaginatedResponse<T> {
  items: T[];
  total: number;
  hasNext: boolean;
}
