export * from './sepulka';
export * from './alloy';
export * from './fleet';
export * from './task';
export * from './auth';
export * from './telemetry';
export * from './policy';
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
export interface PaginationArgs {
    limit?: number;
    offset?: number;
}
export interface PaginatedResponse<T> {
    items: T[];
    total: number;
    hasNext: boolean;
}
