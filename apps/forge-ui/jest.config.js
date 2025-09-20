/** @type {import('jest').Config} */
const config = {
  testEnvironment: 'jest-environment-jsdom',
  setupFiles: ['jest-canvas-mock'],
  setupFilesAfterEnv: ['<rootDir>/jest.setup.ts'],
  moduleNameMapper: {
    '^@/(.*)$': '<rootDir>/src/$1',
    '\\.(css|less|sass|scss)$': 'identity-obj-proxy',
  },
  transform: {
    '^.+\\.(ts|tsx)$': [
      'ts-jest',
      { 
        tsconfig: '<rootDir>/tsconfig.json', 
        isolatedModules: true,
        jsx: 'react-jsx'
      },
    ],
  },
  testMatch: ['<rootDir>/src/**/*.(test|spec).(ts|tsx)'],
  transformIgnorePatterns: [
    'node_modules/(?!(.*\\.mjs$))'
  ],
}

module.exports = config


