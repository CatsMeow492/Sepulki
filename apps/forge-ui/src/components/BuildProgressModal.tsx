'use client';

import { useState, useEffect } from 'react';

interface Ingot {
  id: string;
  sepulkaId: string;
  version: string;
  buildHash: string;
  status: string;
  artifacts: Array<{
    type: string;
    path: string;
    checksum: string;
  }>;
  createdAt: string;
}

interface BuildProgressModalProps {
  isOpen: boolean;
  onClose: () => void;
  designName: string;
  ingot?: Ingot;
}

export function BuildProgressModal({ isOpen, onClose, designName, ingot }: BuildProgressModalProps) {
  const [progress, setProgress] = useState(0);
  const [currentStep, setCurrentStep] = useState('Initializing...');

  useEffect(() => {
    if (!isOpen || !ingot) return;

    // Simulate build progress
    const steps = [
      'Validating design specifications...',
      'Generating URDF configuration...',
      'Compiling robot assets...',
      'Creating deployment package...',
      'Signing build artifacts...',
      'Build complete! 🎉'
    ];

    let stepIndex = 0;
    const interval = setInterval(() => {
      setProgress((prev) => Math.min(prev + 15, 100));
      setCurrentStep(steps[Math.min(stepIndex, steps.length - 1)]);
      stepIndex++;
      
      if (stepIndex >= steps.length) {
        clearInterval(interval);
      }
    }, 800);

    return () => clearInterval(interval);
  }, [isOpen, ingot]);

  if (!isOpen) return null;

  return (
    <div className="fixed inset-0 bg-black bg-opacity-50 flex items-center justify-center z-50">
      <div className="bg-white rounded-lg p-8 max-w-md w-full mx-4">
        <div className="text-center">
          <div className="text-6xl mb-4">🔨</div>
          <h2 className="text-2xl font-bold text-gray-900 mb-2">Building Design</h2>
          <p className="text-gray-600 mb-6">"{designName}"</p>
          
          {ingot && (
            <div className="mb-6 p-4 bg-gray-50 rounded-lg text-left">
              <h3 className="font-semibold text-gray-900 mb-2">Build Details:</h3>
              <div className="space-y-1 text-sm text-gray-600">
                <div><strong>Version:</strong> {ingot.version}</div>
                <div><strong>Build Hash:</strong> {ingot.buildHash}</div>
                <div><strong>Status:</strong> {ingot.status}</div>
                <div><strong>Started:</strong> {new Date(ingot.createdAt).toLocaleString()}</div>
              </div>
            </div>
          )}

          <div className="mb-4">
            <div className="w-full bg-gray-200 rounded-full h-2 mb-2">
              <div 
                className="bg-orange-600 h-2 rounded-full transition-all duration-300"
                style={{ width: `${progress}%` }}
              ></div>
            </div>
            <p className="text-sm text-gray-600">{currentStep}</p>
          </div>

          {progress >= 100 ? (
            <div className="space-y-3">
              <div className="p-3 bg-green-50 rounded-lg">
                <p className="text-green-800 font-medium">✅ Build completed successfully!</p>
                <p className="text-green-600 text-sm">Your robot is ready for deployment.</p>
              </div>
              <div className="flex space-x-3">
                <button
                  onClick={onClose}
                  className="flex-1 px-4 py-2 bg-gray-600 text-white rounded-lg hover:bg-gray-700"
                >
                  Close
                </button>
                <button
                  onClick={() => {
                    // TODO: Navigate to deployment flow
                    onClose();
                  }}
                  className="flex-1 px-4 py-2 bg-orange-600 text-white rounded-lg hover:bg-orange-700"
                >
                  🚀 Deploy
                </button>
              </div>
            </div>
          ) : (
            <button
              onClick={onClose}
              className="px-6 py-2 text-gray-600 hover:text-gray-800"
            >
              Run in Background
            </button>
          )}
        </div>
      </div>
    </div>
  );
}
