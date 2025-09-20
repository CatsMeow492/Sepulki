'use client';

import { useState } from 'react';
import { forgeSepulka, type ForgeInput } from '@/lib/graphql';
import { useAuth } from './AuthProvider';

interface SaveDesignModalProps {
  isOpen: boolean;
  onClose: () => void;
  onSaved: (sepulkaId: string) => void;
  robotSpec: any;
  selectedAlloys: string[];
  patternId?: string;
}

export function SaveDesignModal({ 
  isOpen, 
  onClose, 
  onSaved, 
  robotSpec, 
  selectedAlloys,
  patternId 
}: SaveDesignModalProps) {
  const { smith } = useAuth();
  const [saving, setSaving] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const [formData, setFormData] = useState({
    name: '',
    description: '',
  });

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    
    if (!smith) {
      setError('You must be signed in to save designs');
      return;
    }

    if (!formData.name.trim()) {
      setError('Design name is required');
      return;
    }

    if (selectedAlloys.length === 0) {
      setError('At least one component must be selected');
      return;
    }

    setSaving(true);
    setError(null);

    try {
      // Prepare forge input from current design state
      const forgeInput: ForgeInput = {
        name: formData.name.trim(),
        description: formData.description.trim() || undefined,
        patternId: patternId || undefined,
        alloyIds: selectedAlloys,
        parameters: {
          // Extract parameters from robot spec
          ...(robotSpec?.parameters || {}),
          // Add any additional parameters from the UI
          reach: robotSpec?.reach || 1000,
          payload: robotSpec?.payload || 10,
          precision: robotSpec?.precision || 0.5,
          // Include joint configurations if available
          ...(robotSpec?.joints && { joints: robotSpec.joints }),
          // Include workspace envelope if defined
          ...(robotSpec?.workspace && { workspace: robotSpec.workspace }),
        }
      };

      console.log('🔨 Forging Sepulka with input:', forgeInput);

      const result = await forgeSepulka(forgeInput);

      if (result.errors && result.errors.length > 0) {
        // Handle GraphQL errors
        setError(result.errors[0].message);
        return;
      }

      if (!result.sepulka) {
        setError('Failed to create design - no data returned');
        return;
      }

      console.log('✅ Sepulka forged successfully:', result.sepulka);

      // Success! Reset form and notify parent
      setFormData({ name: '', description: '' });
      onSaved(result.sepulka.id);
      onClose();

    } catch (error) {
      console.error('❌ Failed to forge Sepulka:', error);
      setError(error instanceof Error ? error.message : 'Failed to save design');
    } finally {
      setSaving(false);
    }
  };

  if (!isOpen) return null;

  return (
    <div className="fixed inset-0 bg-black bg-opacity-50 flex items-center justify-center z-50">
      <div className="bg-white rounded-lg shadow-xl max-w-md w-full mx-4">
        <div className="p-6">
          <div className="flex justify-between items-center mb-6">
            <h2 className="text-2xl font-bold text-gray-900">🔥 Forge New Sepulka</h2>
            <button
              onClick={onClose}
              className="text-gray-400 hover:text-gray-600"
              disabled={saving}
            >
              <svg className="w-6 h-6" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M6 18L18 6M6 6l12 12" />
              </svg>
            </button>
          </div>

          {error && (
            <div className="mb-4 p-4 bg-red-50 border border-red-200 rounded-lg">
              <div className="flex">
                <div className="flex-shrink-0">
                  <svg className="h-5 w-5 text-red-400" viewBox="0 0 20 20" fill="currentColor">
                    <path fillRule="evenodd" d="M10 18a8 8 0 100-16 8 8 0 000 16zM8.707 7.293a1 1 0 00-1.414 1.414L8.586 10l-1.293 1.293a1 1 0 101.414 1.414L10 11.414l1.293 1.293a1 1 0 001.414-1.414L11.414 10l1.293-1.293a1 1 0 00-1.414-1.414L10 8.586 8.707 7.293z" clipRule="evenodd" />
                  </svg>
                </div>
                <div className="ml-3">
                  <p className="text-sm text-red-700">{error}</p>
                </div>
              </div>
            </div>
          )}

          <form onSubmit={handleSubmit}>
            <div className="space-y-4">
              <div>
                <label htmlFor="name" className="block text-sm font-medium text-gray-700 mb-2">
                  Design Name *
                </label>
                <input
                  type="text"
                  id="name"
                  value={formData.name}
                  onChange={(e) => setFormData(prev => ({ ...prev, name: e.target.value }))}
                  placeholder="e.g., WarehouseBot-Production-v1"
                  className="w-full px-3 py-2 border border-gray-300 rounded-lg focus:ring-2 focus:ring-orange-500 focus:border-orange-500"
                  disabled={saving}
                  autoFocus
                />
                <p className="mt-1 text-sm text-gray-500">
                  Choose a descriptive name for your robot design
                </p>
              </div>

              <div>
                <label htmlFor="description" className="block text-sm font-medium text-gray-700 mb-2">
                  Description
                </label>
                <textarea
                  id="description"
                  value={formData.description}
                  onChange={(e) => setFormData(prev => ({ ...prev, description: e.target.value }))}
                  placeholder="Describe the robot's purpose and capabilities..."
                  rows={3}
                  className="w-full px-3 py-2 border border-gray-300 rounded-lg focus:ring-2 focus:ring-orange-500 focus:border-orange-500"
                  disabled={saving}
                />
              </div>

              <div className="bg-gray-50 rounded-lg p-4">
                <h3 className="text-sm font-medium text-gray-700 mb-2">Configuration Summary</h3>
                <div className="space-y-1 text-sm text-gray-600">
                  {patternId && (
                    <div>
                      <span className="font-medium">Pattern:</span> {patternId}
                    </div>
                  )}
                  <div>
                    <span className="font-medium">Components:</span> {selectedAlloys.length} selected
                  </div>
                  {robotSpec?.reach && (
                    <div>
                      <span className="font-medium">Reach:</span> {robotSpec.reach}mm
                    </div>
                  )}
                  {robotSpec?.payload && (
                    <div>
                      <span className="font-medium">Payload:</span> {robotSpec.payload}kg
                    </div>
                  )}
                  <div>
                    <span className="font-medium">Created by:</span> {smith?.name} ({smith?.email})
                  </div>
                </div>
              </div>
            </div>

            <div className="flex justify-end space-x-3 mt-6">
              <button
                type="button"
                onClick={onClose}
                className="px-4 py-2 border border-gray-300 rounded-lg text-gray-700 hover:bg-gray-50"
                disabled={saving}
              >
                Cancel
              </button>
              <button
                type="submit"
                className="px-4 py-2 bg-orange-600 text-white rounded-lg hover:bg-orange-700 focus:outline-none focus:ring-2 focus:ring-offset-2 focus:ring-orange-500 disabled:opacity-50 disabled:cursor-not-allowed flex items-center"
                disabled={saving || !formData.name.trim()}
              >
                {saving ? (
                  <>
                    <svg className="animate-spin -ml-1 mr-2 h-4 w-4 text-white" fill="none" viewBox="0 0 24 24">
                      <circle className="opacity-25" cx="12" cy="12" r="10" stroke="currentColor" strokeWidth="4"></circle>
                      <path className="opacity-75" fill="currentColor" d="M4 12a8 8 0 018-8V0C5.373 0 0 5.373 0 12h4zm2 5.291A7.962 7.962 0 014 12H0c0 3.042 1.135 5.824 3 7.938l3-2.647z"></path>
                    </svg>
                    Forging...
                  </>
                ) : (
                  <>
                    🔥 Forge Sepulka
                  </>
                )}
              </button>
            </div>
          </form>
        </div>
      </div>
    </div>
  );
}
