'use client';

import { useState } from 'react';
import { useRouter } from 'next/navigation';

export default function Home() {
  const router = useRouter();
  const [userInput, setUserInput] = useState('');

  const handleSubmit = (e: React.FormEvent) => {
    e.preventDefault();
    if (userInput.trim()) {
      router.push(`/analyze?input=${encodeURIComponent(userInput)}`);
    }
  };

  const handleExampleClick = (example: string) => {
    setUserInput(example);
  };

  return (
    <div className="max-w-4xl mx-auto px-4 py-12">
      {/* Progress Steps */}
      <div className="flex justify-center mb-12">
        <div className="flex items-center space-x-4">
          <div className="flex items-center">
            <div className="w-8 h-8 bg-blue-600 text-white rounded-full flex items-center justify-center">
              1
            </div>
            <span className="ml-2 text-blue-600 font-medium">Use Case</span>
          </div>
          <div className="w-16 h-[2px] bg-gray-200"></div>
          <div className="flex items-center">
            <div className="w-8 h-8 bg-gray-200 text-gray-500 rounded-full flex items-center justify-center">
              2
            </div>
            <span className="ml-2 text-gray-500">Components</span>
          </div>
          <div className="w-16 h-[2px] bg-gray-200"></div>
          <div className="flex items-center">
            <div className="w-8 h-8 bg-gray-200 text-gray-500 rounded-full flex items-center justify-center">
              3
            </div>
            <span className="ml-2 text-gray-500">Review</span>
          </div>
          <div className="w-16 h-[2px] bg-gray-200"></div>
          <div className="flex items-center">
            <div className="w-8 h-8 bg-gray-200 text-gray-500 rounded-full flex items-center justify-center">
              4
            </div>
            <span className="ml-2 text-gray-500">Quote</span>
          </div>
        </div>
      </div>

      {/* Main Content */}
      <div className="text-center mb-12">
        <h1 className="text-4xl font-bold text-gray-900 mb-4">
          Design Your Perfect Automation Solution
        </h1>
        <p className="text-lg text-gray-600 mb-8">
          Tell us about your automation needs and let our AI configure the ideal robot solution
        </p>
      </div>

      {/* Configuration Form */}
      <form onSubmit={handleSubmit} className="space-y-8">
        <div className="bg-white rounded-lg shadow-sm p-6">
          <div className="space-y-6">
            <div>
              <textarea
                value={userInput}
                onChange={(e) => setUserInput(e.target.value)}
                className="w-full p-4 border border-gray-300 rounded-lg focus:ring-2 focus:ring-blue-500 focus:border-blue-500 text-gray-900 text-base"
                rows={4}
                placeholder="Describe your automation needs..."
              />
              
              {/* Suggested Prompts */}
              <div className="mt-4 space-y-3">
                <p className="text-sm font-medium text-gray-500">Example use cases:</p>
                <div className="space-y-2">
                  <div
                    className="p-3 bg-gray-50 rounded-lg hover:bg-gray-100 cursor-pointer transition-colors"
                    onClick={() => handleExampleClick("I need a robot to pick and pack items from shelves in my warehouse")}
                  >
                    <p className="text-gray-700">🏭 Warehouse Automation</p>
                    <p className="text-sm text-gray-500">Example: "I need a robot to pick and pack items from shelves in my warehouse"</p>
                  </div>
                  <div
                    className="p-3 bg-gray-50 rounded-lg hover:bg-gray-100 cursor-pointer transition-colors"
                    onClick={() => handleExampleClick("Looking for a robot to assist with circuit board assembly and soldering")}
                  >
                    <p className="text-gray-700">⚙️ Assembly Line</p>
                    <p className="text-sm text-gray-500">Example: "Looking for a robot to assist with circuit board assembly and soldering"</p>
                  </div>
                  <div
                    className="p-3 bg-gray-50 rounded-lg hover:bg-gray-100 cursor-pointer transition-colors"
                    onClick={() => handleExampleClick("Need automated visual inspection for product defects on our production line")}
                  >
                    <p className="text-gray-700">🔍 Quality Control</p>
                    <p className="text-sm text-gray-500">Example: "Need automated visual inspection for product defects on our production line"</p>
                  </div>
                </div>
              </div>
            </div>
          </div>
        </div>

        {/* Action Button */}
        <div className="flex justify-center">
          <button
            type="submit"
            className="bg-[#14171F] text-white px-6 py-3 rounded-lg font-medium flex items-center space-x-2 hover:bg-gray-800 transition-colors disabled:opacity-50 disabled:cursor-not-allowed"
            disabled={!userInput.trim()}
          >
            Analyze Requirements
            <svg className="w-5 h-5 ml-2" fill="none" stroke="currentColor" viewBox="0 0 24 24">
              <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M9 5l7 7-7 7" />
            </svg>
          </button>
        </div>
      </form>
    </div>
  );
}
