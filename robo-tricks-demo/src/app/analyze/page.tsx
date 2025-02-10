'use client';

import { useEffect, useState } from 'react';
import { useSearchParams, useRouter } from 'next/navigation';
import ReactMarkdown from 'react-markdown';
import remarkGfm from 'remark-gfm';
import type { Components } from 'react-markdown';

interface CodeProps {
  inline?: boolean;
  className?: string;
  children?: React.ReactNode;
}

export default function AnalyzePage() {
  const searchParams = useSearchParams();
  const router = useRouter();
  const [analysis, setAnalysis] = useState<string>('');
  const [loading, setLoading] = useState(true);
  const [error, setError] = useState<string | null>(null);

  useEffect(() => {
    const userInput = searchParams.get('input');
    if (!userInput) {
      router.push('/');
      return;
    }

    const analyzeRequirements = async () => {
      try {
        const response = await fetch('/api/analyze', {
          method: 'POST',
          headers: {
            'Content-Type': 'application/json',
          },
          body: JSON.stringify({ userInput }),
        });

        if (!response.ok) {
          throw new Error('Failed to analyze requirements');
        }

        const data = await response.json();
        setAnalysis(data.analysis);
      } catch (err) {
        setError(err instanceof Error ? err.message : 'An error occurred');
      } finally {
        setLoading(false);
      }
    };

    analyzeRequirements();
  }, [searchParams, router]);

  const markdownComponents: Components = {
    h1: ({children}) => <h1 className="text-2xl font-bold text-gray-900 mt-6 mb-4">{children}</h1>,
    h2: ({children}) => <h2 className="text-xl font-bold text-gray-900 mt-5 mb-3">{children}</h2>,
    h3: ({children}) => <h3 className="text-lg font-bold text-gray-900 mt-4 mb-2">{children}</h3>,
    p: ({children}) => <p className="text-gray-700 mb-4">{children}</p>,
    ul: ({children}) => <ul className="list-disc list-inside mb-4">{children}</ul>,
    ol: ({children}) => <ol className="list-decimal list-inside mb-4">{children}</ol>,
    li: ({children}) => <li className="text-gray-700 mb-2">{children}</li>,
    strong: ({children}) => <strong className="font-semibold text-gray-900">{children}</strong>,
    blockquote: ({children}) => (
      <blockquote className="border-l-4 border-blue-500 pl-4 italic text-gray-700 my-4">{children}</blockquote>
    ),
    code: ({inline, className, children}: CodeProps) => {
      const match = /language-(\w+)/.exec(className || '');
      return inline ? (
        <code className="bg-gray-100 rounded px-1 py-0.5 text-sm text-gray-800">
          {children}
        </code>
      ) : (
        <code className="block bg-gray-100 rounded p-4 text-sm text-gray-800 overflow-x-auto">
          {children}
        </code>
      );
    },
  };

  const handleContinue = () => {
    // Store the analysis in localStorage for reference in later steps
    localStorage.setItem('requirementAnalysis', analysis);
    localStorage.setItem('userInput', searchParams.get('input') || '');
    router.push('/configure?step=2');
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

      <div className="bg-white rounded-lg shadow-sm p-6">
        <h1 className="text-2xl font-bold text-gray-900 mb-6">Requirement Analysis</h1>
        
        {loading ? (
          <div className="flex flex-col items-center justify-center py-12">
            <div className="animate-spin rounded-full h-12 w-12 border-b-2 border-blue-600"></div>
            <p className="mt-4 text-gray-600">Analyzing your requirements...</p>
          </div>
        ) : error ? (
          <div className="bg-red-50 border border-red-200 rounded-lg p-4 text-red-600">
            {error}
          </div>
        ) : (
          <div className="space-y-6">
            <div className="p-4 bg-blue-50 border border-blue-100 rounded-lg">
              <h2 className="font-medium text-blue-900 mb-2">Your Initial Request:</h2>
              <p className="text-blue-800">{searchParams.get('input')}</p>
            </div>
            
            <div className="space-y-4">
              <h2 className="font-medium text-gray-900">Refined Requirements & Considerations:</h2>
              <div className="prose prose-blue max-w-none">
                <ReactMarkdown 
                  remarkPlugins={[remarkGfm]}
                  components={markdownComponents}
                >
                  {analysis}
                </ReactMarkdown>
              </div>
            </div>

            <div className="flex justify-end space-x-4 mt-8">
              <button
                onClick={() => router.push('/')}
                className="px-6 py-3 border border-gray-300 rounded-lg text-gray-700 hover:bg-gray-50"
              >
                Start Over
              </button>
              <button
                onClick={handleContinue}
                className="px-6 py-3 bg-blue-600 text-white rounded-lg hover:bg-blue-700"
              >
                Continue to Configuration
              </button>
            </div>
          </div>
        )}
      </div>
    </div>
  );
} 