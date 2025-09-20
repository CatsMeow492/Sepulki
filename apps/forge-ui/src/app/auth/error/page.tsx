'use client'

import Link from "next/link"
import { useSearchParams } from "next/navigation"

const errors: Record<string, string> = {
  Configuration: "There was a problem with the server configuration.",
  AccessDenied: "Access denied. You do not have permission to sign in.",
  Verification: "The verification token has expired or has already been used.",
  Default: "An unexpected error occurred during sign in.",
}

export default function AuthErrorPage() {
  const searchParams = useSearchParams()
  const error = searchParams.get('error') as keyof typeof errors
  
  return (
    <div className="min-h-screen bg-gradient-to-br from-orange-50 to-gray-100 flex items-center justify-center">
      <div className="max-w-md w-full bg-white rounded-xl shadow-lg p-8">
        {/* Sepulki Branding */}
        <div className="text-center mb-8">
          <div className="text-4xl mb-2">🔥</div>
          <h1 className="text-3xl font-bold text-gray-900">Sepulki Forge</h1>
        </div>

        {/* Error Message */}
        <div className="mb-6">
          <div className="p-4 bg-red-50 border border-red-200 rounded-lg">
            <div className="flex items-start">
              <div className="text-red-500 mr-2 text-xl">⚠️</div>
              <div>
                <h2 className="text-lg font-medium text-red-800 mb-2">Sign In Error</h2>
                <p className="text-sm text-red-600">
                  {errors[error] || errors.Default}
                </p>
              </div>
            </div>
          </div>
        </div>

        {/* Actions */}
        <div className="space-y-3">
          <Link
            href="/auth/signin"
            className="w-full flex items-center justify-center px-4 py-3 border border-transparent rounded-lg shadow-sm bg-orange-600 text-sm font-medium text-white hover:bg-orange-700 focus:outline-none focus:ring-2 focus:ring-offset-2 focus:ring-orange-500"
          >
            Try Again
          </Link>
          
          <Link
            href="/"
            className="w-full flex items-center justify-center px-4 py-3 border border-gray-300 rounded-lg shadow-sm bg-white text-sm font-medium text-gray-700 hover:bg-gray-50 focus:outline-none focus:ring-2 focus:ring-offset-2 focus:ring-orange-500"
          >
            Go Home
          </Link>
        </div>

        {/* Help Text */}
        <div className="mt-6 text-center">
          <p className="text-xs text-gray-500">
            If this problem persists, please contact support or try a different browser.
          </p>
        </div>
      </div>
    </div>
  )
}
