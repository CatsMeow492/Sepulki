'use client'

import { useAuth } from "./AuthProvider"
import Image from "next/image"
import { useState, useRef, useEffect } from "react"

export function SmithProfile() {
  const { smith, loading, signOut } = useAuth()
  const [isOpen, setIsOpen] = useState(false)
  const menuRef = useRef<HTMLDivElement>(null)

  useEffect(() => {
    function handleClickOutside(event: MouseEvent) {
      if (menuRef.current && !menuRef.current.contains(event.target as Node)) {
        setIsOpen(false)
      }
    }

    document.addEventListener('mousedown', handleClickOutside)
    return () => {
      document.removeEventListener('mousedown', handleClickOutside)
    }
  }, [])

  if (loading) {
    return (
      <div className="flex items-center">
        <div className="animate-pulse rounded-full bg-gray-200 h-8 w-8"></div>
      </div>
    )
  }

  if (!smith) {
    return null
  }

  const roleEmoji = {
    SMITH: '🔨',
    OVER_SMITH: '⚒️', 
    ADMIN: '👑'
  }[smith.role] || '🔨'

  return (
    <div className="relative ml-3" ref={menuRef}>
      <button 
        className="flex max-w-xs items-center rounded-full bg-white text-sm focus:outline-none focus:ring-2 focus:ring-orange-500 focus:ring-offset-2 lg:rounded-md lg:p-2 lg:hover:bg-gray-50"
        onClick={() => setIsOpen(!isOpen)}
      >
        <div className="flex items-center">
          {smith.image ? (
            <Image
              className="h-8 w-8 rounded-full"
              src={smith.image}
              alt=""
              width={32}
              height={32}
            />
          ) : (
            <div className="h-8 w-8 rounded-full bg-orange-500 flex items-center justify-center text-white text-sm font-medium">
              {smith.name?.[0] || 'S'}
            </div>
          )}
          <div className="ml-3 hidden lg:block">
            <p className="text-sm font-medium text-gray-700 flex items-center">
              <span className="mr-1">{roleEmoji}</span>
              {smith.name || 'Smith'}
            </p>
            <p className="text-xs text-gray-500 capitalize">{smith.role.replace('_', ' ').toLowerCase()}</p>
          </div>
        </div>
      </button>

      {isOpen && (
        <div className="absolute right-0 z-10 mt-2 w-48 origin-top-right rounded-md bg-white py-1 shadow-lg ring-1 ring-black ring-opacity-5 focus:outline-none">
          <div className="px-4 py-2 border-b border-gray-100">
            <p className="text-sm font-medium text-gray-900 flex items-center">
              <span className="mr-2">{roleEmoji}</span>
              {smith.name || 'Smith'}
            </p>
            <p className="text-xs text-gray-500">{smith.email}</p>
            <p className="text-xs text-orange-600 font-medium capitalize mt-1">
              {smith.role.replace('_', ' ').toLowerCase()}
            </p>
          </div>
          <button
            onClick={() => {
              signOut()
              setIsOpen(false)
            }}
            className="block w-full text-left px-4 py-2 text-sm text-gray-700 hover:bg-gray-100 focus:outline-none focus:bg-gray-100"
          >
            Sign out
          </button>
        </div>
      )}
    </div>
  )
}
