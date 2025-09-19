export function ErrorBanner({ message, onRetry }: { message: string; onRetry?: () => void }) {
  return (
    <div className="flex items-start gap-3 p-3 rounded border border-red-300 bg-red-50 text-red-800">
      <span className="mt-0.5">⚠️</span>
      <div className="flex-1">
        <div className="text-sm font-medium">Problem loading model</div>
        <div className="text-xs mt-1 opacity-90 break-words">{message}</div>
      </div>
      {onRetry && (
        <button
          className="ml-3 px-2 py-1 text-xs rounded bg-white border border-red-300 hover:bg-red-100"
          onClick={onRetry}
        >
          Retry
        </button>
      )}
    </div>
  )
}


