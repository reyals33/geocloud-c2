import { lazy, Suspense } from 'react'

// Lazy-load the heavy Three.js scene so it doesn't slow other routes
const OverviewScene = lazy(() => import('../components/OverviewScene'))

export default function OverviewDashboard() {
  return (
    <div style={{ flex: 1, minHeight: 0, padding: '12px', display: 'flex', flexDirection: 'column' }}>
      <div style={{
        flex: 1, minHeight: 0,
        borderRadius: '8px',
        overflow: 'hidden',
        border: '1px solid var(--border)',
      }}>
        <Suspense fallback={
          <div style={{
            width: '100%', height: '100%',
            display: 'flex', alignItems: 'center', justifyContent: 'center',
            color: 'var(--text-muted)', fontSize: 13, letterSpacing: '0.1em',
          }}>
            LOADING C2 DISPLAY…
          </div>
        }>
          <OverviewScene />
        </Suspense>
      </div>
    </div>
  )
}
