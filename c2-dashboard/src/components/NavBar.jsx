import { NavLink } from 'react-router-dom'
import { useRos } from '../ros/RosContext'
import { AGENTS } from '../config'

export default function NavBar() {
  const { status } = useRos()

  return (
    <nav className="navbar">
      <span className="navbar-brand">GeoCloud C2</span>

      {AGENTS.map(a => (
        <NavLink
          key={a.id}
          to={`/${a.id}`}
          className={({ isActive }) => 'nav-link' + (isActive ? ' active' : '')}
          style={({ isActive }) => isActive ? { borderBottom: `2px solid ${a.color}` } : {}}
        >
          {a.label}
        </NavLink>
      ))}

      <NavLink
        to="/overview"
        className={({ isActive }) => 'nav-link' + (isActive ? ' active' : '')}
      >
        Overview
      </NavLink>

      <span className="nav-spacer" />

      <div className="conn-badge">
        <div className={`conn-dot ${status}`} />
        {status === 'connected' ? 'ROS Live' : status === 'error' ? 'Error' : 'Connecting…'}
      </div>
    </nav>
  )
}
