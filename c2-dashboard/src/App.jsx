import { Routes, Route, Navigate } from 'react-router-dom'
import NavBar from './components/NavBar'
import AgentDashboard from './pages/AgentDashboard'
import OverviewDashboard from './pages/OverviewDashboard'

export default function App() {
  return (
    <div className="page">
      <NavBar />
      <Routes>
        <Route path="/robot1"   element={<AgentDashboard agentId="robot1" />} />
        <Route path="/robot2"   element={<AgentDashboard agentId="robot2" />} />
        <Route path="/drone1"   element={<AgentDashboard agentId="drone1" />} />
        <Route path="/drone2"   element={<AgentDashboard agentId="drone2" />} />
        <Route path="/overview" element={<OverviewDashboard />} />
        <Route path="*"         element={<Navigate to="/overview" replace />} />
      </Routes>
    </div>
  )
}
