import { StrictMode } from 'react'
import { createRoot } from 'react-dom/client'
import { BrowserRouter } from 'react-router-dom'
import { RosProvider } from './ros/RosContext'
import App from './App'
import './index.css'

createRoot(document.getElementById('root')).render(
  <StrictMode>
    <BrowserRouter>
      <RosProvider>
        <App />
      </RosProvider>
    </BrowserRouter>
  </StrictMode>
)
