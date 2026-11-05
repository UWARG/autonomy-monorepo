import { StrictMode } from 'react'
import { createRoot } from 'react-dom/client'
import 'react-grid-layout/css/styles.css';
import 'react-resizable/css/styles.css';
import './index.css'
import App from './App.tsx'

createRoot(document.getElementById('root')!).render(
  <StrictMode>
    <App />
  </StrictMode>,
)
