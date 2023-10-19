import { ThemeProvider } from '@emotion/react';
import { CssBaseline, createTheme } from '@mui/material';
import React from 'react';
import ReactDOM from 'react-dom/client';
import App from './App';
import SafetyDisclaimer from './SafetyDisclaimer';
import InitROS from './ROS';

const root = ReactDOM.createRoot(document.getElementById('root'));
root.render(
  <React.StrictMode>
    <ThemeProvider theme={createTheme({palette: {mode: "dark"}})}> {/* Dark theme */}
      <CssBaseline />
      <InitROS /> {/* Initialize ROSBridge */}
      <SafetyDisclaimer /> {/* Open the safety disclaimer */}
      <App /> {/* Start the app */}
    </ThemeProvider>
  </React.StrictMode>
);