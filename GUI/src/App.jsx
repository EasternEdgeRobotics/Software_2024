import { CssBaseline, ThemeProvider, createTheme, Tabs, Tab } from "@mui/material";
import SafetyDisclaimer from "./SafetyDisclaimer";
import React from "react";
import { CameraAlt, SportsEsports, Build } from "@mui/icons-material";

const darkTheme = createTheme({
  palette: {mode: "dark",},
});

function App() {
  const [tabIndex, setTabIndex] = React.useState(0);
  const handleTabChange = (e, index) => {setTabIndex(index);}

  return (
    <ThemeProvider theme={darkTheme}>
      <CssBaseline />
      <SafetyDisclaimer /> {/* This opens the safety disclaimer dialog */}
      <Tabs value={tabIndex} onChange={handleTabChange} centered>
          <Tab label=<CameraAlt /> />
          <Tab label=<SportsEsports /> />
          <Tab label=<Build /> />
      </Tabs>
      {tabIndex === 0 && (
        <h1>CAMERA WINDOW</h1>
      )}
      {tabIndex === 1 && (
        <h1>CONTROLLER WINDOW</h1>
      )}
      {tabIndex === 2 && (
        <h1>SETTINGS WINDOW</h1>
      )}
    </ThemeProvider>
  );
}

export default App;
