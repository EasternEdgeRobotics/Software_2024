import { CssBaseline, ThemeProvider, createTheme, Tabs, Tab, Grid, Box } from "@mui/material";
import SafetyDisclaimer from "./SafetyDisclaimer";
import React, { useEffect } from "react";
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
      <br/>
      {tabIndex === 0 && (
        <Box>
          <Grid container spacing={2} style={{transform: `translateX(${(window.innerWidth - 1776)/2}px)`}}>
            <Grid item xs margin="0">
              <Box bgcolor="#ff0000" width="1280px" height="720px"><h1>Camera #1</h1></Box>
            </Grid>
            <Grid item xs margin="0">
              <Box bgcolor="#00ff00" width="480px" height="352px"><h1>Camera #2</h1></Box>
              <Box bgcolor="#0000ff" width="480px" height="352px" marginTop={"-6px"}><h1>Camera #3</h1></Box>
            </Grid>
          </Grid>
        </Box>
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