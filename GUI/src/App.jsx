import { CssBaseline, ThemeProvider, createTheme, Tabs, Tab, Grid, Box } from "@mui/material";
import SafetyDisclaimer from "./SafetyDisclaimer";
import Controller from "./Controller";
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
      <br/>
      {tabIndex === 0 && (
        <Grid container spacing={2}>
          <Grid item xs={1/9} />
          <Grid item xs={8} margin="0">
            <Box bgcolor="#ff0000" height="720px" sx={{backgroundImage: "url(http://192.168.2.135:8080/?action=stream)", backgroundSize: "100% 100%"}}><h3>Camera #1</h3></Box>
          </Grid>
          <Grid item xs margin="0">
            <Box bgcolor="#00ff00" height="320px" sx={{backgroundImage: "url(http://192.168.2.135:8080/?action=stream)", backgroundSize: "100% 100%"}}><h3>Camera #2</h3></Box>
            <Box bgcolor="#0000ff" height="320px" sx={{backgroundImage: "url(http://192.168.2.135:8080/?action=stream)", backgroundSize: "100% 100%"}}><h3>Camera #3</h3></Box>
          </Grid>
          <Grid item xs={1/9} />
        </Grid>
      )}
      {tabIndex === 1 && (
        <Box>
          <Controller />
        </Box>
      )}
      {tabIndex === 2 && (
        <h1>SETTINGS WINDOW</h1>
      )}
    </ThemeProvider>
  );
}

export default App;