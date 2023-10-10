import { CssBaseline, ThemeProvider, createTheme, Tabs, Tab, Grid, Box, Paper } from "@mui/material";
import SafetyDisclaimer from "./SafetyDisclaimer";
import Controller from "./ControllerMenu";
import React from "react";
import { CameraAlt, SportsEsports, Build } from "@mui/icons-material";
import Settings from "./Settings";
import { useAtom } from "jotai";
import { CurrentControllerAtom } from "./Atoms";

const darkTheme = createTheme({
  palette: {mode: "dark",},
});

function App() {
  const [tabIndex, setTabIndex] = React.useState(0);
  const handleTabChange = (e, index) => {setTabIndex(index);}

  const [currentController, setCurrentController] = useAtom(CurrentControllerAtom);

  const [, forceUpdate] = React.useReducer(x => x + 1, 0);

  window.addEventListener('gamepadconnected', (e) => {
    if (currentController === -1) {
      setCurrentController(e.gamepad.index);
      forceUpdate();
    }
  });

  window.addEventListener('gamepaddisconnected', (e) => {
    if (e.gamepad.index === currentController) {
      if (navigator.getGamepads.length > 1) {
        for (const controller of navigator.getGamepads()) {
          if(controller.connected && controller !== e.gamepad) {
            setCurrentController(controller.index);
            break;
          }
        }
      } else setCurrentController(-1);
    }
    forceUpdate();
  });
  
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
            <Paper elevation={7}>
              <h3 style={{textAlign: "center", margin: "0"}}>Camera 1</h3>
              <Paper bgcolor="#ff0000" sx={{backgroundImage: "url(http://192.168.2.135:8080/?action=stream), url(./nosignal.jpg)", backgroundSize: "100% 100%", height: "720px"}} />
              <h3 style={{margin: "0", textAlign:"center"}}>{currentController === -1 ? "No Controller Connected!" : `Current Controller: ${navigator.getGamepads()[currentController].id}`}</h3>
            </Paper>
          </Grid>
          <Grid item xs margin="0">
            <Paper elevation={7}>
            <h3 style={{textAlign: "center", margin: "0"}}>Camera 2</h3>
              <Paper bgcolor="#00ff00" sx={{backgroundImage: "url(http://192.168.2.135:8080/?action=stream), url(./nosignal.jpg)", backgroundSize: "100% 100%", height: "320px"}} />
            </Paper>
            <br/><br/>
            <Paper elevation={7} sx={{marginTop: "4px"}}>
            <h3 style={{textAlign: "center", margin: "0"}}>Camera 3</h3>
              <Paper bgcolor="#00ff00" sx={{backgroundImage: "url(http://192.168.2.135:8080/?action=stream), url(./nosignal.jpg)", backgroundSize: "100% 100%", height: "320px"}} />
            </Paper>
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
        <Settings />
      )}
    </ThemeProvider>
  );
}

export default App;