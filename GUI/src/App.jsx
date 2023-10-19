import { Build, CameraAlt, Psychology, SportsEsports } from "@mui/icons-material";
import { Box, Tab, Tabs } from "@mui/material";
import { useEffect, useReducer, useState } from "react";
import CameraTab from "./tabs/CameraTab";
import ControllerTab from "./tabs/ControllerTab";
import SettingsTab from "./tabs/SettingsTab";
import { loadConfig } from "./ROS";
import { useAtom } from "jotai";
import { currentControllerAtom } from "./atoms/CurrentController";
import TaskTab from "./tabs/TaskTab";

/* This is mainly for the tabs of the GUI (camera, controller settings, general settings) */

function App() {
    const [tabIndex, setTabIndex] = useState(0);
    /* function that force reloads the component */
    const [, forceUpdate] = useReducer(x => x + 1, 0);
    const [currentController, setCurrentController] = useAtom(currentControllerAtom);

    function renderTab() {
        switch(tabIndex) {
            case 0: return <CameraTab />; //if tab is 0 render cameras
            case 1: return <ControllerTab />; //if tab is 1 render controller settings
            case 2: return <SettingsTab />; //if tab is 2 render general settings
            case 3: return <TaskTab />; //if tab is 3 render task menu
            default: return;
        }
    }

    useEffect(() => loadConfig(), []);

    /* set controller that was just plugged in as the default controller if none selected */
    window.addEventListener('gamepadconnected', (e) => {
        if (currentController === -1) {
        setCurrentController(e.gamepad.index);
        forceUpdate();
        }
    });

    /* if disconnected, try to set the next found controller, if no controller can be found set the current controller as -1 (none) */
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
        <Box>
            <Tabs value={tabIndex} onChange={(e, newIndex) => setTabIndex(newIndex)} centered>
                <Tab label={<CameraAlt />} />
                <Tab label={<SportsEsports />} />
                <Tab label={<Build />} />
                <Tab label={<Psychology />} />
            </Tabs>
            <br/>
            {renderTab()} {/* Render the tab depending on which tab is selected */}
        </Box>
    )
}

export default App;