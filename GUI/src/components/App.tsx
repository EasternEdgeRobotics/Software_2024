import {Box, Button, Tab, Tabs} from "@mui/material";
import { ArrowUpRightSquare, Bot, Camera, Gamepad2, Wrench } from "lucide-react";
import { useState } from "react";
import CameraTab from "./CameraTab";
import SettingsTab from "./SettingsTab";
import React from "react";
import { useAtom } from "jotai";
import { CameraIPs } from "../api/Atoms";
import ControllerTab from "./ControllerTab";
import SafetyDisclaimer from "./SafetyDisclaimer";
import InitControllers from "../api/Controller";
import { InitROS } from "../api/ROS";
import { BotTab } from "./BotTab";

// Navbar, calls to render each window, and a popout button

export function RenderTab(props: {tab: number}) {
    //return respective tab depending on which tab is chosen, or else return null (which should never happen)
    switch (props.tab) {
        case 0: return(<CameraTab />);
        case 1: return(<ControllerTab />);
        case 2: return(<SettingsTab />);
        case 3: return(<BotTab />);
        default: return(null);
    }
}

export default function App() {
    const [tabIndex, setTabIndex] = useState<number>(0);
    const [, setCameraIPs] = useAtom(CameraIPs);

    React.useEffect(() => {
        /**fetch('/config').then(response => response.json()).then(data => {
            setCameraIPs(data.CameraIPs || ["", "", ""]);
        });**/
    }, [setCameraIPs]);
    

    return(
        <Box>
            <InitControllers /> {/* Initialize controller listeners */}
            <InitROS /> {/* Initialize ROS */}
            <SafetyDisclaimer /> {/* Display the safety disclaimer */}
            <Tabs value={tabIndex} onChange={(e, index) => {setTabIndex(index)}} centered> {/* Navbar on the top of the screen */}
                <Tab label={<Camera />} />
                <Tab label={<Gamepad2 />} />
                <Tab label={<Wrench />} />
                <Tab label={<Bot />} />
            </Tabs>
            <br/>
            {/* Popout window button, only for camera and bot tasks*/}
            <Box position="absolute" top="8px" right="8px">
                {/* To allow new tabs for popout, change index at the array below */}
                <Button variant="outlined" disabled={![0, 3].includes(tabIndex)} onClick={() => {window.open(`/${tabIndex}`, "", "popout")}}><ArrowUpRightSquare /></Button>
            </Box>
            {/* Pass through the tab to the render function in order to render the component */}
            <RenderTab tab={tabIndex} />
        </Box>
    )
}