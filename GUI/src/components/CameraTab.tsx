import { Box, Button, Grid, Paper } from "@mui/material";
import { useAtom } from "jotai";
import { CameraIPs, CurrentController, ImuData } from "../api/Atoms";
import React, { useState } from "react";

// Main camera vs alt camera are the exact same without margin and different height

export function MainCamera(props: {ip: string, width: number, enableHud: boolean}) {
    return(
        <Box height={(props.width-8) * (3/8)} sx={{backgroundImage: `url(${props.ip}), url(./nosignal.jpg)`, backgroundSize: "100% 100%", borderRadius: "12px", margin: "0 4px 0 4px"}}>
            <Box display={props.enableHud ? "block" : "none"} sx={{width:"100%", height:"20%", backgroundColor:"rgba(0,0,0,0.5)", backdropFilter:"blur(1px)", float:"right", borderTopRightRadius: "12px", borderTopLeftRadius: "12px"}}>
                <h1 style={{color: "red", display: "inline"}}>demo text</h1>
                <h1 style={{color: "lime", display: "inline"}}>&nbsp;stats will be here</h1>
            </Box>
        </Box>
    );
}

export function AltCamera(props: {ip: string, width: number}) {
    return(
        <Box height={props.width * (19/108)} sx={{backgroundImage: `url(${props.ip}), url(./nosignal.png)`, backgroundSize: "100% 100%", borderRadius: "12px"}}/>
    )
}

export default function CameraTab() {
    const [width, setWidth] = React.useState<number>(window.innerWidth);
    const [IPs] = useAtom<string[]>(CameraIPs);
    const [currentController] = useAtom<number>(CurrentController);
    const [enableHud, setEnableHud] = useState<boolean>(true);

    // Create a listener to update width on window resize
    React.useEffect(() => {window.addEventListener("resize", () => {setWidth(window.innerWidth)});}, []);

    return(
        <Grid container spacing={2}>
            <Grid item xs={1/9} />
            <Grid item xs={8}>
                <Paper elevation={7} sx={{borderRadius: "12px"}}>
                    <h3 style={{textAlign: "center", margin: "0"}}>Camera 1</h3>
                    <Button onClick={() => {setEnableHud(!enableHud)}} variant="text" style={{height:"28px", margin: "0", marginTop: "-28px", float:"right", marginRight:"4px", borderRadius:"12px"}}>Toggle Hud</Button>
                    <MainCamera ip={IPs[0]} width={width} enableHud={enableHud}/>
                    <h3 style={{margin: "0", textAlign: "center"}}>{currentController === -1 ? "No Controller Connected!" : `Current Controller: ${navigator.getGamepads()[currentController]!.id}`}</h3>
                </Paper>
            </Grid>
            <Grid item xs display="flex" flexDirection="column" justifyContent="space-between">
                <Paper elevation={7} sx={{alignSelf: "flex-start", borderRadius: "12px", width: "100%"}}>
                    <h3 style={{textAlign: "center", margin: "0"}}>Camera 2</h3>
                    <AltCamera ip={IPs[1]} width={width} />
                </Paper>
                <Paper elevation={7} sx={{alignSelf: "flex-end", borderRadius: "12px", width: "100%"}}>
                    <h3 style={{textAlign: "center", margin: "0"}}>Camera 3</h3>
                    <AltCamera ip={IPs[2]} width={width} />
                </Paper>
            </Grid>
            <Grid item xs={1/9} />
        </Grid>
    )
}