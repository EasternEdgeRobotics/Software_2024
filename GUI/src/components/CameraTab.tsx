import { Box, Button, Grid, Paper } from "@mui/material";
import { useAtom } from "jotai";
import { CameraURLs } from "../api/Atoms";
import React from "react";

// Main camera vs alt camera are the exact same without margin and different height

export function MainCamera(props: {ip: string, width: number, enableHud: boolean}) {
    return (
        <Box height={(props.width) * (1473/6118)} width={props.width * 0.428} sx={{backgroundImage: `url(${props.ip}), url(./nosignal.jpg)`, backgroundSize: "100% 100%", borderRadius: "12px", margin: "0 4px 0 4px"}}>
            <Box display={props.enableHud ? "block" : "none"} sx={{width:"100%", height:"20%", backgroundColor:"rgba(0,0,0,0.5)", backdropFilter:"blur(1px)", float:"right", borderTopRightRadius: "12px", borderTopLeftRadius: "12px"}}>
                <h1 style={{color: "red", display: "inline"}}>demo text</h1>
                <h1 style={{color: "lime", display: "inline"}}>&nbsp;stats will be here</h1>
            </Box>
        </Box>
    );
}

export function AltCamera(props: {ip: string, width: number}) {
    return (
        <Box height={props.width * (1620.3/6118)} width={props.width * 0.4708} sx={{backgroundImage: `url(${props.ip}), url(./nosignal.jpg)`, backgroundSize: "100% 100%", borderRadius: "12px"}}/>
    );
}

export default function CameraTab() {
    const [width, setWidth] = React.useState<number>(window.innerWidth);
    const [URLs] = useAtom<string[]>(CameraURLs);
    // const [enableHud, setEnableHud] = React.useState<boolean>(false);

    // Create a listener to update width on window resize
    React.useEffect(() => {window.addEventListener("resize", () => {setWidth(window.innerWidth);});}, []);

    return (
        <Grid container spacing={2}>
            <Grid item xs display="flex" flexDirection="column" justifyContent="space-between">
                <Box display="flex" justifyContent="center">
                    <Paper elevation={7} sx={{alignSelf: "flex-start", borderRadius: "12px", width: `${width * 0.428}`}}>
                        <h3 style={{textAlign: "center", margin: "0"}}>Camera 1</h3>
                        {/* <Button onClick={() => {setEnableHud(!enableHud);}} variant="text" style={{height:"28px", margin: "0", marginTop: "-28px", float:"right", marginRight:"4px", borderRadius:"12px"}}>Toggle Hud</Button> */}
                        {/* <MainCamera ip={URLs[0]} width={width} enableHud={enableHud} /> */}
                        <AltCamera ip={URLs[0]} width={width} />
                    </Paper>
                </Box>
                <Box display="flex" justifyContent="center">
                    <Paper elevation={7} sx={{alignSelf: "flex-end", borderRadius: "12px", width: `${width * 0.428}`}}>
                        <h3 style={{textAlign: "center", margin: "0"}}>Camera 2</h3>
                        <AltCamera ip={URLs[1]} width={width} />
                    </Paper>
                </Box>
            </Grid>
            <Grid item xs display="flex" flexDirection="column" justifyContent="space-between">
                <Box display="flex" justifyContent="center">
                    <Paper elevation={7} sx={{alignSelf: "flex-start", borderRadius: "12px", width: `${width * 0.428}`, marginBottom: "8px"}}>
                        <h3 style={{textAlign: "center", margin: "0"}}>Camera 3</h3>
                        <AltCamera ip={URLs[2]} width={width} />
                    </Paper> 
                </Box>
                <Box display="flex" justifyContent="center">
                    <Paper elevation={7} sx={{alignSelf: "flex-end", borderRadius: "12px", width: `${width * 0.428}`}}>
                        <h3 style={{textAlign: "center", margin: "0"}}>Camera 4</h3>
                        <AltCamera ip={URLs[3]} width={width} />
                    </Paper>
                </Box>
            </Grid>
        </Grid>
    );
}