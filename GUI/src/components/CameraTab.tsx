import { Box, Grid, Paper } from "@mui/material";
import { useAtom } from "jotai";
import { CameraIPs } from "../api/Atoms";
import React from "react";

// Main camera vs alt camera are the exact same without margin and different height

export function MainCamera(props: {ip: string, width: number}) {
    return (
        <Box height={(props.width-8) * (3/8)} sx={{backgroundImage: `url(${props.ip}), url(./nosignal.jpg)`, backgroundSize: "100% 100%", borderRadius: "12px", margin: "0 4px 0 4px"}}/>
    );
}

export function AltCamera(props: {ip: string, width: number}) {
    return (
        <Box height={props.width * (19/108)} sx={{backgroundImage: `url(${props.ip}), url(./nosignal.jpg)`, backgroundSize: "100% 100%", borderRadius: "12px"}}/>
    );
}

export default function CameraTab() {
    const [width, setWidth] = React.useState<number>(window.innerWidth);
    const [IPs] = useAtom<string[]>(CameraIPs);

    // Create a listener to update width on window resize
    React.useEffect(() => {window.addEventListener("resize", () => {setWidth(window.innerWidth);});}, []);

    return (
        <Grid container spacing={2}>
            <Grid item xs display="flex" flexDirection="column" justifyContent="space-between">
                <Paper elevation={7} sx={{alignSelf: "flex-start", borderRadius: "12px", width: "100%"}}>
                    <h3 style={{textAlign: "center", margin: "0"}}>Camera 1</h3>
                    <AltCamera ip={IPs[0]} width={width} />
                </Paper>
                <Paper elevation={7} sx={{alignSelf: "flex-end", borderRadius: "12px", width: "100%"}}>
                    <h3 style={{textAlign: "center", margin: "0"}}>Camera 2</h3>
                    <AltCamera ip={IPs[1]} width={width} />
                </Paper>
            </Grid>
            <Grid item xs display="flex" flexDirection="column" justifyContent="space-between">
                <Paper elevation={7} sx={{alignSelf: "flex-start", borderRadius: "12px", width: "100%"}}>
                    <h3 style={{textAlign: "center", margin: "0"}}>Camera 3</h3>
                    <AltCamera ip={IPs[2]} width={width} />
                </Paper>
                <Paper elevation={7} sx={{alignSelf: "flex-end", borderRadius: "12px", width: "100%"}}>
                    <h3 style={{textAlign: "center", margin: "0"}}>Camera 4</h3>
                    <AltCamera ip={IPs[3]} width={width} />
                </Paper>
            </Grid>
        </Grid>
    );
}