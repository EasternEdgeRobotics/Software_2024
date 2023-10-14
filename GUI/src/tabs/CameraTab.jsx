import { Box, Grid, Paper } from "@mui/material";
import { useAtom } from "jotai";
import { useEffect, useState } from "react";
import { ipAtom } from "../atoms/CameraIP";
import { currentControllerAtom } from "../atoms/CurrentController";

function CameraTab () {
    const [width, setWidth] = useState(window.innerWidth);

    useEffect(() => {
        const updateWidth = () => {setWidth(window.innerWidth);}
        window.addEventListener("resize", updateWidth);
    }, []);

    const [IPs] = useAtom(ipAtom);
    const [currentController] = useAtom(currentControllerAtom);

    return (
        <Grid container spacing={2}>
            <Grid item xs={1/9} />
            <Grid item xs={8}>
                <Paper elevation={7} sx={{borderRadius: "12px"}}>
                    <h3 style={{textAlign: "center", margin: "0"}}>Camera 1</h3>
                    {/* Set the background image to what is saved as Camera 1 IP, or no signal if it cannot connect, and stretch the image */}
                    {/* remember to set this as camera 1 url */}
                    {/* also possibly make this actually 16:9? */}
                    <Box height={(width-8) * (3/8)} sx={{backgroundImage: `url(http://${IPs[0]}), url(./nosignal.jpg)`, backgroundSize: "100% 100%", borderRadius: "12px", marginLeft: "4px", marginRight: "4px"}}/>
                    <h3 style={{margin: "0", textAlign: "center"}}>{currentController === -1 ? "No Controller Connected!" : `Current Controller: ${navigator.getGamepads()[currentController].id}`}</h3>
                </Paper>
            </Grid>
            <Grid item xs sx={{position: "relative"}}>
                <Paper elevation={7} sx={{borderRadius: "12px"}} height={width * (3/8)}>
                    <h3 style={{textAlign: "center", margin: "0"}}>Camera 2</h3>
                    <Box height={width * (19/108)} sx={{backgroundImage: `url(http://${IPs[1]}), url(./nosignal.jpg)`, backgroundSize: "100% 100%", borderRadius: "12px"}}/>
                </Paper>
                <Paper elevation={7} sx={{borderRadius: "12px", position: "absolute", bottom: "0", width: "100%"}} height={width * 0.8}>
                    <h3 style={{textAlign: "center", margin: "0"}}>Camera 3</h3>
                    <Box height={width * (19/108)} sx={{backgroundImage: `url(http://${IPs[2]}), url(./nosignal.jpg)`, backgroundSize: "100% 100%", borderRadius: "12px"}}/>
                </Paper>
            </Grid>
            <Grid item xs={1/9} />
        </Grid>
    )
}

export default CameraTab;