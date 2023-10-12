import { Box, Divider, Grid, TextField, Button } from "@mui/material";
import { useAtom } from "jotai";
import { Camera1IP, Camera2IP, Camera3IP } from "./Atoms";
import { saveConfig } from "./ROS";

var settings = {}

function Settings() {
    const [camera1IP, setCamera1IP] = useAtom(Camera1IP);
    const [camera2IP, setCamera2IP] = useAtom(Camera2IP);
    const [camera3IP, setCamera3IP] = useAtom(Camera3IP);

    const saveSettings = async () => {
        settings.Camera1IP = camera1IP;
        settings.Camera2IP = camera2IP;
        settings.Camera3IP = camera3IP;
        saveConfig(JSON.stringify(settings));
    }

    return (
        <Box>
            <Box left="10%" bottom="16px" position="fixed" width="80%">
                <Button variant="contained" sx={{width: "100%"}} onClick={saveSettings}>Save Settings</Button>
            </Box>
            <Divider>Cameras</Divider>
            <br/>
            <Grid container>
                {/* 3 text fields for camera ips */}
                <Grid item xs={1/2} />
                <Grid item xs={10/3}>
                    <TextField id="camera-1-url" label="Camera 1 URL" variant="outlined" sx={{width: "100%"}} value={camera1IP} onChange={(e) => {setCamera1IP(e.target.value);}}/>
                </Grid>
                <Grid item xs={1/2} />
                <Grid item xs={10/3}>
                    <TextField id="camera-2-url" label="Camera 2 URL" variant="outlined" sx={{width: "100%"}} value={camera2IP} onChange={(e) => {setCamera2IP(e.target.value);}}/>
                </Grid>
                <Grid item xs={1/2} />
                <Grid item xs={10/3}>
                    <TextField id="camera-3-url" label="Camera 3 URL" variant="outlined" sx={{width: "100%"}} value={camera3IP} onChange={(e) => {setCamera3IP(e.target.value);}}/>
                </Grid>
                <Grid item xs={1/2} />
            </Grid>
        </Box>
    )
}

export default Settings;