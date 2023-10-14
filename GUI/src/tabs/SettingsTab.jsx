import { Box, Divider, Grid, TextField, Button } from "@mui/material";
import { useAtom } from "jotai";
import { ipAtom } from "../atoms/CameraIP";
import { saveConfig } from "../ROS";

function SettingsTab() {
    const [IPs, setIPs] = useAtom(ipAtom);

    const setCameraIP = (camera, ip) => {
        setIPs(IPs.map((item, index) => index === camera ? ip : item)); //i hate this is there is a better way to do this please change it
    }

    const saveSettings = () => {
        let settings = {};
        settings.cameraIPs = IPs;
        saveConfig(settings);
    }

    return (
        <Box>
            <Box left="10%" bottom="16px" position="fixed" width="80%">
                <Button variant="contained" sx={{width: "100%"}} onClick={() => saveSettings()}>Save Settings</Button>
            </Box>
            {/* maybe add a textbox to set ros ip rather than an env file? */}
            <Divider>Cameras</Divider>
            <br/>
            <Grid container>
                {/* 3 text fields for camera ips */}
                <Grid item xs={1/2} />
                <Grid item xs={10/3}>
                    <TextField id="camera-1-url" label="Camera 1 URL" variant="outlined" sx={{width: "100%"}} value={IPs[0]} onChange={(e) => {setCameraIP(0, e.target.value);}}/>
                </Grid>
                <Grid item xs={1/2} />
                <Grid item xs={10/3}>
                    <TextField id="camera-2-url" label="Camera 2 URL" variant="outlined" sx={{width: "100%"}} value={IPs[1]} onChange={(e) => {setCameraIP(1, e.target.value);}}/>
                </Grid>
                <Grid item xs={1/2} />
                <Grid item xs={10/3}>
                    <TextField id="camera-3-url" label="Camera 3 URL" variant="outlined" sx={{width: "100%"}} value={IPs[2]} onChange={(e) => {setCameraIP(2, e.target.value);}}/>
                </Grid>
                <Grid item xs={1/2} />
            </Grid>
        </Box>
    )
}

export default SettingsTab;