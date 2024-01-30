import { useAtom } from "jotai";
import { CameraIPs, ROSIP, ProfilesList, CurrentProfile, RequestingProfilesList, RequestingConfig } from "../api/Atoms";
import { Box } from "@mui/system";
import { Button, Divider, FormControl, Grid, InputLabel, Select, TextField, MenuItem } from "@mui/material";
import { Trash2 } from "lucide-react";
import ProfileEditor from "./ProfileEditor";
import { useState } from "react";

export default function SettingsTab() {
    const [IPs, setIPs] = useAtom(CameraIPs);
    const [RosIP, setRosIP] = useAtom(ROSIP);
    const [editorOpen, setEditorOpen] = useState<boolean>(false);
    const [currentProfile, setCurrentProfile] = useAtom(CurrentProfile);
    const [profilesList, ] = useAtom(ProfilesList);
    const [,setRequestingProfilesList] = useAtom(RequestingProfilesList);

    const setCameraIP = (camera: number, ip: string) => {
        setIPs(IPs.map((item, index) => index === camera ? ip : item));
    };

    const save = () => {
        setIPs(IPs.map(ip => !ip.startsWith("http://") && !ip.startsWith("https://") ? "http://" + ip : ip));
        const settings: {[id: string] : string[]} = {};
        settings.CameraIPs = IPs.map(ip => !ip.startsWith("http://") && !ip.startsWith("https://") ? "http://" + ip : ip);
        fetch("/config", {method: "POST", headers: {"Content-Type": "application/json"}, body: JSON.stringify(settings)});
        localStorage.setItem("ROS_IP", RosIP);
    };

    return (
        <Box>
            <Divider>ROS</Divider><br/>
            <Box display="flex" justifyContent="center">
                {/* TODO: may make this a part of the web servers env? */}
                <TextField label="ROS Bridge IP (Must Refresh After Changing)" variant="outlined" sx={{width: "40%"}} value={RosIP} onChange={(e) => setRosIP(e.target.value)} />
            </Box>
            <br/><Divider>Cameras</Divider><br/>
            <Grid container>
                <Grid item xs={1/2} />
                <Grid item xs={10/3}>
                    <TextField label="Camera 1 URL" variant="outlined" sx={{width: "100%"}} value={IPs[0]} onChange={(e) => setCameraIP(0, e.target.value)} />
                </Grid>
                <Grid item xs={1/2} />
                <Grid item xs={10/3}>
                    <TextField label="Camera 2 URL" variant="outlined" sx={{width: "100%"}} value={IPs[1]} onChange={(e) => setCameraIP(1, e.target.value)} />
                </Grid>
                <Grid item xs={1/2} />
                <Grid item xs={10/3}>
                    <TextField label="Camera 2 URL" variant="outlined" sx={{width: "100%"}} value={IPs[2]} onChange={(e) => setCameraIP(2, e.target.value)} />
                </Grid>
                <Grid item xs={1/2} />
            </Grid>
            <Box position="absolute" bottom="8px" left="10%" width="80%">
                <Button variant="contained" sx={{width: "100%"}} onClick={() => {save();}}>Save Settings</Button>
            </Box>
            <br/><Divider>Profiles</Divider><br/>
            <Box display="flex" justifyContent="center">
                <Grid container width="50%" spacing={1}>
                    <Grid item xs={9}>
                        <FormControl fullWidth >
							<InputLabel>Profile</InputLabel>
							<Select value={currentProfile} label="Profile" onClick={()=>{setRequestingProfilesList(1);}} onChange={(e) => {
								setCurrentProfile(e.target.value)
								return;
							}} sx={{width: "100%"}}>
								{profilesList.map((profile) => {
										//Add menu item for every profile
										return <MenuItem value={profile.name}>{profile.name}</MenuItem>;
									})
								}
							</Select>
						</FormControl>
                    </Grid>
                    <Grid item xs={2}>
                        <Button variant="contained" sx={{height: "56px", width: "100%"}}>Load Profile</Button>
                    </Grid>
                    <Grid item xs={1}>
                        <Button variant="outlined" sx={{height: "56px", width: "100%"}}><Trash2 /></Button>
                    </Grid>
                    <Grid item xs={12}>
                        <Button variant="contained" sx={{height: "56px", width: "100%"}} onClick={() => {
                            if (currentProfile !=""){
                                setEditorOpen(true);
                                }}}>Profile Editor</Button>
                        <ProfileEditor open={editorOpen}  onClose={() => setEditorOpen(false)} />
                    </Grid>
                </Grid>
            </Box>
        </Box>
    );
}
