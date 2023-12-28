import { useAtom } from "jotai";
import { CameraIPs, ROSIP, Profiles, ProfilesService } from "../api/Atoms";
import { Box } from "@mui/system";
import { Button, Divider, Grid, TextField, Menu, Typography } from "@mui/material";
import { useState, useEffect } from "react";

export default function SettingsTab() {
    const [IPs, setIPs] = useAtom(CameraIPs);
    const [RosIP, setRosIP] = useAtom(ROSIP);
    const [profileEditorMode, setProfileEditorMode] = useState<boolean>(false); //The settings tab can turn into the profile editor
    const [currentProfile, setCurrentProfile] = useState<string>("No Profile Selected"); //Stores the selected profile for editing
    const [createProfileInputBox, setCreateProfileInputBox] = useState<string>(""); 
    const [profiles, setProfiles] = useAtom(Profiles); //Profiles are stored here
    const [, setProfilesService] = useAtom(ProfilesService); //If ProfilesService == 0, tell the ROS client to overwrite it's stored profiles with what is here. If ProfilesService == 1, tell the ROS client to send its stored profiles. 

    const setCameraIP = (camera: number, ip: string) => {
        setIPs(IPs.map((item, index) => index === camera ? ip : item));
    }

    const save = () => {
        setIPs(IPs.map(ip => !ip.startsWith("http://") && !ip.startsWith("https://") ? "http://" + ip : ip));
        let settings: any = {};
        settings.CameraIPs = IPs.map(ip => !ip.startsWith("http://") && !ip.startsWith("https://") ? "http://" + ip : ip);
        fetch('/config', {method: "POST", headers: {'Content-Type': 'application/json'}, body: JSON.stringify(settings)});
        localStorage.setItem("ROS_IP", RosIP);
    }

    const [anchorElement, setAnchorEL] = useState(null);

    const [mouseAnchored, setMouseAnchored] = useState<boolean>(false);

    const setMouseAnchor = (currentTarget:any) =>{
        if (mouseAnchored){
            setAnchorEL(null);
            setMouseAnchored(false);
        }
        else {
            setAnchorEL(currentTarget);
            setMouseAnchored(true);
        }
    }


    // FUNCTIONS BELOW BELONG TO PROFILE EDITOR
    
    const createNewProfile = () => {

        //Ensure that the profile name is not already in use
        if (profiles!=""){
            const currentProfilesList = JSON.parse(profiles);
            for (let i = 0; i<currentProfilesList.length;i++){
                if (currentProfilesList[i]["info"]["name"]==createProfileInputBox){
                    console.log("Profile already exists");
                    return;
                }
            }
        }

        //Create the new profile
        const newProfile = {
            "info": {
                "name": createProfileInputBox,
                "date_created": new Date().toDateString()
            },
            "controllers": [
                //TODO: Figure out accepted controllers and create approperiate profile presets for each
            ]
        };
        if (profiles ==""){
            setProfiles("["+JSON.stringify(newProfile)+"]");
        }
        else {
            const newProfilesList = JSON.parse(profiles); //Turn the profiles (which are stored as a string) into a JSON object (i.e. list of dictionaries)
            newProfilesList.push(newProfile); //Add the new profile
            setProfiles(JSON.stringify(newProfilesList));
        }
    }

    const deleteProfile = () => {
        const newProfilesList = [];
        const currentProfilesList = JSON.parse(profiles);
        for (let i = 0; i<currentProfilesList.length;i++){
            if (currentProfilesList[i]["info"]["name"]==currentProfile){ 
                continue; //Do not add the profile we want to remove
            }
            else{
                newProfilesList.push(currentProfilesList[i]);
            }
        }
        setProfiles(JSON.stringify(newProfilesList)); //Turn the JSON object into a string then store it in profiles
        setCurrentProfile("No Profile Selected"); //Reset the current profile selected
        setCreateProfileInputBox(""); //Reset the create profile input box
    }


    if (!profileEditorMode){ //Load the normal settings tab
    return(
        <Box>
            <Divider>ROS</Divider><br/>
            <Box display="flex" justifyContent="center">
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
            <br/><Divider>Profiles</Divider><br/>
            <Grid container justifyContent={"space-evenly"}>
                <Grid item>
                    <Button variant={"contained"} onClick={()=>{setProfileEditorMode(true);
                                                                setProfilesService(1);
                                                                setCurrentProfile("No Profile Selected");}}>Launch Profile Editor</Button>
                </Grid>
            </Grid>
            <Box position="absolute" bottom="8px" left="10%" width="80%">
                <Button variant="contained" sx={{width: "100%"}} onClick={() => {save()}}>Save Settings</Button>
            </Box>
        </Box>
    )}
    else{ //load the profile editor
        return(
            <Box>
                <Grid container columns={1} alignItems={"center"} rowGap={1}>
                    <Grid item xs={1}>
                        <Grid container alignItems={"center"} columnSpacing={1} justifyContent={"space-evenly"}>
                            <Grid item>  
                                <Button onClick={()=>{setProfileEditorMode(false)}}>Return to Settings</Button>
                            </Grid>
                            <Grid item xs={1.5}></Grid>
                            <Grid item xs={1.5}>
                                <Typography fontFamily={"monospace"}>Current Profile: </Typography>
                            </Grid>
                            <Grid item xs={2}>
                                <Button variant={"outlined"} aria-controls={"multipliers-preset-menu"} aria-haspopup={"true"} aria-expanded={mouseAnchored} onClick={(e)=>{setMouseAnchor(e.currentTarget);}}>   
                                    {currentProfile}
                                    <Menu id={"multipliers-preset-menu"} open={mouseAnchored} anchorEl={anchorElement} onClick={(e)=>{setMouseAnchor(e.currentTarget)}}>
                                        {JSON.parse(profiles || '[{"info":{"name":"No Profile Selected"}}]').map((profile: { info: { name: string; }; })=>{ //If profiles is an empty string, replace it by a placeholder dictionary in the JSON.parse() argument
                                            return(
                                                <Button onClick={()=>{setCurrentProfile(profile.info.name)}}>{profile.info.name}</Button>
                                            )
                                        })}
                                    </Menu>
                                </Button>
                            </Grid>
                            <Grid item xs={1}></Grid>
                            <Grid item xs={3}> 
                                <TextField label="Create New Profile" variant="outlined" sx={{width: "100%"}} value={createProfileInputBox} onChange={(e) => setCreateProfileInputBox(e.target.value)} />
                            </Grid>
                            <Grid item xs={1}>
                                <Button variant={"contained"} onClick={createNewProfile}>
                                    Create
                                </Button>
                            </Grid>
                        </Grid>
                    </Grid>
                    <Grid item xs={1}>
                        <Grid container alignItems={"center"} columnSpacing={1} justifyContent={"space-evenly"}>
                            <Grid item xs={2}>
                                <Button variant={"contained"} onClick={deleteProfile}>
                                    Delete Profile
                                </Button>
                            </Grid>
                            <Grid item xs={2}>       
                                <Button variant={"contained"} onClick={()=>{setProfilesService(0);}}>
                                    Save Changes
                                </Button>
                            </Grid>
                        </Grid>
                    </Grid>
                </Grid>
            </Box>
        )
    }
}
