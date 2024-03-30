import { AlertCircle, CheckCircle2 } from "lucide-react";
import { Grid, Paper, Table, TableBody, TableCell, TableContainer, TableHead, TableRow, Slider, Box, Button} from "@mui/material";
import { useAtom } from "jotai";
import { IsROSConnected, ThrusterMultipliers, ProfilesList, CurrentProfile, Mappings, ControllerInput } from "../api/Atoms";
import { useState, useEffect, } from "react";
import React from "react";

export function StatusIndicator(props: {statement: boolean}) {
    if (props.statement) return <CheckCircle2 color="lime" />;
    else return <AlertCircle color="red" />;
}

export function BotTab() {
    const [isRosConnected] = useAtom(IsROSConnected);

    const [thrusterMultipliers, setThrusterMultipliers] = useAtom(ThrusterMultipliers);

    const [profilesList] = useAtom(ProfilesList);
    const [currentProfile] = useAtom(CurrentProfile);
    const [mappings] = useAtom(Mappings);
    const [,setControllerInput] = useAtom(ControllerInput);

    const [controllerDetected, setControllerDetected] = useState<boolean>(false);

    let controller1 = "null";
    let controller2 = "null";

    let initialPageLoad = true;

    const status = [
        {"name": "ROS", "status": isRosConnected},
        {"name":"Controller Detected", "status": controllerDetected}
    ];
      
    const [, reloadComponent] = useState<number>(0);

    useEffect(() => { //Whenever the current profile changes, set controller 1 and 2 to that profile's controllers
        for (let i = 0; i < profilesList.length;i++){
            if (currentProfile == profilesList[i].name){
                controller1 = profilesList[i].controller1;
                controller2 = profilesList[i].controller2;
            }
        }
    }, [currentProfile]);

    //All inputs will be in the format of a list, where the first element is input value (1 for button and between (-1,1) for axis) 
    //and the second element is the action

    //Likely one of the most important functions in the software package
    //This function listens to the controller input and updates the ControllerInput global state. When this state is changed, 
    //ROS.tsx recognizes this and publishes the input to the device running the rosbridge server (RPi4 inside the enclosure) 
    const input_listener = (controller: number) => {
        let controllerDetected = false;
        for (let i =0; i<navigator.getGamepads().length; i++){
            if (navigator.getGamepads()[i] != null){ //The browser (accessed through the predefined navigator object) natively recognizes controllers 
                                                     //If navigator.getGamepads() contains 1 non-null object, it means that the browser has detected a controller
                controllerDetected = true;
            }
        }
        if (!controllerDetected){
            return; //Don't bother with the rest of the function if no controller is detected
        } else if (Object.keys(mappings[controller]).length == 0) { //This indicates that no mappings have been loaded.
            if (initialPageLoad){
                console.log("No mappings have been loaded. Go to the settings and load a profile.");
                initialPageLoad = false;
            }
            return; 
        } else if (Object.keys(mappings[controller]["buttons"]).length == 0) { //This indicates that mappings have been loaded, 
                                                                              //but that this input_listener function is being called for controller 2
                                                                              //which happens to have no mappings (it is nullable in the database)
            return; //Don't bother with the rest of the function 
        }

        setControllerDetected(true); //Used to visually indicate a controller has been detected, and associated mappings have been loaded

        //The below for loop is quick, so it will detect a button press on any button even if it was for a short time
        
        const gamepadsInstance = navigator.getGamepads(); 

        const controller_input_list:(string|number|undefined)[][] = [];

        for (let i = 0; i < gamepadsInstance.length; i++){
            if (gamepadsInstance[i]?.id == [controller1, controller2][controller]){
                for (let j = 0; j<(gamepadsInstance[i]?.buttons.length as number);j++){
                    if (gamepadsInstance[i]?.buttons[j].pressed){ //Button press detected
                        if (mappings[controller]["buttons"][j] != "None"){
                            const controller_input = [];
                            controller_input.push(1);
                            controller_input.push(mappings[controller]["buttons"][j]);
                            controller_input_list.push(controller_input);
                        }
                    }
                }
                for (let j = 0; j<(gamepadsInstance[i]?.axes.length as number);j++){
                    const deadzone = Number(mappings[controller]["deadzones"][j]) > 0.05 ? Number(mappings[controller]["deadzones"][j]): 0.05;
                    if (Math.abs(gamepadsInstance[i]?.axes[j] as number) >= deadzone){ //The axis has been moved beyond it's "deadzone", which means that this is actual pilot input and not controller drift
                        if (mappings[controller]["axes"][j] != "None"){
                            const controller_input = [];
                            controller_input.push(gamepadsInstance[i]?.axes[j]);
                            controller_input.push(mappings[controller]["axes"][j]);
                            controller_input_list.push(controller_input);
                        }
                    }
                }
            }
        }
        if (controller_input_list.length > 0) {setControllerInput(JSON.stringify(controller_input_list));
                                                console.log(controller_input_list);} // The ROS.tsx script will detect changes to the global variable "ControllerInput"
    }

    useEffect(() => { //Constantly run the input listeners for controllers 1 and 2 (often addressed as controllers 0 and 1)
        setInterval(() => {
            reloadComponent(Math.random());
            input_listener(0);
            input_listener(1);
            //input_publisher();
        }, 100);
    }, []);
    
    return (
        <Box flexGrow={1}>
            <Grid container justifyContent={"center"} spacing={1}>
                {["Power", "Surge", "Sway", "Heave", "Pitch", "Roll", "Yaw"].map((label, index) => {
                        return (
                            <Grid item xs={1} key={index} display="flex" justifyContent="center" alignItems="center" flexWrap="wrap" height="300px">
                                <Slider orientation="vertical" valueLabelDisplay="auto" step={5} defaultValue={thrusterMultipliers[index]} onChange={(_,value) => setThrusterMultipliers(thrusterMultipliers.map((v, i) => {if (i == index) return value as number; else return v;}))} />
                                <Box flexBasis="100%" height="0" />
                                <h2>{label}: {thrusterMultipliers[index]}</h2>
                            </Grid>
                        );})
                }
                <Grid item xs={3}>
                    <Grid container justifyContent={"center"} rowSpacing={2}>
                        <Grid item xs={12}>
                            <TableContainer component={Paper}>
                                <Table>
                                    <TableHead>
                                        <TableRow>
                                            <TableCell align="center">Service</TableCell>
                                            <TableCell align="center">Status</TableCell>
                                        </TableRow>
                                    </TableHead>
                                    <TableBody>
                                    {status.map((data) => {
                                        return (
                                            <TableRow key={data.name} sx={{ "&:last-child td, &:last-child th": { border: 0 } }}>
                                                <TableCell align="center">{data.name}</TableCell>
                                                <TableCell align="center"><StatusIndicator statement={data.status} /></TableCell>
                                            </TableRow>
                                        );
                                    })}
                                    </TableBody>
                                </Table>
                            </TableContainer>
                        </Grid>
                    </Grid>
                </Grid>
            </Grid>
            <Grid container justifyContent={"center"} spacing={1} sx={{marginTop: "64px"}}>
                <Grid item xs={3.5}>
                    <Button variant="contained" sx={{width: "100%", height: "3rem"}}>Load Power Preset</Button>
                </Grid>
                <Grid item xs={3.5}>
                    <Button variant="contained" sx={{width: "100%", height: "3rem"}}>Save Power Preset</Button>
                </Grid>
                <Grid item xs={3} />
            </Grid>
        </Box>
    );
}