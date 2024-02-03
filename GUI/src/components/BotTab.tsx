import { AlertCircle, CheckCircle2 } from "lucide-react";
import { Grid, Paper, Table, TableBody, TableCell, TableContainer, TableHead, TableRow, Slider, Box, Button} from "@mui/material";
import { useAtom } from "jotai";
import { IsROSConnected, ThrusterMultipliers, ProfilesList, CurrentProfile, Mappings, ControllerInput } from "../api/Atoms";
import { useState, useEffect, } from "react";

export function StatusIndicator(props: {statement: boolean}) {
    if (props.statement) return <CheckCircle2 color="lime" />;
    else return <AlertCircle color="red" />;
}

export function BotTab() {
    const [isRosConnected] = useAtom(IsROSConnected);
    const [thrusterMultipliers, setThrusterMultipliers] = useAtom(ThrusterMultipliers);
    const [profilesList, ] = useAtom(ProfilesList);
    const [currentProfile, ] = useAtom(CurrentProfile);
    const [controller1, setController1] = useState<string>("null");
    const [controller2, setController2] = useState<string>("null");
    const [initalPageLoad, setInitialPageLoad] = useState<boolean>(true);
    const [mappings, ] = useAtom(Mappings);
    const [,setControllerInput] = useAtom(ControllerInput);

    const status = [
        {"name": "ROS", "status": isRosConnected}
    ];
      
    const [, reloadComponent] = useState<number>(0);

    useEffect(() => {
        for (let i = 0; i<profilesList.length;i++){
            if (currentProfile==profilesList[i].name){
                setController1(profilesList[i].controller1);
                setController2(profilesList[i].controller2);
            }
        }
    }, [currentProfile]);

    //All inputs will be in the format of a list, where the first element is input value (1 for button and between (-1,1) for axis) 
    //and the second element is the action
    const input_listener =(controller: number) => {
        let controllerDetected = false;
        for (let i =0; i<navigator.getGamepads().length; i++){
            if (navigator.getGamepads()[i] != null){
                controllerDetected = true;
            }
        }
        if (!controllerDetected){
            return; 
        } else if (Object.keys(mappings[controller]).length == 0) { //This indicates that no mappings have been loaded.
            if (initalPageLoad){
                console.log("No mappings have been loaded. Go to the settings and load a profile.");
                setInitialPageLoad(false);
            }
            return; 
        } else if (Object.keys(mappings[controller]["buttons"]).length == 0) { //This indicates that mappings have been loaded, 
                                                                              //but that this is controller 1 and no bindings have 
                                                                              //been set for it (since it is nullable in the database)
            return; 
        }

        for (let i = 0; i < navigator.getGamepads().length; i++){
            if (navigator.getGamepads()[i]?.id == controller1){
                for (let j = 0; j<(navigator.getGamepads()[i]?.buttons.length as number);j++){
                    if (navigator.getGamepads()[i]?.buttons[j].pressed){
                        if (mappings[controller]["buttons"][j] != "None"){
                            const controller_input = [];
                            controller_input.push(1);
                            controller_input.push(mappings[controller]["buttons"][j]);
                            setControllerInput(JSON.stringify(controller_input));
                        }
                    }
                }
                for (let j = 0; j<(navigator.getGamepads()[i]?.axes.length as number);j++){
                    const deadzone = Number(mappings[controller]["deadzones"][j]) > 0.05 ? Number(mappings[controller]["deadzones"][j]): 0.05;
                    if (Math.abs(navigator.getGamepads()[i]?.axes[j] as number) >= deadzone){
                        if (mappings[controller]["axes"][j] != "None"){
                            const controller_input = [];
                            controller_input.push(navigator.getGamepads()[i]?.axes[j]);
                            controller_input.push(mappings[controller]["axes"][j]);
                            setControllerInput(JSON.stringify(controller_input));
                        }
                    }
                }
            }
        }
    }

    useEffect(() => {
        setInterval(() => {
            reloadComponent(Math.random());
            input_listener(0);
            input_listener(1);
        }, 100);
    }, []);
    
    return (
        <Box flexGrow={1}>
            <Grid container justifyContent={"center"} spacing={1}>
                {["Power", "Surge", "Sway", "Heave", "Pitch", "Roll", "Yaw"].map((label, index) => {
                        return (
                            <Grid item xs={1} display="flex" justifyContent="center" alignItems="center" flexWrap="wrap" height="300px">
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
