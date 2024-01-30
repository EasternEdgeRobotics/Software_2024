import { AlertCircle, CheckCircle2 } from "lucide-react";
import { Grid, Paper, Table, TableBody, TableCell, TableContainer, TableHead, TableRow, Slider, Box, Button} from "@mui/material";
import { useAtom } from "jotai";
import { IsROSConnected, PowerMultipliers, ProfilesList, CurrentProfile, Mappings, RequestingConfig } from "../api/Atoms";
import { useState, useEffect, } from "react";

export function StatusIndicator(props: {statement: boolean}) {
    if (props.statement) return <CheckCircle2 color="lime" />;
    else return <AlertCircle color="red" />;
}

export function BotTab() {
    const [isRosConnected] = useAtom(IsROSConnected);
    const [powerMultipliers, setPowerMultipliers] = useAtom(PowerMultipliers);
    const [profilesList, ] = useAtom(ProfilesList);
    const [currentProfile, ] = useAtom(CurrentProfile);
    const [,setRequestingConfig] = useAtom(RequestingConfig);
    const [controller1, setController1] = useState<string>("null");
    const [controller2, setController2] = useState<string>("null");
    const [mappings, ] = useAtom(Mappings);

    const status = [
        {"name": "ROS", "status": isRosConnected}
    ];
      
    const [, reloadComponent] = useState<number>(0);
    useEffect(() => {
        setInterval(() => {
            reloadComponent(Math.random());
        }, 100);
    }, []);

    useEffect(() => {
        for (let i = 0; i<profilesList.length;i++){
            if (currentProfile==profilesList[i].name){
                setController1(profilesList[i].controller1)
                setController2(profilesList[i].controller2)
            }
        }
    }, [currentProfile]);
    for (let i = 0; i < navigator.getGamepads().length; i++){
        if (navigator.getGamepads()[i]?.id == controller1){
            for (let j = 0; j<(navigator.getGamepads()[i]?.buttons.length as number);j++){
                if (navigator.getGamepads()[i]?.buttons[j].pressed){
                    console.log(mappings[0]["buttons"][j])
                }
            }
            for (let j = 0; j<(navigator.getGamepads()[i]?.axes.length as number);j++){
                if (navigator.getGamepads()[i]?.buttons[j].pressed){
                    console.log(mappings[0]["axes"][j])
                    console.log(mappings[0]["deadzone"][j])
                }
            }
        }
        if (navigator.getGamepads()[i]?.id == controller2){
            for (let j = 0; j<(navigator.getGamepads()[i]?.buttons.length as number);j++){
                if (navigator.getGamepads()[i]?.buttons[j].pressed){
                    console.log(mappings[1]["buttons"][j])
                }
            }
            for (let j = 0; j<(navigator.getGamepads()[i]?.axes.length as number);j++){
                if (navigator.getGamepads()[i]?.buttons[j].pressed){
                    console.log(mappings[1]["axes"][j])
                    console.log(mappings[1]["deadzone"][j])
                }
            }
        }
    }
    
    return (
        <Box flexGrow={1}>
            <Grid container justifyContent={"center"} spacing={1}>
                {["Power", "Surge", "Sway", "Heave", "Pitch", "Roll", "Yaw"].map((label, index) => {
                        return (
                            <Grid item xs={1} display="flex" justifyContent="center" alignItems="center" flexWrap="wrap" height="300px">
                                <Slider orientation="vertical" valueLabelDisplay="auto" step={5} defaultValue={powerMultipliers[index]} onChange={(_,value) => setPowerMultipliers(powerMultipliers.map((v, i) => {if (i == index) return value as number; else return v;}))} />
                                <Box flexBasis="100%" height="0" />
                                <h2>{label}: {powerMultipliers[index]}</h2>
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
