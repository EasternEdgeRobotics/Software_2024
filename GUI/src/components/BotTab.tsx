import { AlertCircle, CheckCircle2 } from "lucide-react";
import { Grid, Paper, Table, TableBody, TableCell, TableContainer, TableHead, 
    TableRow, Slider, Box, Typography, Button, Menu} from "@mui/material";
import { useAtom } from "jotai";
import { IsROSConnected, PowerMultipliers, Profiles, ProfilesService } from "../api/Atoms";
import React from "react";
import { useState } from "react";

var sliderLabels: {[key: number]: string} = {
    0: "Power",
    1: "Surge",
    2: "Sway",
    3: "Heave",
    4: "Pitch",
    5: "Roll",
    6: "Yaw"
}

//Function to return vertical slider component
export function VerticalSlider(props: {sliderIndex: number, defaultValue:number, height:number, sliderChangedCallback:(sliderIndex: number, value: number) => void}){
        const sliderLabel = sliderLabels[props.sliderIndex];
        let currentValue = props.defaultValue;
    
        const [width, setWidth] = React.useState<number>(window.innerWidth);
    
        React.useEffect(() => {window.addEventListener("resize", () => {setWidth(window.innerWidth)});}, []);
    
        return(
            <Box height={(0.65)*props.height} justifyContent={"right"}>
                <Slider sx={{'& input[type="range"]': {WebkitAppearance: 'slider-vertical',},}}
                        orientation="vertical" defaultValue={props.defaultValue}
                        onChange={(e,val)=>{props.sliderChangedCallback(props.sliderIndex, val as number);
                                                                        currentValue=(val as number);}}
                        onKeyDown={preventHorizontalKeyboardNavigation} min={0} max={100} step={5}/>
                <Typography fontFamily={"monospace"} fontSize={0.015*(width)}>{sliderLabel}={currentValue}</Typography>
            </Box>
        )
    }

// Function to prevent control of slider using left and right arrow
function preventHorizontalKeyboardNavigation(event: React.KeyboardEvent) {
    if (event.key === 'ArrowLeft' || event.key === 'ArrowRight') {
      event.preventDefault(); 
    }
  }

export function StatusIndicator(props: {statement: any}) {
    if (props.statement) return <CheckCircle2 color="lime" />
    else return (<AlertCircle color="red" />)
}

export function BotTab() {
    const [isRosConnected] = useAtom(IsROSConnected);
    const [height, setHeight] = useState<number>(window.innerHeight);
    const [powerMultipliers, setPowerMultipliers] = useAtom(PowerMultipliers);
    const [currentProfile, setCurrentProfile] = useState<string>("Pick Profile"); //Stores the current selected profile 
    const [, setProfilesService] = useAtom(ProfilesService); //If ProfilesService == 0, tell the ROS client to overwrite it's stored profiles with what is here. If ProfilesService == 1, tell the ROS client to send its stored profiles. 
    const [profiles,] = useAtom(Profiles);
    const [currentPowerMultiplierPreset, setCurrentPowerMultiplierPreset] = useState<string>("Pick Power Multiplier Preset");
    React.useEffect(() => {window.addEventListener("resize", () => {setHeight(window.innerHeight)});}, []);

    const [anchorElement, setAnchorEL] = React.useState(null);

    const [mouseAnchoredOnProfilePicker, setMouseAnchoredOnProfilePicker] = React.useState<boolean>(false);
    const [mouseAnchoredOnPresetPicker, setMouseAnchoredOnPresetPicker] = React.useState<boolean>(false);

    const setMouseAnchor = (currentTarget:any, anchor:string) =>{
        if (anchor=="profile picker"){
            if (mouseAnchoredOnProfilePicker){
                setAnchorEL(null);
                setMouseAnchoredOnProfilePicker(false);
            }
            else {
                setAnchorEL(currentTarget);
                setMouseAnchoredOnProfilePicker(true);
            }
        }
        else if (anchor=="power multiplier picker"){
            if (mouseAnchoredOnPresetPicker){
                setAnchorEL(null);
                setMouseAnchoredOnPresetPicker(false);
            }
            else {
                setAnchorEL(currentTarget);
                setMouseAnchoredOnPresetPicker(true);
            }
        }
        
    }

    const status = [
        {"name": "ROS", "status": isRosConnected}
    ]

    // Function to handle changing slider value
    const sliderChanged = (sliderIndex: number, value: number) => {
        setPowerMultipliers(powerMultipliers.map((item, index) => index === sliderIndex ? value : item));
    } 
    
    return(
        <Box flexGrow={1}>
            <Grid container justifyContent={"center"} spacing={1}>
                {[0,1,2,3,4,5,6].map((index) => {
                    return(
                        <Grid item xs={1}>
                            <VerticalSlider sliderIndex={index} height={height} defaultValue={powerMultipliers[index]} sliderChangedCallback={sliderChanged}/>
                        </Grid>)})
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
                                    return(
                                        <TableRow key={data.name} sx={{ '&:last-child td, &:last-child th': { border: 0 } }}>
                                            <TableCell align="center">{data.name}</TableCell>
                                            <TableCell align="center"><StatusIndicator statement={data.status} /></TableCell>
                                        </TableRow>
                                    )
                                })}
                                </TableBody>
                            </Table>
                        </TableContainer>
                        </Grid>
                        <Grid item xs={4}>
                            <Button variant={"outlined"} aria-controls={"multipliers-preset-menu"} aria-haspopup={"true"} aria-expanded={mouseAnchoredOnPresetPicker} onClick={(e)=>{setMouseAnchor(e.currentTarget, "power multiplier picker")}}>
                                {currentPowerMultiplierPreset}      
                                <Menu id={"multipliers-preset-menu"} open={mouseAnchoredOnPresetPicker} anchorEl={anchorElement} onClick={(e)=>{setMouseAnchor(e.currentTarget, "power multiplier picker")}}>
                                    {[currentPowerMultiplierPreset].map((preset) => { //The mapped list here should be changed to a list of available presets
                                        return(
                                            <Button onClick={()=>{setCurrentPowerMultiplierPreset(preset)}}>{preset}</Button>
                                        )
                                    })}
                                </Menu>
                            </Button>
                        </Grid>
                    </Grid>
                </Grid>
            </Grid>
            <Grid container item margin={6} justifyContent={"center"}>
                <Button variant={"outlined"} aria-controls={"profile-menu"} aria-haspopup={"true"} aria-expanded={mouseAnchoredOnProfilePicker} onClick={(e)=>{setMouseAnchor(e.currentTarget, "profile picker");
                                                                                                                                                                setProfilesService(1);}}>
                    {currentProfile}
                    <Menu id={"multipliers-preset-menu"} open={mouseAnchoredOnProfilePicker} anchorEl={anchorElement} onClick={(e)=>{setMouseAnchor(e.currentTarget, "profile picker")}}>
                        {JSON.parse(profiles || '[{"info":{"name":"No Profile Selected"}}]').map((profile: { info: { name: string; }; })=>{ //If profiles is an empty string, replace it by a placeholder dictionary in the JSON.parse() argument
                            return(
                                <Button onClick={()=>{setCurrentProfile(profile.info.name)}}>{profile.info.name}</Button>
                            )
                        })}
                    </Menu>
                </Button>
            </Grid>
        </Box>
    );
}