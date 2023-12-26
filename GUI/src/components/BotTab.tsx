import { AlertCircle, CheckCircle2 } from "lucide-react";
import { Grid, Paper, Table, TableBody, TableCell, TableContainer, TableHead, 
    TableRow, Slider, Box, Typography, Button, Menu, MenuItem} from "@mui/material";
import { useAtom } from "jotai";
import { IsROSConnected, PowerMultipliers } from "../api/Atoms";
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

// Function to prevent control of slider using left and right arrow
function preventHorizontalKeyboardNavigation(event: React.KeyboardEvent) {
    if (event.key === 'ArrowLeft' || event.key === 'ArrowRight') {
      event.preventDefault(); 
    }
  }

// Function to return vertical slider component
export function VerticalSlider(props: {sliderIndex: number, defaultValue:number, height:number, sliderChangedCallback:(sliderIndex: number, value: number) => void}){
    const sliderLabel = sliderLabels[props.sliderIndex];
    let currentValue = props.defaultValue;
    
    const [width, setWidth] = React.useState<number>(window.innerWidth);

    React.useEffect(() => {window.addEventListener("resize", () => {setWidth(window.innerWidth)});}, []);

    return(
        <Box height={(0.85)*props.height} justifyContent={"right"}>
            <Slider sx={{'& input[type="range"]': {WebkitAppearance: 'slider-vertical',},}} 
                    orientation="vertical" defaultValue={props.defaultValue} 
                    onChange={(e,val)=>{props.sliderChangedCallback(props.sliderIndex, val as number);
                                                                    currentValue=(val as number);}}
                    onKeyDown={preventHorizontalKeyboardNavigation} min={0} max={100} step={5}/>
            <Typography fontFamily={"monospace"} fontSize={0.015*(width)}>{sliderLabel}={currentValue}</Typography>
        </Box>
    )
}

export function StatusIndicator(props: {statement: any}) {
    if (props.statement) return <CheckCircle2 color="lime" />
    else return (<AlertCircle color="red" />)
}

export function BotTab() {
    const [isRosConnected] = useAtom(IsROSConnected);
    const [height, setHeight] = useState<number>(window.innerHeight);
    const [powerMultipliers, setPowerMultipliers] = useAtom(PowerMultipliers);
    const [currentProfile, setCurrentProfile] = useState<string>("Pick Profile");
    React.useEffect(() => {window.addEventListener("resize", () => {setHeight(window.innerHeight)});}, []);

    const [anchorElement, setAnchorEL] = React.useState(null);

    const [mouseAnchored, setMouseAnchored] = React.useState<boolean>(false);

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
                            <VerticalSlider sliderIndex={index} height={(0.8)*height} defaultValue={powerMultipliers[index]} sliderChangedCallback={sliderChanged}/>
                        </Grid>)})
                }
                <Grid item xs={2}>
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
            </Grid>
            <Grid container item margin={6} justifyContent={"center"}>
                <Button aria-controls={"profile-menu"} aria-haspopup={"true"} aria-expanded={mouseAnchored} onClick={(e)=>{setMouseAnchor(e.currentTarget)}}>
                        {currentProfile}      
                        <Menu id={"profile-menu"} open={mouseAnchored} anchorEl={anchorElement} onClick={(e)=>{setMouseAnchor(e.currentTarget)}}>
                            <Button onClick={()=>{setCurrentProfile("Profile 1")}}>Profile 1</Button>
                        </Menu>
                </Button>
            </Grid>
        </Box>
    );
}