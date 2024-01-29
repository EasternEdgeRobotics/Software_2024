import { Box, Button, Dialog, DialogActions, DialogContent, FormControl, InputLabel, MenuItem, Select, Tab, Tabs, TextField , Grid, Divider} from "@mui/material";
import { useEffect, useState } from "react";
import ControllerTab from "./ProfileEditor/ControllerTab";
import { useAtom } from "jotai";
import { Mappings, ProfilesList, RequestingConfig, RequestingProfilesList } from "../api/Atoms";

export default function ProfileEditor(props: {open: boolean; onClose: () => void; currentProfile: string}) {
	const [tabIndex, setTabIndex] = useState<number>(0);
	const [controller1, setController1] = useState<number>(-1);
	const [controller2, setController2] = useState<number>(-1);
	const [currentProfile, setCurrentProfile] = useState<string>(props.currentProfile);
	const [profilesList, ] = useAtom(ProfilesList)
	const [profileTextField, setProfileTextField] = useState<string>("");
	const [,setRequestingConfig] = useAtom(RequestingConfig);
	const [,setRequestingProfilesList] = useAtom(RequestingProfilesList);
	const [mappings, setMappings] = useAtom(Mappings);

	const [, reloadComponent] = useState<number>(0);

	if (currentProfile != props.currentProfile) { //I don't know why this ended up being necessary but it fixed currentProfile being = "";
		setCurrentProfile(props.currentProfile);
	}

	useEffect(() => {
		window.addEventListener("gamepadconnected", () => {
			reloadComponent(Math.random());
		});
		window.addEventListener("gamepaddisconnected", (e) => {
			setTabIndex(0);
			if (controller1 == e.gamepad.index) {
				setController1(-1);
				const tmp = mappings;
				tmp[0] = {};
				setMappings(tmp);
			}
			if (controller2 == e.gamepad.index) {
				setController2(-1);
				const tmp = mappings;
				tmp[1] = {};
				setMappings(tmp);
			}
			reloadComponent(Math.random());
		});
	}, []);

	const profileExists = (profile : string) => {
		for (let i = 0; i<profilesList.length;i++){
			if (profilesList[i].name == profile){
				return true;
			} 
		}
		return false;
	}

	return (
		<Dialog open={props.open} onClose={props.onClose} fullWidth maxWidth="lg">
			<DialogContent sx={{paddingX: "24px", paddingY: "0", height: "65vh"}}>
				<h2 style={{position: "absolute", marginTop: "12px"}}>Profile Editor</h2>
				<Tabs value={tabIndex} onChange={(_, index) => setTabIndex(index)} sx={{paddingBottom: "12px"}} centered>
					<Tab label="General" />
					<Tab label="Controller 1" disabled={controller1 == -1} />
					<Tab label="Controller 2" disabled={controller2 == -1} />
				</Tabs>
				<h3>Editing Profile: {currentProfile}</h3>
				{tabIndex == 0 &&
					<Box>
						<FormControl fullWidth sx={{marginY: "16px"}}>
							<InputLabel>Controller 1</InputLabel>
							<Select value={controller1} label="Controller 1" onChange={(e) => {
								if (e.target.value as number < 0) { 
									return;
								}
								setController1(e.target.value as number); 
								setRequestingConfig({state:1, profileName:currentProfile, controller1:"null", controller2:"null"}); //We just want to update the profiles list
								for (let i = 0; i<profilesList.length; i++){
									if (profilesList[i].name == currentProfile){
										if (profilesList[i].controller1 != navigator.getGamepads()[e.target.value as number]?.id){ //If this controller is not recognized by profile, do not apply the mappings
											const tmp = mappings;
											if (e.target.value as number < 0) {
												tmp[0] = {}; 
												return;
											}
											tmp[0] = {"buttons": {}, "axes": {}};
											tmp[0]["buttons"] = {...Array.from({ length: navigator.getGamepads()[e.target.value as number]?.buttons.length ?? 0 }, () => "None")};
											tmp[0]["axes"] = {...Array.from({ length: navigator.getGamepads()[e.target.value as number]?.axes.length ?? 0 }, () => "None")};
											tmp[0]["deadzones"] = {...Array.from({ length: navigator.getGamepads()[e.target.value as number]?.axes.length ?? 0 }, () => "0")};
											setMappings(tmp);
										}
									}
								}
							}} sx={{width: "100%"}}>
								<MenuItem value={-1}>None</MenuItem>
								{navigator.getGamepads().length > 0 &&
									navigator.getGamepads().filter((gamepad) => gamepad !== null && gamepad.index !== controller2).map((gamepad) => {
										//Add menu item for every controller
										return <MenuItem value={gamepad?.index}>{gamepad?.id}</MenuItem>;
									})
								}
							</Select>
						</FormControl>
						<FormControl fullWidth sx={{marginTop: "2px"}}>
							<InputLabel>Controller 2</InputLabel>
							<Select value={controller2} label="Controller 2" onChange={(e) => {
								if (e.target.value as number < 0) { 
									return;
								}
								setController2(e.target.value as number); 
								setRequestingConfig({state:1, profileName:currentProfile, controller1:"null", controller2:"null"}); //We just want to update the profiles list
								for (let i = 0; i<profilesList.length; i++){
									if (profilesList[i].name == currentProfile){
										if (profilesList[i].controller2 != navigator.getGamepads()[e.target.value as number]?.id){ //If this controller is not recognized by profile, do not apply the mappings
											const tmp = mappings;
											if (e.target.value as number < 0) {
												tmp[1] = {}; 
												return;
											}
											tmp[1] = {"buttons": {}, "axes": {}};
											tmp[1]["buttons"] = {...Array.from({ length: navigator.getGamepads()[e.target.value as number]?.buttons.length ?? 0 }, () => "None")};
											tmp[1]["axes"] = {...Array.from({ length: navigator.getGamepads()[e.target.value as number]?.axes.length ?? 0 }, () => "None")};
											tmp[1]["deadzones"] = {...Array.from({ length: navigator.getGamepads()[e.target.value as number]?.axes.length ?? 0 }, () => "0")};
											setMappings(tmp);
										}
									}
								}
							}} sx={{width: "100%"}}>
								<MenuItem value={-1}>None</MenuItem>
								{navigator.getGamepads().length > 0 &&
									navigator.getGamepads().filter((gamepad) => gamepad !== null && gamepad.index !== controller1).map((gamepad) => {
										//Add menu item for every controller
										return <MenuItem value={gamepad?.index}>{gamepad?.id}</MenuItem>;
									})
								}
							</Select>
						</FormControl>
						<Box marginTop={0.5}></Box>
						<br/><Divider>Create a New Profile Based on These Mappings</Divider><br/>
						<Grid container alignItems={"center"} spacing={2}>
							<Grid item xs={10}>
								<TextField label="Profile Name" variant="outlined" sx={{width: "100%"}} value={profileTextField} 
								onChange={(e) => setProfileTextField(e.target.value)}/>
							</Grid>
							<Grid item xs={2}>
								<Button variant="contained" onClick={() => {
									if ((profileTextField.replaceAll(" ","") != "") && !(profileExists(profileTextField))){
									setCurrentProfile(profileTextField);
								}}}>Create Profile</Button>
							</Grid>
						</Grid>
						
					</Box>
				}
				{tabIndex == 1 &&
					<ControllerTab controller={controller1} index={0} />
				}
				{tabIndex == 2 &&
					<ControllerTab controller={controller2} index={1} />
				}
			</DialogContent>
			<DialogActions>
					<Button onClick={() => {setRequestingConfig({state:1, profileName:currentProfile, controller1:"null", controller2:"null"});}}>Temporary</Button>
					<Button onClick={() => {setRequestingProfilesList(1);}}>Temporary2</Button>
					<Button onClick={props.onClose}>Close</Button>
					<Button onClick={() => {
						let controller1Name: any = "null";
						let controller2Name: any = "null"; 
						if (controller1 == -1){
							return
						} else {
							controller1Name = navigator.getGamepads()[controller1]?.id;
						}
						if (controller2 != -1){
							controller2Name = navigator.getGamepads()[controller2]?.id;
						} 
						setRequestingConfig({state:0, profileName:currentProfile, controller1:controller1Name, controller2:controller2Name}); 
						props.onClose();
						}}>Save</Button>
			</DialogActions>
		</Dialog>
	);
}