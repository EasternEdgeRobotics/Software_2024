import { Box, Button, Dialog, DialogActions, DialogContent, FormControl, InputLabel, MenuItem, Select, Tab, Tabs, TextField } from "@mui/material";
import { useEffect, useState } from "react";
import ControllerTab from "./ProfileEditor/ControllerTab";
import { useAtom } from "jotai";
import { Mappings } from "../api/Atoms";

export default function ProfileEditor(props: {open: boolean; onClose: () => void}) {
	const [tabIndex, setTabIndex] = useState<number>(0);
	const [controller1, setController1] = useState<number>(-1);
	const [controller2, setController2] = useState<number>(-1);
	const [profileName, setProfileName] = useState<string>("");
	const [mappings, setMappings] = useAtom(Mappings);

	const [, reloadComponent] = useState<number>(0);
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

	return (
		<Dialog open={props.open} onClose={props.onClose} fullWidth maxWidth="lg">
			<DialogContent sx={{paddingX: "24px", paddingY: "0", height: "65vh"}}>
				<h2 style={{position: "absolute", marginTop: "12px"}}>Profile Editor</h2>
				<Tabs value={tabIndex} onChange={(_, index) => setTabIndex(index)} sx={{paddingBottom: "12px"}} centered>
					<Tab label="General" />
					<Tab label="Controller 1" disabled={controller1 == -1} />
					<Tab label="Controller 2" disabled={controller2 == -1} />
				</Tabs>
				{tabIndex == 0 &&
					<Box>
						<TextField label="Profile Name" variant="outlined" sx={{width: "100%"}} value={profileName} onChange={(e) => setProfileName(e.target.value)} error={profileName.replaceAll(" ", "").length == 0} />
						<FormControl fullWidth sx={{marginY: "16px"}}>
							<InputLabel>Controller 1</InputLabel>
							<Select value={controller1} label="Controller 1" onChange={(e) => {
								setController1(e.target.value as number); 
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
								setController2(e.target.value as number); const tmp = mappings;
								if (e.target.value as number < 0) {
									tmp[1] = {}; 
									return;
								}
								tmp[1] = {"buttons": {}, "axes": {}};
								tmp[1]["buttons"] = {...Array.from({ length: navigator.getGamepads()[e.target.value as number]?.buttons.length ?? 0 }, () => "None")};
								tmp[1]["axes"] = {...Array.from({ length: navigator.getGamepads()[e.target.value as number]?.axes.length ?? 0 }, () => "None")};
								tmp[1]["deadzones"] = {...Array.from({ length: navigator.getGamepads()[e.target.value as number]?.axes.length ?? 0 }, () => "0")};
								setMappings(tmp);
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
					<Button onClick={props.onClose}>Close</Button>
					<Button onClick={() => {console.log(mappings); props.onClose();}}>Save</Button>
			</DialogActions>
		</Dialog>
	);
}