import { Box, Button, Dialog, DialogActions, DialogContent, DialogTitle, FormControl, InputLabel, MenuItem, Select, Tab, Tabs, TextField } from "@mui/material";
import { useAtom } from "jotai";
import { useEffect, useState } from "react";
import { Mappings } from "../api/Atoms";
import ControllerMapping from "./ProfileEditor/ControllerMapping";

export default function ProfileEditor(props: {open: boolean; onClose: () => void}) {
	const [tabIndex, setTabIndex] = useState<number>(0);
	const [controller1, setController1] = useState<number>(-1);
	const [controller1Buttons, setController1Buttons] = useState<number>(-1);
	const [controller1Axes, setController1Axes] = useState<number>(-1);
	const [controller2, setController2] = useState<number>(-1);
	const [controller2Buttons, setController2Buttons] = useState<number>(-1);
	const [controller2Axes, setController2Axes] = useState<number>(-1);
	const [profileName, setProfileName] = useState<string>("");
	const [mappings] = useAtom(Mappings);

    const [, setJankSolution] = useState<number>(0);
    useEffect(() => {
        window.addEventListener("gamepadconnected", () => {
            setJankSolution(Math.random());
        });
    }, []);

	return (
		<Box>
			<Dialog open={props.open} onClose={props.onClose} fullWidth={true} maxWidth="lg">
				<DialogTitle>Profile Editor</DialogTitle>
				<DialogContent sx={{paddingX: "24px", paddingY: "0", height: "65vh"}}>
					<Tabs value={tabIndex} onChange={(_, index) => setTabIndex(index)} sx={{paddingBottom: "8px"}} centered>
						<Tab label="General" />
						<Tab label="Controller 1" disabled={controller1 == -1} />
						<Tab label="Controller 2" disabled={controller2 == -1} />
					</Tabs>
					{tabIndex == 0 &&
						<Box>
							<TextField label="Profile Name" variant="outlined" fullWidth value={profileName} onChange={(e) => setProfileName(e.target.value)} error={profileName.replace(" ", "").length == 0}/>
							<FormControl fullWidth sx={{marginY: "12px"}}>
								<InputLabel>Controller 1</InputLabel>
								<Select value={controller1} label="Controller 1" onChange={(e) => {setController1(e.target.value as number); setController1Buttons(navigator.getGamepads()[e.target.value as number]?.buttons.length || 0); setController1Axes(navigator.getGamepads()[e.target.value as number]?.axes.length || 0);}} sx={{width: "100%"}}>
									<MenuItem value={-1}>None</MenuItem>
									{navigator.getGamepads().length > 0 &&
										navigator.getGamepads().filter((gamepad) => gamepad !== null && gamepad.index !== controller2).map((gamepad) => {
											//Add menu item for every controller
											return <MenuItem value={gamepad?.index}>{gamepad?.id}</MenuItem>;
										})
									}
								</Select>
							</FormControl>
							<FormControl fullWidth>
								<InputLabel>Controller 2</InputLabel>
								<Select value={controller2} label="Controller 2" onChange={(e) => {setController2(e.target.value as number); setController2Buttons(navigator.getGamepads()[e.target.value as number]?.buttons.length || 0); setController2Axes(navigator.getGamepads()[e.target.value as number]?.axes.length || 0);}} sx={{width: "100%"}}>
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
						<ControllerMapping controller={controller1} buttons={controller1Buttons} axes={controller1Axes} />			
					}
					{tabIndex == 2 &&
						<ControllerMapping controller={controller2} buttons={controller2Buttons} axes={controller2Axes} />
					}
				</DialogContent>
				<DialogActions>
					<Button onClick={props.onClose}>Close</Button>
					<Button onClick={() => {console.log(mappings); props.onClose();}}>Save</Button>
				</DialogActions>
			</Dialog>
		</Box>
	);
}