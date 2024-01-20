import { Box, Button, Checkbox, Dialog, DialogActions, DialogContent, DialogTitle, FormControl, FormControlLabel, InputLabel, MenuItem, Select, Tab, Tabs } from "@mui/material";
import { useState } from "react";

export default function ProfileEditor(props: {open: boolean; onClose: () => void}) {
	const [tabIndex, setTabIndex] = useState<number>(0);
	const [controller1, setController1] = useState<number>(-1);
	const [controller2, setController2] = useState<number>(-1);
	const [controller2Enabled, setController2Enabled] = useState<boolean>(false);

    window.addEventListener("gamepadconnected", (e) => {
        if (controller1 === -1) setController1(e.gamepad.index);
	});

    window.addEventListener("gamepaddisconnected", (e) => {
		setTabIndex(0);
        [{controller: controller1, setController: setController1}, {controller: controller2, setController: setController2}].forEach(c => {
			if (e.gamepad.index === c.controller) {
				if (navigator.getGamepads.length > 1) {
					for (const controller of navigator.getGamepads()) {
						if (controller?.connected && controller !== e.gamepad) {
							c.setController(controller?.index);
							break;
						}
					}
				} else c.setController(-1);
			}
		});
    });

	return (
		<Box>
			<Dialog open={props.open} onClose={props.onClose} fullWidth={true} maxWidth="lg">
				<DialogTitle>Editing Profile: MAKE THE PROFILE NAME SHOW UP</DialogTitle>
				<DialogContent sx={{paddingX: "24px", paddingY: "0"}}>
					<Tabs value={tabIndex} onChange={(_, index) => setTabIndex(index)} sx={{paddingBottom: "16px"}} centered>
						<Tab label="General" />
						<Tab label="Controller 1" disabled={controller1 === -1} />
						<Tab label="Controller 2" disabled={controller2 === -1 || !controller2Enabled} />
					</Tabs>
					<Box sx={{height: "65vh"}} display={"flex"} justifyContent={"center"}>
					{tabIndex == 0 &&
							<Box sx={{width: "100%"}}>
								<FormControl sx={{width: "100%"}}>
									<InputLabel>Controller 1</InputLabel>
									<Select value={controller1} label="Controller 1" onChange={(e) => setController1(e.target.value as number)} error={(controller1 === -1)} sx={{width: "100%"}}>
										{navigator.getGamepads().length > 0 &&
											navigator.getGamepads().filter((gamepad) => gamepad !== null && gamepad.index !== controller2).map((gamepad) => {
												//Add menu item for every controller
												return <MenuItem value={gamepad?.index}>{gamepad?.id}</MenuItem>;
											})
										}
										{navigator.getGamepads().filter((gamepad) => gamepad !== null && gamepad.index !== controller2).length <= 0 &&
											<MenuItem value={-1} disabled={true}>No Controllers Detected!</MenuItem>
										}
									</Select>
								</FormControl>
								<FormControlLabel control={<Checkbox />} label="Controller 2" checked={controller2Enabled} onChange={(_, checked) => setController2Enabled(checked)} sx={{marginY: "8px"}}/>
								<FormControl sx={{width: "100%"}}>
									<InputLabel>Controller 2</InputLabel>
									<Select value={controller2} label="Controller 2" onChange={(e) => setController2(e.target.value as number)} error={(controller2 === -1)} sx={{width: "100%"}} disabled={!controller2Enabled}>
									{navigator.getGamepads().length > 0 &&
										navigator.getGamepads().filter((gamepad) => gamepad !== null && gamepad.index !== controller1).map((gamepad) => {
											//Add menu item for every controller
											return <MenuItem value={gamepad?.index}>{gamepad?.id}</MenuItem>;
										})
									}
									{navigator.getGamepads().filter((gamepad) => gamepad !== null && gamepad.index !== controller1).length <= 0 &&
										<MenuItem value={-1} disabled={true}>No Controllers Detected!</MenuItem>
									}
									</Select>
								</FormControl>
							</Box>			
					}
					{tabIndex == 1 &&
						<Box sx={{width: "100%"}}>
							{navigator.getGamepads()[controller1]?.buttons.map((button, index) => (
								<h1 style={{color: button.pressed ? "green" : "red"}}>Button {index}</h1>
							))}
						</Box>
					}
					</Box>
				</DialogContent>
				<DialogActions>
					<Button onClick={props.onClose}>Close</Button>
					<Button onClick={() => props.onClose()}>Save</Button>
				</DialogActions>
			</Dialog>
		</Box>
	);
}