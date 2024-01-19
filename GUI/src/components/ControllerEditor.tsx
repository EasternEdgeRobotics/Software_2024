import { Box, Button, Checkbox, Dialog, DialogActions, DialogContent, DialogTitle, FormControl, FormControlLabel, InputLabel, MenuItem, Select, Tab, Tabs } from "@mui/material";
import { useState } from "react";

export default function ControllerEditor() {
	const [open, setOpen] = useState<boolean>(false);
	const [tabIndex, setTabIndex] = useState<number>(0);
	const [controller1, setController1] = useState<number>(-1);
	const [controller2, setController2] = useState<number>(-1);
	const [controller2Enabled, setController2Enabled] = useState<boolean>(false);

	return (
		<Box>
			<Box display="flex" justifyContent="center" marginBottom="16px">
                <Button sx={{width: "40%", height: "56px"}} variant="outlined" onClick={() => setOpen(true)}>Open Profile Editor</Button>
            </Box>
			<Dialog open={open} onClose={() => setOpen(false)} fullWidth={true} maxWidth="lg">
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
											navigator.getGamepads().map((gamepad) => {
												//Add menu item for every controller
												if (gamepad !== null && gamepad.index !== controller2){
													return <MenuItem value={gamepad?.index}>{gamepad?.id}</MenuItem>;
												}
											})
										}
										{navigator.getGamepads().length <= 0 &&
											<MenuItem value={-1}>No Controllers Detected!</MenuItem>
										}
									</Select>
								</FormControl>
								<FormControlLabel control={<Checkbox />} label="Controller 2" checked={controller2Enabled} onChange={(_, checked) => setController2Enabled(checked)} sx={{marginY: "8px"}}/>
								<FormControl sx={{width: "100%"}}>
									<InputLabel>Controller 2</InputLabel>
									<Select value={controller2} label="Controller 2" onChange={(e) => setController2(e.target.value as number)} error={(controller2 === -1)} sx={{width: "100%"}} disabled={!controller2Enabled}>
									{navigator.getGamepads().length > 0 &&
										navigator.getGamepads().map((gamepad) => {
											//Add menu item for every controller
											if (gamepad !== null && gamepad.index !== controller1){
												return <MenuItem value={gamepad?.index}>{gamepad?.id}</MenuItem>;
											}
										})
									}
									{navigator.getGamepads().length <= 0 &&
										<MenuItem value={-1}>No Controllers Detected!</MenuItem>
									}
									</Select>
								</FormControl>
							</Box>			
					}
					</Box>
				</DialogContent>
				<DialogActions>
					<Button onClick={() => setOpen(false)}>Close</Button>
					<Button onClick={() => setOpen(false)}>Save</Button>
				</DialogActions>
			</Dialog>
		</Box>
	);
}