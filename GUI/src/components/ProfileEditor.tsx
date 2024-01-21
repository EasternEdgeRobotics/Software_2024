import { Box, Button, Dialog, DialogActions, DialogContent, DialogTitle, Divider, FormControl, Grid, InputLabel, LinearProgress, MenuItem, Select, Tab, Tabs, TextField } from "@mui/material";
import { useEffect, useState } from "react";

export default function ProfileEditor(props: {open: boolean; onClose: () => void}) {
	const [tabIndex, setTabIndex] = useState<number>(0);
	const [controller1, setController1] = useState<number>(-1);
	const [controller2, setController2] = useState<number>(-1);
	
	//refresh component every 10 ms if on controller tabs
	const [, setJankSolution] = useState<number>(0);
	useEffect(() => {
		setInterval(() => {
			setJankSolution(Math.random());
		}, 100);
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
							<TextField label="Profile Name" variant="outlined" fullWidth />
							<FormControl fullWidth sx={{marginY: "12px"}}>
								<InputLabel>Controller 1</InputLabel>
								<Select value={controller1} label="Controller 1" onChange={(e) => setController1(e.target.value as number)} sx={{width: "100%"}}>
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
								<Select value={controller2} label="Controller 2" onChange={(e) => setController2(e.target.value as number)} sx={{width: "100%"}}>
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
						<Box>
							<Divider sx={{marginBottom: "8px"}}>Buttons</Divider>
							<Grid container spacing={1}>
								{navigator.getGamepads()[controller1]?.buttons.map((button, index) => (
									<Grid item xs={3}>
										<FormControl fullWidth>
											<InputLabel sx={{color: button.pressed ? "#87CEEB" : "light-grey"}}>Button {index}</InputLabel>
											<Select label={`Button ${index}`} sx={{width: "100%"}}>

											</Select>
										</FormControl>
									</Grid>
								))}
							</Grid>
							<Divider sx={{marginY: "8px"}}>Axes</Divider>
							<Grid container spacing={1}>
								{navigator.getGamepads()[controller1]?.axes.map((axis, index) => (
									<Grid item xs={12}>
										<Box display="flex" alignItems="center" marginBottom="4px">
											<LinearProgress variant="determinate" value={axis*50+50} sx={{width: "100%"}} />
										</Box>
										<FormControl fullWidth>
											<InputLabel>Axis {index} action</InputLabel>
											<Select label={`Axis ${index} action`}>

											</Select>
										</FormControl>
									</Grid>
								))}
							</Grid>

						</Box>			
					}
					{tabIndex == 2 &&
						<Box>
							<Divider sx={{marginBottom: "8px"}}>Buttons</Divider>
							<Grid container spacing={1}>
								{navigator.getGamepads()[controller2]?.buttons.map((button, index) => (
									<Grid item xs={3}>
										<FormControl fullWidth>
											<InputLabel sx={{color: button.pressed ? "#87CEEB" : "light-grey"}}>Button {index}</InputLabel>
											<Select label={`Button ${index}`} sx={{width: "100%"}}>

											</Select>
										</FormControl>
									</Grid>
								))}
							</Grid>
							<Divider sx={{marginY: "8px"}}>Axes</Divider>
							<Grid container spacing={1}>
								{navigator.getGamepads()[controller2]?.axes.map((axis, index) => (
									<Grid item xs={12}>
										<Box display="flex" alignItems="center" marginBottom="4px">
											<LinearProgress variant="determinate" value={axis*50+50} sx={{width: "100%"}} />
										</Box>
										<Grid container spacing={1}>
											<Grid item xs={10}>
												<FormControl fullWidth>
													<InputLabel>Axis {index} action</InputLabel>
													<Select label={`Axis ${index} action`}>
														
													</Select>
												</FormControl>
											</Grid>
											<Grid item xs={2}>
												<TextField InputProps={{inputProps: {min: 0, max: 1, step: 0.1}}} type="number" label="Axis Deadzone" /> {/* TODO: error if textfield is NaN or is not between 0 and 1 */}
											</Grid>
										</Grid>
									</Grid>
								))}
							</Grid>

						</Box>			
					}
				</DialogContent>
				<DialogActions>
					<Button onClick={props.onClose}>Close</Button>
					<Button onClick={() => props.onClose()}>Save</Button>
				</DialogActions>
			</Dialog>
		</Box>
	);
}