import { Divider, FormControl, Select, MenuItem, InputLabel, TextField, Box, Grid } from "@mui/material";
import AxisBar from "./AxisBar";
import ButtonLabel from "./ButtonLabel";
import { useAtom } from "jotai";
import { Mappings } from "../../api/Atoms";

export default function ControllerMapping(props: {controller: number, buttons: number, axes: number}) {
    const [mappings, setMappings] = useAtom(Mappings);

    Array(props.buttons).map((_, index) => {
        console.log(index);
    });

    return (
        <Box>
            <Divider sx={{marginBottom: "8px"}}>Buttons</Divider>
            <Grid container spacing={1}>
                {Array.from(Array(props.buttons).keys()).map((_, index) => (
                    <Grid item xs={3}>
                        <FormControl fullWidth>
                            <ButtonLabel controller={navigator.getGamepads()[props.controller]} index={index} />
                            <Select defaultValue="None" label={`Button ${index}`} sx={{width: "100%"}} onChange={(e) => {const tmp = mappings; tmp[1][index] = e.target.value as string; setMappings(tmp);}}>
                                <MenuItem value="None">None</MenuItem>
                                <MenuItem value={"openClaw"}>Open Claw</MenuItem>
                                <MenuItem value={"closeClaw"}>Close Claw</MenuItem>
                            </Select>
                        </FormControl>
                    </Grid>
                ))}
            </Grid>
            <Divider sx={{marginY: "8px"}}>Axes</Divider>
                <Grid container spacing={1}>
                    {navigator.getGamepads()[props.controller]?.axes.map((_, index) => (
                        <Grid item xs={6}>
                            <AxisBar controllerIndex={navigator.getGamepads()[props.controller]?.index ?? -1} index={index} />
                            <Grid container spacing={1}>
                                <Grid item xs={9.25}>
                                    <FormControl fullWidth>
                                        <InputLabel>Axis {index} action</InputLabel>
                                        <Select label={`Axis ${index} action`}>
                                            
                                        </Select>
                                    </FormControl>
                                </Grid>
                                <Grid item xs={2.75}>
                                    <TextField InputProps={{inputProps: {min: 0, max: 1, step: 0.1}}} type="number" label="Deadzone" /> {/* TODO: error if textfield is NaN or is not between 0 and 1 */}
                                </Grid>
                            </Grid>
                        </Grid>
                    ))}
        </Grid>
    </Box>
    );
}