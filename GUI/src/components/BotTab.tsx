import { AlertCircle, CheckCircle2 } from "lucide-react";
import { Grid, Paper, Table, TableBody, TableCell, TableContainer, TableHead, TableRow, Slider, Box, Button} from "@mui/material";
import { useAtom } from "jotai";
import { IsROSConnected, PowerMultipliers } from "../api/Atoms";

export function StatusIndicator(props: {statement: boolean}) {
    if (props.statement) return <CheckCircle2 color="lime" />;
    else return <AlertCircle color="red" />;
}

export function BotTab() {
    const [isRosConnected] = useAtom(IsROSConnected);
    const [powerMultipliers, setPowerMultipliers] = useAtom(PowerMultipliers);

    const status = [
        {"name": "ROS", "status": isRosConnected}
    ];
    
    return (
        <Box flexGrow={1}>
            <Grid container justifyContent={"center"} spacing={1}>
                {["Power", "Surge", "Sway", "Heave", "Pitch", "Roll", "Yaw"].map((label, index) => {
                        return (
                            <Grid item xs={1} display="flex" justifyContent="center" alignItems="center" flexWrap="wrap" height="300px">
                                <Slider orientation="vertical" valueLabelDisplay="auto" step={5} defaultValue={0} onChange={(_,value) => setPowerMultipliers(powerMultipliers.map((v, i) => {if (i == index) return value as number; else return v;}))} />
                                <Box flexBasis="100%" height="0" />
                                <h1>{label}: {powerMultipliers[index]}</h1>
                                {index == 2 &&
                                    <Box>
                                        <Box flexBasis="100%" height="0" />
                                        <Button variant="outlined">Load Power Preset</Button>
                                        {/* TODO: make the load/save buttons work */}
                                    </Box>
                                }
                                {index == 4 &&
                                    <Box>
                                        <Box flexBasis="100%" height="0" />
                                        <Button variant="outlined">Save Power Preset</Button>
                                    </Box>
                                }
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
        </Box>
    );
}