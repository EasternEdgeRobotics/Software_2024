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
                                <Slider orientation="vertical" valueLabelDisplay="auto" step={5} defaultValue={powerMultipliers[index]} onChange={(_,value) => setPowerMultipliers(powerMultipliers.map((v, i) => {if (i == index) return value as number; else return v;}))} />
                                <Box flexBasis="100%" height="0" />
                                <h2>{label}: {powerMultipliers[index]}</h2>
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
            <Grid container justifyContent={"center"} spacing={1} sx={{marginTop: "64px"}}>
                <Grid item xs={3.5}>
                    <Button variant="contained" sx={{width: "100%", height: "3rem"}}>Load Power Preset</Button>
                </Grid>
                <Grid item xs={3.5}>
                    <Button variant="contained" sx={{width: "100%", height: "3rem"}}>Save Power Preset</Button>
                </Grid>
                <Grid item xs={3} />
            </Grid>
        </Box>
    );
}