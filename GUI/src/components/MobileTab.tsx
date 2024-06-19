import React from "react";
import { Button, Box, Grid, Paper, Table, TableBody, TableCell, TableContainer, TableHead, TableRow } from "@mui/material";
import { useAtom } from 'jotai';
import { ControllerInput, ADCArray, TemperatureArray, IMUARRAY } from '../api/Atoms'; 

//  hook for opening the claw
const useOpenClaw = () => {
    const [, setControllerInput] = useAtom(ControllerInput);

    const openClaw = () => {
        setControllerInput(prevState => {
            const newControllerInput = [...prevState];
            newControllerInput[5] = 1;   // 5 is used for open_claw from the ROS file
            return newControllerInput;
        });
        // To reset the values this will input 0 to the controller input after 1 second to reset from the button click.
        setTimeout(() => {
            setControllerInput(prevState => {
                const newControllerInput = [...prevState];
                newControllerInput[5] = 0; // Should reset the claw
                return newControllerInput;
            });
        }, 100); // 0.1 seconds
    };

    return openClaw;
};

// hook for closing the claw
const useCloseClaw = () => {
    const [, setControllerInput] = useAtom(ControllerInput);

    const closeClaw = () => {
        setControllerInput(prevState => {
            const newControllerInput = [...prevState];
            newControllerInput[6] = 1;  // 6 is used for close_claw from the ROS file
            return newControllerInput;
        });

        setTimeout(() => {
            setControllerInput(prevState => {
                const newControllerInput = [...prevState];
                newControllerInput[6] = 0; // Should reset the claw
                return newControllerInput;
            });
        }, 100); // 0.1 seconds
    };

    return closeClaw;
};

// hook for lighting the LED
const useLightLED = () => {
    const [, setControllerInput] = useAtom(ControllerInput);

    const lightLED = () => {
        setControllerInput(prevState => {
            const newControllerInput = [...prevState];
            newControllerInput[11] = 1; //11 is used for brighten_led from ROS file  
            return newControllerInput;
        });

        setTimeout(() => {
            setControllerInput(prevState => {
                const newControllerInput = [...prevState];
                newControllerInput[11] = 0; // Should reset the LED
                return newControllerInput;
            });
        }, 100); // 0.1 seconds
    };

    return lightLED;
};

// hook for dimming the LED
const useDimLED = () => {
    const [, setControllerInput] = useAtom(ControllerInput);

    const dimLED = () => {
        setControllerInput(prevState => {
            const newControllerInput = [...prevState];
            newControllerInput[12] = 1; //12 is used for dim_led from ROS file
            return newControllerInput;
        });

        setTimeout(() => {
            setControllerInput(prevState => {
                const newControllerInput = [...prevState];
                newControllerInput[12] = 0; // Should reset the LED
                return newControllerInput;
            });
        }, 100); // 0.1 seconds
    };

    return dimLED;
};

// Hook for surging up
const useSurgeUp = () => {
    const [, setControllerInput] = useAtom(ControllerInput);

    const surgeup = () => {
        setControllerInput(prevState => {
            const newControllerInput = [...prevState];
            newControllerInput[0] = 100;   // 0 is used for surge from the ROS file
            return newControllerInput;
        });
    };

    const stopsurgeup = () => {
        setControllerInput(prevState => {
            const newControllerInput = [...prevState];
            newControllerInput[0] = 0; // Should reset the surge
            return newControllerInput;
        });
    };

    return { surgeup, stopsurgeup };
};

// Hook for surging down
const useSurgeDown = () => {
    const [, setControllerInput] = useAtom(ControllerInput);

    const surgedown = () => {
        setControllerInput(prevState => {
            const newControllerInput = [...prevState];
            newControllerInput[0] = -100;   // 0 is used for surge from the ROS file
            return newControllerInput;
        });
    };

    const stopsurgedown = () => {
        setControllerInput(prevState => {
            const newControllerInput = [...prevState];
            newControllerInput[0] = 0; // Should reset the surge
            return newControllerInput;
        });
    };

    return { surgedown, stopsurgedown };
};


export default function MobileTab() {
    const openClaw = useOpenClaw();
    const closeClaw = useCloseClaw();
    const lightLED = useLightLED();
    const dimLED = useDimLED();
    const { surgeup: surgeUpClick, stopsurgeup: surgeUpRelease } = useSurgeUp();
    const { surgedown: surgeDownClick, stopsurgedown: surgeDownRelease } = useSurgeDown();

    const [read_ADCArray] = useAtom(ADCArray);
    const [read_TemperatureArray] = useAtom(TemperatureArray);
    const [read_IMUARRAY] = useAtom(IMUARRAY);

    const Arrays = [
        {"name": "ADC", "status": JSON.stringify(read_ADCArray)},
        {"name":"TEMPERATURE", "status": JSON.stringify(read_TemperatureArray)},
        {"name": "IMU", "status": JSON.stringify(read_IMUARRAY)}
    ];

    return (
        <Box>
        <Box mb={8}>
            <Button onClick={openClaw}>Open Claw</Button>
            <Button onClick={closeClaw}>Close Claw</Button>
        </Box>
        <Box mb={8}>
            <Button onClick={lightLED}>Light the LED</Button>
            <Button onClick={dimLED}>Dim the LED</Button>
        </Box>
        <Box>
        <Button onMouseDown={surgeUpClick} onMouseUp={surgeUpRelease}>Surge UP</Button>
        <Button onMouseDown={surgeDownClick} onMouseUp={surgeDownRelease}>Surge DOWN</Button>
        </Box>
        
        <Grid container justifyContent={"center"} spacing={200} sx={{ marginTop: "40px", transform: "scale(0.7)" }}>
                <Grid container justifyContent={"right"} spacing={17}>
                    <Grid item xs={3}>
                        <Grid container justifyContent={"center"} rowSpacing={3}>
                            <Grid item xs={50}>
                                <TableContainer component={Paper}>
                                    <Table>
                                        <TableHead>
                                            <TableRow>
                                                <TableCell align="center">Items</TableCell>
                                                <TableCell align="center">Values</TableCell>
                                            </TableRow>
                                        </TableHead>
                                        <TableBody>
                                            {Arrays.map((data) => (
                                                <TableRow key={data.name} sx={{ "&:last-child td, &:last-child th": { border: 0 } }}>
                                                    <TableCell align="center">{data.name}</TableCell>
                                                    <TableCell align="center">{data.status}</TableCell>
                                                </TableRow>
                                            ))}
                                        </TableBody>
                                    </Table>
                                </TableContainer>
                            </Grid>
                        </Grid>
                    </Grid>
                </Grid>
            </Grid>
        </Box>
    );
}

