import { AlertCircle, CheckCircle2 } from "lucide-react";
import {
    Grid,
    Paper,
    Table,
    TableBody,
    TableCell,
    TableContainer,
    TableHead,
    TableRow,
    Slider,
    Box,
    Switch,
} from "@mui/material";
import { useAtom } from "jotai";
import {
    IsROSConnected,
    ThrusterMultipliers,
    ProfilesList,
    CurrentProfile,
    Mappings,
    ControllerInput,
    PilotActions,
    ADCArray,
    TemperatureArray,
    KeyboardInputMap
} from "../api/Atoms";
import { useState, useEffect } from "react";

export function StatusIndicator(props: { statement: boolean }) {
    if (props.statement) return <CheckCircle2 color="lime" />;
    else return <AlertCircle color="red" />;
}

export function BotTab() {
    const [isRosConnected] = useAtom(IsROSConnected);

    const [thrusterMultipliers, setThrusterMultipliers] =
        useAtom(ThrusterMultipliers);
    const [pilotActions] = useAtom(PilotActions);
    const [keyboardInputMap] = useAtom(KeyboardInputMap);

    const [profilesList] = useAtom(ProfilesList);
    const [currentProfile] = useAtom(CurrentProfile);
    const [mappings] = useAtom(Mappings);
    const [, setControllerInput] = useAtom(ControllerInput);
    const [controller1Detected, setController1Detected] = useState(false);
    const [controller2Detected, setController2Detected] = useState(false);
    const [keyboardMode, setKeyboardMode] = useState(false);

    const [controller1Name, setController1Name] = useState("Not Assigned");
    const [controller2Name, setController2Name] = useState("Not Assigned");

    // ADC and TEMP data
    const [read_ADCArray] = useAtom(ADCArray);
    const [read_TemperatureArray] = useAtom(TemperatureArray);

    let initialPageLoad = true;

    let pressed_key = "";

    const status = [
        { name: "ROS", status: isRosConnected },
        { name: "Controller 1 Detected", status: controller1Detected },
        { name: "Controller 2 Detected", status: controller2Detected },
    ];

    const Arrays = [
        { name: "ADC", status: JSON.stringify(read_ADCArray) },
        { name: "TEMPERATURE", status: JSON.stringify(read_TemperatureArray) },
    ];

    const [, reloadComponent] = useState<number>(0);

    // All inputs will be in the format of a list, where the first element is input value (1 for button and between (-1,1) for axis)
    // and the second element is the action

    // Likely one of the most important functions in the software package
    // This function listens to the controller input and updates the ControllerInput global state. When this state is changed,
    // ROS.tsx recognizes this and publishes the input to the device running the rosbridge server (RPi4 inside the enclosure)
    const input_listener = () => {
        let controller1 = "null";
        let controller2 = "null";

        let controller1Index = -1;
        let controller2Index = -1;

        let controller1Set = false;

        // Find out the controller names, stored in both a normal javascript variable for this function and as a react state to be displayed in GUI
        for (let i = 0; i < profilesList.length; i++) {
            if (currentProfile == profilesList[i].name) {
                controller1 = profilesList[i].controller1;
                setController1Name(controller1);

                controller2 =
                    profilesList[i].controller2 != null
                        ? profilesList[i].controller2
                        : "null";
                setController2Name(controller2);
            }
        }

        // Find out the controller index in navigator
        for (
            let controllerIndex = 0;
            controllerIndex < navigator.getGamepads().length;
            controllerIndex++
        ) {
            if (
                navigator.getGamepads()[controllerIndex]?.id == controller1 &&
                !controller1Set
            ) {
                controller1Set = true; // This is in case controller 1 and controller 2 have the same name
                controller1Index = controllerIndex;
                setController1Detected(true);
            } else if (navigator.getGamepads()[controllerIndex]?.id == controller2) {
                controller2Index = controllerIndex;
                setController2Detected(true);
            }
        }

        const controllerIndexes = [controller1Index, controller2Index];
        const controllerInput: (number | undefined)[] = [
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        ]; // Current controller input from pilot. Must match length of ControllerInput defined in Atmos.tsx

        if (controllerIndexes[0] == -1) {
            setController1Detected(false);
        }
        if (controllerIndexes[1] == -1) {
            setController2Detected(false);
        }

        // Iterate through controller 1 and controller 2
        for (let i = 0; i < 2; i++) {
            if (Object.keys(mappings[i]).length == 0) {
                // This indicates that no mappings have been loaded for this controller.
                if (initialPageLoad) {
                    console.log(
                        "No mappings have been loaded. Go to the settings and load a profile."
                    );
                    initialPageLoad = false;
                }
                continue;
            }

            // The below for loop is quick, so it will detect a button press on any button even if it was for a short time

            const gamepadInstance = navigator.getGamepads()[controllerIndexes[i]];

            for (let j = 0; j < (gamepadInstance?.buttons.length as number); j++) {
                if (gamepadInstance?.buttons[j].pressed) {
                    //Button press detected
                    if (mappings[i]["buttons"][j] != "None") {
                        for (let k = 0; k < pilotActions.length; k++) {
                            if (pilotActions[k] == mappings[i]["buttons"][j]) {
                                controllerInput[k - 1] = 1;
                                break;
                            }
                        }
                    }
                }
            }
            for (let j = 0; j < (gamepadInstance?.axes.length as number); j++) {
                const deadzone =
                    Number(mappings[i]["deadzones"][j]) > 0.05
                        ? Number(mappings[i]["deadzones"][j])
                        : 0.05;
                if (Math.abs(gamepadInstance?.axes[j] as number) >= deadzone) {
                    // The axis has been moved beyond it's "deadzone", which means that this is actual pilot input and not controller drift
                    if (mappings[i]["axes"][j] != "None") {
                        for (let k = 0; k < pilotActions.length; k++) {
                            if (pilotActions[k] == mappings[i]["axes"][j]) {
                                controllerInput[k - 1] = Math.round(
                                    j % 2 == 0
                                        ? (gamepadInstance?.axes[j] as number) * 100
                                        : (gamepadInstance?.axes[j] as number) * -100
                                );
                                break;
                            }
                        }
                    }
                }
            }
        }

        if (keyboardMode && pressed_key) {
            for (let i = 0; i < keyboardInputMap.length; i++) {
                if (keyboardInputMap[i][0] == pressed_key) {

                    for (let j = 0; j < pilotActions.length; j++) {
                        if (pilotActions[j] == keyboardInputMap[i][1]) {
                            controllerInput[j - 1] = keyboardInputMap[i][2] as number;
                            break;
                        }
                    }

                    break;
                }
            }
        }
        
        setControllerInput(controllerInput);
    };

    

    useEffect(() => {

        if (keyboardMode){
            // Add keyboard input listeners for keyboard mode
            window.addEventListener(
                "keydown",
                function (e) {
                    pressed_key = e.key;
                },
                false
            );
            window.addEventListener(
                "keyup",
                function (e) {
                    pressed_key = "";
                },
                false
            );
        } else {
            // Remove keyboard input listeners for keyboard mode
            window.removeEventListener(
                "keydown",
                function (e) {
                    pressed_key = e.key;
                },
                false
            );
            window.removeEventListener(
                "keyup",
                function (e) {
                    pressed_key = "";
                },
                false
            );
        }
        document.addEventListener("keypress", (event) => {
            switch (event.key) {
                case "0":
                    setThrusterMultipliers([0, 0, 0, 0, 0, 0]);
                    break;
                case "1":
                    setThrusterMultipliers([100, 100, 100, 100, 100, 70]);
                    break;
                case "2":
                    setThrusterMultipliers([100, 0, 0, 100, 0, 0]);
                    break;
                case "3":
                    setThrusterMultipliers([0, 100, 0, 0, 0, 0]);
                    break;
                default:
                    break;
            }
        });

        // Constantly run the input listener
        const interval = setInterval(() => {
            reloadComponent(Math.random());
            input_listener();
        }, 100); // 100 ms or 10Hz
        return () => clearInterval(interval); // Stop listening for input when we click off the BotTab
    }, [keyboardMode]);

    return (
        <Box flexGrow={1}>
            <Grid container justifyContent={"center"} spacing={1}>
                {["Power", "Surge", "Sway", "Heave", "Pitch", "Yaw"].map(
                    (label, index) => {
                        return (
                            <Grid
                                item
                                xs={1}
                                key={index}
                                display="flex"
                                justifyContent="center"
                                alignItems="center"
                                flexWrap="wrap"
                                height="300px"
                            >
                                <Slider orientation="vertical" valueLabelDisplay="auto" step={5} value={thrusterMultipliers[index]} onChange={(_,value) => setThrusterMultipliers(thrusterMultipliers.map((v, i) => {if (i == index) return value as number; else return v;}))} />
                                <Box flexBasis="100%" height="0" />
                                <h2>
                                    {label}: {thrusterMultipliers[index]}
                                </h2>
                            </Grid>
                        );
                    }
                )}

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
                                                <TableRow
                                                    key={data.name}
                                                    sx={{
                                                        "&:last-child td, &:last-child th": { border: 0 },
                                                    }}
                                                >
                                                    <TableCell align="center">{data.name}</TableCell>
                                                    <TableCell align="center">
                                                        <StatusIndicator statement={data.status} />
                                                    </TableCell>
                                                </TableRow>
                                            );
                                        })}
                                    </TableBody>
                                </Table>
                            </TableContainer>
                        </Grid>
                        <Grid item xs={3} />
                    </Grid>
                    <Grid container justifyContent={"top"} sx={{ marginTop: "-18px" }}>
                        <Grid item xs="auto">
                            Current Profile: {currentProfile}
                        </Grid>

                        <Grid item xs="auto">
                            Controller 1: {controller1Name}
                        </Grid>

                        <Grid item xs="auto">
                            Controller 2: {controller2Name}
                        </Grid>

                        <Grid item xs={12}>
                            Keyboard Mode {keyboardMode ? "on" : "off"}
                            <Switch
                                onChange={(event: React.ChangeEvent<HTMLInputElement>) => { setKeyboardMode(event.target.checked); }}
                            />
                        </Grid>
                    </Grid>
                </Grid>
            </Grid>

            <Box display="flex" justifyContent="center" width="100%" paddingTop="16px" paddingX="10%" marginTop={"64px"}>
                <TableContainer component={Paper}>
                    <Table>
                        <TableHead>
                            <TableRow>
                                <TableCell align="center">Items</TableCell>
                                <TableCell align="center">Values</TableCell>
                            </TableRow>
                        </TableHead>
                        <TableBody>
                        {Arrays.map((data) => {
                            return (
                                <TableRow key={data.name} sx={{ "&:last-child td, &:last-child th": { border: 0 } }}>
                                    <TableCell align="center">{data.name}</TableCell>
                                    <TableCell align="center">{data.status}</TableCell>
                                </TableRow>
                            );
                        })}
                        </TableBody>
                    </Table>
                </TableContainer>
            </Box>

            <Box display="flex" justifyContent="center" width="100%" paddingTop="16px" paddingX="10%">
                <p>0 - all off, 1 - all high (70% yaw), 2 - all vertical, 3 - all surge</p>
            </Box>
        </Box>
    );
}
