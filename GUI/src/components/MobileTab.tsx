import React from "react";
import { Button, Box } from "@mui/material";
import { useAtom } from 'jotai';
import { ControllerInput } from '../api/Atoms'; 

//  hook for opening the claw
const useOpenClaw = () => {
    const [, setControllerInput] = useAtom(ControllerInput);

    const openClaw = () => {
        setControllerInput(prevState => {
            const newControllerInput = [...prevState];
            newControllerInput[5] = 1;   // 5 is used for open_claw from the ROS file
            return newControllerInput;
        });
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
    };

    return dimLED;
};

export default function MobileTab() {
    const openClaw = useOpenClaw();
    const closeClaw = useCloseClaw();
    const lightLED = useLightLED();
    const dimLED = useDimLED();

    return (
        <Box>
            <Button onClick={openClaw}>Open Claw</Button>
            <Button onClick={closeClaw}>Close Claw</Button>
            <Button onClick={lightLED}>Light the LED</Button>
            <Button onClick={dimLED}>Dim the LED</Button>
        </Box>
    );
}
