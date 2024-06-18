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
const SurgingUP = () => {
    const [, setControllerInput] = useAtom(ControllerInput);

    const surgeup = () => {
        setControllerInput(prevState => {
            const newControllerInput = [...prevState];
            newControllerInput[0] = 100;   // 0 is used for surge from the ROS file
            return newControllerInput;
        });
        
        setTimeout(() => {
            setControllerInput(prevState => {
                const newControllerInput = [...prevState];
                newControllerInput[0] = 0; // Should reset the surge
                return newControllerInput;
            });
        }, 100); // 0.1 seconds
    };

    return surgeup;
};
// Hook for surging down
const SurgingDOWN = () => {
    const [, setControllerInput] = useAtom(ControllerInput);

    const surgedown = () => {
        setControllerInput(prevState => {
            const newControllerInput = [...prevState];
            newControllerInput[0] = -100;   // 0 is used for surge from the ROS file
            return newControllerInput;
        });
        
        setTimeout(() => {
            setControllerInput(prevState => {
                const newControllerInput = [...prevState];
                newControllerInput[0] = 0; // Should reset the surge
                return newControllerInput;
            });
        }, 100); // 0.1 seconds
    };

    return surgedown;
};


export default function MobileTab() {
    const openClaw = useOpenClaw();
    const closeClaw = useCloseClaw();
    const lightLED = useLightLED();
    const dimLED = useDimLED();
    const surgeup = SurgingUP();
    const surgedown = SurgingDOWN();

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
            <Button onClick={surgeup}>Surge UP</Button>
            <Button onClick={surgedown}>Surge DOWN</Button>
        </Box>
    </Box>
    );
}
