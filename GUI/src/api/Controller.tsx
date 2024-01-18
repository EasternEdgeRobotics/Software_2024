import { useAtom } from "jotai";
import React from "react";
import { CurrentController } from "./Atoms";

export default function InitControllers() {
    const [, forceUpdate] = React.useReducer(x => x + 1, 0);
    const [currentController, setCurrentController] = useAtom(CurrentController);

    /* set controller that was just plugged in as the default controller if none selected */
    window.addEventListener("gamepadconnected", (e) => {
        if (currentController === -1) {
            setCurrentController(e.gamepad.index);
            forceUpdate();
        }
    });

    /* if disconnected, try to set the next found controller, if no controller can be found set the current controller as -1 (none) */
    window.addEventListener("gamepaddisconnected", (e) => {
        if (e.gamepad.index === currentController) {
            if (navigator.getGamepads.length > 1) {
                for (const controller of navigator.getGamepads()) {
                    if (controller?.connected && controller !== e.gamepad) {
                        setCurrentController(controller?.index);
                        break;
                    }
                }
            } else setCurrentController(-1);
        }
        forceUpdate();
    });

    return (null);
}