import { useAtom } from "jotai";
import { CurrentController } from "../api/Atoms";
import { Box, Button, Divider, FormControl, InputLabel, MenuItem, Select, TextField } from "@mui/material";

export default function ControllerTab() {
    const [currentController, setCurrentController] = useAtom(CurrentController);

    return (
        <Box>
            <Box display="flex" justifyContent="center" marginBottom="16px">
            <FormControl sx={{width: "40%"}}>
                    {/* Currently just basic controller select option, becomes red if there is no controller selected */}
                    {/* This also sends warnings to the browser every time it loads, nothings broken it's becuase the default option is -1 which doesnt exist */}
                    <InputLabel id="controller-select-label">Controller</InputLabel>
                    <Select labelId="controller-select-label" id="controller-select" value={currentController} label="Controller" onChange={(e) => setCurrentController(e.target.value as number)} error={(currentController === -1)}>
                        {navigator.getGamepads().length > 0 &&
                            navigator.getGamepads().map((gamepad) => {
                                //Add menu item for every controller
                                if (gamepad !== null){
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
            <Divider>Profiles</Divider><br/>
            <Box display="flex" justifyContent="center" marginBottom="16px">
                <FormControl sx={{width: "33%"}}>
                    {/* Currently just basic controller select option, becomes red if there is no controller selected */}
                    {/* This also sends warnings to the browser every time it loads, nothings broken it's becuase the default option is -1 which doesnt exist */}
                    <InputLabel id="controller-profile-select-label">Select Profile</InputLabel>
                    <Select labelId="controller-profile-select-label" id="controller-profile-select" value={currentController} label="Select Profile">
                        {/* get each profile from config */}
                    </Select>
                </FormControl>
                <Button variant="outlined" sx={{marginLeft: "8px", width: "7%"}}>Load Profile</Button>
            </Box>
            <Box display="flex" justifyContent="center" marginBottom="16px"> {/* error if profile name exists maybe? */}
                <TextField label="Profile Name" variant="outlined" sx={{width: "33%"}} />
                <Button variant="outlined" sx={{marginLeft: "8px", width: "7%"}}>Save Profile</Button>
            </Box>
            <Box display="flex" justifyContent="center" marginBottom="16px">
                <Button sx={{width: "40%", height: "56px"}} variant="outlined">Open Profile Editor</Button>
            </Box>
        </Box>
    );
}