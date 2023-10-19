import { Box, FormControl, InputLabel, MenuItem, Select } from '@mui/material';
import { useAtom } from 'jotai';
import { currentControllerAtom } from '../atoms/CurrentController';

function ControllerTab() {
    const [currentController, setCurrentController] = useAtom(currentControllerAtom)

    return(
        <Box>
            <Box display="flex" justifyContent="center" alignItems="center">
                <FormControl sx={{width: "30%"}}>
                    {/* Currently just basic controller select option, becomes red if there is no controller selected */}
                    {/* This also sends warnings to the browser every time it loads, nothings broken it's becuase the default option is -1 which doesnt exist */}
                    <InputLabel id="controller-select-label">Controller</InputLabel>
                    <Select labelId="controller-select-label" id="controller-select" value={currentController} label="Controller" onChange={setCurrentController} error={(currentController === -1)}>
                        {navigator.getGamepads().length > 0 &&
                            navigator.getGamepads().map((gamepad) => {
                                //Add menu item for every controller
                                return <MenuItem value={gamepad.index}>{gamepad.id}</MenuItem>;
                            })
                        }
                    </Select>
                </FormControl>
            </Box>
        </Box>
    )
}

export default ControllerTab;