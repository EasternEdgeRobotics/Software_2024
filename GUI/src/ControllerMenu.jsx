import { Box, Button, FormControl, InputLabel, MenuItem, Select } from '@mui/material';
import { useAtom } from 'jotai';
import React from 'react';
import { CurrentControllerAtom } from './Atoms';

function Controller() {
    const [currentController, setCurrentController] = useAtom(CurrentControllerAtom);

    return(
        <Box>
            <Box display="flex" justifyContent="center" alignItems="center">
                <FormControl sx={{width: "30%"}}>
                    <InputLabel id="controller-select-label">Controller</InputLabel>
                    <Select labelId="controller-select-label" id="controller-select" value={currentController} label="Controller" onChange={setCurrentController} error={(currentController === -1)}>
                        {navigator.getGamepads().length > 0 &&
                            navigator.getGamepads().map((gamepad) => {
                                return <MenuItem value={gamepad.index}>{gamepad.id}</MenuItem>;
                            })
                        }
                    </Select>
                </FormControl>
            </Box>
        </Box>
    )
}

export default Controller;