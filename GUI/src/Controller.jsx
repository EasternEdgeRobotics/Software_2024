/* imma keep it real there is a very likely change this entire thing needs to be rewritten it sucks i hate it */

import { FormControl, InputLabel, Select, Box, MenuItem, Grid, Button, Divider } from "@mui/material";
import React from "react";

/* TODO add event listener for controller connected and disconnected */

function Controller() {
    const [selectedController, setSelectedController] = React.useState(-1);
    const handleChange = (event) => {setSelectedController(event.target.value);}

    const [_, forceUpdate] = React.useReducer((x) => x + 1, 0);

    const update = () => {
        if (navigator.getGamepads()[selectedController] === null) setSelectedController(-1);
        forceUpdate();
    }

    const [controllerSelect] = React.useState({});
    const RefreshControllers = () => {
    }

    return(
        <Box>
            <Box display="flex" justifyContent="center" alignItems="center" flexWrap="wrap">
                <FormControl sx={{minWidth: "30%"}} error={(selectedController === -1)}>
                    <InputLabel id="controller-select-label">Controller</InputLabel>
                    <Select
                    labelId = "controller-select-label"
                    label = "Controller"
                    value = {selectedController}
                    id = "controller-select"
                    onChange={handleChange}>
                        {navigator.getGamepads().length > 0 &&
                            navigator.getGamepads().map((controller, index) => {
                                if (controller !== null) return <MenuItem value={index} id="controller-select-item">{controller.id} (id: {index})</MenuItem>
                            })
                        }
                    </Select>
                </FormControl>
            </Box>
            <br/>
            <Box display="flex" justifyContent="center" alignItems="center">
                <Button variant="contained" onClick={update}>Refresh Controllers</Button>
            </Box>
            <br/>
            <Divider>Configs</Divider>
            {/* Saving and loading configs/presets will go here */}
            <br/>
            <Divider>Bindings</Divider>
            {/* The actual control bindings for the controller will go here */}
        </Box>
    )
}

export default Controller;