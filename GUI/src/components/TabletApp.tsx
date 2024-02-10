import {Box, Button, Checkbox, FormGroup, FormControlLabel, Grid, rgbToHex} from "@mui/material";
import { useState } from "react";
import React from "react";
import { useAtom } from "jotai";
import { grey } from "@mui/material/colors";

export default function TabletApp() {

    return (
        <FormGroup>
            <Grid>    
                <FormControlLabel control = {<Checkbox />} label = "TASK 1: OOI: Coastal Pioneer Array" />
            </Grid>
            <Grid>
                <Checkbox  sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> 
                <FormControlLabel control = {<Checkbox />} label = "1.1 Release the multi-function node" /> 
            </Grid>

            <Grid>
                <FormControlLabel control = {<Checkbox />} label = "TASK 2: SMART Cables for Ocean Observing" />
            </Grid>
            <Grid>
                <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} />
                <FormControlLabel control = {<Checkbox />} label = "2.1 Deploy SMART Cable" />
            </Grid>

            <Grid>
                <FormControlLabel control = {<Checkbox />} label = "TASK 3: From the Red Sea to Tenesse" />
            </Grid>
            <Grid>
                <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} />
                <FormControlLabel control = {<Checkbox />} label = "3.1 Probiotics 2" />
            </Grid>
        </FormGroup>
    );
}