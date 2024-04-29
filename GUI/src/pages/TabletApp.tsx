import {Box, Button, Checkbox, FormGroup, FormControlLabel, Grid, Typography, AppBar, Stack} from "@mui/material";
import { useState } from "react";
import React from "react";
import { useAtom } from "jotai";
import { grey, green, yellow, red, blue} from "@mui/material/colors";
import { Task } from "../types/Task";
import taskJSON from "./tasks.json";


/* 
The purpose of the TabletApp component is to provide the deck chief with a GUI which is to be used on a tablet that:

    1. Provides a checklist of tasks and subtasks that can be ticked/unticked 

    2. Provides a main timer, which can be started, paused and reset, permanently displayed as bar at the top of the screen

    TIMERS TOO FAST!

    3. Provides secondary timers, one for each task, which can be started paused and reset independently of the main timers and other task-specific timers

Additional features include:

    1. Should any timer reach 15 minutes, all timers "break". That is, all timers are paused and are no longer interactive

    2. A settings menu located underneath the checklist for debugging purposes
*/

export default function TabletApp() {
    // Main-Timer: Variables
    let milliseconds = 0;
    let displayedMilliseconds = "";
    let seconds = 0;
    let displayedSeconds = "";
    let minutes = 0;
    let displayedMinutes = "";
    let displayedTime = "";
    let timerOn = 0;
    const timerPaused = 0; // Necessary variable? Unless it's value is changed at any point, it must be kept to const
    let timeOver = 0;

    // Main Timer: Function
    function repeat() {       
        setTimeout(function() {
            if (timerOn == 1) {
                if (timerPaused == 0) {
                    if (timeOver == 0) {
                        milliseconds = milliseconds + 1;

                        // If millesconds and/or seconds have reached their limits, increment/reset variables accordingly
                        if (milliseconds >= 100) {
                            seconds = seconds + 1;
                            milliseconds = 0;
                        }
                        if (seconds >= 60) {
                            minutes = minutes + 1;
                            seconds = 0;
                        }

                        // Set displayedMilliseconds, displayedSeconds, and displayedMinutes based upon if their corresponding non-displayed values are single-digit or double-digit
                        if (milliseconds < 10) {
                            displayedMilliseconds = ":0" + milliseconds;
                        }
                        else {
                            displayedMilliseconds = ":" + milliseconds;
                        }
                        if (seconds < 10) {
                            displayedSeconds = ":0" + seconds;
                        }
                        else {
                            displayedSeconds = ":" + seconds;
                        }
                        if (minutes < 10) {
                            displayedMinutes = "0" + minutes; 
                        }
                        else {
                            displayedMinutes = "" + minutes;
                        }
                             
                        // If 15 minutes have passed within the Main Timer, break all timers
                        if (minutes == 15) {
                            timeOver = 1;
                        }

                        // Update and display displayedTime
                        displayedTime = displayedMinutes + displayedSeconds + displayedMilliseconds;
                        document.getElementById("timerDisplay")!.innerHTML = displayedTime;

                        // Repeat the Main Timer's function
                        repeat();
                    }
                }
            }
        });
    }


    // Task 1 Timer: Variables 
    let milliseconds1 = 0;
    let displayedMilliseconds1 = "";
    let seconds1 = 0;
    let displayedSeconds1 = "";
    let minutes1 = 0;
    let displayedMinutes1 = "";
    let timerOn1 = false;
    let displayedTime1 = "";

    // Task 1 Timer: Function
    function repeat1() {
        setTimeout(function() {
            if (timerOn1 == true) { 
                if (timeOver == 0) {
                    milliseconds1 = milliseconds1 + 1;

                    // If millesconds1 and/or seconds1 have reached their limits, increment/reset variables accordingly
                    if(milliseconds1 >= 100) {
                        seconds1 = seconds1 + 1;
                        milliseconds1 = 0;
                    }
                    if(seconds1 >= 60) {
                        minutes1 = minutes1 + 1;
                        seconds1 = 0;
                    }

                    // Set displayedMilliseconds1, displayedSeconds1, and displayedMinutes1 based upon if their corresponding non-displayed values are single-digit or double-digit
                    if (milliseconds1 < 10) {
                        displayedMilliseconds1 = ":0" + milliseconds1;
                    }
                    else {
                        displayedMilliseconds1 = ":" + milliseconds1;
                    }
                    if (seconds1 < 10) {
                        displayedSeconds1 = ":0" + seconds1;
                    }
                    else {
                        displayedSeconds1 = ":" + seconds1;
                    }
                    if (minutes1 < 10) {
                        displayedMinutes1 = "0" + minutes1;
                    }
                    else {
                        displayedMinutes1 = "" + minutes1;
                    }

                    // If 15 minutes have passed within the Task 1 Timer, break all timers
                    if(minutes1 == 15) {
                        timeOver = 1;
                    }

                    // Update and display displayedTime1
                    displayedTime1 = displayedMinutes1 + displayedSeconds1 + displayedMilliseconds1;
                    document.getElementById("timerDisplay1")!.innerHTML = displayedTime1;

                    // Repeat the Task 1 Timer's function
                    repeat1();
                }
            }
        });
    }

    // Task 2 Timer: Variables 
    let milliseconds2 = 0;
    let displayedMilliseconds2 = "";
    let seconds2 = 0;
    let displayedSeconds2 = "";
    let minutes2 = 0;
    let displayedMinutes2 = "";
    let timerOn2 = false;
    let displayedTime2 = "";

    // Task 2 Timer: Function
    function repeat2() {
        setTimeout(function() {
            if (timerOn2 == true) {
                if (timeOver == 0) {
                    milliseconds2 = milliseconds2 + 1;

                    // If millesconds2 and/or seconds2 have reached their limits, increment/reset variables accordingly
                    if(milliseconds2 >= 100) {
                        seconds2 = seconds2 + 1;
                        milliseconds2 = 0;
                    }
                    if(seconds2 >= 60) {
                        minutes2 = minutes2 + 1;
                        seconds2 = 0;
                    }

                    // Set displayedMilliseconds2, displayedSeconds2, and displayedMinutes2 based upon if their corresponding non-displayed values are single-digit or double-digit
                    if (milliseconds2 < 10) {
                        displayedMilliseconds2 = ":0" + milliseconds2;
                    }
                    else {
                        displayedMilliseconds2 = ":" + milliseconds2;
                    }
                    if (seconds2 < 10) {
                        displayedSeconds2 = ":0" + seconds2;
                    }
                    else {
                        displayedSeconds2 = ":" + seconds2;
                    }
                    if (minutes2 < 10) {
                        displayedMinutes2 = "0" + minutes2;
                    }
                    else {
                        displayedMinutes2 = "" + minutes2;
                    }

                    // If 15 minutes have passed wihtin the Task 2 Timer, break all timers
                    if(minutes2 == 15) {
                        timeOver = 1;
                    }

                    // Update and display displayedTime2
                    displayedTime2 = displayedMinutes2 + displayedSeconds2 + displayedMilliseconds2;
                    document.getElementById("timerDisplay2")!.innerHTML = displayedTime2;
                    
                    // Repeat the Task 2 Timer's function
                    repeat2();
                }
            }
        });
    }

    // Task 3 Timer: Variables 
    let milliseconds3 = 0;
    let displayedMilliseconds3 = "";
    let seconds3 = 0;
    let displayedSeconds3 = "";
    let minutes3 = 0;
    let displayedMinutes3 = "";
    let timerOn3 = false;
    let displayedTime3 = "";

    // Task 3 Timer: Function
    function repeat3() {
        setTimeout(function() {
            if (timerOn3 == true) {
                if (timeOver == 0) {
                    milliseconds3 = milliseconds3 + 1;

                    // If millesconds3 and/or seconds3 have reached their limits, increment/reset variables accordingly
                    if(milliseconds3 >= 100) {
                        seconds3 = seconds3 + 1;
                        milliseconds3 = 0;
                    }
                    if(seconds3 >= 60) {
                        minutes3 = minutes3 + 1;
                        seconds3 = 0;
                    }

                    // Set displayedMilliseconds3, displayedSeconds3, and displayedMinutes3 based upon if their corresponding non-displayed values are single-digit or double-digit
                    if (milliseconds3 < 10) {
                        displayedMilliseconds3 = ":0" + milliseconds3;
                    }
                    else {
                        displayedMilliseconds3 = ":" + milliseconds3;
                    }
                    if (seconds3 < 10) {
                        displayedSeconds3 = ":0" + seconds3;
                    }
                    else {
                        displayedSeconds3 = ":" + seconds3;
                    }
                    if (minutes3 < 10) {
                        displayedMinutes3 = "0" + minutes3;
                    }
                    else {
                        displayedMinutes3 = "" + minutes3;
                    }

                    // If 15 minutes have passed within the Task 3 Timer, break all timers
                    if(minutes3 == 15) {
                        timeOver = 1;
                    }

                    // Update and display displayedTime3
                    displayedTime3 = displayedMinutes3 + displayedSeconds3 + displayedMilliseconds3;
                    document.getElementById("timerDisplay3")!.innerHTML = displayedTime3;

                    // Repeat the Task 3 Timer's function
                    repeat3();
                }
            }
        });
    }

 // Task 4 Timer: Variables 
 let milliseconds4 = 0;
 let displayedMilliseconds4 = "";
 let seconds4 = 0;
 let displayedSeconds4 = "";
 let minutes4 = 0;
 let displayedMinutes4 = "";
 let timerOn4 = false;
 let displayedTime4 = "";

 // Task 4 Timer: Function
 function repeat4() {
     setTimeout(function() {
         if (timerOn4 == true) {
            if (timeOver == 0) {
                milliseconds4 = milliseconds4 + 1;

                // If millesconds4 or seconds4 have reached their limits, increment/reset variables accordingly
                if(milliseconds4 >= 100) {
                    seconds4 = seconds4 + 1;
                    milliseconds4 = 0;
                }
                if(seconds4 >= 60) {
                    minutes4 = minutes4 + 1;
                    seconds4 = 0;
                }

                // Set displayedMilliseconds4, displayedSeconds4, and displayedMinutes4 based upon if their corresponding non-displayed values are single-digit or double-digit
                if (milliseconds4 < 10) {
                    displayedMilliseconds4 = ":0" + milliseconds4;
                }
                else {
                    displayedMilliseconds4 = ":" + milliseconds4;
                }
                if (seconds4 < 10) {
                    displayedSeconds4 = ":0" + seconds4;
                }
                else {
                    displayedSeconds4 = ":" + seconds4;
                }
                if (minutes4 < 10) {
                    displayedMinutes4 = "0" + minutes4;
                }
                else {
                    displayedMinutes4 = "" + minutes4;
                }


                // If 15 minutes have passed within the Task 4 Timer, break all timers
                if(minutes4 == 15) {
                    timeOver = 1;
                }

                 // Update and display displayedTime4
                displayedTime4 = displayedMinutes4 + displayedSeconds4 + displayedMilliseconds4;
                document.getElementById("timerDisplay4")!.innerHTML = displayedTime4;

                // Repeat the Task 4 Timer's function
                repeat4();
            }
         }
     });
 }

    return(
        // Main Timer & Task 1 Timer: Control Layouts
        <FormGroup>              
            {/* Main Timer: Controls */}
            <AppBar sx = {{bgcolor: "grey", textAlign: "center"}}>
                {/* Main Timer: Displayed Time */}
                <Typography component = "div" id = "timerDisplay" fontSize={50}>
                    00:00:00 
                </Typography> 

                {/* Main Timer: Start Button */}
                <Button variant = "contained" sx = {{background: green[400], color: grey[50]}} onClick = {() => {if(timeOver == 0) {timerOn = 1;} repeat();}}>                  
                    <Typography fontSize = {25}> 
                        START 
                    </Typography>
                </Button>
                
                {/* Main Timer: Pause Button */}
                <Button variant = "contained" sx = {{background: yellow[400], color: [grey[50]]}} onClick = {() => {if(timeOver == 0) {timerOn = 0;}}}>                 
                    <Typography fontSize = {25}>
                        PAUSE
                    </Typography>
                </Button>

                {/* Main Timer: Reset Button */}
                <Button variant = "contained" sx = {{background: red[400], color: grey[50]}} onClick = {() => {if(timeOver == 0) {timerOn = 0; milliseconds = 0; seconds = 0; minutes = 0; document.getElementById("timerDisplay")!.innerHTML = "00:00:00";}}}>                    
                    <Typography fontSize = {25}>
                        RESET
                    </Typography>
                </Button>
            </AppBar>
            
            <Grid>    
                <Checkbox  sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox  sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox  sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox  sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox  sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox  sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox  sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox  sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox  sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox  sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox  sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox  sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox  sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox  sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox  sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox  sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox  sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox  sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox  sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox  sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox  sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox  sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox  sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox  sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox  sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox  sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox  sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox  sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox  sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox  sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox  sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox  sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox  sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox  sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox  sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox  sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox  sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox  sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox  sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox  sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox  sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox  sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox  sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox  sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox  sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox  sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox  sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox  sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox  sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox  sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox  sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox  sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox  sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox  sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox  sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox  sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox  sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox  sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox  sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox  sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox  sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox  sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox  sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox  sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox  sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox  sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox  sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox  sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox  sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox  sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox  sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox  sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox  sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox  sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox  sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox  sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox  sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox  sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox  sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox  sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox  sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox  sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox  sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox  sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox  sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox  sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox  sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox  sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox  sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox  sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox  sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox  sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox  sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox  sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox  sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox  sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox  sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox  sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox  sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox  sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox  sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox  sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} />
                
                {/* Task 1 Timer : Controls */}
                <Box sx = {{width: [600], textAlign : "left"}}>
                    <Stack spacing = {2} direction = "row">
                        {/* Task 1 Timer: Start Button */}
                        <Button variant = "outlined" sx = {{borderColor: green[900], color: green[900]}} onClick = {() => {if(timeOver == 0) {timerOn1 = true; repeat1();}}}> 
                            START 
                        </Button>

                        {/* Task 1 Timer: Pause Button*/}
                        <Button variant = "outlined" sx = {{borderColor: yellow[900], color: yellow[900]}} onClick = {() => {if (timeOver == 0) {timerOn1 = false;}}}> 
                            PAUSE 
                        </Button>
                        
                        {/* Task 1 Timer: Displayed Time */}
                        <Box textAlign = "center" sx = {{width: [80]}}>  {/* Include 'border: "2px solid grey"'? */}                            
                            <Typography component = "div" id = "timerDisplay1" fontSize = {20} sx = {{color: grey[400]}}>
                                00:00:00
                            </Typography> 
                        </Box>

                        {/* Task 1 Timer: Reset button */}
                        <Button variant = "outlined" sx ={{borderColor: red[900], color: red[900]}} onClick = {() => {if(timeOver == 0) {timerOn1 = false; milliseconds1 = 0; seconds1 = 0; minutes1 = 0; document.getElementById("timerDisplay1")!.innerHTML = "00:00:00";}}}>
                            RESET
                        </Button>
                    </Stack>
                </Box>
                
                {/* Task 1: Checklist Header */}
                <FormControlLabel control = {<Checkbox />} label = "TASK 1: OOI: Coastal Pioneer Array" />
            </Grid>

            {/* Task 1: Indentation & Checkboxes */}
            {/* <div style={{ height: "20px" }}></div> */}
            {Tasks("TASK 1 : Coastal Pioneer Array", taskJSON.task1.tasks)};
            {/* Task 2 Timer: Control Layout */}
            <Grid>    
                {/* Task 2 Timer : Controls */}
                <Box sx = {{width: [600], textAlign : "left"}}>
                    <Stack spacing = {2} direction = "row">
                        {/* Task 2 Timer: Start Button */}
                        <Button variant = "outlined" sx = {{borderColor: green[900], color: green[900]}} onClick = {() => {if(timeOver == 0) {timerOn2 = true; repeat2();}}}> 
                            START 
                        </Button>

                        {/* Task 2 Timer: Pause Button*/}
                        <Button variant = "outlined" sx = {{borderColor: yellow[900], color: yellow[900]}} onClick = {() => {if (timeOver == 0) {timerOn2 = false;}}}> 
                            PAUSE 
                        </Button>
                        
                        {/* Task 2 Timer: Displayed Time */}
                        <Box textAlign = "center" sx = {{width: [80]}}>  {/* Include 'border: "2px solid grey"'? */}                            
                            <Typography component = "div" id = "timerDisplay2" fontSize = {20} sx = {{color: grey[400]}}>
                                00:00:00
                            </Typography> 
                        </Box>

                        {/* Task 2 Timer: Reset button */}
                        <Button variant = "outlined" sx ={{borderColor: red[900], color: red[900]}} onClick = {() => {if(timeOver == 0) {timerOn2 = false; milliseconds2 = 0; seconds2 = 0; minutes2 = 0; document.getElementById("timerDisplay2")!.innerHTML = "00:00:00";}}}>
                            RESET
                        </Button>
                    </Stack>
                </Box>

                {/* Task 2: Checklist Header */}
                <FormControlLabel control = {<Checkbox />} label = "TASK 2: SMART Cables for Ocean Observing" />
            </Grid>
            {/* Task 2: Indentation & Checkboxes */}
            {Tasks("TASK 2 : Coastal Pioneer Array", taskJSON.task2.tasks)};

            {/* Task 3 Timer: Control Layout */}
            <Grid>    
                {/* Task 3 Timer : Controls */}
                <Box sx = {{width: [600], textAlign : "left"}}>
                    <Stack spacing = {2} direction = "row">
                        {/* Task 3 Timer: Start Button */}
                        <Button variant = "outlined" sx = {{borderColor: green[900], color: green[900]}} onClick = {() => {if(timeOver == 0) {timerOn3 = true; repeat3();}}}> 
                            START 
                        </Button>

                        {/* Task 3 Timer: Pause Button*/}
                        <Button variant = "outlined" sx = {{borderColor: yellow[900], color: yellow[900]}} onClick = {() => {if (timeOver == 0) {timerOn3 = false;}}}> 
                            PAUSE 
                        </Button>
                        
                        {/* Task 3 Timer: Displayed Time */}
                        <Box textAlign = "center" sx = {{width: [80]}}>  {/* Include 'border: "2px solid grey"'? */}                            
                            <Typography component = "div" id = "timerDisplay3" fontSize = {20} sx = {{color: grey[400]}}>
                                00:00:00
                            </Typography> 
                        </Box>

                        {/* Task 3 Timer: Reset button */}
                        <Button variant = "outlined" sx ={{borderColor: red[900], color: red[900]}} onClick = {() => {if(timeOver == 0) {timerOn3 = false; milliseconds3 = 0; seconds3 = 0; minutes3 = 0; document.getElementById("timerDisplay3")!.innerHTML = "00:00:00";}}}>
                            RESET
                        </Button>
                    </Stack>
                </Box>

                {/* Task 3: Checklist Header */}
                <FormControlLabel control = {<Checkbox />} label = "TASK 3: From the Red Sea to Tennesse" />
            </Grid>

            {/* Task 3: Indentation & Checkboxes */}
           {Tasks("TASK 3 : Coastal Pioneer Array", taskJSON.task3.tasks)};
            {/* Task 4 Timer: Control Layout */}
            <Grid>    
                {/* Task 4 Timer: Controls */}
                <Box sx = {{width: [600], textAlign : "left"}}>
                    <Stack spacing = {2} direction = "row">
                        {/* Task 4 Timer: Start Button */}
                        <Button variant = "outlined" sx = {{borderColor: green[900], color: green[900]}} onClick = {() => {if(timeOver == 0) {timerOn4 = true; repeat4();}}}> 
                            START 
                        </Button>

                        {/* Task 4 Timer: Pause Button*/}
                        <Button variant = "outlined" sx = {{borderColor: yellow[900], color: yellow[900]}} onClick = {() => {if (timeOver == 0) {timerOn4 = false;}}}> 
                            PAUSE 
                        </Button>
                        
                        {/* Task 4 Timer: Displayed Time */}
                        <Box textAlign = "center" sx = {{width: [80]}}>  {/* Include 'border: "2px solid grey"'? */}                            
                            <Typography component = "div" id = "timerDisplay4" fontSize = {20} sx = {{color: grey[400]}}>
                                00:00:00
                            </Typography> 
                        </Box>

                        {/* Task 4 Timer: Reset button */}
                        <Button variant = "outlined" sx ={{borderColor: red[900], color: red[900]}} onClick = {() => {if(timeOver == 0) {timerOn4 = false; milliseconds4 = 0; seconds4 = 0; minutes4 = 0; document.getElementById("timerDisplay4")!.innerHTML = "00:00:00";}}}>
                            RESET
                        </Button>
                    </Stack>
                </Box>

                {/* Task 4: Checklist Header */}
                <FormControlLabel control = {<Checkbox />} label = "TASK 4: MATE Floats!" />
            </Grid>
    
            {/* Task 4: Indentation & Checkboxes */}
          {Tasks("TASK 4 : Coastal Pioneer Array", taskJSON.task4.tasks)};

        {/* Settings Menu */}
        <Stack>
            {/* Main Timer: Setting */}
            <Button sx = {{width: [190], color: blue[900]}} onClick = {() => {minutes = 14; seconds = 50; milliseconds = 0; document.getElementById("timerDisplay")!.innerHTML = "14:50:00";}}>
                - Main Timer: [14:50:00]
            </Button>

            {/* Task 1 Timer: Setting */}
            <Button sx = {{width: [200], color: blue[900]}} onClick = {() => {minutes1 = 14; seconds1 = 50; milliseconds = 0; document.getElementById("timerDisplay1")!.innerHTML = "14:50:00";}}>
                - Task 1 Timer: [14:50:00]
            </Button>

            {/* Task 2 Timer: Seting */}
            <Button sx = {{width: [200], color: blue[900]}} onClick = {() => {minutes2 = 14; seconds2 = 50; milliseconds2 = 0; document.getElementById("timerDisplay2")!.innerHTML = "14:50:00";}}>
                - Task 2 Timer: [14:50:00]
            </Button>

            {/* Task 3 Timer: Setting */}
            <Button sx = {{width: [200], color: blue[900]}} onClick = {() => {minutes3 = 14; seconds3 = 50; milliseconds3 = 0; document.getElementById("timerDisplay3")!.innerHTML = "14:50:00";}}>
                - Task 3 Timer: [14:50:00]
            </Button>

            {/* Task 4 Timer: Setting */}
            <Button sx = {{width: [200], color: blue[900]}} onClick = {() => {minutes4 = 14; seconds4 = 50; milliseconds4 = 0; document.getElementById("timerDisplay4")!.innerHTML = "14:50:00";}}>
                - Task 4 Timer: [14:50:00]
            </Button>
        </Stack>

        </FormGroup>
    );
}  

function RenderTasks(tasks: Task[],level = 0, id = "") {
    return tasks.map((task, index) => {
        const currentID = id ? `${id}:${index + 1}` : `${index + 1}`;

        return (
          <div style={{ paddingLeft: `${level ** 0.5 * 2}em` }}>
            {!task.subTasks && (
              <FormControlLabel control={<Checkbox />} label={task.name} style={{fontWeight: "0.9em"}}/>
            )}
            {task.subTasks && (
            <div
                style={{
                paddingLeft: "0px",
                wordBreak: "break-word",
                alignItems: "flex-start",
                fontWeight: "bolder",
                fontSize: "1.1em",
                }}
            >
            <Checkbox sx={{ color: grey[900], "&.Mui-checked": { color: grey[900] } }}
                      disabled={true} style={{paddingLeft: "0px", opacity: 0.5}}
                />
                {task.name} :
              </div>
            )}
            {task.subTasks &&
              RenderTasks(
                task.subTasks,
                level + 1,
                currentID
                // publisher,
                // saved
              )}
          </div>
        );
    });
}

function Tasks(name: string, tasks: Task[]) {
    return (
        <FormGroup style={{padding: "2em"}}>
            <h2>{name}</h2>
            {RenderTasks(tasks)}
        </FormGroup>
    );
}