import {Box, Button, Checkbox, FormGroup, FormControlLabel, Grid, Typography, AppBar, Stack} from "@mui/material";
import { useState } from "react";
import React from "react";
import { useAtom } from "jotai";
import { grey, green, yellow, red, blue} from "@mui/material/colors";

/* 
The purpose of the TabletApp component is to provide the deck chief with a GUI which is to be used on a tablet that:

    1. Provides a checklist of tasks and subtasks that can be ticked/unticked 

    2. Provides a main timer, which can be started, paused and reset, permanently displayed as bar at the top of the screen

    3. Provides secondary timers, one for each task, which can be started paused and reset independently of the main timers and other task-specific timers

Additional features include:

    1. Should any timer reach 15 minutes, all timers "break". That is, all timers are paused and are no longer interactive

    2. A settings menu located underneath the checklist for debugging purposes

Notes:

    1. Timers are too fast; for every actual minute that passes, unfixed timers are approximately 2 seconds ahead

    2. Testing the usage device will be necessary as to ensure correct formatting and appropriate timer delay
*/

export default function TabletApp() {
    // Main-Timer: Variables
    var milliseconds = 0
    var displayedMilliseconds = ""
    var seconds = 0
    var displayedSeconds = ""
    var minutes = 0
    var displayedMinutes = ""
    var displayedTime = ""
    var timerOn = 0
    var timerPaused = 0 // Necessary variable?
    var timeOver = 0
    var doubleNextSecond = false
    var tripleNextSecond = false

    // Main Timer: Function (FIXED)
    function repeat() {       
        setTimeout(function() {
            if (timerOn == 1) {
                if (timerPaused == 0) {
                    if (timeOver == 0) {
                        milliseconds = milliseconds + 1
 
                        // If millesconds and/or seconds have reached their limits, increment/reset variables accordingly. tirpleNextSecond and doubleNextSecond are used reduce the discrepancy between the displayed time elapsed and total time elapsed
                        if (milliseconds >= 100) {
                            if (tripleNextSecond == false && doubleNextSecond == false) {
                            seconds = seconds + 1
                            milliseconds = 0
                            }
                            else if (tripleNextSecond == true) {
                                milliseconds = 0
                                tripleNextSecond = false
                            }
                            else {
                                milliseconds = 0
                                doubleNextSecond = false
                            }
                        }
                        if (seconds >= 60) {
                            minutes = minutes + 1
                            seconds = 0
                            tripleNextSecond = true
                            doubleNextSecond = true
                        }

                        // Set displayedMilliseconds, displayedSeconds, and displayedMinutes based upon if their corresponding non-displayed values are single-digit or double-digit
                        if (milliseconds < 10) {
                            displayedMilliseconds = ":0" + milliseconds
                        }
                        else {
                            displayedMilliseconds = ":" + milliseconds
                        }
                        if (seconds < 10) {
                            displayedSeconds = ":0" + seconds
                        }
                        else {
                            displayedSeconds = ":" + seconds
                        }
                        if (minutes < 10) {
                            displayedMinutes = "0" + minutes 
                        }
                        else {
                            displayedMinutes = "" + minutes
                        }
                             
                        // If 15 minutes have passed within the Main Timer, break all timers
                        if (minutes == 15) {
                            timeOver = 1
                        }

                        // Update and display displayedTime
                        displayedTime = displayedMinutes + displayedSeconds + displayedMilliseconds
                        document.getElementById("timerDisplay")!.innerHTML = displayedTime

                        // Repeat the Main Timer's function
                        repeat()
                    }
                }
            }
        }, 9)
    }


    // Task 1 Timer: Variables 
    var milliseconds1 = 0
    var displayedMilliseconds1 = ""
    var seconds1 = 0
    var displayedSeconds1 = ""
    var minutes1 = 0
    var displayedMinutes1 = ""
    var timerOn1 = false
    var displayedTime1 = ""

    // Task 1 Timer: Function
    function repeat1() {
        setTimeout(function() {
            if (timerOn1 == true) { 
                if (timeOver == 0) {
                    milliseconds1 = milliseconds1 + 1

                    // If millesconds1 and/or seconds1 have reached their limits, increment/reset variables accordingly
                    if(milliseconds1 >= 100) {
                        seconds1 = seconds1 + 1
                        milliseconds1 = 0
                    }
                    if(seconds1 >= 60) {
                        minutes1 = minutes1 + 1
                        seconds1 = 0
                    }

                    // Set displayedMilliseconds1, displayedSeconds1, and displayedMinutes1 based upon if their corresponding non-displayed values are single-digit or double-digit
                    if (milliseconds1 < 10) {
                        displayedMilliseconds1 = ":0" + milliseconds1
                    }
                    else {
                        displayedMilliseconds1 = ":" + milliseconds1
                    }
                    if (seconds1 < 10) {
                        displayedSeconds1 = ":0" + seconds1
                    }
                    else {
                        displayedSeconds1 = ":" + seconds1
                    }
                    if (minutes1 < 10) {
                        displayedMinutes1 = "0" + minutes1
                    }
                    else {
                        displayedMinutes1 = "" + minutes1
                    }

                    // If 15 minutes have passed within the Task 1 Timer, break all timers
                    if(minutes1 == 15) {
                        timeOver = 1
                    }

                    // Update and display displayedTime1
                    displayedTime1 = displayedMinutes1 + displayedSeconds1 + displayedMilliseconds1
                    document.getElementById("timerDisplay1")!.innerHTML = displayedTime1

                    // Repeat the Task 1 Timer's function
                    repeat1()
                }
            }
        }, 9)
    }

    // Task 2 Timer: Variables 
    var milliseconds2 = 0
    var displayedMilliseconds2 = ""
    var seconds2 = 0
    var displayedSeconds2 = ""
    var minutes2 = 0
    var displayedMinutes2 = ""
    var timerOn2 = false
    var displayedTime2 = ""

    // Task 2 Timer: Function
    function repeat2() {
        setTimeout(function() {
            if (timerOn2 == true) {
                if (timeOver == 0) {
                    milliseconds2 = milliseconds2 + 1

                    // If millesconds2 and/or seconds2 have reached their limits, increment/reset variables accordingly
                    if(milliseconds2 >= 100) {
                        seconds2 = seconds2 + 1
                        milliseconds2 = 0
                    }
                    if(seconds2 >= 60) {
                        minutes2 = minutes2 + 1
                        seconds2 = 0
                    }

                    // Set displayedMilliseconds2, displayedSeconds2, and displayedMinutes2 based upon if their corresponding non-displayed values are single-digit or double-digit
                    if (milliseconds2 < 10) {
                        displayedMilliseconds2 = ":0" + milliseconds2
                    }
                    else {
                        displayedMilliseconds2 = ":" + milliseconds2
                    }
                    if (seconds2 < 10) {
                        displayedSeconds2 = ":0" + seconds2
                    }
                    else {
                        displayedSeconds2 = ":" + seconds2
                    }
                    if (minutes2 < 10) {
                        displayedMinutes2 = "0" + minutes2
                    }
                    else {
                        displayedMinutes2 = "" + minutes2
                    }

                    // If 15 minutes have passed wihtin the Task 2 Timer, break all timers
                    if(minutes2 == 15) {
                        timeOver = 1
                    }

                    // Update and display displayedTime2
                    displayedTime2 = displayedMinutes2 + displayedSeconds2 + displayedMilliseconds2
                    document.getElementById("timerDisplay2")!.innerHTML = displayedTime2
                    
                    // Repeat the Task 2 Timer's function
                    repeat2()
                }
            }
        })
    }

    // Task 3 Timer: Variables 
    var milliseconds3 = 0
    var displayedMilliseconds3 = ""
    var seconds3 = 0
    var displayedSeconds3 = ""
    var minutes3 = 0
    var displayedMinutes3 = ""
    var timerOn3 = false
    var displayedTime3 = ""

    // Task 3 Timer: Function
    function repeat3() {
        setTimeout(function() {
            if (timerOn3 == true) {
                if (timeOver == 0) {
                    milliseconds3 = milliseconds3 + 1

                    // If millesconds3 and/or seconds3 have reached their limits, increment/reset variables accordingly
                    if(milliseconds3 >= 100) {
                        seconds3 = seconds3 + 1
                        milliseconds3 = 0
                    }
                    if(seconds3 >= 60) {
                        minutes3 = minutes3 + 1
                        seconds3 = 0
                    }

                    // Set displayedMilliseconds3, displayedSeconds3, and displayedMinutes3 based upon if their corresponding non-displayed values are single-digit or double-digit
                    if (milliseconds3 < 10) {
                        displayedMilliseconds3 = ":0" + milliseconds3
                    }
                    else {
                        displayedMilliseconds3 = ":" + milliseconds3
                    }
                    if (seconds3 < 10) {
                        displayedSeconds3 = ":0" + seconds3
                    }
                    else {
                        displayedSeconds3 = ":" + seconds3
                    }
                    if (minutes3 < 10) {
                        displayedMinutes3 = "0" + minutes3
                    }
                    else {
                        displayedMinutes3 = "" + minutes3
                    }

                    // If 15 minutes have passed within the Task 3 Timer, break all timers
                    if(minutes3 == 15) {
                        timeOver = 1
                    }

                    // Update and display displayedTime3
                    displayedTime3 = displayedMinutes3 + displayedSeconds3 + displayedMilliseconds3
                    document.getElementById("timerDisplay3")!.innerHTML = displayedTime3

                    // Repeat the Task 3 Timer's function
                    repeat3()
                }
            }
        })
    }

 // Task 4 Timer: Variables 
 var milliseconds4 = 0
 var displayedMilliseconds4 = ""
 var seconds4 = 0
 var displayedSeconds4 = ""
 var minutes4 = 0
 var displayedMinutes4 = ""
 var timerOn4 = false
 var displayedTime4 = ""

 // Task 4 Timer: Function
 function repeat4() {
     setTimeout(function() {
         if (timerOn4 == true) {
            if (timeOver == 0) {
                milliseconds4 = milliseconds4 + 1

                // If millesconds4 or seconds4 have reached their limits, increment/reset variables accordingly
                if(milliseconds4 >= 100) {
                    seconds4 = seconds4 + 1
                    milliseconds4 = 0
                }
                if(seconds4 >= 60) {
                    minutes4 = minutes4 + 1
                    seconds4 = 0
                }

                // Set displayedMilliseconds4, displayedSeconds4, and displayedMinutes4 based upon if their corresponding non-displayed values are single-digit or double-digit
                if (milliseconds4 < 10) {
                    displayedMilliseconds4 = ":0" + milliseconds4
                }
                else {
                    displayedMilliseconds4 = ":" + milliseconds4
                }
                if (seconds4 < 10) {
                    displayedSeconds4 = ":0" + seconds4
                }
                else {
                    displayedSeconds4 = ":" + seconds4
                }
                if (minutes4 < 10) {
                    displayedMinutes4 = "0" + minutes4
                }
                else {
                    displayedMinutes4 = "" + minutes4
                }


                // If 15 minutes have passed within the Task 4 Timer, break all timers
                if(minutes4 == 15) {
                    timeOver = 1
                }

                 // Update and display displayedTime4
                displayedTime4 = displayedMinutes4 + displayedSeconds4 + displayedMilliseconds4
                document.getElementById("timerDisplay4")!.innerHTML = displayedTime4

                // Repeat the Task 4 Timer's function
                repeat4()
            }
         }
     })
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
                <Button variant = "contained" sx = {{background: green[400], color: grey[50]}} onClick = {() => {if(timeOver == 0) {timerOn = 1} repeat()}}>                  
                    <Typography fontSize = {25}> 
                        START 
                    </Typography>
                </Button>
                
                {/* Main Timer: Pause Button */}
                <Button variant = "contained" sx = {{background: yellow[400], color: [grey[50]]}} onClick = {() => {if(timeOver == 0) {timerOn = 0}}}>                 
                    <Typography fontSize = {25}>
                        PAUSE
                    </Typography>
                </Button>

                {/* Main Timer: Reset Button */}
                <Button variant = "contained" sx = {{background: red[400], color: grey[50]}} onClick = {() => {if(timeOver == 0) {timerOn = 0; milliseconds = 0; seconds = 0; minutes = 0; document.getElementById("timerDisplay")!.innerHTML = "00:00:00"}}}>                    
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
                        <Button variant = "outlined" sx = {{borderColor: green[900], color: green[900]}} onClick = {() => {if(timeOver == 0) {timerOn1 = true; repeat1()}}}> 
                            START 
                        </Button>

                        {/* Task 1 Timer: Pause Button*/}
                        <Button variant = "outlined" sx = {{borderColor: yellow[900], color: yellow[900]}} onClick = {() => {if (timeOver == 0) {timerOn1 = false}}}> 
                            PAUSE 
                        </Button>
                        
                        {/* Task 1 Timer: Displayed Time */}
                        <Box textAlign = "center" sx = {{width: [80]}}>  {/* Include 'border: "2px solid grey"'? */}                            
                            <Typography component = "div" id = "timerDisplay1" fontSize = {20} sx = {{color: grey[400]}}>
                                00:00:00
                            </Typography> 
                        </Box>

                        {/* Task 1 Timer: Reset button */}
                        <Button variant = "outlined" sx ={{borderColor: red[900], color: red[900]}} onClick = {() => {if(timeOver == 0) {timerOn1 = false; milliseconds1 = 0; seconds1 = 0; minutes1 = 0; document.getElementById("timerDisplay1")!.innerHTML = "00:00:00"}}}>
                            RESET
                        </Button>
                    </Stack>
                </Box>
                
                {/* Task 1: Checklist Header */}
                <FormControlLabel control = {<Checkbox />} label = "TASK 1: OOI: Coastal Pioneer Array" />
            </Grid>

            {/* Task 1: Indentation & Checkboxes */}
            <Grid>
                {/* Rows of darkened checkboxes are used to simulate indentation */}
                <Checkbox  sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> 
                <FormControlLabel control = {<Checkbox />} label = "1.1 Release the multi-function node" /> 
                <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} />
                <FormControlLabel control = {<Checkbox />} label = "'Trigger' the release of the multi-function node's recovery float - 10 points" />
                <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} />
                <FormControlLabel control = {<Checkbox />} label = "Visually determine the failed recovery float - 5 points" />
                <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} />  <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} />
                <FormControlLabel control = {<Checkbox />} label = "Pull a pin to release the failed recovery float to the surface - 10 points" />
                <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} />
                <FormControlLabel control = {<Checkbox />} label = "Return the failed covery float to the surface, side of the pool - 5 points" />
                <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} />
                <FormControlLabel control = {<Checkbox />} label = "Connect a recovery line to the multi-function node for manual recovery - 20 points" />
            </Grid>

            {/* Task 2 Timer: Control Layout */}
            <Grid>    
                {/* Task 2 Timer : Controls */}
                <Box sx = {{width: [600], textAlign : "left"}}>
                    <Stack spacing = {2} direction = "row">
                        {/* Task 2 Timer: Start Button */}
                        <Button variant = "outlined" sx = {{borderColor: green[900], color: green[900]}} onClick = {() => {if(timeOver == 0) {timerOn2 = true; repeat2()}}}> 
                            START 
                        </Button>

                        {/* Task 2 Timer: Pause Button*/}
                        <Button variant = "outlined" sx = {{borderColor: yellow[900], color: yellow[900]}} onClick = {() => {if (timeOver == 0) {timerOn2 = false}}}> 
                            PAUSE 
                        </Button>
                        
                        {/* Task 2 Timer: Displayed Time */}
                        <Box textAlign = "center" sx = {{width: [80]}}>  {/* Include 'border: "2px solid grey"'? */}                            
                            <Typography component = "div" id = "timerDisplay2" fontSize = {20} sx = {{color: grey[400]}}>
                                00:00:00
                            </Typography> 
                        </Box>

                        {/* Task 2 Timer: Reset button */}
                        <Button variant = "outlined" sx ={{borderColor: red[900], color: red[900]}} onClick = {() => {if(timeOver == 0) {timerOn2 = false; milliseconds2 = 0; seconds2 = 0; minutes2 = 0; document.getElementById("timerDisplay2")!.innerHTML = "00:00:00"}}}>
                            RESET
                        </Button>
                    </Stack>
                </Box>

                {/* Task 2: Checklist Header */}
                <FormControlLabel control = {<Checkbox />} label = "TASK 2: SMART Cables for Ocean Observing" />
            </Grid>

            {/* Task 2: Indentation & Checkboxes */}
            <Grid>
                {/* Rows of darkened checkboxes are to simulate indentation */}
                <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} />
                <FormControlLabel control = {<Checkbox />} label = "2.1 Deploy SMART Cable" />
                <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} />
                <FormControlLabel control = {<Checkbox />} label = "Deploy SMART cable through three waypoints - up to 20 points" />
                <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} />
                <FormControlLabel control = {<Checkbox />} label = "Deploy SMART cable through two waypoints located on the seafloor" />
                <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} />
                <FormControlLabel control = {<Checkbox />} label = "(A) Deployed through both waypoints - 10 points" />
                <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} />
                <FormControlLabel control = {<Checkbox />} label = "(B) Deployed through one waypoint - 5 points " />
                <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} />
                <FormControlLabel control = {<Checkbox />} label = "Deploy SMART cable through a waypoint located on a seamount - 10 points" />
                <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} />
                <FormControlLabel control = {<Checkbox />} label = "Place the SMART repeater in the designated area - 10 points" />
                <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} />
                <FormControlLabel control = {<Checkbox />} label = "Return the end of the cable to the surface, side of the pool - 5 points" />
                <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} />
                <FormControlLabel control = {<Checkbox />} label = "Measure the temperature to check the SMART cable sensor readings" />
                <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} />
                <FormControlLabel control = {<Checkbox />} label = "(A) Within 1°C - 15 points" />
                <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} />
                <FormControlLabel control = {<Checkbox />} label = "(B) Within 2°C - 5 points" />
                <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} />
                <FormControlLabel control = {<Checkbox />} label = "Connect the AUV docking station the the SMART cable repeater" />
                <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} />
                <FormControlLabel control = {<Checkbox />} label = "Retrieve the power connector from the AUV docking station - 5 points" />
                <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} />
                <FormControlLabel control = {<Checkbox />} label = "Install the power connector - 15 points" />
            </Grid>

            {/* Task 3 Timer: Control Layout */}
            <Grid>    
                {/* Task 3 Timer : Controls */}
                <Box sx = {{width: [600], textAlign : "left"}}>
                    <Stack spacing = {2} direction = "row">
                        {/* Task 3 Timer: Start Button */}
                        <Button variant = "outlined" sx = {{borderColor: green[900], color: green[900]}} onClick = {() => {if(timeOver == 0) {timerOn3 = true; repeat3()}}}> 
                            START 
                        </Button>

                        {/* Task 3 Timer: Pause Button*/}
                        <Button variant = "outlined" sx = {{borderColor: yellow[900], color: yellow[900]}} onClick = {() => {if (timeOver == 0) {timerOn3 = false}}}> 
                            PAUSE 
                        </Button>
                        
                        {/* Task 3 Timer: Displayed Time */}
                        <Box textAlign = "center" sx = {{width: [80]}}>  {/* Include 'border: "2px solid grey"'? */}                            
                            <Typography component = "div" id = "timerDisplay3" fontSize = {20} sx = {{color: grey[400]}}>
                                00:00:00
                            </Typography> 
                        </Box>

                        {/* Task 3 Timer: Reset button */}
                        <Button variant = "outlined" sx ={{borderColor: red[900], color: red[900]}} onClick = {() => {if(timeOver == 0) {timerOn3 = false; milliseconds3 = 0; seconds3 = 0; minutes3 = 0; document.getElementById("timerDisplay3")!.innerHTML = "00:00:00"}}}>
                            RESET
                        </Button>
                    </Stack>
                </Box>

                {/* Task 3: Checklist Header */}
                <FormControlLabel control = {<Checkbox />} label = "TASK 3: From the Red Sea to Tennesse" />
            </Grid>

            {/* Task 3: Indentation & Checkboxes */}
            <Grid>
                {/* Rows of darkened checkboxes are used to simulate indentation */}
                <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} />
                <FormControlLabel control = {<Checkbox />} label = "3.1 Probiotics 2" />
                <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} />
                <FormControlLabel control = {<Checkbox />} label = "Place a probiotic irrigation system in the designated location - 10 points" />
                <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} />
                <FormControlLabel control = {<Checkbox />} label = "Deploy the probiotic sprinkler on coral head - 10 points" />
                <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} />
                <FormControlLabel control = {<Checkbox />} label = "Activate the irrigation system - 10 points" />
                <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} />
                <FormControlLabel control = {<Checkbox />} label = "3.2 Coral Restoration" />
                <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} />
                <FormControlLabel control = {<Checkbox />} label = "Transplant branching coral - 10 points" />
                <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} />
                <FormControlLabel control = {<Checkbox />} label = "Transplant brain coral" />
                <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} />
                <FormControlLabel control = {<Checkbox />} label = "(A) Autonomously - 30 points" />
                <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} />
                <FormControlLabel control = {<Checkbox />} label = "(B) Manually - 10 points" />
                <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} />
                <FormControlLabel control = {<Checkbox />} label = "3.3 3D Coral Modelling" />
                <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} />
                <FormControlLabel control = {<Checkbox />} label = "(A) Autonomously create a 3D model of the coral restoration area - up to 40 points" />
                <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} />
                <FormControlLabel control = {<Checkbox />} label = "Create a 3D model of the coral restoration area - 20 points" />
                <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} />
                <FormControlLabel control = {<Checkbox />} label = "Measure the length of the coral restoration area (within 5 cm) - 10 points" />
                <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} />
                <FormControlLabel control = {<Checkbox />} label = "Scale the 3D model using the length of the coral restoration area - 5 points" />
                <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} />                
                <FormControlLabel control = {<Checkbox />} label = "Use the scaled 3D model to estimate the height (within 5 cm) - 5 points" />
                <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} />
                <FormControlLabel control = {<Checkbox />} label = "(B) Manually create a 3D model of the coral restoration area - up to 30 points" />
                <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} />
                <FormControlLabel control = {<Checkbox />} label = "Measure the length of the coral restoration area (within 5 cm) - 10 points" />
                <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} />
                <FormControlLabel control = {<Checkbox />} label = "Measure the height of the coral restoration area (within 5 cm) - 10 points" />
                <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} />
                <FormControlLabel control = {<Checkbox />} label = "Create a scaled 3D model displaying both measurements - 10 points" />
                <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} />
                <FormControlLabel control = {<Checkbox />} label = "3.4 Determine the location of sturgeon spawning grounds" />
                <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} />
                <FormControlLabel control = {<Checkbox />} label = "Recover an acoustic receiver - 10 points" />
                <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} />
                <FormControlLabel control = {<Checkbox />} label = "Determine the location of a potential spawning site" />
                <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} />
                <FormControlLabel control = {<Checkbox />} label = "Create a graph of sturgeon locations from the receiver data - 15 points" />
                <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} />
                <FormControlLabel control = {<Checkbox />} label = "Determine the potential spawning site - 5 points" />
                <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} />
                <FormControlLabel control = {<Checkbox />} label = "Characterize the habitat at potential spawning site" />
                <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} />
                <FormControlLabel control = {<Checkbox />} label = "Place an ADCP - 10 points" />
                <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} />
                <FormControlLabel control = {<Checkbox />} label = "Recover a sediment sample - 10 points" />
            </Grid>

            {/* Task 4 Timer: Control Layout */}
            <Grid>    
                {/* Task 4 Timer: Controls */}
                <Box sx = {{width: [600], textAlign : "left"}}>
                    <Stack spacing = {2} direction = "row">
                        {/* Task 4 Timer: Start Button */}
                        <Button variant = "outlined" sx = {{borderColor: green[900], color: green[900]}} onClick = {() => {if(timeOver == 0) {timerOn4 = true; repeat4()}}}> 
                            START 
                        </Button>

                        {/* Task 4 Timer: Pause Button*/}
                        <Button variant = "outlined" sx = {{borderColor: yellow[900], color: yellow[900]}} onClick = {() => {if (timeOver == 0) {timerOn4 = false}}}> 
                            PAUSE 
                        </Button>
                        
                        {/* Task 4 Timer: Displayed Time */}
                        <Box textAlign = "center" sx = {{width: [80]}}>  {/* Include 'border: "2px solid grey"'? */}                            
                            <Typography component = "div" id = "timerDisplay4" fontSize = {20} sx = {{color: grey[400]}}>
                                00:00:00
                            </Typography> 
                        </Box>

                        {/* Task 4 Timer: Reset button */}
                        <Button variant = "outlined" sx ={{borderColor: red[900], color: red[900]}} onClick = {() => {if(timeOver == 0) {timerOn4 = false; milliseconds4 = 0; seconds4 = 0; minutes4 = 0; document.getElementById("timerDisplay4")!.innerHTML = "00:00:00"}}}>
                            RESET
                        </Button>
                    </Stack>
                </Box>

                {/* Task 4: Checklist Header */}
                <FormControlLabel control = {<Checkbox />} label = "TASK 4: MATE Floats!" />
            </Grid>
    
            {/* Task 4: Indentation & Checkboxes */}
            <Grid>
                {/* Rows of darkened checkboxes are used to simulate indentation */}
                <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} />
                <FormControlLabel control = {<Checkbox />} label = " (A) Design and construct an operational vertical profiling float - 5 points " />
                <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} />
                <FormControlLabel control = {<Checkbox />} label = "Prior to the competition, design a construct and vertical profiling float - 5 points" />
                <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} />
                <FormControlLabel control = {<Checkbox />} label = "Deploy the float into a designated area - 5 points" />
                <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} />
                <FormControlLabel control = {<Checkbox />} label = "Float communicates with the mission station prior to descending - 10 points" />
                <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} />
                <FormControlLabel control = {<Checkbox />} label = "Float completes up to two vertical profiles - " />
                <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} />
                <FormControlLabel control = {<Checkbox />} label = "Vertical profile 1" />
                <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} />
                <FormControlLabel control = {<Checkbox />} label = "Float completes first profile using a buoyancy engine - 10 points" />
                <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} />
                <FormControlLabel control = {<Checkbox />} label = "Float communicates data to the mission station - 5 points" />
                <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} />
                <FormControlLabel control = {<Checkbox />} label = "Data is graphed as depth over time - 10 points" />
                <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} />
                <FormControlLabel control = {<Checkbox />} label = "Vertical profile 2" />
                <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} />
                <FormControlLabel control = {<Checkbox />} label = "Float completes a second profile using a buoyancy engine - 10 points" />
                <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} />
                <FormControlLabel control = {<Checkbox />} label = "Float communicates data to the mission station - 5 points" />
                <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} />
                <FormControlLabel control = {<Checkbox />} label = "Data is graphed as depth over time - 10 points" />
                <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} /> <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} />
                <FormControlLabel control = {<Checkbox />} label = "(B) MATE-provided data is used to graph depth over time - 10 points" />
            </Grid>

        {/* Settings Menu */}
        <Stack>
            {/* Main Timer: Setting */}
            <Button sx = {{width: [190], color: blue[900]}} onClick = {() => {minutes = 14; seconds = 50; milliseconds = 0; document.getElementById("timerDisplay")!.innerHTML = "14:50:00"}}>
                - Main Timer: [14:50:00]
            </Button>

            {/* Task 1 Timer: Setting */}
            <Button sx = {{width: [200], color: blue[900]}} onClick = {() => {minutes1 = 14; seconds1 = 50; milliseconds = 0; document.getElementById("timerDisplay1")!.innerHTML = "14:50:00"}}>
                - Task 1 Timer: [14:50:00]
            </Button>

            {/* Task 2 Timer: Seting */}
            <Button sx = {{width: [200], color: blue[900]}} onClick = {() => {minutes2 = 14; seconds2 = 50; milliseconds2 = 0; document.getElementById("timerDisplay2")!.innerHTML = "14:50:00"}}>
                - Task 2 Timer: [14:50:00]
            </Button>

            {/* Task 3 Timer: Setting */}
            <Button sx = {{width: [200], color: blue[900]}} onClick = {() => {minutes3 = 14; seconds3 = 50; milliseconds3 = 0; document.getElementById("timerDisplay3")!.innerHTML = "14:50:00"}}>
                - Task 3 Timer: [14:50:00]
            </Button>

            {/* Task 4 Timer: Setting */}
            <Button sx = {{width: [200], color: blue[900]}} onClick = {() => {minutes4 = 14; seconds4 = 50; milliseconds4 = 0; document.getElementById("timerDisplay4")!.innerHTML = "14:50:00"}}>
                - Task 4 Timer: [14:50:00]
            </Button>
        </Stack>

        </FormGroup>
    );
}  