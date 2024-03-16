import {Box, Button, Checkbox, FormGroup, FormControlLabel, Grid, Typography, AppBar, Stack} from "@mui/material";
import { useState } from "react";
import React from "react";
import { useAtom } from "jotai";
import { grey, green, yellow, red} from "@mui/material/colors";

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
    var timerPaused = 0
    var timeOver = 0

    // Main Timer: Function
    function repeat() {       
        setTimeout(function() {
            if (timerOn == 1) {
                if (timerPaused == 0) {
                    if (timeOver == 0) {
                        milliseconds = milliseconds + 1

                        if (milliseconds >= 100) {
                            seconds = seconds + 1
                            milliseconds = 0
                        }
                        if (seconds >= 60) {
                            minutes = minutes + 1
                            seconds = 0
                        }

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
                                    
                        if (minutes == 15) {
                            timeOver = 1
                        }

                        displayedTime = displayedMinutes + displayedSeconds + displayedMilliseconds
                        document.getElementById("timerDisplay")!.innerHTML = displayedTime
                        repeat()
                    }
                }
            }
        })
    }


    // Task 1 Timer: Variables 
    var milliseconds1 = 0
    var seconds1 = 0
    var minutes1 = 0
    var timerOn1 = false
    var displayedTime1 = ""

    // Task 1 Timer: Function
    function repeat1() {
        setTimeout(function() {
            if(timerOn1 == true){
                milliseconds1 = milliseconds1 + 1

                if(milliseconds1 >= 100) {
                    seconds1 = seconds1 + 1
                    milliseconds1 = 0
                }
                if(seconds1 >= 60) {
                    minutes1 = minutes1 + 1
                    seconds1 = 0
                }

                if(minutes1 == 15) {
                    timeOver = 1
                }

                displayedTime1 = minutes1 + ":" + seconds1 + ":" + milliseconds1
                document.getElementById("timerDisplay1")!.innerHTML = displayedTime1
                repeat1()
            }
        })
    }

    // Task 2 Timer: Variables 
    var milliseconds2 = 0
    var seconds2 = 0
    var minutes2 = 0
    var timerOn2 = false
    var displayedTime2 = ""

    // Task 2 Timer: Function
    function repeat2() {
        setTimeout(function() {
            if(timerOn2 == true){
                milliseconds2 = milliseconds2 + 1

                if(milliseconds2 >= 100) {
                    seconds2 = seconds2 + 1
                    milliseconds2 = 0
                }
                if(seconds2 >= 60) {
                    minutes2 = minutes2 + 1
                    seconds2 = 0
                }

                if(minutes2 == 15) {
                    timeOver = 1
                }

                displayedTime2 = minutes2 + ":" + seconds2 + ":" + milliseconds2
                document.getElementById("timerDisplay2")!.innerHTML = displayedTime2
                repeat2()
            }
        })
    }

    // Task 3 Timer: Variables 
    var milliseconds3 = 0
    var seconds3 = 0
    var minutes3 = 0
    var timerOn3 = false
    var displayedTime3 = ""

    // Task 3 Timer: Function
    function repeat3() {
        setTimeout(function() {
            if(timerOn3 == true){
                milliseconds3 = milliseconds3 + 1

                if(milliseconds3 >= 100) {
                    seconds3 = seconds3 + 1
                    milliseconds3 = 0
                }
                if(seconds3 >= 60) {
                    minutes3 = minutes3 + 1
                    seconds3 = 0
                }

                if(minutes3 == 15) {
                    timeOver = 1
                }

                displayedTime3 = minutes3 + ":" + seconds3 + ":" + milliseconds3
                document.getElementById("timerDisplay3")!.innerHTML = displayedTime3
                repeat3()
            }
        })
    }

 // Task 4 Timer: Variables 
 var milliseconds4 = 0
 var seconds4 = 0
 var minutes4 = 0
 var timerOn4 = false
 var displayedTime4 = ""

 // Task 4 Timer: Function
 function repeat4() {
     setTimeout(function() {
         if(timerOn4 == true){
             milliseconds4 = milliseconds4 + 1

             if(milliseconds4 >= 100) {
                 seconds4 = seconds4 + 1
                 milliseconds4 = 0
             }
             if(seconds4 >= 60) {
                 minutes4 = minutes4 + 1
                 seconds4 = 0
             }

             if(minutes4 == 15) {
                 timeOver = 1
             }

             displayedTime4 = minutes4 + ":" + seconds4 + ":" + milliseconds4
             document.getElementById("timerDisplay4")!.innerHTML = displayedTime4
             repeat4()
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
                        <Box textAlign = "center" sx = {{width: [80] }}>  {/* Include 'border: "2px solid grey"'? */}                            
                            <Typography component = "div" id = "timerDisplay1" fontSize = {20} sx = {{color: grey[400]}}>
                                0:0:0
                            </Typography> 
                        </Box>

                        {/* Task 1 Timer: Reset button */}
                        <Button variant = "outlined" sx ={{borderColor: red[900], color: red[900]}} onClick = {() => {if(timeOver == 0) {timerOn1 = false; milliseconds1 = 0; seconds1 = 0; minutes1 = 0; document.getElementById("timerDisplay1")!.innerHTML = "0:0:0"}}}>
                            RESET
                        </Button>
                    </Stack>
                </Box>
                
                <FormControlLabel control = {<Checkbox />} label = "TASK 1: OOI: Coastal Pioneer Array" />
            </Grid>

            <Grid>
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
                        <Box textAlign = "center" sx = {{width: [80] }}>  {/* Include 'border: "2px solid grey"'? */}                            
                            <Typography component = "div" id = "timerDisplay2" fontSize = {20} sx = {{color: grey[400]}}>
                                0:0:0
                            </Typography> 
                        </Box>

                        {/* Task 2 Timer: Reset button */}
                        <Button variant = "outlined" sx ={{borderColor: red[900], color: red[900]}} onClick = {() => {if(timeOver == 0) {timerOn2 = false; milliseconds2 = 0; seconds2 = 0; minutes2 = 0; document.getElementById("timerDisplay2")!.innerHTML = "0:0:0"}}}>
                            RESET
                        </Button>
                    </Stack>
                </Box>

                <FormControlLabel control = {<Checkbox />} label = "TASK 2: SMART Cables for Ocean Observing" />
            </Grid>

            <Grid>
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
                        <Box textAlign = "center" sx = {{width: [80] }}>  {/* Include 'border: "2px solid grey"'? */}                            
                            <Typography component = "div" id = "timerDisplay3" fontSize = {20} sx = {{color: grey[400]}}>
                                0:0:0
                            </Typography> 
                        </Box>

                        {/* Task 3 Timer: Reset button */}
                        <Button variant = "outlined" sx ={{borderColor: red[900], color: red[900]}} onClick = {() => {if(timeOver == 0) {timerOn3 = false; milliseconds3 = 0; seconds3 = 0; minutes3 = 0; document.getElementById("timerDisplay3")!.innerHTML = "0:0:0"}}}>
                            RESET
                        </Button>
                    </Stack>
                </Box>

                <FormControlLabel control = {<Checkbox />} label = "TASK 3: From the Red Sea to Tennesse" />
            </Grid>

            <Grid>
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

            <Grid>    
                {/* Task 4 Timer : Controls */}
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
                        <Box textAlign = "center" sx = {{width: [80] }}>  {/* Include 'border: "2px solid grey"'? */}                            
                            <Typography component = "div" id = "timerDisplay4" fontSize = {20} sx = {{color: grey[400]}}>
                                0:0:0
                            </Typography> 
                        </Box>

                        {/* Task 4 Timer: Reset button */}
                        <Button variant = "outlined" sx ={{borderColor: red[900], color: red[900]}} onClick = {() => {if(timeOver == 0) {timerOn4 = false; milliseconds4 = 0; seconds4 = 0; minutes4 = 0; document.getElementById("timerDisplay4")!.innerHTML = "0:0:0"}}}>
                            RESET
                        </Button>
                    </Stack>
                </Box>

                <FormControlLabel control = {<Checkbox />} label = "TASK 4: MATE Floats!" />
            </Grid>
    
            <Grid>
                <Checkbox sx = {{color: grey[900], '&.Mui-checked': {color: grey[900]}}} />
                <FormControlLabel control = {<Checkbox />} label = " (A) Design and construct an opeartational vetical porfiling float - 5 points " />
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
        </FormGroup>
    );
}