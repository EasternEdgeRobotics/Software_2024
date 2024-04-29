import {Box, Button, Checkbox, FormGroup, FormControlLabel, Grid, Typography, AppBar, Stack} from "@mui/material";
import { useState } from "react";
import React from "react";
import { useAtom } from "jotai";
import { grey, green, yellow, red, blue} from "@mui/material/colors";
import { Task } from "../types/Task";
import taskJSON from "./tasks.json";
import { Row } from "react-bootstrap";


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



    return (
      // Main Timer & Task 1 Timer: Control Layouts
      <FormGroup>
        {/* Main Timer: Controls */}
        <AppBar sx={{ bgcolor: "grey", textAlign: "center" }}>
          {/* Main Timer: Displayed Time */}
          <Typography component="div" id="timerDisplay" fontSize={50}>
            00:00:00
          </Typography>

          {/* Main Timer: Start Button */}
          <Button
            variant="contained"
            sx={{ background: green[400], color: grey[50] }}
            onClick={() => {
              if (timeOver == 0) {
                timerOn = 1;
              }
              repeat();
            }}
          >
            <Typography fontSize={25}>START</Typography>
          </Button>

          {/* Main Timer: Pause Button */}
          <Button
            variant="contained"
            sx={{ background: yellow[400], color: [grey[50]] }}
            onClick={() => {
              if (timeOver == 0) {
                timerOn = 0;
              }
            }}
          >
            <Typography fontSize={25}>PAUSE</Typography>
          </Button>

          {/* Main Timer: Reset Button */}
          <Button
            variant="contained"
            sx={{ background: red[400], color: grey[50] }}
            onClick={() => {
              if (timeOver == 0) {
                timerOn = 0;
                milliseconds = 0;
                seconds = 0;
                minutes = 0;
                document.getElementById("timerDisplay")!.innerHTML = "00:00:00";
              }
            }}
          >
            <Typography fontSize={25}>RESET</Typography>
          </Button>
        </AppBar>
        <div style={{ height: "240px" }}></div>
        {Tasks("TASK 1 : Coastal Pioneer Array", taskJSON.task1.tasks)};
        {Tasks("TASK 2 : Deploy SMART cables", taskJSON.task2.tasks)};
        {Tasks("TASK 3 : From the Red Sea to Tenesse", taskJSON.task3.tasks)};
        {Tasks("TASK 4 : MATE Floats", taskJSON.task4.tasks)};
      </FormGroup>
    );
}  

function RenderTasks(tasks: Task[],level = 0, id = "") {
    return tasks.map((task, index) => {
        const currentID = id ? `${id}:${index + 1}` : `${index + 1}`;

        return (
          <div style={{ paddingLeft: `${level ** 0.5 * 2}em` }}>
            {!task.subTasks && (
              <FormControlLabel
                control={<Checkbox />}
                label={<div style={{ fontSize: "1.6vw" }}>{task.name}</div>}
                style={{ fontSize: 20 }}
              />
            )}
            {task.subTasks && (
              <div
                style={{
                  paddingLeft: "0px",
                  wordBreak: "break-word",
                  alignItems: "flex-start",
                  fontWeight: "bolder",
                  fontSize: "1.9vw",
                }}
              >
                <Checkbox
                  sx={{
                    color: grey[900],
                    "&.Mui-checked": { color: grey[900] },
                  }}
                  disabled={true}
                  style={{ paddingLeft: "0px", opacity: 0.5 }}
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
      <FormGroup style={{ paddingLeft: "10vw", paddingRight: "10vw" }}>
        <Row style={{ display: "flex", justifyContent: "flex-start" , alignItems: "center" , marginBottom: "1vh"}}>
                <h1 style={{ marginTop: "0px", marginBottom: "0px" }}>{name}</h1>
                <div style={{ width: "2vw" }}></div>
          <Timer />
        </Row>
        {RenderTasks(tasks)}
      </FormGroup>
    );
}

function Timer() {
        let milliseconds1 = 0;
        let displayedMilliseconds1 = "";
        let seconds1 = 0;
        let displayedSeconds1 = "";
        let minutes1 = 0;
        let displayedMinutes1 = "";
        let timerOn1 = false;
        let displayedTime1 = "";
        let timeOver = 0;

        // Task 1 Timer: Function
        function repeat1() {
          setTimeout(function () {
            if (timerOn1 == true) {
              if (timeOver == 0) {
                milliseconds1 = milliseconds1 + 1;

                // If millesconds1 and/or seconds1 have reached their limits, increment/reset variables accordingly
                if (milliseconds1 >= 100) {
                  seconds1 = seconds1 + 1;
                  milliseconds1 = 0;
                }
                if (seconds1 >= 60) {
                  minutes1 = minutes1 + 1;
                  seconds1 = 0;
                }

                // Set displayedMilliseconds1, displayedSeconds1, and displayedMinutes1 based upon if their corresponding non-displayed values are single-digit or double-digit
                if (milliseconds1 < 10) {
                  displayedMilliseconds1 = ":0" + milliseconds1;
                } else {
                  displayedMilliseconds1 = ":" + milliseconds1;
                }
                if (seconds1 < 10) {
                  displayedSeconds1 = ":0" + seconds1;
                } else {
                  displayedSeconds1 = ":" + seconds1;
                }
                if (minutes1 < 10) {
                  displayedMinutes1 = "0" + minutes1;
                } else {
                  displayedMinutes1 = "" + minutes1;
                }

                // If 15 minutes have passed within the Task 1 Timer, break all timers
                if (minutes1 == 15) {
                  timeOver = 1;
                }

                // Update and display displayedTime1
                displayedTime1 =
                  displayedMinutes1 +
                  displayedSeconds1 +
                  displayedMilliseconds1;
                document.getElementById("timerDisplay1")!.innerHTML =
                  displayedTime1;

                // Repeat the Task 1 Timer's function
                repeat1();
              }
            }
          });
        }
    
    return (
      <Box sx={{ width: [400], textAlign: "left" }}>
        <Stack spacing={2} direction="row">
          {/* Task 1 Timer: Start Button */}
          <Button
            variant="outlined"
            sx={{ borderColor: green[900], color: green[900] }}
            onClick={() => {
              if (timeOver == 0) {
                timerOn1 = true;
                repeat1();
              }
            }}
          >
            START
          </Button>

          {/* Task 1 Timer: Pause Button*/}
          <Button
            variant="outlined"
            sx={{ borderColor: yellow[900], color: yellow[900] }}
            onClick={() => {
              if (timeOver == 0) {
                timerOn1 = false;
              }
            }}
          >
            PAUSE
          </Button>

          {/* Task 1 Timer: Displayed Time */}
          <Box textAlign="center" sx={{ width: [80] }}>
            {" "}
            {/* Include 'border: "2px solid grey"'? */}
            <Typography
              component="div"
              id="timerDisplay1"
              fontSize={20}
              sx={{ color: grey[400] }}
            >
              00:00:00
            </Typography>
          </Box>

          {/* Task 1 Timer: Reset button */}
          <Button
            variant="outlined"
            sx={{ borderColor: red[900], color: red[900] }}
            onClick={() => {
              if (timeOver == 0) {
                timerOn1 = false;
                milliseconds1 = 0;
                seconds1 = 0;
                minutes1 = 0;
                document.getElementById("timerDisplay1")!.innerHTML =
                  "00:00:00";
              }
            }}
          >
            RESET
          </Button>
        </Stack>
      </Box>
    );
}