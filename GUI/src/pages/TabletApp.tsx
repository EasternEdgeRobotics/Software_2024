import {Box, Button, Checkbox, FormGroup, FormControlLabel, Grid, Typography, AppBar, Stack} from "@mui/material";
import React, { useState, useEffect } from "react";
import { atom, useAtom } from "jotai";
import { grey, green, yellow, red, blue} from "@mui/material/colors";
import { Task } from "../types/Task";
import taskJSON from "./tasks.json";
import { Row } from "react-bootstrap";
import { ROSIP } from "../api/Atoms";
import ROSLIB, { Ros } from "roslib";

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

const taskPublisherAtom = atom<ROSLIB.Topic<ROSLIB.Message> | null>(null);

export default function TabletApp() {
  const [ros, setRos] = React.useState<Ros>(new ROSLIB.Ros({}));
  const [RosIP] = useAtom(ROSIP);
  const [saved_tasks, setTasks] = useState<{ [key: string]: boolean }>({});
  const [, setTaskPublisher] = useAtom(taskPublisherAtom);


  ros.on("error", () => {
    0;
  }); // to prevent page breaking

  React.useEffect(() => {
    ros.connect(`ws://${RosIP}:9090`);
    setInterval(() => {
      if (!ros.isConnected) {
        setRos(new ROSLIB.Ros({}));
        ros.connect(`ws://${RosIP}:9090`);
      }
    }, 1000);
  }, []);


  useEffect(() => {
    const taskClient = new ROSLIB.Service({
      ros: ros,
      name: "/all_tasks",
      serviceType: "eer_messages/Config",
    });
    const request = new ROSLIB.ServiceRequest({});
    taskClient.callService(
      request,
      function (result) {
        console.log("All tasks:", result.result);
        setTasks(JSON.parse(result.result));
      },
      function (error) {
        console.log("Error calling service:", error);
      }
    );

      const task_publisher = new ROSLIB.Topic({
        ros: ros,
        name: "task_updates",
        messageType: "std_msgs/String",
      });


    task_publisher.subscribe((message) => {
      const data: { id: string; sender: string; status: boolean } = JSON.parse(
        (message as any).data
      );
      console.log("Received message UPDATE ", data);
      // if (data.sender === "tablet") return;

      setTasks((prevSaved) => ({
        ...prevSaved,
        [data.id]: data.status,
      }));
    });

    setTaskPublisher(task_publisher);
  }, [ros]);




  return (
    <FormGroup>
      <MainTimer />
        <div style={{ height: "250px" }}></div>
        {Tasks({
          name: "TASK 1 : Coastal Pioneer Array",
          tasks: taskJSON.task1.tasks,
          saved: saved_tasks,
          setTasks,
        })}

        {Tasks({
          name: "TASK 2 : Deploy SMART cables",
          tasks: taskJSON.task2.tasks,
          saved: saved_tasks,
          setTasks
        })}

        {Tasks({
          name: "TASK 3 : From the Red Sea to Tenesse",
          tasks: taskJSON.task3.tasks,
          saved: saved_tasks,
          setTasks
        })}

        {Tasks({
          name: "TASK 4 : MATE Floats",
          tasks: taskJSON.task4.tasks,
          saved: saved_tasks,
          setTasks
        })}
    </FormGroup>
  );
}

function Tasks(props: {
  name: string;
  tasks: Task[];
  saved?: { [key: string]: boolean };
  setTasks: React.Dispatch<React.SetStateAction<{ [key: string]: boolean }>>;
}) {
  
  return (
    <FormGroup style={{ paddingLeft: "10vw", paddingRight: "8vw" }}>
      <Row
        style={{
          display: "flex",
          justifyContent: "flex-start",
          alignItems: "center",
          marginBottom: "1vh",
        }}
      >
        <h1 style={{ marginTop: "0px", marginBottom: "0px" }}>{props.name}</h1>
        <div style={{ width: "2vw" }}></div>
        <Timer />
      </Row>
      {RenderTasks(
        props.tasks,
        undefined,
        props.name.split(":")[0].trim().replace(" ", "_"),
        props.saved,
        props.setTasks,
      )}
      <div style={{ height: "3vh" }}></div>
    </FormGroup>
  );
}

function RenderTasks(
  tasks: Task[],
  level = 0,
  id = "",
  saved?: { [key: string]: boolean },
  setTasks?: React.Dispatch<React.SetStateAction<{ [key: string]: boolean }>>,
) {
  const [task_publisher] = useAtom(taskPublisherAtom);

  return tasks.map((task, index) => {
    const currentID = id ? `${id}:${index + 1}` : `${index + 1}`;
    const currentState = saved ? (saved[currentID] ?? false) : false;

    return (
      <div
        key={currentID}
        style={{
          paddingLeft: `${level ** 0.5 * 2}em`,
          paddingBottom: "5px",
        }}
      >
        {!task.subTasks && (
          <FormControlLabel
            control={
              <Checkbox
                key={currentID}
                defaultChecked={task.checked}
                aria-checked={task.checked}
                checked={currentState}
                onChange={() => {
                  setTasks &&
                    setTasks((prevSaved) => ({
                      ...prevSaved,
                      [currentID]: !prevSaved[currentID],
                    }));

                  const message = new ROSLIB.Message({
                    data: JSON.stringify({
                      id: currentID,
                      status: !currentState,
                      sender: "tablet",
                    }),
                  });
                  task_publisher?.publish(message);
                }}
              />
            }
            label={
              <div
                style={{
                  fontSize: `max(calc(1.6vw), 15px)`,
                  fontWeight: "lighter",
                }}
              >
                {task.name}
              </div>
            }
            style={{ fontSize: 20 }}
          />
        )}
        {task.subTasks && (
          <div
            style={{
              paddingLeft: "0px",
              wordBreak: "break-word",
              alignItems: "flex-start",
              fontWeight: "bold",
              fontSize: `max(calc(1.8vw), 17px)`,
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
            currentID,
            saved,
            setTasks
            // task_publisher,
            // saved
          )}
      </div>
    );
  });
}


function MainTimer() {
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
     setTimeout(function () {
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
             } else {
               displayedMilliseconds = ":" + milliseconds;
             }
             if (seconds < 10) {
               displayedSeconds = ":0" + seconds;
             } else {
               displayedSeconds = ":" + seconds;
             }
             if (minutes < 10) {
               displayedMinutes = "0" + minutes;
             } else {
               displayedMinutes = "" + minutes;
             }

             // If 15 minutes have passed within the Main Timer, break all timers
             if (minutes == 15) {
               timeOver = 1;
             }

             // Update and display displayedTime
             displayedTime =
               displayedMinutes + displayedSeconds + displayedMilliseconds;
             document.getElementById("timerDisplay")!.innerHTML = displayedTime;

             // Repeat the Main Timer's function
             repeat();
           }
         }
       }
     });
   }

  return (
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
  );
}

function Timer() {
  const [milliseconds, setMilliseconds] = useState(0);
  const [seconds, setSeconds] = useState(0);
  const [minutes, setMinutes] = useState(0);
  const [timerOn, setTimerOn] = useState(false);
  const [timeOver, setTimeOver] = useState(false);

  useEffect(() => {
    let interval: NodeJS.Timeout;

    if (timerOn && !timeOver) {
      interval = setInterval(() => {
        setMilliseconds((prevMilliseconds) => {
          if (prevMilliseconds >= 99) {
            setSeconds((prevSeconds) => prevSeconds + 1);
            return 0;
          }
          return prevMilliseconds + 1;
        });

        setSeconds((prevSeconds) => {
          if (prevSeconds >= 59) {
            setMinutes((prevMinutes) => prevMinutes + 1);
            return 0;
          }
          return prevSeconds;
        });

        if (minutes >= 15) {
          setTimeOver(true);
          setTimerOn(false);
        }
      }, 10);
    }

    return () => {
      if (interval) {
        clearInterval(interval);
      }
    };
  }, [timerOn, timeOver, minutes]);

  const resetTimer = () => {
    setMilliseconds(0);
    setSeconds(0);
    setMinutes(0);
    setTimeOver(false);
  };

  const formatTime = (time: number) => (time < 10 ? `0${time}` : `${time}`);

  return (
    <Box sx={{ width: [400], textAlign: "left" }}>
      <Stack spacing={2} direction="row">
        <Button
          variant="outlined"
          sx={{ borderColor: green[900], color: green[900] }}
          onClick={() => {
            if (!timeOver) {
              setTimerOn(true);
            }
          }}
        >
          START
        </Button>

        <Button
          variant="outlined"
          sx={{ borderColor: yellow[900], color: yellow[900] }}
          onClick={() => {
            if (!timeOver) {
              setTimerOn(false);
            }
          }}
        >
          PAUSE
        </Button>

        <Box textAlign="center" sx={{ width: [80] }}>
          <Typography component="div" fontSize={20} sx={{ color: grey[400] }}>
            {`${formatTime(minutes)}:${formatTime(seconds)}:${formatTime(
              milliseconds
            )}`}
          </Typography>
        </Box>

        <Button
          variant="outlined"
          sx={{ borderColor: red[900], color: red[900] }}
          onClick={resetTimer}
        >
          RESET
        </Button>
      </Stack>
    </Box>
  );
}

