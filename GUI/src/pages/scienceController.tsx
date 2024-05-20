import { Checkbox, IconButton, Button, Box, Typography } from "@mui/material";
import { atom, useAtom } from "jotai";
//fonts
import "@fontsource/roboto/300.css";
import "@fontsource/roboto/400.css";
import "@fontsource/roboto/500.css";
import "@fontsource/roboto/700.css";

import { useEffect, useState } from "react";

import { Sidebar, Menu, MenuItem } from "react-pro-sidebar";
import { Home } from "@mui/icons-material";
import { Col, Row } from "react-bootstrap";
import taskJSON from "./tasks.json";
import { Task } from "../types/Task";
import { MenuSquareIcon } from "lucide-react";
import SquareIcon from "@mui/icons-material/Square";

import { Line, Bar } from "react-chartjs-2";
import {
  Chart as ChartJS,
  CategoryScale,
  LinearScale,
  PointElement,
  LineElement,
  Title,
  Tooltip,
  Legend,
  ChartData,
  BarElement,
} from "chart.js";
import ROSLIB, { Ros } from "roslib";
import React from "react";
import { ROSIP } from "../api/Atoms";
const taskPublisherAtom = atom<ROSLIB.Topic<ROSLIB.Message> | null>(null);

const redirectToScreenshot = async (urls: string[], i: 0 | 1 | 2 | 3) => {
  //open a new tab with the stream url
  let streamUrl = urls[i];
  streamUrl = streamUrl.replace("/stream", "/snapshot");
  if (!streamUrl) {
    alert("The selected camera is not connected.");
    return;
  }
  window.open(streamUrl, "_blank");
};

const ScreenshotView = ({ urls }: { urls: string[] }) => {
  if (urls.length === 0) urls = ["", "", "", ""];
  for (let i = 0; i < 4; i++) {
    if (!urls[i]) urls[i] = "";
  }
  return (
    <Box
      sx={{
        width: "68vw",
        minWidth: 350,
        height: 159,
        display: "flex",
        position: "relative",
        alignItems: "flex-start",
        borderRadius: 8,
        backgroundColor: "#D9D9D9",
        padding: "5px 14px",
      }}
    >
      <Box
        sx={{
          gap: 3,
          flex: 0,
          width: "65vw",
          // minWidth: 320,
          height: 128,
          display: "flex",
          opacity: 0.9,
          padding: "0 14px",
          zIndex: 100,
          alignSelf: "flex-end",
          alignItems: "center",
          borderRadius: 3,
          justifyContent: "center",
          backgroundColor: "rgba(208, 208, 208, 0)",
          position: "relative", // add this line
        }}
      >
        {urls.map((url, index) => (
          <Button
            sx={{
              color: "#e8e8e8",
              width: "20vw", // adjust this value as needed
              height: 60, // adjust this value as needed
              borderRadius: 3.5, // adjust this value as needed
              backgroundColor: "#252525",
            }}
            onClick={() => redirectToScreenshot(urls, index as 0 | 1 | 2 | 3)}
          >
            <Typography
              sx={{
                fontSize: 20,
                fontStyle: "normal",
                fontWeight: 600,
              }}
            >{`Camera ${index + 1}`}</Typography>
          </Button>
        ))}
        <Typography
          sx={{
            position: "absolute",
            top: 0,
            bottom: 10,
            marginTop: "-10px",
            color: "rgba(0, 0, 0, 1)",
            fontSize: 28,
            fontStyle: "bold",
            textAlign: "center",
            fontFamily: "Inter",
            fontWeight: 700,
            textDecoration: "none",
          }}
        >
          Screenshot
        </Typography>
      </Box>
    </Box>
  );
};

function SideBar() {
  //[To-do] acoount for mobile view
  const [collapsed, setCollapsed] = useState(false);
  const [RosIP] = useAtom(ROSIP);
  const [ros, setRos] = React.useState<Ros>(new ROSLIB.Ros({}));
  const [saved_tasks, setTasks] = useState<{ [key: string]: boolean }>({});
  const [, setTaskPublisher] = useAtom(taskPublisherAtom);
  const [score, setScore] = useState<number>(0);

  useEffect(() => {
    const rosInstance = new ROSLIB.Ros({});
    rosInstance.connect(`ws://${RosIP}:9090`);
    setRos(rosInstance);
  }, [RosIP]);

  const taskClient = new ROSLIB.Service({
    ros: ros,
    name: "/all_tasks",
    serviceType: "eer_messages/Config",
  });
  const request = new ROSLIB.ServiceRequest({});

  useEffect(() => {
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
      if (data.sender === "science") return;

      setTasks((prevSaved) => ({
        ...prevSaved,
        [data.id]: data.status,
      }));
    });

    setTaskPublisher(task_publisher);
  }, [ros, collapsed]);

    useEffect(() => {
      setScore(
        calculateAchivedScore(
          saved_tasks)
      );
    }, [saved_tasks]);

  ros.on("connection", () => console.log("Connected to ROS"));

  const handleToggleSidebar = () => {
    setCollapsed(!collapsed);
    // task_publisher.publish(message);
  };

  const styles = {
    sideBarHeight: {
      height: "100vh",
      overflow: "auto",
    },
    menuIcon: {
      float: "right",
      margin: "10px",
    },
  };

  // if (loading) {
  //   return <div>Loading...</div>; // Display a loading message while the callService method is running
  // }

  return (
    <div style={{ display: "flex", position: "fixed", right: 0, zIndex: 1000 }}>
      <IconButton
        onClick={handleToggleSidebar}
        style={{
          position: "fixed",
          top: "50%",
          right: !collapsed ? "700px" : "0",
          transform: "translateY(-50%) rotate(180deg)",
          backgroundColor: "#e0e0e0",
          color: "black",
          transition: "right 300ms",
          borderRadius: "5px 50px 50px 5px",
          padding: "10px",
          paddingTop: "30px",
          paddingBottom: "30px",
          paddingLeft: "5px",
          writingMode: "vertical-rl",
          textOrientation: "mixed",
        }}
      >
        Tasks
        <div style={{ height: "5px" }} />
        <MenuSquareIcon />
      </IconButton>
      <Sidebar
        style={{
          ...styles.sideBarHeight,
          backdropFilter: "blur(10px)",
        }}
        collapsed={collapsed}
        rtl={false}
        width="700px"
        collapsedWidth="0px"
        backgroundColor="rgb(0, 0, 60, 0.8)"
      >
        <Menu>
          <MenuItem>
            <Box
              sx={{
                marginTop: "25px",
                display: "inline-block",
                backgroundColor: "rgba(20, 255, 255, 0.4)", // white, 50% transparent
                borderRadius: "10px", // rounded edges
                padding: "10px", // some padding
              }}
            >
              <Typography
                sx={{
                  fontSize: 30,
                  fontStyle: "normal",
                  fontWeight: 600,
                }}
              >
                Total: {score}/{350}
              </Typography>
            </Box>
          </MenuItem>
          <SubList
            name="TASK 1 : Coastal Pioneer Array"
            tasks={taskJSON.task1.tasks}
            saved={saved_tasks}
            setTasks={setTasks}
          />
          <SubList
            name="TASK 2 : Deploy SMART cables"
            tasks={taskJSON.task2.tasks}
            saved={saved_tasks}
            setTasks={setTasks}
          />
          <SubList
            name="TASK 3 : From the Red Sea to Tenesse"
            tasks={taskJSON.task3.tasks}
            saved={saved_tasks}
            setTasks={setTasks}
          />
          <SubList
            name="TASK 4 : MATE Floats"
            tasks={taskJSON.task4.tasks}
            max={taskJSON.task4.single}
            saved={saved_tasks}
            setTasks={setTasks}
          />
          {/* More menu items... */}
        </Menu>
      </Sidebar>
    </div>
  );
}

function SubList(props: {
  name: string;
  tasks: Task[];
  max?: boolean;
  saved: { [key: string]: boolean };
  setTasks: React.Dispatch<React.SetStateAction<{ [key: string]: boolean }>>;
}) {
  const [score, setScore] = useState<number>(0);
  const [totalScore, setTotalScore] = useState<number>(0);
  const [task_publisher] = useAtom(taskPublisherAtom);

  useEffect(() => {
    //recursive function to calculate the total score of the tasks
    //if max is true, return the maximum score of the tasks [used for when only 1 task needs to be completed]
    const calculateTotalScore = (tasks: Task[], max = false): number => {
      if (max)
        return Math.max(
          ...tasks.map(
            (t) => t.points ?? calculateTotalScore(t.subTasks!, t.single)
          )
        );

      let total = 0;
      tasks.forEach((subtask) => {
        if (subtask.single)
          console.log("subtask", subtask, subtask.points ?? 0, subtask.single);
        if (subtask.subTasks) {
          total += calculateTotalScore(
            subtask.subTasks,
            subtask.single ?? false
          );
        } else {
          total += subtask.points ?? 0;
        }
      });

      return total;
    };

    setTotalScore(calculateTotalScore(props.tasks, props.max));
  }, [props.tasks]);

  useEffect(() => {
    setScore(
      calculateAchivedScore(
        props.saved,
        props.name.split(":")[0].trim().replace(" ", "_") as
          | "TASK_1"
          | "TASK_2"
          | "TASK_3"
          | "TASK_4"
      )
    );
  }, [props.saved]);

  return (
    <div
      style={{
        paddingLeft: "20px",
        paddingRight: "20px",
        paddingBottom: "10px",
      }}
    >
      <Row>
        <h2>
          {props.name} ({score}/{totalScore})
        </h2>
        <div>
          {renderTasks(
            props.tasks,
            undefined,
            props.name.split(":")[0].trim().replace(" ", "_"),
            // props.publisher,
            task_publisher!,
            props.saved,
            props.setTasks
          )}
        </div>
      </Row>
    </div>
  );
}

//recursive function to render the tasks
//level is used to calculate the padding of the tasks
function renderTasks(
  tasks: Task[],
  level = 0,
  id = "",
  publisher: ROSLIB.Topic,
  saved?: { [key: string]: boolean },
  setTasks?: React.Dispatch<React.SetStateAction<{ [key: string]: boolean }>>
) {
  const handleCheckboxChange = (task: Task, taskID: string) => {
    task.checked = !task.checked;
    console.log("Task checked:", taskID, task.checked);
    getTaskFromID(taskID);

    setTasks &&
      setTasks((prevSaved) => ({
        ...prevSaved,
        [taskID]: !prevSaved[taskID],
      }));

    const message = new ROSLIB.Message({
      data: JSON.stringify({
        id: taskID,
        status: task.checked,
        sender: "science",
      }),
    });
    publisher.publish(message);
  };

  return tasks.map((task: Task, index) => {
    const currentID = id ? `${id}:${index + 1}` : `${index + 1}`;
    const currentState = saved ? saved[currentID] ?? false : false;

    const Icon = () => {
      if (saved && saved[currentID] === true) task.checked = true;
      else task.checked = false;

      if (!task.subTasks && tasks.length > 1)
        return (
          <div>
            <Checkbox
              style={{ marginBottom: "10px", padding: "0px" }}
              aria-label={task.name}
              defaultChecked={currentState}
              color="success"
              aria-checked={currentState}
              checked={currentState}
              onChange={() => handleCheckboxChange(task, currentID)}
              key={currentID}
            />
          </div>
        );
      else if (task.subTasks)
        return (
          <SquareIcon
            style={{ marginBottom: "10px", padding: "0px" }}
            scale={0.5}
          />
        );
      else return <a></a>;
    };

    return (
      <div style={{ paddingLeft: `${level ** 0.5 * 30}px` }}>
        <Col style={{ minHeight: "45px" }}>
          <div
            style={{
              display: "flex",
              flex: 1,
              minHeight: "2px",
              alignItems: "flex-start",
            }}
          >
            <Icon />
            {!task.subTasks && (
              <div
                style={{
                  marginLeft: "10px",
                  flex: 1,
                  wordBreak: "break-word",
                  alignItems: "flex-start",
                }}
              >
                {task.name} ({task.points})
              </div>
            )}
            {task.subTasks && (
              <div
                style={{
                  marginLeft: "10px",
                  flex: 1,
                  wordBreak: "break-word",
                  alignItems: "flex-start",
                  fontWeight: "bolder",
                  paddingBottom: "10px",
                }}
              >
                {task.name} :
              </div>
            )}
          </div>
          {task.subTasks &&
            renderTasks(
              task.subTasks,
              level + 1,
              currentID,
              publisher,
              saved,
              setTasks
            )}
        </Col>
      </div>
    );
  });
}

//
function CSVHandler() {
  //[To-do] handle a CSV fike with title Row
  //[To-do] formatting and layout
  ChartJS.register(
    CategoryScale,
    LinearScale,
    PointElement,
    LineElement,
    Title,
    Tooltip,
    Legend,
    BarElement
  );
  const [chartData, setChartData] = useState<ChartData>({
    labels: [],
    datasets: [],
  });
  const [fileSelected, setFileSelected] = useState(false);

  const options = {
    responsive: true,
    plugins: {
      legend: {
        position: "top" as const,
      },
      title: {
        display: true,
        text: "Number of Sturgeon detected at each receiver over time",
      },
    },
  };

  const handleFileChange = (event: React.ChangeEvent<HTMLInputElement>) => {
    if (!event.target.files) return;
    console.log("event:", event.target.files[0]);
    const file = event.target.files[0];
    const reader = new FileReader();

    reader.onload = (e) => {
      if (e.target) {
        // Parsing CSV file contents.
        const contents = e.target.result as string;
        const headers = contents.split("\n").map((line) => line.split(",")[0]);
        const lines = contents
          .split("\n")
          .map((line) => line.split(",").slice(1).map(Number));

        for (let i = 0; i < lines.length; i++) {
          if (lines[i].length === 0 || headers[i].length === 0) {
            lines.splice(i, 1);
            headers.splice(i, 1);
            i--;
          }
        }
        console.log("Headers:", headers, "Lines:", lines);

        const datasets: ChartData["datasets"] = [];
        let labels: string[] = [];

        const colors = [
          "rgb(0, 143, 136)",
          "rgb(245, 200, 0)",
          "rgb(241, 98, 8)",
        ];

        // Looping through headers to create datasets
        for (let i = 1; i < headers.length; i++) {
          const key = headers[i]?.trim() ?? "key ??";

          const color = colors[i % colors.length];

          datasets.push({
            label: key,
            data: lines[i],
            borderColor: `${color.slice(0, -1)}, 0.75)`,
            backgroundColor: `${color.slice(0, -1)}, 0.95)`,
          });
        }
        labels = lines[0].map((item) => item.toString());

        console.log("Data sets:", datasets, "Labels:", labels);

        const data: ChartData = {
          labels,
          datasets,
        };

        setChartData(data);
      }
    };

    if (file) {
      reader.readAsText(file);
      setFileSelected(true);
    }
  };

  return (
    <div>
      <input type="file" accept=".csv" onChange={handleFileChange} />
      <div
        style={{
          backgroundColor: "rgba(225, 225, 225, 0.8)",
          margin: "3vh",
          padding: "10px",
          borderRadius: "10px",
          backdropFilter: "blur(10px)",
          WebkitBackdropFilter: "blur(10px)",
          border: "1px solid rgba(255, 255, 255, 0.18)",
          // height: chartData.datasets.length > 0 ? undefined : "150px",
        }}
      >
        <Line data={chartData as any} options={options} />
      </div>
      <div
        style={{
          backgroundColor: "rgba(225, 225, 225, 0.8)",
          margin: "3vh",
          padding: "10px",
          borderRadius: "10px",
          backdropFilter: "blur(10px)",
          WebkitBackdropFilter: "blur(10px)",
          border: "1px solid rgba(255, 255, 255, 0.18)",
        }}
      >
        <Bar data={chartData as any} options={options} />
      </div>
    </div>
  );
}

export function ControllerApp() {
  let camUrls: string[] = [];
  const [ros, setRos] = React.useState<Ros>(new ROSLIB.Ros({}));
  const [RosIP] = useAtom(ROSIP);
  const [urls, setURLs] = React.useState<string[]>([]);
  const [rosConnected, setRosConnected] = React.useState(true);

  ros.on("error", () => {
   setRosConnected(false);
  }); // to prevent page breaking

  React.useEffect(() => {
    ros.connect(`ws://${RosIP}:9090`);
    setInterval(() => {
      if (!ros.isConnected) {
        setRos(new ROSLIB.Ros({}));
        ros.connect(`ws://${RosIP}:9090`);
      } else {
        setRosConnected(true);
      }
    }, 1000);
  }, []);

  // Create a ROS service on the "/camera_urls" topic, using a custom EER service type (see eer_messages folder in ROS2/colcon_ws/src)
  const cameraURLsClient = new ROSLIB.Service({
    ros: ros,
    name: "/camera_urls",
    serviceType: "eer_messages/Config",
  });

  if (urls.length == 0) {
    const request = new ROSLIB.ServiceRequest({
      state: 1,
      data: "FetchCameraURLs",
    }); // What's in the data field is not important in this case
    cameraURLsClient.callService(request, function (result) {
      try {
        if (result.result.trim() != "[]") {
          // Trim spaces before comparison
          camUrls = JSON.parse(result.result);
          setURLs(camUrls);
          //   console.log("camera_urls", camUrls);
        } else {
          console.log("No camera URLs stored in database");
        }
      } catch (error) {
        console.error("Error parsing result:", error); // Log parsing errors
      }
    });
  }

  console.log("URLS", urls);

  const styles = {
    contentDiv: {
      display: "flex",
    },
    contentMargin: {
      marginLeft: "10px",
      width: "100%",
    },
  };

  return (
    <>
      {!rosConnected && ( // Add this block
        <div
          style={{
            backgroundColor: "red",
            color: "white",
            textAlign: "center",
          }}
        >
          ROS not connected
        </div>
      )}`
      <div
        style={{
          ...styles.contentDiv,
          position: "fixed",
          right: 0,
          zIndex: 1000,
        }}
      >
        <SideBar />
      </div>

      <Row className="justify-content-center">
        <Col lg={3}>
          <div style={styles.contentDiv}>
            <div style={{ display: "flex", justifyContent: "center" }}>
              <ScreenshotView urls={urls} />
            </div>
          </div>
          <CSVHandler />
        </Col>
      </Row>
    </>
  );
}

function getTaskFromID(
  id: string,
  allTasks: { [key: string]: { name: string; tasks: Task[] } } = taskJSON
) {
  0;
  const indexlist = id
    .split(":")
    .slice(1)
    .map((a) => Number(a) - 1);
  const parentTask = id.split(":")[0].replace("_", "").toLowerCase();
  const mainTasks = allTasks[parentTask].tasks;

  let tempTask: Task = mainTasks[indexlist[0]];
  for (let i = 1; i < indexlist.length; i++) {
    const index = indexlist[i];
    tempTask = tempTask.subTasks![index];
  }
  return tempTask;
}

function calculateAchivedScore(
  stored: { [key: string]: boolean },
  filter?: "TASK_1" | "TASK_2" | "TASK_3" | "TASK_4",
  allTasks: { [key: string]: { name: string; tasks: Task[] } } = taskJSON
) {
  let score = 0;
  const keys = Object.keys(stored);
  for (let i = 0; i < keys.length; i++) {
    const key = keys[i];

    if (filter && !key.includes(filter)) continue;

    if (Object.prototype.hasOwnProperty.call(stored, key)) {
      const task = getTaskFromID(key, allTasks);
      if (stored[key]) score += task?.points ?? 0;
    }
  }
  console.log("our Score:", score);
  return score;
}
