import { Checkbox, IconButton } from "@mui/material";
import { useAtom } from "jotai";
//fonts
import "@fontsource/roboto/300.css";
import "@fontsource/roboto/400.css";
import "@fontsource/roboto/500.css";
import "@fontsource/roboto/700.css";

import { useEffect, useState } from "react";

import "../styles/science.css";
import { Sidebar, Menu, MenuItem } from "react-pro-sidebar";
import { Home } from "@mui/icons-material";
import "../styles/science.css";
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

const redirectToScreenshot = async (urls: string[], i: 0 | 1 | 2 | 3) => {
  //open a new tab with the stream url
  let streamUrl = urls[i];
  streamUrl = streamUrl.replace("/stream", "/snapshot");
  window.open(streamUrl, "_blank");
};

const ScreenshotVeiw = ({ urls }: { urls: string[] }) => {
  //[To-do] handle the case when ROS not connected / urls are not available
  //[To-do] better styling for the components
  console.log(urls, "URLS");
  return (
    <div className="desktop1-rectangle1">
      <div className="desktop1-frame1">
        <button
          type="button"
          className="desktop1-button button"
          onClick={() => redirectToScreenshot(urls, 0)}
        >
          <span className="desktop1-text">
            <span>Camera 1</span>
            <br></br>
          </span>
        </button>
        <button
          type="button"
          className="desktop1-button1 button"
          onClick={() => redirectToScreenshot(urls, 1)}
        >
          <span className="desktop1-text03">
            <span>Camera 2</span>
            <br></br>
          </span>
        </button>
        <button
          type="button"
          className="desktop1-button2 button"
          onClick={() => redirectToScreenshot(urls, 2)}
        >
          <span className="desktop1-text06">
            <span>Camera 3</span>
            <br></br>
          </span>
        </button>
        <button
          type="button"
          className="desktop1-button3 button"
          onClick={() => redirectToScreenshot(urls, 3)}
        >
          <span className="desktop1-text09">
            <span>Camera 4</span>
            <br></br>
          </span>
        </button>
      </div>
      <span className="desktop1-text12">
        <span>Screenshot</span>
      </span>
    </div>
  );
};

//recursive function to render the tasks
//level is used to calculate the padding of the tasks
function renderTasks(tasks: Task[], level = 0) {

  const handleCheckboxChange = (task: Task) => {
    //[To-do] Update the tast status globally with ROS
    //[To-do] Listen and update the task status from ROS
    task.checked = !task.checked;
    const updatedTask = { ...task, checked: !task.checked };
    localStorage.setItem(task.name, JSON.stringify(updatedTask));
  };

  return tasks.map((task: Task) => {
    const Icon = () => {
      if (!task.subTasks && tasks.length > 1)
        return (
          <Checkbox
            style={{ marginBottom: "10px", padding: "0px" }}
            aria-label={task.name}
            defaultChecked={task.checked}
            color="success"
            aria-checked={task.checked}
            onChange={() => handleCheckboxChange(task)}
          />
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
          {task.subTasks && renderTasks(task.subTasks, level + 1)}
        </Col>
      </div>
    );
  });
}

function SubList(props: { name: string; tasks: Task[]; max?: boolean }) {
  const [tasks, setTasks] = useState<Task[]>([]);
  const [score, setScore] = useState<number>(0);
  const [totalScore, setTotalScore] = useState<number>(0);

  useEffect(() => {
    const loadedTasks = props.tasks.map((task: Task) => {
      const savedTask = localStorage.getItem(task.name);
      return savedTask ? JSON.parse(savedTask) : task;
    });
    setTasks(loadedTasks);
  }, [props.tasks]);

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

    setTotalScore(calculateTotalScore(tasks, props.max));
  }, [tasks]);

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
        <div>{renderTasks(tasks)}</div>
      </Row>
    </div>
  );
}

function SideBar() {
  //[To-do] acoount for mobile view
  const [collapsed, setCollapsed] = useState(false);

  const handleToggleSidebar = () => {
    setCollapsed(!collapsed);
  };
  const styles = {
    sideBarHeight: {
      height: "145vh",
    },
    menuIcon: {
      float: "right",
      margin: "10px",
    },
  };

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
        style={styles.sideBarHeight}
        collapsed={collapsed}
        rtl={false}
        width="700px"
        collapsedWidth="0px"
        backgroundColor="rgb(0, 0, 69, 0.7)"
      >
        <Menu>
          <MenuItem icon={<Home />}>Home</MenuItem>
          <SubList
            name="TASK 1 : Coastal Pioneer Array"
            tasks={taskJSON.task1.tasks}
          />
          <SubList
            name="TASK 2 : Deploy SMART cables"
            tasks={taskJSON.task2.tasks}
          />
          <SubList
            name="TASK 3 : From the Red Sea to Tenesse"
            tasks={taskJSON.task3.tasks}
          />
          <SubList
            name="TASK 4 : MATE Floats"
            tasks={taskJSON.task4.tasks}
            max={taskJSON.task4.single}
          />
          {/* More menu items... */}
        </Menu>
      </Sidebar>
    </div>
  );
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
            data: lines[i - 1],
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

  ros.on("error", () => {0}); // to prevent page breaking

  React.useEffect(() => {
    ros.connect(`ws://${RosIP}:9090`);
    setInterval(() => {
      if (!ros.isConnected) {
        setRos(new ROSLIB.Ros({}));
        ros.connect(`ws://${RosIP}:9090`);
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
            <ScreenshotVeiw urls={urls} />
          </div>
          <CSVHandler />
        </Col>
      </Row>
    </>
  );
}