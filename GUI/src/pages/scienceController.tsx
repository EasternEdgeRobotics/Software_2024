import { Checkbox, IconButton } from "@mui/material";
import { useAtom } from "jotai";
//fonts
import "@fontsource/roboto/300.css";
import "@fontsource/roboto/400.css";
import "@fontsource/roboto/500.css";
import "@fontsource/roboto/700.css";
import { CameraURLs } from "../api/Atoms";

import { useEffect, useRef, useState } from "react";

import "../styles/science.css";
import { Sidebar, Menu, MenuItem } from "react-pro-sidebar";
import { Home } from "@mui/icons-material";
import "../styles/science.css";
import { Col, Row } from "react-bootstrap";
import taskJSON from "./tasks.json";
import { Task } from "../types/Task";
import { MenuSquareIcon } from "lucide-react";
import SquareIcon from "@mui/icons-material/Square";

import { Chart, Line, ChartProps, Bar } from "react-chartjs-2";
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
  ChartType,
  BarElement,
} from "chart.js";

const ScreenshotVeiw = () => {
  return (
    <div className="desktop1-rectangle1">
      <div className="desktop1-frame1">
        <button type="button" className="desktop1-button button">
          <span className="desktop1-text">
            <span>Camera 1</span>
            <br></br>
          </span>
        </button>
        <button type="button" className="desktop1-button1 button">
          <span className="desktop1-text03">
            <span>Camera 2</span>
            <br></br>
          </span>
        </button>
        <button type="button" className="desktop1-button2 button">
          <span className="desktop1-text06">
            <span>Camera 3</span>
            <br></br>
          </span>
        </button>
        <button type="button" className="desktop1-button3 button">
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

function renderTasks(tasks: Task[], level = 0) {
  // if (!(level >= 1 || tasks.length > 2)) level = level - 1;

  const handleCheckboxChange = (task: Task) => {
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

function SubList(props: { name: string; tasks: Task[] }) {
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
    // const calculateScore = (tasks: Task[]): number => {
    //   return tasks.reduce((total, task) => total + (task.checked ? task.points : 0) + (task.subTasks ? calculateScore(task.subTasks) : 0), 0);
    // };

    const calculateTotalScore = (tasks: Task[]): number => {
      let total = 0;
      tasks.forEach((subtask) => {
        if (subtask.subTasks) {
          total += calculateTotalScore(subtask.subTasks);
        } else {
          total += subtask.points ?? 0;
        }
      });

      return total;
    };

    // setScore(calculateScore(tasks));
    setTotalScore(calculateTotalScore(tasks));
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
          <SubList name="TASK 4 : MATE Floats" tasks={taskJSON.task4.tasks} />
          {/* More menu items... */}
        </Menu>
      </Sidebar>
    </div>
  );
}

function CSVHandler() {
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
        const contents = e.target.result as string;
        const lines = contents.split("\n");
        const header = lines[0].split(",");

        const datasets: ChartData["datasets"] = [];
        const dataLists: { [key: string]: number }[] = [];
        const labels: string[] = [];

        for (let i = 1; i < lines.length; i++) {
          const line = lines[i].split(",");
          const item: { [key: string]: number } = {};
          labels.push(i.toString());

          for (let j = 0; j < header.length; j++) {
            const key = header[j]?.trim() ?? "key ??";
            const value = Number(line[j]?.trim()) ?? 0;
            item[key] = Number(value);
          }
          dataLists.push(item);
        }

        const colors = [
          "rgb(0, 143, 136)",
          "rgb(245, 200, 0)",
          "rgb(241, 98, 8)",
        ];

        Object.keys(dataLists[0]).forEach((key, index) => {
          const color = colors[index % colors.length];
          datasets.push({
            label: key,
            data: dataLists.map((item) => item[key]),
            borderColor: `${color.slice(0, -1)}, 0.75)`,
            backgroundColor: `${color.slice(0, -1)}, 0.95)`,
          });
        });
        console.log("Parsed Data:", dataLists);
        console.log("Data sets:", datasets);

        const data: ChartData = {
          labels, //parsedData.map(item => item.x), // Replace 'x' with the actual property name for the x-axis
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
  const [IPs] = useAtom<string[]>(CameraURLs);

  const videoRef = useRef<HTMLVideoElement>(null);

  const captureScreenshot = async () => {
    // Fetch the video data from the server
    const response = await fetch("http://localhost:8880/");
    const data = await response.blob();

    // Create a Blob URL from the data
    const url = URL.createObjectURL(data);

    // Create a hidden video element
    const video = document.createElement("video");

    // Set the source to the Blob URL
    video.src = url;

    // Wait for the video to load metadata (dimensions)
    await video.play();

    // Create a canvas and draw the current video frame onto it
    const canvas = document.createElement("canvas");
    canvas.width = video.videoWidth;
    canvas.height = video.videoHeight;
    canvas.getContext("2d")?.drawImage(video, 0, 0);

    // Use toBlob method to create blob link to download
    canvas.toBlob((blob) => {
      if (!blob) return console.error("Failed to capture canvas to blob");
      const url = URL.createObjectURL(blob);
      const link = document.createElement("a");
      link.href = url;
      link.setAttribute("download ", "screenshot.png");

      // Append the link to the document body and click it to start the download
      document.body.appendChild(link);
      link.click();

      // Clean up by removing the link from the body and revoking the blob URL
      document.body.removeChild(link);
      URL.revokeObjectURL(url);
    }, "image/png");
  };

  const streamUrl = "http://localhost:8880/";
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
            <ScreenshotVeiw />
          </div>
          <CSVHandler />
        </Col>
      </Row>
    </>
  );
}
