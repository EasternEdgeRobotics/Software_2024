import { ThemeProvider } from "@emotion/react";
import { Box, Button, Checkbox, CssBaseline, createTheme } from "@mui/material";
import { useAtom } from "jotai";
//fonts
import "@fontsource/roboto/300.css";
import "@fontsource/roboto/400.css";
import "@fontsource/roboto/500.css";
import "@fontsource/roboto/700.css";
import SafetyDisclaimer from "../components/SafetyDisclaimer";
import { InitROS } from "../api/ROS";
import { CameraIPs } from "../api/Atoms";
import React, { useRef, useState } from "react";

import '../styles/science.css'
import { AltCamera } from "../components/CameraTab";
import { Sidebar, Menu, MenuItem } from 'react-pro-sidebar';
import { Home, Brightness4, SwapHoriz, CheckBox } from '@mui/icons-material';
import "../styles/science.css";
import { Col, Row } from 'react-bootstrap';
import taskJSON from './tasks.json';
import {Task, SubTask} from '../types/Task';

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

  )
}

function renderTasks(tasks: Task[], level = 0) {
  return tasks.map((task: Task) => (
    <div style={{ paddingLeft: `${level * 35}px` }}>
      <Col style={{minHeight: "5px"}}>
        <div style={{ display: 'flex',flex:1, minHeight: "2px", alignItems: 'flex-start' }}>
          <Checkbox style={{ marginBottom: "10px" }} aria-label={task.name} defaultChecked color="success" aria-checked={false} />
          <div style={{ marginLeft: '10px', flex: 1, wordBreak: 'break-word', alignItems:"flex-start" }}>{task.name}</div>
        </div>
        {task.subTasks && renderTasks(task.subTasks, level + 1)}
      </Col>
    </div>
  ));
}

function SubList(props:{name: string, tasks: Task[]}) { 
  return (
    <div style={{ paddingLeft: '20px', paddingRight: '20px' , paddingBottom: "10px" }}>
      <Row>
        <h2>{props.name}</h2>
        <div>
          {renderTasks(props.tasks)}
          {/* ... rest of your code ... */}
        </div>
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
      height: "145vh"
    },
    menuIcon: {
      float: "right",
      margin: "10px"
    }
  }

  return (
    <div style={{ display: 'flex', position: 'fixed', right: 0, zIndex: 1000 }}>
      <Button
        onClick={handleToggleSidebar}
        style={{
          position: 'fixed',
          top: '50%',
          left: !collapsed ? '200px' : '0',
          transform: 'translateY(-50%)',
          backgroundColor: '#007bff',
          borderColor: '#007bff',
          color: 'white'
        }}
      >
        {/* {!collapsed ? <FaArrowLeft size={20} /> : <FaArrowRight size={20} />} */}
      </Button>
      <Sidebar style={styles.sideBarHeight} collapsed={collapsed} rtl={false} width="600px" collapsedWidth="0px" backgroundColor="rgb(0, 0, 69, 0.7)">
      <Menu>
          <MenuItem icon={<Home />}>Home</MenuItem>
          <SubList name="TASK 1 : Coastal Pioneer Array" tasks={taskJSON.task1.tasks} />
          <SubList name="TASK 2 : Deploy SMART cables" tasks={taskJSON.task2.tasks} />
          <SubList name="TASK 3 : From the Red Sea to Tenesse" tasks={taskJSON.task3.tasks} />
          <SubList name="TASK 4 : MATE Floats" tasks={taskJSON.task4.tasks} />
          {/* More menu items... */}
          
      </Menu>
      </Sidebar>
    </div>
  );
}

export function ControllerApp() {
  const [IPs] = useAtom<string[]>(CameraIPs);

  const videoRef = useRef<HTMLVideoElement>(null);

  const captureScreenshot = async () => {
    // Fetch the video data from the server
    const response = await fetch('http://localhost:8880/');
    const data = await response.blob();

    // Create a Blob URL from the data
    const url = URL.createObjectURL(data);

    // Create a hidden video element
    const video = document.createElement('video');

    // Set the source to the Blob URL
    video.src = url;

    // Wait for the video to load metadata (dimensions)
    await video.play();

    // Create a canvas and draw the current video frame onto it
    const canvas = document.createElement('canvas');
    canvas.width = video.videoWidth;
    canvas.height = video.videoHeight;
    canvas.getContext('2d')?.drawImage(video, 0, 0);

    // Use toBlob method to create blob link to download
    canvas.toBlob((blob) => {
      if (!blob) return console.error('Failed to capture canvas to blob');
      const url = URL.createObjectURL(blob);
      const link = document.createElement('a');
      link.href = url;
      link.setAttribute('download ', 'screenshot.png');

      // Append the link to the document body and click it to start the download
      document.body.appendChild(link);
      link.click();

      // Clean up by removing the link from the body and revoking the blob URL
      document.body.removeChild(link);
      URL.revokeObjectURL(url);
    }, 'image/png');
  };

  

    const streamUrl = 'http://localhost:8880/';
  const styles = {
    contentDiv: {
      display: "flex"
    },
    contentMargin: {
      marginLeft: "10px",
      width: "100%"
    },
    
  };
  
  return (
    <>
      <div style={{ ...styles.contentDiv, position: 'fixed', right: 0, zIndex: 1000 }}>
        <SideBar />
      </div>

      <Row>
        <Col lg={3}>
          <div style={styles.contentDiv}>
            <ScreenshotVeiw />
          </div>
        </Col>
      </Row>
    </>
  );

}
