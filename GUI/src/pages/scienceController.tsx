import { ThemeProvider } from "@emotion/react";
import { Box, Button, CssBaseline, createTheme } from "@mui/material";
import { useAtom } from "jotai";
//fonts
import "@fontsource/roboto/300.css";
import "@fontsource/roboto/400.css";
import "@fontsource/roboto/500.css";
import "@fontsource/roboto/700.css";
import SafetyDisclaimer from "../components/SafetyDisclaimer";
import { InitROS } from "../api/ROS";
import { CameraIPs } from "../api/Atoms";
import React, { useRef } from "react";
import html2canvas from "html2canvas";

import '../styles/science.css'
import { AltCamera } from "../components/CameraTab";

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


function ScreenshotView2() {
  const boxRef = useRef(null);
  let lin = "";

  const captureScreenshot = async () => {
    if (boxRef.current) {
      const canvas = await html2canvas(boxRef.current);
      const imgData = canvas.toDataURL();
      lin= imgData
      // Do something with imgData (base64 encoded image)
      window.open(imgData);
    }
  };

  return (
    <Box>
      <InitROS /> {/* Initialize ROS */}
      <Button variant="outlined" onClick={captureScreenshot}>Screenshot</Button>
      <Box ref={boxRef} height={200} sx={{ backgroundImage: `url(${"http://localhost:8880/"}), url(./nosignal.jpg)`, backgroundSize: "100% 100%", borderRadius: "12px" }} />
      
    </Box>
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
    
  return (
    <Box>
      <InitROS /> {/* Initialize ROS */}
      {/* <ScreenshotVeiw/> */}
      {/* <SafetyDisclaimer /> {Display the safety disclaimer} */}
      <Button variant="outlined" onClick={captureScreenshot}>Screenshot</Button>
      {/* <Box height={200} sx={{ backgroundImage: `url(${streamUrl}), url(./nosignal.jpg)`, backgroundSize: "100% 100%", borderRadius: "12px" }} /> */}
      <ScreenshotView2/>
    </Box>
  );
}
