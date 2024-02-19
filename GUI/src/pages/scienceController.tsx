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
import React, { useRef } from 'react';
// export {}
const fetch = require('node-fetch');
const { createCanvas, Image } = require('canvas');
const { JSDOM } = require('jsdom');

export function ControllerApp() {
  const [IPs] = useAtom<string[]>(CameraIPs);

  const videoRef = useRef<HTMLVideoElement>(null);

  const captureScreenshot = async () => {
    // Fetch the video data from the server
    const response = await fetch('http://localhost:8880/');
    const data = await response.buffer();

    // Create an Image object from the video data
    const img = new Image();
    img.src = data;

    // Create a canvas and draw the image onto it
    const canvas = createCanvas(img.width, img.height);
    const ctx = canvas.getContext('2d');
    ctx.drawImage(img, 0, 0, img.width, img.height);
  };

  const streamUrl = 'http://localhost:8880/';
  return (
    <Box>
      <InitROS /> {/* Initialize ROS */}
      {/* <SafetyDisclaimer /> {Display the safety disclaimer} */}
      <Button variant="outlined" onClick={captureScreenshot}>Screenshot</Button>
    </Box>
  );
}
