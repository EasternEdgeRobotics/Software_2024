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

// export {}

export function ControllerApp() {
  const [IPs] = useAtom<string[]>(CameraIPs);

  return (
    <Box>
      <InitROS /> {/* Initialize ROS */}
      <SafetyDisclaimer /> {/* Display the safety disclaimer */}
            // a button to screenshot selected camera
      <Button variant="outlined" onClick={async () => {
        fetch(IPs[0], {
          method: 'GET',
          headers: {
            'Content-Type': 'image/png',
          },
        })
          .then((response) => response.blob())
          .then((blob) => {
            // Create blob link to download
            const url = window.URL.createObjectURL(
              new Blob([blob]),
            );
            const link = document.createElement('a');
            link.href = url;
            link.setAttribute(
              'download',
              `FileName.png`,
            );

            // Append to html link element page
            document.body.appendChild(link);

            // Start download
            link.click();

            // Clean up and remove the link
            // link.parentNode.removeChild(link);
          });
      }}>Screenshot</Button>
    </Box>
  );
}
