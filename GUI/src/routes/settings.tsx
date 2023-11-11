import { useAtom } from "jotai";
import { CameraIPs, ROSIP } from "../components/Atoms";
import { Box } from "@mui/system";
import { Button, Divider, Grid, TextField } from "@mui/material";

export default function SettingsTab() {
  const [IPs, setIPs] = useAtom(CameraIPs);
  const [RosIP, setRosIP] = useAtom(ROSIP);

  const setCameraIP = (camera: number, ip: string) => {
    setIPs(IPs.map((item, index) => (index === camera ? ip : item)));
  };

  const save = () => {
    setIPs(
      IPs.map((ip) =>
        !ip.startsWith("http://") && !ip.startsWith("https://")
          ? "http://" + ip
          : ip
      )
    );
    let settings: any = {};
    settings.CameraIPs = IPs.map((ip) =>
      !ip.startsWith("http://") && !ip.startsWith("https://")
        ? "http://" + ip
        : ip
    );
    fetch("/config", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(settings),
    });
    localStorage.setItem("ROS_IP", RosIP);
  };

  return (
    <Box>
      <Divider>ROS</Divider>
      <br />
      <Box display="flex" justifyContent="center">
        <TextField
          label="ROS Bridge IP (Must Refresh After Changing)"
          variant="outlined"
          sx={{ width: "40%" }}
          value={RosIP}
          onChange={(e) => setRosIP(e.target.value)}
        />
      </Box>
      <br />
      <Divider>Cameras</Divider>
      <br />
      <Grid container>
        <Grid item xs={1 / 2} />
        <Grid item xs={10 / 3}>
          <TextField
            label="Camera 1 URL"
            variant="outlined"
            sx={{ width: "100%" }}
            value={IPs[0]}
            onChange={(e) => setCameraIP(0, e.target.value)}
          />
        </Grid>
        <Grid item xs={1 / 2} />
        <Grid item xs={10 / 3}>
          <TextField
            label="Camera 2 URL"
            variant="outlined"
            sx={{ width: "100%" }}
            value={IPs[1]}
            onChange={(e) => setCameraIP(1, e.target.value)}
          />
        </Grid>
        <Grid item xs={1 / 2} />
        <Grid item xs={10 / 3}>
          <TextField
            label="Camera 2 URL"
            variant="outlined"
            sx={{ width: "100%" }}
            value={IPs[2]}
            onChange={(e) => setCameraIP(2, e.target.value)}
          />
        </Grid>
        <Grid item xs={1 / 2} />
      </Grid>
      <Box position="absolute" bottom="8px" left="10%" width="80%">
        <Button
          variant="contained"
          sx={{ width: "100%" }}
          onClick={() => {
            save();
          }}
        >
          Save Settings
        </Button>
      </Box>
    </Box>
  );
}
