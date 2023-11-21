import {
  Button,
  Dialog,
  DialogActions,
  DialogContent,
  DialogContentText,
  DialogTitle,
} from "@mui/material";
import { useState } from "react";

export default function SafetyDisclaimer() {
  const [open, setOpen] = useState(true);

  return (
    <Dialog open={open}>
      <DialogTitle>Eastern Edge Safety Disclaimer</DialogTitle>
      <DialogContent>
        <DialogContentText component="span">
          Prior to using the Eastern Edge ROV control interface, please ensure
          the following:
          <br />
          <ul>
            {" "}
            {/* CHANGE THESE */}
            <li>You will not throw the ROV.</li>
            <li>You will not eat the ROV.</li>
            <li>You will not yell at the ROV.</li>
            <li>You will not kick the ROV.</li>
            <li>You will not swing the ROV around by the tether.</li>
            <li>You will not battle other ROVs with the ROV.</li>
          </ul>
          After reading this, press the "I Agree" button to continue.
        </DialogContentText>
      </DialogContent>
      <DialogActions>
        <Button onClick={() => setOpen(false)}>I Agree</Button>
      </DialogActions>
    </Dialog>
  );
}
