import React from 'react';
import { Button, Dialog, DialogTitle, DialogContent, DialogContentText, DialogActions } from '@mui/material';

function SafetyDisclaimer() {
    const [open, setOpen] = React.useState(true);

    const handleClose = () => {setOpen(false);}

    return (
        <Dialog
            open={open}
            aria-labelledby="alert-dialog-title"
            aria-describedby="alert-dialog-description"
        >
        <DialogTitle id="alert-dialog-title">Eastern Edge Safety Disclaimer</DialogTitle>
        <DialogContent>
          <DialogContentText id="alert-dialog-description">
          Prior to using the Eastern Edge ROV control interface, please ensure the following:<br/>
            <ul> {/* CHANGE THESE */}
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
          <Button onClick={handleClose}>I Agree</Button>
        </DialogActions>
      </Dialog>
    );
}

export default SafetyDisclaimer;