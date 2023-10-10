import { Box, Divider, Grid, TextField } from "@mui/material";

function Settings() {
    return (
        <Box>
            <Divider>Cameras</Divider>
            <br/>
            <Grid container>
                <Grid item xs={1/2} />
                <Grid item xs={10/3}>
                    <TextField id="camera-1-url" label="Camera 1 URL" variant="outlined" sx={{width: "100%"}}/>
                </Grid>
                <Grid item xs={1/2} />
                <Grid item xs={10/3}>
                    <TextField id="camera-2-url" label="Camera 2 URL" variant="outlined" sx={{width: "100%"}}/>
                </Grid>
                <Grid item xs={1/2} />
                <Grid item xs={10/3}>
                    <TextField id="camera-3-url" label="Camera 3 URL" variant="outlined" sx={{width: "100%"}}/>
                </Grid>
                <Grid item xs={1/2} />
            </Grid>
        </Box>
    )
}

export default Settings;