import { Box, Grid } from "@mui/material";
import { useEffect, useState } from "react";

function TaskTab() {
    const [width, setWidth] = useState(window.innerWidth);
    useEffect(() => {
        const updateWidth = () => {setWidth(window.innerWidth);}
        window.addEventListener("resize", updateWidth);
    }, []);

    return(
        <Grid container spacing={2}>
            <Grid item sx={8}>
                <Box height={(width) * (3/8)} sx={{backgroundColor: "#323232", backgroundSize: "100% 100%", borderRadius: "12px"}}/>
            </Grid>
            <h1>probably image display</h1>
            <h1>buttons for taking an image</h1>
            <h1>maybe buttons on the side to perform each task</h1>
            <h1>this is a low priority we dont even know the tasks</h1>
        </Grid>
    )
}

export default TaskTab;