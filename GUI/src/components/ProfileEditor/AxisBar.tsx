import { Box, LinearProgress } from "@mui/material";
import { useEffect, useState } from "react";

export default function AxisBar(props: {controllerIndex: number, index: number}) {
	//refresh component every 10 ms if on controller tabs
	const [, setJankSolution] = useState<number>(0);
	useEffect(() => {
		setInterval(() => {
			setJankSolution(Math.random());
		}, 100);
	}, []);


    return (
        <Box display="flex" alignItems="center" marginBottom="4px">
            <LinearProgress variant="determinate" value={(navigator.getGamepads()[props.controllerIndex]?.axes[props.index] ?? 0)*50+50} sx={{width: "100%", marginBottom: "4px"}} />
        </Box>
    );
}