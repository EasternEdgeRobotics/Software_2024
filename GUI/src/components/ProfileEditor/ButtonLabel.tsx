import { InputLabel } from "@mui/material";
import { useEffect, useState } from "react";

export default function ButtonLabel(props: {controller: Gamepad | null, index: number}) {
	//refresh component every 10 ms if on controller tabs
	const [, setJankSolution] = useState<number>(0);
	useEffect(() => {
		setInterval(() => {
			setJankSolution(Math.random());
		}, 100);
	}, []);

    return (
        <InputLabel sx={{color: (props.controller?.buttons[props.index].pressed ?? false) ? "#87CEEB" : "light-grey"}}>Button {props.index}</InputLabel>
    );
}