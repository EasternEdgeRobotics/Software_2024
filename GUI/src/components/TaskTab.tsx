import { Box, Button } from "@mui/material";
import { ChangeEvent, useState } from "react";

export default function TaskTab() {
    const [test, setTest] = useState(["", "", ""]);

    const onChange = (event: ChangeEvent<HTMLInputElement>) => {
        const loader = new FileReader();
        console.log(event.target.files)
        loader.onload = function (loadEvent: ProgressEvent<FileReader>) {
            if (loadEvent.target?.error) {
                setTest(["Error", "Error", "Error"]);
            }
            const recievers: [number, number][] = [[0, 0], [0, 0], [0, 0]]
            String(loadEvent.target?.result).split("\n").forEach((line, index) => {
                if (index === 0) return;
                const data = line.split(",");
                if (recievers[0][0] < Number(data[0])) recievers[0] = [Number(data[0]), index];
                if (recievers[1][0] < Number(data[1])) recievers[1] = [Number(data[1]), index];
                if (recievers[2][0] < Number(data[2])) recievers[2] = [Number(data[2]), index]
            });
            setTest([`${recievers[0][0]} (Day: ${recievers[0][1]})`, `${recievers[1][0]} (Day: ${recievers[1][1]})`, `${recievers[2][0]} (Day: ${recievers[2][1]})`])
        }
        loader.readAsText(event.target.files![event.target.files!.length - 1]);
        event.target.value = "";
    }

    return(
        <Box>
            <Button variant="contained" component="label"><input type="file" accept="text/csv" hidden onChange={onChange} />Upload</Button>
            <h1>Reciever 1 Max: {test[0]}</h1>
            <h1>Reciever 2 Max: {test[1]}</h1>
            <h1>Reciever 3 Max: {test[2]}</h1>
        </Box>
    )
}