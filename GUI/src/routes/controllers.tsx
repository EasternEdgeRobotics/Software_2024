import { useAtom } from "jotai";
import { CurrentController } from "../components/Atoms";
import { Box, FormControl, InputLabel, MenuItem, Select } from "@mui/material";

export default function ControllerTab() {
  const [currentController, setCurrentController] = useAtom(CurrentController);

  return (
    <Box display="flex" justifyContent="center">
      <FormControl sx={{ width: "40%" }}>
        {/* Currently just basic controller select option, becomes red if there is no controller selected */}
        {/* This also sends warnings to the browser every time it loads, nothings broken it's becuase the default option is -1 which doesnt exist */}
        <InputLabel id="controller-select-label">Controller</InputLabel>
        <Select
          labelId="controller-select-label"
          id="controller-select"
          value={currentController}
          label="Controller"
          onChange={(e) => setCurrentController(e.target.value as number)}
          error={currentController === -1}
        >
          {navigator.getGamepads().length > 0 &&
            navigator.getGamepads().map((gamepad) => {
              //Add menu item for every controller
              if (gamepad !== null) {
                return (
                  <MenuItem value={gamepad!.index}>{gamepad!.id}</MenuItem>
                );
              }
            })}
          {navigator.getGamepads().length <= 0 && (
            <MenuItem value={-1}>No Controllers Detected!</MenuItem>
          )}
        </Select>
      </FormControl>
    </Box>
  );
}
