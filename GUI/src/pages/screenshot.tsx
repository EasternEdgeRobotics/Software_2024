import { Box } from "@mui/material";
import React, { useRef, useState } from "react";
import { EyeDropper, OnChangeEyedrop } from "react-eyedrop";

export function SCColorPicker() {
  const [color, setColor] = useState("#ffffff");
  const [active, setActive] = useState(false);
  
  

  const handleColorPick = (color: OnChangeEyedrop) => {
    setColor(color.rgb);
    setActive(false);
  };

  return (
    <div>
      <EyeDropper
        onChange={handleColorPick}
        wrapperClasses={`my-css-class ${active ? "my-active-css-class" : ""}`}
      />
      <button onClick={() => setActive(true)}>Activate Eyedropper</button>
      <div
        style={{ backgroundColor: color, width: "100px", height: "100px" }}
      />
      <Box
        height={2000 * (1620.3 / 6118)}
        width={2000 * 0.4708}
        sx={{
          backgroundImage: `url(http://localhost:8080/snapshot)`,
          backgroundSize: "100% 100%",
          borderRadius: "12px",
        }}
      />
      <div>
    <img src="http://localhost:8080/snapshot" width="640" height="500" />
      <canvas style={{ display: 'none' }} width="640" height="500" />
      <button>Capture frame</button>
      </div>
    </div>
  );
}
