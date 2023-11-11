import { Box, CssBaseline } from "@mui/material";
import React from "react";
import { createRoot } from "react-dom/client";
import {
  Outlet,
  RouterProvider,
  ScrollRestoration,
  createBrowserRouter,
} from "react-router-dom";
import InitControllers from "./components/Controller";
import ResponsiveAppBar from "./components/ResponsiveAppBar";
import { RosProvider } from "./components/ROS";

const pages = ["bots", "cameras", "controllers", "settings"];

function Root() {
  return (
    <RosProvider rosURL={"ws://localhost:9090"}>
      <Box>
        <ResponsiveAppBar pages={pages} />
        <Outlet />
      </Box>
      {/*<SafetyDisclaimer /> {/* Display the safety disclaimer */}
      <InitControllers /> {/* Initialize controller listeners */}
      <ScrollRestoration />
    </RosProvider>
  );
}

const router = createBrowserRouter([
  {
    path: "/",
    element: <Root />,
  },
]);

const root = createRoot(document.getElementById("root")!);
root.render(
  <React.StrictMode>
    <RouterProvider router={router} />
    <CssBaseline />
  </React.StrictMode>
);
