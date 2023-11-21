import { Box, CssBaseline, ThemeProvider, createTheme } from "@mui/material";
import React from "react";
import { createRoot } from "react-dom/client";
import {
  Outlet,
  RouterProvider,
  ScrollRestoration,
  createBrowserRouter,
} from "react-router-dom";
import InitControllers from "./components/Controller";
import { NavBar } from "./components/NavBar";
import { RosProvider } from "./components/ROS";
import SafetyDisclaimer from "./components/SafetyDisclaimer";
import { routes as remixRoutes } from "virtual:routes";
import { Bot, Camera, Gamepad2, Wrench } from "lucide-react";

const routes = remixRoutes;
routes[0].lazy = async () => await import("./root");

export function Component() {
  const pages = [
    { title: <Bot />, path: "bot" },
    { title: <Camera />, path: "cameras" },
    { title: <Gamepad2 />, path: "controllers" },
    { title: <Wrench />, path: "settings" },
  ];

  return (
    <ThemeProvider theme={createTheme({ palette: { mode: "dark" } })}>
      <RosProvider rosURL={"ws://localhost:9090"}>
        <Box>
          <NavBar pages={pages} />
          <Outlet />
        </Box>
        <SafetyDisclaimer /> {/* Display the safety disclaimer */}
        <InitControllers /> {/* Initialize controller listeners */}
        <CssBaseline />
        <ScrollRestoration />
      </RosProvider>
    </ThemeProvider>
  );
}

const router = createBrowserRouter(routes);

const root = createRoot(document.getElementById("root")!);
root.render(
  <React.StrictMode>
    <RouterProvider router={router} />
  </React.StrictMode>
);
