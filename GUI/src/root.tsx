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
import SafetyDisclaimer from "./components/SafetyDisclaimer";
import { routes as remixRoutes } from "virtual:routes";

const routes = remixRoutes;
routes[0].lazy = async () => await import("./root");

export function Component() {
  const pages = routes[0]
    .children!.map((route) => route.path!)
    .filter((path) => path !== undefined);

  return (
    <RosProvider rosURL={"ws://localhost:9090"}>
      <Box>
        <ResponsiveAppBar pages={pages} />
        <Outlet />
      </Box>
      <SafetyDisclaimer /> {/* Display the safety disclaimer */}
      <InitControllers /> {/* Initialize controller listeners */}
      <ScrollRestoration />
    </RosProvider>
  );
}

const router = createBrowserRouter(routes);

const root = createRoot(document.getElementById("root")!);
root.render(
  <React.StrictMode>
    <RouterProvider router={router} />
    <CssBaseline />
  </React.StrictMode>
);
