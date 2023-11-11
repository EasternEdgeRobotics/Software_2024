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

export const pages = routes[0]
  .children!.filter((child) => {
    return child.path != "*" && child.path != undefined;
  })
  .map((child) => {
    return child.path!;
  });
console.log(pages);

export function Root() {
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
    children: routes,
  },
]);

const root = createRoot(document.getElementById("root")!);
root.render(
  <React.StrictMode>
    <RouterProvider router={router} />
    <CssBaseline />
  </React.StrictMode>
);
