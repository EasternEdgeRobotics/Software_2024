import { atom } from "jotai";

//global states for things that are needed from multiple files
export const CameraIPs = atom<string[]>(["", "", ""]);
export const CurrentController = atom<number>(-1);
export const ROSIP = atom<string>(localStorage.getItem("ROS_IP") || window.location.hostname);