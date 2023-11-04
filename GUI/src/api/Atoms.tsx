import { atom } from "jotai";

//global states for things that are needed from multiple files
export const CameraIPs = atom<string[]>(["", "", ""]);
export const PowerMultipliers = atom<number[]>([0,0,0,0,0,0,0]); //Power:0, Surge:1, Sway:2, Heave:3, Pitch:4, Roll:5, Yaw:6
export const CurrentController = atom<number>(-1);
export const ROSIP = atom<string>(localStorage.getItem("ROS_IP") || window.location.hostname);
export const IsROSConnected = atom<boolean>(false);