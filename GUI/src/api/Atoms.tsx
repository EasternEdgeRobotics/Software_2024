import { atom } from "jotai";

//global states for things that are needed from multiple files
export const CameraIPs = atom<string[]>(["", "", ""]);
export const PowerMultipliers = atom<number[]>([0,0,0,0,0,0,0]); //Power:0, Surge:1, Sway:2, Heave:3, Pitch:4, Roll:5, Yaw:6
export const ROSIP = atom<string>(localStorage.getItem("ROS_IP") || window.location.hostname);
export const IsROSConnected = atom<boolean>(false);

export const Mappings = atom<{[controller: number]: {[type: string]: {[index: number]: string}}}>({0: {}, 1: {}});

export const CurrentProfile = atom<string>(""); 
export const ProfilesList = atom<[{id: number, name: string, controller1: string, controller2: string}]>([{id:0, name:"default",controller1:"null",controller2:"null"}]);

export const RequestingConfig = atom<{state:number, profileName: string, controller1: string, controller2: string}>({state:2, profileName:"default", controller1: "null", controller2: "null"});
export const RequestingProfilesList = atom<number>(2);
/*{
  "0": {
      "buttons": {
          "0": "None",
          "1": "openClaw",
          "2": "None",
          "3": "None",
          "4": "None",
          "5": "None",
          "6": "None",
          "7": "None",
          "8": "None",
          "9": "None",
          "10": "None",
          "11": "None",
          "12": "None",
          "13": "None",
          "14": "None",
          "15": "None"
      },
      "axes": {
          "0": "None",
          "1": "None",
          "2": "None",
          "3": "None"
      },
      "deadzones": {
          "0": "0.3",
          "1": "0",
          "2": "0",
          "3": "0"
      }
  },
  "1": {}
}
{
    "0": {
        "axes": {
            "0": "None",
            "1": "None",
            "2": "None",
            "3": "None"
        },
        "buttons": {
            "0": "None",
            "1": "openClaw",
            "2": "None",
            "3": "None",
            "4": "None",
            "5": "None",
            "6": "None",
            "7": "None",
            "8": "None",
            "9": "None",
            "10": "None",
            "11": "None",
            "12": "None",
            "13": "None",
            "14": "None",
            "15": "None"
        },
        "deadzones": {
            "0": "0.3",
            "1": "0",
            "2": "0",
            "3": "0"
        }
    },
    "1": {
        "axes": {},
        "buttons": {},
        "deadzones": {}
    }
}*/