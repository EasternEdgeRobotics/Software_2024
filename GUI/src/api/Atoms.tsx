import { atom } from "jotai";


// In the React Javascript Framework, normal Javascript variables still exist. However, a new type of variable is introduced which is called a state.
// When a state is changed, components using that state (the same way you'd use a normal variable) are updated.

// States are local to the scope of the function they are defined in. However, using the Atom states from the jotai library allows for the creating of global states
// Which can be accessed in multiple files


// Global State Definitions
export const CameraIPs = atom<string[]>(["", "", "", ""]); 
export const ROSIP = atom<string>(localStorage.getItem("ROS_IP") || window.location.hostname);
export const IsROSConnected = atom<boolean>(false);

export const ImuData = atom<string>(""); 

export const Mappings = atom<{[controller: number]: {[type: string]: {[index: number]: string}}}>({0: {}, 1: {}}); // Current controller mappings

export const ThrusterMultipliers = atom<number[]>([20,0,0,0,0,0,0]); // Power:0, Surge:1, Sway:2, Heave:3, Pitch:4, Roll:5, Yaw:6
export const ControllerInput = atom<string>(""); // Current controller input from pilot

export const CurrentProfile = atom<string>("Not Assigned"); // Current pilot profile
export const ProfilesList = atom<[{id: number, name: string, controller1: string, controller2: string}]>([{id:0, name:"default",controller1:"null",controller2:"null"}]); // List of known pilot profiles

export const RequestingConfig = atom<{state:number, profileName: string, controller1: string, controller2: string}>({state:2, profileName:"default", controller1: "null", controller2: "null"}); // Used for requesting mappings from OR loading mappings into the Database
export const RequestingProfilesList = atom<number>(2); // Used for deleting a certain profile in the database or requesting a list of profiles. 
// See the ROS.tsx script for how the above two Atom states are used
