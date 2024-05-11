import { atom } from "jotai";


// In the React Javascript Framework, normal Javascript variables still exist. However, a new type of variable is introduced which is called a state.
// When a state is changed, components using that state (the same way you'd use a normal variable) are updated.

// States are local to the scope of the function they are defined in. However, using the Atom states from the jotai library allows for the creating of global states
// Which can be accessed in multiple files


// Global State Definitions
export const CameraURLs = atom<string[]>(["", "", "", ""]);
export const ROSIP = atom<string>(localStorage.getItem("ROS_IP") || window.location.hostname);
export const IsROSConnected = atom<boolean>(false);

export const ImuData = atom<string>("");

export const Mappings = atom<{[controller: number]: {[type: string]: {[index: number]: string}}}>({0: {}, 1: {}}); // Current controller mappings

export const ThrusterMultipliers = atom<number[]>([20,0,0,0,0,0]); // Power:0, Surge:1, Sway:2, Heave:3, Pitch:4, Yaw:5
export const ControllerInput = atom<(number|undefined)[]>([0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]); // Current controller input from pilot
export const PilotActions = atom<string[]>([ // Possible pilot inputs
  "None",
  "surge",
  "sway",
  "heave",
  "pitch",
  "yaw",
  "open_claw",
  "close_claw",
  "heave_up",
  "heave_down",
  "pitch_up",
  "pitch_down",
  "brighten_led",
  "dim_led",
  "turn_claw_cw",
  "turn_claw_ccw",
  "read_outside_temperature_probe",
  "enter_auto_mode"
])

export const CurrentProfile = atom<string>("Not Assigned"); // Current pilot profile
export const ProfilesList = atom<[{id: number, name: string, controller1: string, controller2: string}]>([{id:0, name:"default",controller1:"null",controller2:"null"}]); // List of known pilot profiles

export const RequestingConfig = atom<{state:number, profileName: string, controller1: string, controller2: string}>({state:2, profileName:"default", controller1: "null", controller2: "null"}); // Used for requesting mappings from OR loading mappings into the Database
export const RequestingProfilesList = atom<number>(2); // Used for deleting a certain profile in the database or requesting a list of profiles.
export const RequestingCameraURLs = atom<number>(2); // Used for requesting/saving camera URLs from/to the database.
// See the ROS.tsx script for how the above three Atom states are used

export const ADC_ARRAY = atom<string>("43, 22, 32, 22, 10"); // ADC DATA
export const TEMPERATURE_ARRAY = atom<string>("43, 22, 32, 22, 10, 15"); // TEMP DATA
