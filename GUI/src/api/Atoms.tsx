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
  "turn_stepper_cw",
  "turn_stepper_ccw",
  "read_outside_temperature_probe",
  "enter_auto_mode"
])

export const KeyboardInputMap = atom<(string|number)[][]>([ // Possible pilot inputs
  ["w","surge",100],
  ["a","sway",-100],
  ["s","surge",-100],
  ["d","sway",100],
  ["q","yaw",-100],
  ["e","yaw",100],
  ["w","surge",100],
  ["r","heave",100],
  ["f","heave",-100],
  ["t","pitch",100],
  ["g","pitch",-100],
  ["z","open_claw",1],
  ["x","close_claw",1],
  ["c","brighten_led",1],
  ["v","dim_led",1],
  ["b","turn_stepper_cw",1],
  ["n","turn_stepper_ccw",1],
  ["m","read_outside_temperature_probe",1],
  [",","enter_auto_mode",1],
])

export const CurrentProfile = atom<string>("Not Assigned"); // Current pilot profile
export const ProfilesList = atom<[{id: number, name: string, controller1: string, controller2: string}]>([{id:0, name:"default",controller1:"null",controller2:"null"}]); // List of known pilot profiles

export const RequestingConfig = atom<{state:number, profileName: string, controller1: string, controller2: string}>({state:2, profileName:"default", controller1: "null", controller2: "null"}); // Used for requesting mappings from OR loading mappings into the Database
export const RequestingProfilesList = atom<number>(2); // Used for deleting a certain profile in the database or requesting a list of profiles.
export const RequestingCameraURLs = atom<number>(2); // Used for requesting/saving camera URLs from/to the database.
// See the ROS.tsx script for how the above three Atom states are used

export const ADCArray = atom<{adc_48v_bus:number,adc_12v_bus:number,adc_5v_bus:number}>({adc_48v_bus:0,adc_12v_bus:0,adc_5v_bus:0}); // ADC DATA
export const TemperatureArray = atom<{power_board_u8:number,power_board_u9:number,power_board_u10:number,
  mega_board_ic2:number,power_board_u11:number,mega_board_ic1:number}>({power_board_u8:0,power_board_u9:0,power_board_u10:0,
    mega_board_ic2:0,power_board_u11:0,mega_board_ic1:0}); // TEMP DATA

export const AutonomousModeStatus = atom<string>("Autonomous Mode Off"); // ADC DATA