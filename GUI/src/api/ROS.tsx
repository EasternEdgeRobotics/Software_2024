import { useAtom } from "jotai";
import ROSLIB, { Ros } from "roslib";
import { IsROSConnected, ROSIP, ThrusterMultipliers, RequestingConfig, RequestingProfilesList, Mappings, ProfilesList, CurrentProfile, ControllerInput } from "./Atoms";
import React from "react";

export function InitROS() {
    const [RosIP] = useAtom(ROSIP); // The ip of the device running the rosbridge server (will be the Pi4 in enclosure)
    const [, setIsRosConnected] = useAtom(IsROSConnected); // Used in BotTab, indicates if we are communicating with the rosbridge server
    const [thrusterMultipliers, setThrusterMultipliers] = useAtom(ThrusterMultipliers);     
    const [requestingConfig, setRequestingConfig] = useAtom(RequestingConfig); // Used below for requesting controller mappings data from the database running in the Pi4 
    const [requestingProfilesList, setRequestingProfilesList] = useAtom(RequestingProfilesList); // Used below for requesting profiles data from the database running in the Pi4
    const [mappings, setMappings] = useAtom(Mappings); // Controller mappings
    const [, setProfilesList] = useAtom(ProfilesList); // The known list of pilot profiles
    const [currentProfile,] = useAtom(CurrentProfile); 
    const [controllerInput, setControllerInput] = useAtom(ControllerInput); // The current controller input from the pilot

    const [hasRecieved, setHasRecieved] = React.useState<boolean>(false);
    const [ros, setRos] = React.useState<Ros>(new ROSLIB.Ros({}));

    ros.on("connection", () => {
        console.log("ROS Connected!");
        setIsRosConnected(true);
        thrusterRequestService.callService(0, (response: {power: number, surge: number, sway: number, heave: number, pitch: number, roll: number, yaw: number}) => 
            {setThrusterMultipliers([response.power, response.surge, response.sway, response.heave, response.pitch, response.roll, response.yaw]);});
        setHasRecieved(true);
    });
    ros.on("close", () => {
        console.log("ROS Disconnected!");
        setIsRosConnected(false);
        setHasRecieved(false);
    });
    // eslint-disable-next-line @typescript-eslint/no-empty-function
    ros.on("error", () => {}); // to prevent page breaking
    
    React.useEffect(() => {
        ros.connect(`ws://${RosIP}:9090`);
        setInterval(() => {
            if (!ros.isConnected) {
                setRos(new ROSLIB.Ros({}));
                ros.connect(`ws://${RosIP}:9090`);
            }
        }, 1000);
    }, []);

    // Create a publisher on the "/thruster_multipliers" ros2 topic, using a custom EER message type (see eer_messages folder in ROS2/colcon_ws/src)
    const thrusterValsTopic = new ROSLIB.Topic({ros:ros, 
                                        name:"/thruster_multipliers", 
                                        messageType: "eer_messages/ThrusterMultipliers"});

    const thrusterRequestService = new ROSLIB.Service({ros:ros, 
        name:"/multipliers_query", 
        serviceType: "eer_messages/Multipliers"});

    // Publish the new power multipliers whenever they change
    React.useEffect(() => {
        const thrusterVals = new ROSLIB.Message({
            power : thrusterMultipliers[0],
            surge: thrusterMultipliers[1],
            sway: thrusterMultipliers[2],
            heave: thrusterMultipliers[3],
            pitch: thrusterMultipliers[4],
            roll: thrusterMultipliers[5],
            yaw: thrusterMultipliers[6]});
        if (hasRecieved) thrusterValsTopic.publish(thrusterVals);
    }    
    ,[thrusterMultipliers]);

    // Create a publisher on the "/controller_input" ros2 topic, using the default String message which will be used from transporting JSON data 
    const controllerInputTopic = new ROSLIB.Topic({ros:ros, 
                                        name:"/controller_input", 
                                        messageType: "std_msgs/String"});

    // Publish the new controller input whenever it changes
    React.useEffect(()=>{
        if (controllerInput == ""){
            return;
        }
        const controllerInputVals = new ROSLIB.Message({data: controllerInput});
        controllerInputTopic.publish(controllerInputVals);
        setControllerInput("");
        }    
    ,[controllerInput]);

    // Create a ROS service on the "/profile_config" topic, using a custom EER service type (see eer_messages folder in ROS2/colcon_ws/src)
    const configClient = new ROSLIB.Service({ros:ros, 
                                            name:"/profile_config", 
                                            serviceType: "eer_messages/Config"});

    // Run the block of code below whenever the RequestingConfig global state is changed
    React.useEffect(()=>{
        if (requestingConfig.state == 0){ // State 0 indicates that we are saving mappings for a certain profile to the database
            const request = new ROSLIB.ServiceRequest({
                state:requestingConfig.state,
                data:JSON.stringify({"profileName": requestingConfig.profileName,"controller1": requestingConfig.controller1,
                                    "controller2": requestingConfig.controller2,"associated_mappings": mappings})}); // Turn the JSON object into a string to send over ROS
            configClient.callService(request, (function(){null;}));
        }
        else if (requestingConfig.state==1){ // State 1 indicates that we are requesting mappings for a certain profile from the database
            const request = new ROSLIB.ServiceRequest({
                state:requestingConfig.state,
                data:requestingConfig.profileName});
            
            // When a response is recieved for a ROS service request, it is expected to be run through a "callback function". In this case, the function definition is being
            // used as a parameter to configClient.callService instead of a reference to the function.    
            configClient.callService(request, function(result){ 
                const databaseStoredMappings = JSON.parse(result.result); // Turn the recieved string into a JSON object
                const tmp = mappings; 

                // For each controller, store the mappings in a temporary object

                for (let i = 0; i<2; i++){
                    if ([requestingConfig.controller1, requestingConfig.controller2][i]=="recognized"){
                        tmp[i] = {"buttons": {}, "axes": {}, "deadzones":{}};
                        tmp[i]["buttons"] = databaseStoredMappings[i]["buttons"];
                        tmp[i]["axes"] = databaseStoredMappings[i]["axes"];
                        tmp[i]["deadzones"] = databaseStoredMappings[i]["deadzones"];
                    } 
                }

                // Set the global Mappings state to the tmp object which now holds the mappings recieved from the database. Note that just running setMappings(databaseStoredMappings) didn't work
				setMappings(tmp);
            });
        }
        if (requestingConfig.state != 2){ // State 2 doesn't do anything, and is used as the default state. Whenever a service call is done for state 0 or 1, RequestingConfig returns to state 2
            setRequestingConfig({state:2, profileName:"default", controller1:"null", controller2:"null"});
        }
        }    
    ,[requestingConfig]);


    // Create a ROS service on the "/profile_list" topic, using a custom EER service type (see eer_messages folder in ROS2/colcon_ws/src)
    const profilesListClient = new ROSLIB.Service({ros:ros, 
                                            name:"/profiles_list", 
                                            serviceType: "eer_messages/Config"});

    // Run the block of code below whenever the RequestingProfilesList global state is changed
    React.useEffect(()=>{
        if (requestingProfilesList==0){ // State 0 indicates that we would like to delete a profile from the database. 
                                        // Note that profiles are created when RequestingConfig state 0 is called and the database doesn't recognize the name of the profile coming from the GUI
            const request = new ROSLIB.ServiceRequest({
                state:0, 
                data:currentProfile});
                profilesListClient.callService(request, function(result){
                    console.log(result.result); // Should return "Profile Deleted" or "Profile Not Found"
            });
        }
        else if (requestingProfilesList==1){ // State 1 indicates that we are requesting the entire list of profiles from the database
            const request = new ROSLIB.ServiceRequest({
                state:1, 
                data:JSON.stringify(mappings)}); // Turn the mappings JSON object into a string to send over ROS

            profilesListClient.callService(request, function(result){
                if (result.result !="[]"){ // Do not load the result if there are no profiles stored (i.e. empty list is returned)
                    setProfilesList(JSON.parse(result.result)); 
                }
                else { // If we recieve an empty list, then just set the profile to "default". The pilot will have to create a profile on the fly that will only be saved locally, and will be gone on page reload
                    setProfilesList([{id:0, name:"default",controller1:"null",controller2:"null"}]);
                }
            });
        }
        setRequestingProfilesList(2);
        }    
    ,[requestingProfilesList]);


    // const ImuDataListener = new ROSLIB.Topic({ros:ros, 
    //     name:"/imu", 
    //     messageType: "std_msgs/String"});

    // ImuDataListener.subscribe(function(message) {
    //     console.log(message);
    // });
    
    return (null);
}