import { useAtom } from "jotai";
import ROSLIB from "roslib";
import { IsROSConnected, ROSIP, ThrusterMultipliers, RequestingConfig, RequestingProfilesList, Mappings, ProfilesList, CurrentProfile, ControllerInput } from "./Atoms";
import React from "react";

const ros = new ROSLIB.Ros({});

export function InitROS() {
    const [RosIP] = useAtom(ROSIP);
    const [, setIsRosConnected] = useAtom(IsROSConnected);
    const [thrusterMultipliers] = useAtom(ThrusterMultipliers);    
    const [requestingConfig, setRequestingConfig] = useAtom(RequestingConfig);
    const [requestingProfilesList, setRequestingProfilesList] = useAtom(RequestingProfilesList);
    const [mappings, setMappings] = useAtom(Mappings);
    const [, setProfilesList] = useAtom(ProfilesList);
    const [currentProfile,] = useAtom(CurrentProfile);
    const [controllerInput, setControllerInput] = useAtom(ControllerInput);

    ros.on("connection", () => {
        console.log("ROS Connected!");
        setIsRosConnected(true);
    });
    ros.on("close", () => {
        console.log("ROS Disconnected!");
        setIsRosConnected(false);
        ros.connect(`ws://${RosIP}:9090`);
    });
    // eslint-disable-next-line @typescript-eslint/no-empty-function
    ros.on("error", () => {}); //to prevent page breaking
    ros.connect(`ws://${RosIP}:9090`);

    const thrusterValsTopic = new ROSLIB.Topic({ros:ros, 
                                        name:"/thruster_multipliers", 
                                        messageType: "eer_messages/ThrusterMultipliers"});

    // Publish the new power multipliers whenever they change
    React.useEffect(()=>{
        const thrusterVals = new ROSLIB.Message({
            power : thrusterMultipliers[0],
            surge: thrusterMultipliers[1],
            sway: thrusterMultipliers[2],
            heave: thrusterMultipliers[3],
            pitch: thrusterMultipliers[4],
            roll: thrusterMultipliers[5],
            yaw: thrusterMultipliers[6]});
        thrusterValsTopic.publish(thrusterVals);
        }    
        ,[thrusterMultipliers]);

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

    const configClient = new ROSLIB.Service({ros:ros, 
                                            name:"/profile_config", 
                                            serviceType: "eer_messages/Config"});

    //setMappings(JSON.parse(result.result.replaceAll("'",'"'))); //Zaid: JSON only likes double quotes
    React.useEffect(()=>{
        if (requestingConfig.state ==0){
            const request = new ROSLIB.ServiceRequest({
                state:requestingConfig.state,
                data:JSON.stringify({"profileName": requestingConfig.profileName,"controller1": requestingConfig.controller1,
                                    "controller2": requestingConfig.controller2,"associated_mappings": mappings})}); //Load data into specified profile
                configClient.callService(request, function(result){ const i =1; })
        }
        else if (requestingConfig.state==1){
            const request = new ROSLIB.ServiceRequest({
                state:requestingConfig.state,
                data:requestingConfig.profileName});
                configClient.callService(request, function(result){ 
                    const databaseStoredMappings = JSON.parse(result.result);
                    const tmp = mappings; 
					tmp[0] = {"buttons": {}, "axes": {}, "deadzones":{}};
                    if (requestingConfig.controller1=="recognized"){
                        tmp[0]["buttons"] = databaseStoredMappings[0]["buttons"];
                        tmp[0]["axes"] = databaseStoredMappings[0]["axes"];
                        tmp[0]["deadzones"] = databaseStoredMappings[0]["deadzones"];
                    }
                    tmp[1] = {"buttons": {}, "axes": {}, "deadzones":{}};
                    if (requestingConfig.controller2=="recognized"){
                        tmp[1]["buttons"] = databaseStoredMappings[1]["buttons"];
                        tmp[1]["axes"] = databaseStoredMappings[1]["axes"];
                        tmp[1]["deadzones"] = databaseStoredMappings[1]["deadzones"];}
					setMappings(tmp);
                }) 
        }
        if (requestingConfig.state != 2){
            setRequestingConfig({state:2, profileName:"default", controller1:"null", controller2:"null"});
        }
        }    
        ,[requestingConfig]);


    const profilesListClient = new ROSLIB.Service({ros:ros, 
                                            name:"/profiles_list", 
                                            serviceType: "eer_messages/Config"});

    React.useEffect(()=>{
        if (requestingProfilesList==0){
            const request = new ROSLIB.ServiceRequest({
                state:0, //Delete Profile
                data:currentProfile});
                profilesListClient.callService(request, function(result){
                    console.log(result.result);
            })
        }
        else if (requestingProfilesList==1){
            const request = new ROSLIB.ServiceRequest({
                state:1, //Get Profiles
                data:JSON.stringify(mappings)});
                profilesListClient.callService(request, function(result){
                if (result.result !="[]"){ //Do not load the result if there are no profiles stored 
                    //setProfilesList(JSON.parse(result.result.replaceAll("'",'"'))); //The String() function replaces the outer "" with ''
                    setProfilesList(JSON.parse(result.result)); 
                }
                else {
                    setProfilesList([{id:0, name:"default",controller1:"null",controller2:"null"}]);
                }
            })
        }
        setRequestingProfilesList(2);
        }    
        ,[requestingProfilesList]);
    
    return (null);
}

export function isRosConnected() {
    return (ros.isConnected);
}