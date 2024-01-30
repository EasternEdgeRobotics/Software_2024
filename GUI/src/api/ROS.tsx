import { useAtom } from "jotai";
import ROSLIB from "roslib";
import { IsROSConnected, ROSIP, PowerMultipliers, RequestingConfig, RequestingProfilesList, Mappings, ProfilesList  } from "./Atoms";
import React from "react";

const ros = new ROSLIB.Ros({});

export function InitROS() {
    const [RosIP] = useAtom(ROSIP);
    const [, setIsRosConnected] = useAtom(IsROSConnected);
    const [powerMultipliers] = useAtom(PowerMultipliers);    
    const [requestingConfig, setRequestingConfig] = useAtom(RequestingConfig);
    const [requestingProfilesList, setRequestingProfilesList] = useAtom(RequestingProfilesList);
    const [mappings, setMappings] = useAtom(Mappings);
    const [, setProfilesList] = useAtom(ProfilesList);

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
                                        name:"/thruster_vals", 
                                        messageType: "eer_messages/ThrusterVals"});

    // Publish the new power multipliers whenever they change
    React.useEffect(()=>{
        const thrusterVals = new ROSLIB.Message({
            power : powerMultipliers[0],
            surge: powerMultipliers[1],
            sway: powerMultipliers[2],
            heave: powerMultipliers[3],
            pitch: powerMultipliers[4],
            roll: powerMultipliers[5],
            yaw: powerMultipliers[6]});
        thrusterValsTopic.publish(thrusterVals);
        }    
        ,[powerMultipliers]);

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
        if (requestingProfilesList==1){
            const request = new ROSLIB.ServiceRequest({
                state:requestingProfilesList,
                data:JSON.stringify(mappings)});
                profilesListClient.callService(request, function(result){
                if (requestingProfilesList==1){ //If Profile service==0, we don't care about the result since we are loading config into database
                    if (result.result !="[]"){ //Do not load the result if there are no profiles stored 
                        //setProfilesList(JSON.parse(result.result.replaceAll("'",'"'))); //The String() function replaces the outer "" with ''
                        setProfilesList(JSON.parse(result.result)); 
                    }
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