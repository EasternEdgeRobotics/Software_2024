import { useAtom } from "jotai";
import ROSLIB from "roslib";
import { IsROSConnected, ROSIP, PowerMultipliers, Profiles, ProfilesService } from "./Atoms";
import React from "react";

const ros = new ROSLIB.Ros({});

export function InitROS() {
    const [RosIP] = useAtom(ROSIP);
    const [, setIsRosConnected] = useAtom(IsROSConnected);
    const [powerMultipliers] = useAtom(PowerMultipliers);

    const [profiles, setProfiles] = useAtom(Profiles);
    const [profilesService, setProfilesService] = useAtom(ProfilesService);

    

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

    const profileConfigClient = new ROSLIB.Service({ros: ros, 
                                            name:"/profiles_config", 
                                            serviceType: "eer_messages/Config"});
    
    React.useEffect(()=>{
        if (profilesService==0 || profilesService==1){
            const request = new ROSLIB.ServiceRequest({
                state:profilesService,
                data:profiles});
            profileConfigClient.callService(request, function(result){
                if (profilesService==1){ //If Profile service==0, we don't care about the result since we are just overwriting what is stored in the ROS2 docker volume.
                    if (result.result !=""){ //Do not load the result if there are no profiles stored  
                        setProfiles(result.result);
                    }
                }
            });
        }
        setProfilesService(2);
        }    
        ,[profilesService]);
    
    return (null);
}

export function isRosConnected() {
    return (ros.isConnected);
}