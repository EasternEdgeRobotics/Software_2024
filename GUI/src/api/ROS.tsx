import { useAtom } from 'jotai';
import ROSLIB from 'roslib';
import { IsROSConnected, ROSIP, PowerMultipliers } from './Atoms';
import React from 'react';

const ros = new ROSLIB.Ros({});

export function InitROS() {
    const [RosIP] = useAtom(ROSIP);
    const [, setIsRosConnected] = useAtom(IsROSConnected);
    const [powerMultipliers] = useAtom(PowerMultipliers);

    

    ros.on("connection", () => {
        console.log("ROS Connected!");
        setIsRosConnected(true);
    });
    ros.on("close", () => {
        console.log("ROS Disconnected!");
        setIsRosConnected(false);
        ros.connect(`ws://${RosIP}:9090`)
    });
    ros.on("error", () => {}); //to prevent page breaking
    ros.connect(`ws://${RosIP}:9090`);

    const thrusterValsTopic = new ROSLIB.Topic({ros:ros, 
                                        name:"/thruster_vals", 
                                        messageType: "eer_messages/ThrusterVals"})

    // Create listeners to detect when power multipliers change
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
        ,[powerMultipliers])

    return(null);
}

export function isRosConnected() {
    return(ros.isConnected);
}