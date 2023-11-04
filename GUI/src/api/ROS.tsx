import { useAtom } from 'jotai';
import ROSLIB from 'roslib';
import { IsROSConnected, ROSIP, PowerMultipliers } from './Atoms';
import React from 'react';

const ros = new ROSLIB.Ros({});

export function InitROS() {
    const [RosIP] = useAtom(ROSIP);
    const [, setIsRosConnected] = useAtom(IsROSConnected);
    const [powerMultipliers] = useAtom(PowerMultipliers);

    // Create listeners to detect when power multipliers change
    React.useEffect(()=>{
        console.log(powerMultipliers)},[powerMultipliers])

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

    return(null);
}

export function isRosConnected() {
    return(ros.isConnected);
}