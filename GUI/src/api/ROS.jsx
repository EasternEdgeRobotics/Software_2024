import { useAtom } from 'jotai';
import ROSLIB from 'roslib';
import { ROSIP } from './Atoms';

const ros = new ROSLIB.Ros();
var hasErrored = false;

export function InitROS() {
    const [RosIP] = useAtom(ROSIP);

    ros.on("connection", () => console.log("ROS Connected!"));
    ros.on("close", () => console.log("ROS Disconnected!"));
    ros.on("error", () => {
        if (!hasErrored) {
            alert(`Could not connect to ROS with IP: ws://${RosIP}:9090\nPlease start ROS Bridge and refresh the page (or change the IP in settings).`); 
            hasErrored = true;
    }});
    ros.connect(`ws://${RosIP}:9090`);

    return(null);
}

export function isRosConnected() {
    return(ros.isConnected);
}