import ROSLIB from 'roslib';
import * as Atoms from "./Atoms";
import { useAtom } from 'jotai';

const ros = new ROSLIB.Ros();

const topic = new ROSLIB.Topic({
    ros : ros,
    name : '/config',
    messageType : 'eer_messages/msg/Config'
});

const InitROS = () => {
    //TODO: potentially save settings without setting each atom?
    const [, setCamera1IP] = useAtom(Atoms.Camera1IP);
    const [, setCamera2IP] = useAtom(Atoms.Camera2IP);
    const [, setCamera3IP] = useAtom(Atoms.Camera3IP);

    ros.on('connection', () => {console.log('ROS Connected!');});
    ros.on('close', () => {console.log('ROS Disconnected!');});    
    ros.connect(`ws://${process.env.REACT_APP_ROS_IP}:9090`); //reminder to set the .env file to the ip of whatever is running the rosbridge
    topic.subscribe((message) => {
        if (message.state !== 3) return; //return if message is not config return (where it will then set the config)
        let data = JSON.parse(message.data);
        //TODO: do the same thing as above, automatically set each variable
        if (data.Camera1IP !== undefined) setCamera1IP(data.Camera1IP);
        if (data.Camera2IP !== undefined) setCamera2IP(data.Camera2IP);
        if (data.Camera3IP !== undefined) setCamera3IP(data.Camera3IP);

    });
}

export const loadConfig = () => {
    topic.publish({state: 0, data: ""}); //send load packet for the subscriber to deal with
}

export const saveConfig = (message) => {
    topic.publish({state: 1, data: message}); //send the save packet with the config
}

export default InitROS;