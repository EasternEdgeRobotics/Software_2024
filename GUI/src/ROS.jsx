import ROSLIB from 'roslib';
import { ipAtom } from './atoms/CameraIP';
import { useAtom } from 'jotai';

const ros = new ROSLIB.Ros();

const topic = new ROSLIB.Topic({
    ros : ros,
    name : '/config',
    messageType : 'eer_messages/msg/Config'
});

export const loadConfig = () => {
    topic.publish({state: 0, data: ""}); //send load packet for the subscriber to deal with
}

export const saveConfig = (message) => {
    console.log(message);
    topic.publish({state: 1, data: JSON.stringify(message)}); //send the save packet with the config
}

function InitROS() {
    const [, setIPs] = useAtom(ipAtom);
    ros.on('connection', () => {console.log('ROS Connected!');});
    ros.on('close', () => {console.log('ROS Disconnected!');});
    ros.connect(`ws://${process.env.REACT_APP_ROS_IP}:9090`);
    topic.subscribe((message) => {
        if (message.state !== 3) return; //return if message is not config return (where it will then set the config)
        let data = JSON.parse(message.data);
        if (data.cameraIPs === undefined) return;
        setIPs(data.cameraIPs);
    });
}

export default InitROS;