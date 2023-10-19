import ROSLIB from 'roslib';
import { ipAtom } from './atoms/CameraIP';
import { useAtom } from 'jotai';
import { ROSIPAtom } from './atoms/LocalStorage';

const ros = new ROSLIB.Ros();
const ws = new WebSocket("ws://127.0.0.1:5001");

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

//just to show the alert once, could probably use a useeffect hook? this works i guess
var hasErrored = false;

function InitROS() {
    const [ROSIP, setROSIP] = useAtom(ROSIPAtom);
    ws.onopen = function(event) {
        ws.send(ROSIP);
    }
    const [, setIPs] = useAtom(ipAtom);
    ros.on('connection', () => {console.log('ROS Connected!');});
    ros.on('close', () => {console.log('ROS Disconnected!');});
    ros.on('error', function() {if (!hasErrored) {alert(`Could not connect to ROS with IP: ws://${ROSIP}:9090\nPlease start ROS Bridge and refresh the page (or change the IP in settings).`); hasErrored = true}});
    try {
        ros.connect(`ws://${ROSIP}:9090`);
    } catch (err) {
        //if there is an error with loading ros (invalid string), set the ip to a valid string and refresh.
        localStorage.setItem("ROS_IP", window.location.hostname);
        window.location.reload();
    }
    topic.subscribe((message) => {
        if (message.state !== 3) return; //return if message is not config return (where it will then set the config)
        let data = JSON.parse(message.data);
        if (data.cameraIPs === undefined) return;
        setIPs(data.cameraIPs);
    });
}

export default InitROS;