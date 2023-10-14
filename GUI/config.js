const ROSLIB = require("roslib");
require("dotenv").config();
const fs = require("fs");

const ros = new ROSLIB.Ros();

ros.connect(`ws://${process.env.REACT_APP_ROS_IP}:9090`);

ros.on('connection', () => {console.log('ROS Connected!');});
ros.on('close', () => {console.log('ROS Disconnected!');});

const receiver = new ROSLIB.Topic({
    ros: ros,
    name: "/config",
    messageType: 'eer_messages/msg/Config'
});

receiver.subscribe((message) => {
    switch(message.state) {
        case 0:
            if (!fs.existsSync("config/config.json")) return;
            console.log("Config requested...");
            receiver.publish({state: 3, data: fs.readFileSync("config/config.json").toString()}); //send config with 'config return' response
            break;
        case 1:
            console.log("Saving config...");
            if (!fs.existsSync("config/")) fs.mkdirSync("config/", (err) => {if (err) console.log(err);}); //if config dir doesnt exist, create it
            fs.writeFileSync("config/config.json", message.data, (err) => {if (err) console.log(err);}); //write config
            console.log("Save success!");
            break;
    }
});