const ROSLIB = require("roslib");
const fs = require("fs");
const WebSocket = require('ws');

const ros = new ROSLIB.Ros();

const wss = new WebSocket.Server({port: 5001});
wss.on('connection', function connection(ws) {
    ws.on('message', function message(data) {
        //if ros is not connected, connect using specified ip
        if (!ros.isConnected) ros.connect(`ws://${data}:9090`);
        //since the first loadconfig sent when ros is not setup here, send the config
        if (!fs.existsSync("config/config.json")) return;
        console.log("Config requested...");
        receiver.publish({state: 3, data: fs.readFileSync("config/config.json").toString()}); //send config with 'config return' response
    });
});

//simple logging for ros
ros.on('connection', () => {console.log('ROS Connected!');});
ros.on('close', () => {console.log('ROS Disconnected!');});
ros.on('error', () => {console.log('A ROS error has occurred!');});

//create config topic
const receiver = new ROSLIB.Topic({
    ros: ros,
    name: "/config",
    messageType: 'eer_messages/msg/Config'
});

receiver.subscribe((message) => {
    switch(message.state) {
        case 0:
            //send config if load config request was received
            if (!fs.existsSync("config/config.json")) return;
            console.log("Config requested...");
            receiver.publish({state: 3, data: fs.readFileSync("config/config.json").toString()}); //send config with 'config return' response
            break;
        case 1:
            //save config with recieved data if save request was receieved
            console.log("Saving config...");
            if (!fs.existsSync("config/")) fs.mkdirSync("config/", (err) => {if (err) console.log(err);}); //if config dir doesnt exist, create it
            fs.writeFileSync("config/config.json", message.data, (err) => {if (err) console.log(err);}); //write config
            console.log("Save success!");
            break;
    }
});