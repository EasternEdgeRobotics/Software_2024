# React GUI
## Install
### You'll need a ROSBridge server running on port 9090 for this to work!
Install yarn (it's just better than npm)
```
npm install -g yarn
```
Install dependencies
```
yarn install
```
Set IP in .env to the device running the ROSBridge server  

Run it
```
yarn start
```
Win
## TODO
 - Actual Controller Support (it just listens for controllers right now)
 - Controller bindings
 - Controller presets that are written to a config
 - Send off cameras to whatever for the AI task (when we get AI code done)
 - General cleanup (im bad at react)