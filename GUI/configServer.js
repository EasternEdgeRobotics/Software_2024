const fs = require('fs');
const express = require('express');
const app = express();

app.use(express.json());

app.get('/config', (req, res) => {
    if (!fs.existsSync("config/config.json")) res.send("{}");
    else res.send(fs.readFileSync("config/config.json"));
});

app.post('/config', (req, res) => {
    if (!fs.existsSync("config/")) fs.mkdirSync("config/", (err) => {if (err) console.log(err)});
    fs.writeFileSync("config/config.json", JSON.stringify(req.body), (err) => {if (err) console.log(err)});
    res.send();
});

//TODO: preset saving and loading

app.listen(3001);
console.log("Config web server started at port 3001");