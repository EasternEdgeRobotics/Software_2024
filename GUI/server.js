const express = require("express");
const app = express();
const fs = require("fs");

app.use(express.json());

app.post('/writeConfig', (request, response) => {
    fs.writeFile("config.json", JSON.stringify(request.body), (err) => {
        if (err) console.log(err);
        else console.log("Config saved!");
    });
    response.send();
});

app.get('/readConfig', (request, response) => {
    fs.readFile("config.json", "utf-8", (err, data) => {
        console.log(data);
        response.json(data);
    })
});

app.listen(5000, () => {console.log("Server started on port 5000!");});