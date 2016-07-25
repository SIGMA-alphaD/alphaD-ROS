var express = require('express');
var app = express();
var path = require("path");

app.use('/static', express.static("static"));

app.get('/', function(req, res){
	res.sendFile(path.join(__dirname + '/Controller.html'));
});

app.listen(3000, function(){
	console.log("Examle app listening on port 3000");
});
