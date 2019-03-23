// Initialize ROS connection
var ros = new ROSLIB.Ros({url:'ws://localhost:9090'});

ros.on('connection', function(){
    console.log('Connected to websocket server.');
});
ros.on('error', function(error){
    console.log('Error connecting to websocket server: ', error);
});
ros.on('close',function(){
    console.log('Connection to websocket server closed.');
});

// Buttons
var openButton = document.querySelector("#trayOpen");
var timerID;
var counter = 0;
var pressHoldEvent = new CustomEvent("pressHold");

var pressHoldDuration = 50;

openButton.addEventListener("mousedown", pressingDown, false);
openButton.addEventListener("mouseup", notPressingDown, false);
openButton.addEventListener("mouseleave", notPressingDown, false);
openButton.addEventListener("touchstart", pressingDown, false);
openButton.addEventListener("touchend", notPressingDown, false);
openButton.addEventListener("pressHold", doSomething, false);

function pressingDown(e){
    requestAnimationFrame(timer);
    e.preventDefault();
}

function notPressingDown(e){
    cancelAnimationFrame(timerID);
    counter = 0;
}
function timer(){
    if (counter < pressHoldDuration){
        timerID = requestAnimationFrame(timer);
        counter++;
        var msg = new ROSLIB.Message({
            data:'opening'
        });
        cmdVel.publish(msg);
        console.log("Opening...")
    }
    else{
        openButton.dispatchEvent(pressHoldEvent);
    }
}

function doSomething(e){
    console.log("Done!")
}

// Publishing

var cmdVel = new ROSLIB.Topic({
    ros:ros,
    name:'/cmd_vel',
    messageType: 'std_msgs/String'
});

// Subscribing

var listener = new ROSLIB.Topic({
    ros:ros,
    name:'/listener',
    messageType:'std_msgs/String'
});

listener.subscribe(function(message){
    console.log('Received message ' + message.data);
    document.getElementById("trayClose").innerHTML = message.data;
});

