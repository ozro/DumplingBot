// Initialize ROS connection
var ros = new ROSLIB.Ros({url:'ws://localhost:9090'});
ros.on('connection', function(){
    console.log('Connected to websocket server.');
    $('#ros_connected').show();
});
ros.on('error', function(error){
    console.log('Error connecting to websocket server: ', error);
    $('#ros_failed').show();
});
ros.on('close',function(){
    console.log('Connection to websocket server closed.');
    $('#ros_closed').show();
});

// Publishers
var trayVel = new ROSLIB.Topic({
    ros:ros,
    name:'/tray_vel',
    messageType: 'std_msgs/Float32'
});

// Subscribers
var listener = new ROSLIB.Topic({
    ros:ros,
    name:'/listener',
    messageType:'std_msgs/String'
});

listener.subscribe(function(message){
    console.log('Received message ' + message.data);
    document.getElementById("trayClose").innerHTML = message.data;
});
