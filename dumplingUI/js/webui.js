var robot_IP = "localhost";
var camera_topic = "/camera/image_raw";
var map_topic = "/camera/image_raw";
var ros;
// Initialize ROS connection
function initROS(){
    ros = new ROSLIB.Ros({url:"ws://" + robot_IP + ':9090'});
    ros.on('connection', function(){
        console.log('Connected to websocket server.');
        $('#connection_card').removeClass('border-left-danger').addClass('border-left-success');
        $('#connection_card .text-md').removeClass('text-danger').addClass('text-success');
        $('#connection_card .h5').text('Connected');
    });
    ros.on('error', function(error){
        console.log('Error connecting to websocket server: ', error);
        $('#connection_card').removeClass('border-left-success').addClass('border-left-danger');
        $('#connection_card .text-md').removeClass('text-success').addClass('text-danger');
        $('#connection_card .h5').text('Error');
    });
    ros.on('close',function(){
        console.log('Connection to websocket server closed.');
        $('#connection_card').removeClass('border-left-success').addClass('border-left-danger');
        $('#connection_card .text-md').removeClass('text-success').addClass('text-danger');
        $('#connection_card .h5').text('Disconnected');
    });
}

function initVideo(){
    $('#camera .text-primary').text("Feed: " + camera_topic);
    video = $('#camera img')[0];
    video.src = "http://" + robot_IP + ":8080/stream?topic=" + camera_topic + "&type=mjpeg&quality=80";

    $('#map .text-primary').text("Feed: " + map_topic);
    video = $('#map img')[0];
    video.src = "http://" + robot_IP + ":8080/stream?topic=" + map_topic + "&type=mjpeg&quality=80";
}

function initNodes(){
    // Publishers
    var trayVel = new ROSLIB.Topic({
        ros:ros,
        name:'/tray_vel',
        messageType: 'std_msgs/Float32'
    });

    // Subscribers
    var motor_error = new ROSLIB.Topic({
        ros:ros,
        name:'/motor_status/errors',
        messageType:'std_msgs/UInt16MultiArray'
    });

    motor_error.subscribe(function(message){
        console.log('Received message ' + message.data);
    });
}

// Window initialization
window.onload = function () {
    initROS()
    initNodes()
    initVideo()
}
