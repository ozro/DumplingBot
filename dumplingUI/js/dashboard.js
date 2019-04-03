var robot_IP = "localhost";
var camera_topic = "/camera/image_raw";
var map_topic = "/camera/image_raw";
var error_topic = "/motor_status/errors";
var limit_topic = "/motor_status/limit_status";
var target_speed_topic = "/motor_status/target_speeds";
var actual_speed_topic = "/motor_status/current_speeds";
var temps_topic = "/motor_status/temps";
var curr_topic = "/motor_status/currents";
var volt_topic = "/motor_status/voltages";
var enc_topic = "/motor_status/encoder_counts";
var target_topic = "";
var state_topic = "";

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
        $('#connection_card .h5').text('Closed');
    });
}

function initVideo(){
    $('#camera .text-primary').text("Feed: " + camera_topic);
    video = $('#camera img')[0];
    video.src = "http://" + robot_IP + ":8080/stream?topic=" + camera_topic + "&type=mjpeg&quality=80";

//     $('#map .text-primary').text("Feed: " + map_topic);
//     video = $('#map img')[0];
//     video.src = "http://" + robot_IP + ":8080/stream?topic=" + map_topic + "&type=mjpeg&quality=80";
}

function initNodes(){
//    // Publishers
//    var trayVel = new ROSLIB.Topic({
//        ros:ros,
//        name:'/tray_vel',
//        messageType: 'std_msgs/Float32'
//    });

    // Subscribers
    var error_sub = new ROSLIB.Topic({
        ros:ros,
        name:error_topic,
        messageType:'std_msgs/UInt16MultiArray'
    });

    error_sub.subscribe(function(message){
        var len = message.data.length;
        for (var i = 0; i < len; i++){
            var msg = message.data[i];
            $('#motor_status_'+i+" .error_status").text = msg;
        }
    });
    
    var target_speed_sub = new ROSLIB.Topic({
        ros:ros,
        name:target_speed_topic,
        messageType:'std_msgs/Int16MultiArray'
    });

    target_speed_sub.subscribe(function(message){
        var len = message.data.length;
        for (var i = 0; i < len; i++){
            var msg = message.data[i];
            $('#motor_status_'+i+" .target .speed").text = msg;
            $('#motor_status_'+i+" .target .progress-bar").val(msg/6400 + 0.5)
        }
    });
    
    var actual_speed_sub = new ROSLIB.Topic({
        ros:ros,
        name:actual_speed_topic,
        messageType:'std_msgs/Int16MultiArray'
    });

    actual_speed_sub.subscribe(function(message){
        var len = message.data.length;
        for (var i = 0; i < len; i++){
            var msg = message.data[i];
            $('#motor_status_'+i+" .actual .speed").text = msg;
            $('#motor_status_'+i+" .actual .progress-bar").val(msg/6400 + 0.5)
        }
    });
    
    var volt_sub = new ROSLIB.Topic({
        ros:ros,
        name:volt_topic,
        messageType:'std_msgs/UInt16MultiArray'
    });

    volt_sub.subscribe(function(message){
        var len = message.data.length;
        var sum = 0;
        for (var i = 0; i < len; i++){
            var msg = message.data[i];
            sum += msg;
            $('#motor_status_'+i+" .volt").text = msg + " V";
        }
        var avg = sum/4;
        var percent = (avg - 12)/(14.7-12) * 100;
        $('#battery_card .h5').txt = avg + " V" + " - " + percent + " %";
        if(percent > 75){
            $('#battery_card .text-md').removeClass('text-danger').removeClass('text-warning').addClass('text-success');
        }
        else if (percent < 25){
            $('#battery_card .text-md').removeClass('text-danger').removeClass('text-success').addClass('text-warning');
        }
        else{
            $('#battery_card .text-md').removeClass('text-success').removeClass('text-warning').addClass('text-danger');
        }
    });
    
    var curr_sub = new ROSLIB.Topic({
        ros:ros,
        name:curr_topic,
        messageType:'std_msgs/UInt16MultiArray'
    });

    curr_sub.subscribe(function(message){
        var len = message.data.length;
        for (var i = 0; i < len; i++){
            var msg = message.data[i];
            $('#motor_status_'+i+" .amps").text = msg + " A";
        }
    });
    
    var temp_sub = new ROSLIB.Topic({
        ros:ros,
        name:temps_topic,
        messageType:'std_msgs/UInt16MultiArray'
    });

    temp_sub.subscribe(function(message){
        var len = message.data.length;
        for (var i = 0; i < len; i++){
            var msg = message.data[i];
            $('#motor_status_'+i+" .temp").text = msg + " &deg;C";
        }
    });

    var enc_sub = new ROSLIB.Topic({
        ros:ros,
        name:enc_topic,
        messageType:'std_msgs/UInt16MultiArray'
    });

    enc_sub.subscribe(function(message){
        var len = message.data.length;
        for (var i = 0; i < len; i++){
            var msg = message.data[i];
            sum += msg;
            $('#motor_status_'+i+" .encoder").text = msg;
        }
    });
        
    var state_sub = new ROSLIB.Topic({
        ros:ros,
        name:state_topic,
        messageType:'std_msgs/String'
    });

    state_sub.subscribe(function(message){
        $('#state_card .h5').txt = message.data;
    });
    
    var target_sub = new ROSLIB.Topic({
        ros:ros,
        name:target_topic,
        messageType:'std_msgs/String'
    });

    target_sub.subscribe(function(message){
        $('#target_card .h5').txt = message.data;
    });
}

// Window initialization
window.onload = function () {
    initROS()
    initNodes()
    initVideo()
}
