var camera_topic = "/usb_cam/image_raw";
var map_topic = "/usb_cam/image_raw";
var error_topic = "/motor_status/errors";
var limit_topic = "/motor_status/limit_status";
var target_speed_topic = "/motor_status/target_speeds";
var actual_speed_topic = "/motor_status/current_speeds";
var temps_topic = "/motor_status/temps";
var curr_topic = "/motor_status/currents";
var volt_topic = "/motor_status/voltages";
var enc_topic = "/motor_status/encoder_counts";
var target_topic = "/target";
var state_topic = "/state";

$('#camera .text-primary').text("Feed: " + camera_topic);
video = $('#camera img')[0];
video.src = "http://" + robot_IP + ":8080/stream?topic=" + camera_topic + "&type=mjpeg&quality=80";

// Subscribers
var error_sub = new ROSLIB.Topic({
    ros:ros,
    name:error_topic,
    messageType:'std_msgs/UInt16MultiArray'
});

error_sub.subscribe(function(message){
    var len = message.data.length;
    for (var i = 0; i < len; i++){
        var data = message.data[i];
        switch(data){
            case 0:
                msg = "Running";
                $('#motor_status_'+i+" .progress-bar").removeClass('bg-danger').removeClass('bg-warning').addClass('bg-primary');
                break;
            case 1:
                msg = "Stopped";
                break;
            default:
                msg = "Error";
                $('#motor_status_'+i+" .progress-bar").removeClass('bg-primary').removeClass('bg-warning').addClass('bg-danger');
                break;
        }

        $('#motor_status_'+i+" .error_status").text(msg);
    }
});

var target_speed_sub = new ROSLIB.Topic({
    ros:ros,
    name:target_speed_topic,
    messageType:'std_msgs/Int16MultiArray'
});

target_speed_sub.subscribe(function(message){
    console.log("Received" + message.data);
    var len = message.data.length;
    for (var i = 0; i < len; i++){
        var msg = message.data[i];
        if(i > 1){
            msg *= -1;
        }
        $('#motor_status_'+i+" .target .speed").text(msg);
        $('#motor_status_'+i+" .target .progress-bar").css('width', (msg/6400 + 0.5) * 100 + '%');
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
        if(i > 1){
            msg *= -1;
        }
        $('#motor_status_'+i+" .actual .speed").text(msg);
        $('#motor_status_'+i+" .actual .progress-bar").css('width', (msg/6400 + 0.5)*100 + '%');
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
        $('#motor_status_'+i+" .volt").text(msg/1000);
    }
    var avg = sum/4;
    var percent = (avg/1000 - 12)/(14.7-12) * 100;
    avg = Math.round(avg*10/1000)/10;
    percent = Math.round(percent);
    $('#battery_card .h5').text(avg + " V" + " - " + percent + " %");
    if(percent > 75){
        $('#battery_card .text-md').removeClass('text-danger').removeClass('text-warning').addClass('text-success');
        $('#battery_card').removeClass('border-left-danger').removeClass('border-left-warning').addClass('border-left-success');
    }
    else if (percent < 25){
        $('#battery_card .text-md').removeClass('text-danger').removeClass('text-success').addClass('text-warning');
        $('#battery_card').removeClass('border-left-danger').removeClass('border-left-success').addClass('border-left-warning');
    }
    else{
        $('#battery_card .text-md').removeClass('text-success').removeClass('text-warning').addClass('text-danger');
        $('#battery_card').removeClass('border-left-success').removeClass('border-left-warning').addClass('border-left-danger');
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
        $('#motor_status_'+i+" .amps").text(msg/1000);
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
        $('#motor_status_'+i+" .temp").text(msg/10);
    }
});

var enc_sub = new ROSLIB.Topic({
    ros:ros,
    name:enc_topic,
    messageType:'rasp/EncoderCounts'
});

enc_sub.subscribe(function(message){
    var len = message.data.length;
    for (var i = 0; i < len; i++){
        var msg = message.data[i];
        sum += msg;
        $('#motor_status_'+i+" .encoder").text(msg);
    }
});
    
var state_sub = new ROSLIB.Topic({
    ros:ros,
    name:state_topic,
    messageType:'std_msgs/String'
});

state_sub.subscribe(function(message){
    $('#state_card .h5').text(message.data);
});

var target_sub = new ROSLIB.Topic({
    ros:ros,
    name:target_topic,
    messageType:'std_msgs/String'
});

target_sub.subscribe(function(message){
    $('#target_card .h5').text(message.data);
});