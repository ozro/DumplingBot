var error_topic = "/motor_status/errors";

var trayVel = new ROSLIB.Topic({
    ros:ros,
    name:'/tray_vel',
    messageType: 'std_msgs/Float32'
});

var openVel = new ROSLIB.Message({
    data:1
});
var closeVel = new ROSLIB.Message({
    data:-1
});
var brakeVel = new ROSLIB.Message({
    data:0
});

var error_sub = new ROSLIB.Topic({
    ros:ros,
    name:error_topic,
    messageType:'std_msgs/UInt16MultiArray'
});

error_sub.subscribe(function(message){
    var data = message.data[4];
    if(data == 0){
        $('#openTray').attr("disabled", false);
    }
    else{
        $('#openTray').attr("disabled", true);
    }
});

$('#openTray').mousedown(function () {
    trayVel.publish(openVel);
    return false;
});
$('#openTray').mouseup(function () {
    trayVel.publish(brakeVel);
    return false;
});
$('#openTray').mouseout(function () {
    trayVel.publish(brakeVel);
    return false;
});
$('#closeTray').mousedown(function () {
    trayVel.publish(closeVel);
    return false;
});
$('#closeTray').mouseup(function () {
    trayVel.publish(brakeVel);
    return false;
});
$('#closeTray').mouseout(function () {
    trayVel.publish(brakeVel);
    return false;
});