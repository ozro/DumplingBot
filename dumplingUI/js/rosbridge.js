var robot_IP = "localhost";

var ros;
// Initialize ROS connection
console.log('Initializing connection to websocket at \"ws://\"' + robot_IP + ":9090\"");
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

var volt_topic = "/motor_status/voltages";
var target_topic = "/target";
var state_topic = "/state";

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

var estop_pub = new ROSLIB.Topic({
    ros:ros,
    name:'/motor_cmd/start',
    messageType:'std_msgs/Bool'
})

$("#estop").click(function(){
    msg = new ROSLIB.Message({
        data:false
    });

    estop_pub.publish(msg);
});

$("#start").click(function(){
    msg = new ROSLIB.Message({
        data:true
    });

    estop_pub.publish(msg);
});