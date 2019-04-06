$("#call").click(function(){
   alert("Help is on the way") 
});

$("#done").click(function(){
    trayVel.publish(closeVel);
});