// Run after page load
window.addEventListener("DOMContentLoaded", function(){
    'use strict';

    function setStatus(str) {
        document.getElementById("status").innerHTML = str;
        document.getElementById("output_spikes").src = "/plots/agg_output_spikes.png?" + new Date().getTime();
        document.getElementById("hidden_layer").src = "/plots/spiketrain.png?" + new Date().getTime();
    }

    var ros = new ROSLIB.Ros({
        url : 'ws://ids-shuttle-2:9090'
    });

    ros.on('connection', function() {
        console.log('Connected to websocket server.');
        setStatus("Connected");
    });

    ros.on('error', function(error) {
        console.log('Error connecting to websocket server: ', error);
        setStatus("Disconnected");
    });

    ros.on('close', function() {
        console.log('Connection to websocket server closed.');
        setStatus("Closing");
    });

    var statusSub = new ROSLIB.Topic({
        ros : ros,
        name : '/status',
        messageType : 'std_msgs/String'
    });

    statusSub.subscribe(function(message) {
        console.log('Received message on ' + statusSub.name + ': ' + message.data);
        setStatus(message.data);
    });

//    var outputSpikeSub = new ROSLIB.Topic({
//        ros : ros,
//        name : '/output_spikes',
//        messageType : 'sacc_rec_grasp/SpikeArray'
//    });
//
//    outputSpikeSub.subscribe(function(message) {
//        console.log('Received message on ' + outputSpikeSub.name + ': ' + message.data);
//    });
})
