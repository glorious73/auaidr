/*---------- Essentials ---------- */
const express    = require('express');
const router     = express.Router();
const cors       = require('cors');
const bodyParser = require('body-parser');
const {checkRobotConnection} = require('../helpers/checkRobotConnection'); // used to confirm connection
/*----- Enable CORS -----*/
router.use(function(req, res, next) {
  res.header("Access-Control-Allow-Origin", "*");
  res.header('Access-Control-Allow-Methods', 'GET,PUT,POST,DELETE');
  res.header('Access-Control-Allow-Headers', 'Content-Type');
  next();
});
/*---------- ROS ----------*/
// ROS itself
const ROSLIB  = require('roslib');
global.ros    = null;
let robotName = ''; // Initially
//----- Variables -----
// Keyboard (not implemented yet)
let twist  = null;
let teleop = null;
let cmdVel = null;
// LED
let arduinoled = null;
let cmdArduino = null;
// Duty Ratios (joystick)
let cmdDuty   = null;
let dutyMsg   = null; // ROS Message to be sent
let dutyArray = [0, 0]; // Will be defined as d1-d2 in moveAction() before publishing
// Live video stream
let vidStreamNode = null;
let imageData = null;
// Battery Management System
let bmsNode  = null;
let bmsArray = [0]; // will be 13 string values
// Battery percentage
let percentNode = null; // initially
let percentage = 0; // initially
/*---------------- Requests ----------------*/
// check if robot is already connected
router.post('/checkConnected', (req, res) => {
  if(ros !== null) {
    console.log('ROS exists');
    return res.json({success: true, name: robotName});
  } else {
    console.log('ROS doesnt exist');
    return res.json({success: false});
  }
});
// connect to robot
router.post('/connect', (req, res) => {
  ros = new ROSLIB.Ros({url : 'ws://' + req.body.ip + ':' + req.body.port});
  ros.on('connection', function() {
    console.log('Connected to websocket server.');
    robotName = req.body.ip.split(".")[0]; // whatever is the subdomain becomes robot name
    initVideoSubscriberTopic();
    initBMSSubscriberTopic();
    initDutyPublisherTopic();
    initPercentSubscriberTopic();
    initLedPublisherTopic();
    return res.json({success: true,  name: robotName, toastMessage: 'Connected to robot.'});
  })
  ros.on('error', function(error) {
    console.log('Error connecting to websocket server: ', error);
    return res.json({success: false, toastMessage: 'Error connecting to robot.'});
  });
});
// disconnect from robot
router.post('/disconnect', (req, res) => {
    // close ros connection and turn off all nodes
    if(ros !== null) {
        ros.close();
        turnOffAllNodes();
        ros = null;
    }
    return res.json({success: true, toastMessage: 'Connection to websocket server closed.'});
});

//----- stream video post request -----
router.post('/streamVideo', checkRobotConnection, (req, res) => {
  // subscribe and stream video. Then, return that as JSON to front-end
  vidStreamNode.subscribe(function(message) {
      imageData  = "data:image/jpg;base64," + message.data;
  });
  return res.json({imageData: imageData});
});
// stop video post request
router.post('/stopVideo', checkRobotConnection, (req, res) => {
  // Everything becomes null and unsubscribe from /usb_cam/Image_raw/compressed topic
  vidStreamNode.unsubscribe();
  imageData = null;
  console.log('Video not streamed anymore.');
  return res.json({imageData: imageData});
});

// Monitor BMS Voltage
router.post('/monitorBMSVoltage', checkRobotConnection, (req, res) => {
  // subscribe and receive voltage values. Then, return that as JSON to front-end
  bmsNode.subscribe(function(message) {
    // console.log(`Received String msg on /bms: ${JSON.stringify(message)}`); WORKING
    bmsArray = message.data.split(",");
  });
  return res.json({bmsArray: bmsArray});
});
router.post('/stopMonitorBMSVoltage', checkRobotConnection, (req, res) => {
  // Everything becomes null and unsubscribe from /bms
  bmsNode.unsubscribe();
  bmsArray = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
  console.log('BMS not monitored anymore.');
  return res.json({bmsArray: bmsArray});
});
// Monitor percentage
router.post('/monitorPercentage', (req, res) => {
  if(ros !== null) {
    // subscribe and receive percentage. Then, return that as JSON to front-end
    percentNode.subscribe(function(message) {
      percentage = message.data;
    });
    return res.json({monitoredPercent: true, percentage: percentage});
  }
  else {
    return res.json({monitoredPercent: false});
  }
});
//----- move robot manually -----
router.post('/moveRobot', checkRobotConnection, (req, res) => {
  // 1. determine whether req.body.linear velocity (Y-axis) is forward or backward (if it works, don't fix it).
  if(req.body.linear > 0) {
    // positive --> map a positive req.body.linear value (sin(theta))
    dutyArray[0] = parseInt(scale(req.body.linear, 0, 5, 0, req.body.currentSpeed));
  } else if(req.body.linear < 0) {
    // negative --> map a negative req.body.linear value
    dutyArray[0] = parseInt(scale(req.body.linear, 0, -5, 0, -req.body.currentSpeed));
  } else {
    dutyArray[0] = 0;
  }
  // 2. Determine whether req.body.angular velocity (X-axis) is right or left (or even 0) (note that sin() inverts the sign)
  if(req.body.angular < 0) {
    // positive --> robot moves right (left motor is faster)
    // Convert the declining X-axis readings from 0 to 5 (cos(theta)) into increasing 0 to -rpm value
    dutyArray[1] = parseInt(scale(req.body.angular, 0, 5, 0, -req.body.currentSpeed));
  } else if(req.body.angular > 0) {
    // negative --> robot moves left (right motor is faster)
    // Convert the declining X-axis readings from 0 to -5 into increasing 0 to rpm value
    dutyArray[1] = parseInt(scale(req.body.angular, 0, -5, 0, req.body.currentSpeed));
  } else {
    dutyArray[1] = dutyArray[0]; // same as the other side
  }
    // 3. publish to ROS
    cmdDuty.publish(dutyMsg);
    return res.json({toastMessage: 'cmdDuty published.'});
});
// light up LED lights request
router.post('/lightLed', checkRobotConnection, (req, res) => {
  if(req.body.buttonType === 'green') {
    cmdArduino = new ROSLIB.Topic({
      ros: ros,
      name: 'toggle_led_green',
      messageType: 'std_msgs/Empty'
    });
  } else if (req.body.buttonType === 'yellow') {
    cmdArduino = new ROSLIB.Topic({
      ros: ros,
      name: 'toggle_led_yellow',
      messageType: 'std_msgs/Empty'
    });
  } else if (req.body.buttonType === 'red') {
    cmdArduino = new ROSLIB.Topic({
      ros: ros,
      name: 'toggle_led_red',
      messageType: 'std_msgs/Empty'
    });
  } else {
    cmdArduino = new ROSLIB.Topic({
      ros: ros,
      name: 'toggle_led_blue',
      messageType: 'std_msgs/Empty'
    });
  }
  console.log('cmdArduino.name = ' + cmdArduino.name);
  cmdArduino.publish(arduinoled);
});
/*---------------- /Requests ----------------*/

/*---------------- Functions ----------------*/
//----- video subscriber -----
function initVideoSubscriberTopic() {
  // Initialize ROS Node for video
  vidStreamNode = new ROSLIB.Topic({
    ros: ros,
    name: '/usb_cam/image_raw/compressed',
    messageType: 'sensor_msgs/CompressedImage'
  });
  console.log("Video subscriber subscribed :)");
}
//----- BMS subscriber -----
function initBMSSubscriberTopic() {
  // Initialize ROS Node for BMS
  bmsNode = new ROSLIB.Topic({
    ros: ros,
    name: '/bms',
    messageType: 'std_msgs/String'
  });
  console.log('BMSNode subscribed');
}
//----- Percent subscriber -----
function initPercentSubscriberTopic() {
  // Initialize ROS Node for BMS
  percentNode = new ROSLIB.Topic({
    ros: ros,
    name: '/percentage',
    messageType: 'std_msgs/String'
  });
  console.log('PercentNode subscribed');
}
//----- duty ratios publisher -----
function initDutyPublisherTopic() {
  // Init Duty message
  dutyMsg = new ROSLIB.Message({
    data: dutyArray
  });
  cmdDuty = new ROSLIB.Topic({
    ros: ros,
    name: '/duty_ratios',
    messageType: 'std_msgs/Int16MultiArray'
  });
  // Register publisher within ROS system
  cmdDuty.advertise();
  console.log('cmdDuty is advertising');
}
//-----a function to create keyboard controller object | TO BE BACK -----
function initTeleopKeyboard() {
  console.log("W, A, S, D can be used to control the robot.");
    // Check if keyboard controller was already created
    if (teleop === null) {
        // Initialize the teleop.
        teleop = new KEYBOARDTELEOP.Teleop({
            ros: ros,
            topic: '/cmd_vel'
        });
    }
}
// LedPublisher
function initLedPublisherTopic() {
  // Init message to be sent to the Arduino
  arduinoled = new ROSLIB.Message({}); // message has nothing since the subscriber has a callback that does the job
  cmdArduino = new ROSLIB.Topic({
    ros: ros,
    name: 'toggle_led_yellow',
    messageType: 'std_msgs/Empty'
  });
  // Register Arduino JS publisher with the ROS system
  cmdArduino.advertise();
  console.log('cmdArduino is advertising');
}

// Turn off all communication with ROS
global.turnOffAllNodes = function () {
  if(vidStreamNode !== null) { vidStreamNode = null;  }
  if(dutyMsg !== null) {  dutyMsg = null;  }
  if(cmdDuty !== null) {  cmdDuty = null;  }
  if(teleop !== null) {  teleop = null;  }
  if(twist !== null) {  twist = null;  }
  if(cmdVel !== null) {  cmdVel = null;  }
  if(cmdArduino !== null) {  cmdArduino = null;  }
  if(arduinoled !== null) {  arduinoled = null;  }
  if(percentNode !== null) {  percentNode = null;  }
}
//----------- a function that maps a range of numbers to another -----------*/
// Credit: https://stackoverflow.com/questions/10756313/javascript-jquery-map-a-range-of-numbers-to-another-range-of-numbers
const scale = (num, in_min, in_max, out_min, out_max) => {
  return (num - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
/*---------------- /Functions ----------------*/
/*-------------------- Finally, export ros router --------------------*/
module.exports = router;
