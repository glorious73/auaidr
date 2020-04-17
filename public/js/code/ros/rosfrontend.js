'use strict'; // https://johnresig.com/blog/ecmascript-5-strict-mode-json-and-more/
// Note that 'canvasjsCode.js' is also a part of rosfrontend
/*-----Front-end Variables-----*/
let   liveRobotVideo     = null; // Will be defined in display video
let   publishImmediately = true; // Initially
let   robot_IP           = 'aidr.serveo.net';
let   robot_port         = '80';
let   robotSpeedRange    = null // Initially (will be defined in initTeleopKeyboard function)
let   currentSpeed       = 128; //Initially (will be the speed of the robot)
let   joystickManager    = null; // Initially (will be initialized in createJoystick function)
let   joystickContainer  = null; // Initially (will be initialized in createJoystick function)
let   videoFlag          = true;
let   operateFlag        = true;
let   fetchVideoIntervalID = null; // initially
const apiUrl = 'auaidr.herokuapp.com';
/*----------------------Event Listeners---------------------*/
// Add event listener for slider moves ("robot-speed" is the UI element on the Web Interface)
robotSpeedRange = document.getElementById("robot-speed");
robotSpeedRange.oninput = function () {
    currentSpeed = parseInt(robotSpeedRange.value);
    console.log("robot speed changed to " + currentSpeed + ".");
}
/*----------------------Event Listeners---------------------*/

function connectToRobot() {
  /*------ Read IP Address and Port # -----*/
  let robotIPElement   = document.getElementById('robotIP');
  let robotPortElement = document.getElementById('robotPort');
  robot_IP   = robotIPElement.value.length===0 ? 'aidr.serveo.net' : robotIPElement.value;
  robot_port = robotPortElement.value.length===0 ? '80' : robotPortElement.value;
  /*----- 'Connecting' button -----*/
  buttonConnecting();
  /*------ Communicate with server to connect to robot -----*/
  fetch(`https://${apiUrl}/ros/connect`, {
    method: 'post',
    body: JSON.stringify({ip: robot_IP, port: robot_port}),
    headers: new Headers({
      'Content-Type': 'application/json'
    })
  })
  .then(response => response.json())
  .then(data => {
    if(data.success===true) {
      showSuccess(data.toastMessage);
      buttonConnected(data.name);
    } else {
      showError(data.toastMessage);
      buttonConnect();
    }
  })
  .catch(err => {
    showError('Error connecting to Robot: ' + err);
    buttonConnect();
  });
}

function disconnectRobot() {
  fetch(`https://${apiUrl}/ros/disconnect`, {
    method: 'post',
    headers: new Headers({
      'Content-Type': 'application/json'
    })
  })
  .then(response => response.json())
  .then(data => {
    showInfo(data.toastMessage);
    buttonConnect();
  })
  .catch(err => {
    showError('Not Connected to Robot.');
    buttonConnect();
  });
}

/*---------------------------------Joystick functions---------------------------------*/
function activateJoystick() {
    operateFlag = operateFlag=== false? true : false; //switch implementation
    if(operateFlag === false) {
      createJoystick();
      joystickContainer.style.visibility = 'Visible';
    } else {
      joystickContainer.style.visibility = 'hidden';
      // joystickManager = null;
    }
}
function createJoystick() {
    // Scope resolution for some variables
    let lin, ang = 0;
    let joystick = null;
    let keptPublishingIntervalId = 0;
    // Check if joystick was already created
    if (joystickManager == null) {
      joystickContainer = document.getElementById('joystick');
      // https://yoannmoinet.github.io/nipplejs/ | Amjad configured the options (change for different behaviour)
      let options = {
          zone: joystickContainer,
          position: { left: 50 + '%', top: 50 + '%' },
          multitouch: false,
          maxNumberOfNipples: 1,
          mode: 'static',
          size: 150,
          color: '#72bcd4',
          restJoystick: true
      };
      joystickManager = nipplejs.create(options);
    }
    // event listener for joystick drag
    joystickManager.on('move', function (evt, j) {
        joystick = j;
        // First, if the joystick was stationary and publishing, stop it with clearInterval()
        clearInterval(keptPublishingIntervalId);
        // joystickjs returns direction is screen coordiantes
        // We need to rotate it in such a way dragging towards screen top will move robot forward
        var direction = joystick.angle.degree - 90;
        if (direction > 180) {
            direction = -(450 - joystick.angle.degree);
        }
        // Convert angles to radians and scale linear and angular speed
        lin = Math.cos(direction / 57.29) * joystick.distance * 0.05;
        ang = Math.sin(direction / 57.29) * joystick.distance * 0.05;
        // Delay between consecutive message publications to prevent DOS (100 ms)
        if (publishImmediately) {
            publishImmediately = false;
            moveRobot(lin, ang);
            setTimeout(function () {
                publishImmediately = true;
            }, 100);
        }
        // Check if we're not in center to keep publishing velocities
        keptPublishingIntervalId = setInterval(restJoystick, 10);
    });
    // event listener for joystick release, always send stop message
    joystickManager.on('end', function () {
        moveRobot(0, 0);
        clearInterval(keptPublishingIntervalId);
    });
    // a function that publishes velocities when joystick is rested not in center
    const restJoystick = () => {
      if(joystick.position.x != 0 && joystick.position.y != 0) {
        moveRobot(lin, ang);
      }
    }
}
// move the robot through the back-end server
function moveRobot(linear, angular) {
  // Send an asynchronous request to the server to move the robot
  fetch(`https://${apiUrl}/ros/moveRobot`, {
      method: 'post',
      body: JSON.stringify({linear: linear, angular: angular, currentSpeed: currentSpeed}),
      headers: new Headers({
        'Content-Type': 'application/json'
      })
  })
  .then(response => response.json())
  .then(data => {
    if(data.message === 'Robot not connected.') {
      // Tell user robot is not connected
      showError(data.message);
    }
    console.log(`Moved robot by ${data.toastMessage}`);
  })
  .catch(err => {
    console.log(err);
    showError('Error. Please make sure Robot is connected.');
  });
}

/*---------- Display live video on the interface function ----------*/
function displayLiveVideo() {
  videoFlag = videoFlag===false ? true : false; // switch implementation
    // retreive image holder on DOM
    liveRobotVideo = document.getElementById("liveRobotVideo");
    if(videoFlag===false) {
      // Make the video tag visible
      liveRobotVideo.style.display = 'block';
      // Send an asynchronous request to the server to get the video
      fetchVideoIntervalID = setInterval(() => {
        if(videoFlag===false) {
          fetch(`https://${apiUrl}/ros/streamVideo`, {
              method: 'post',
              body: JSON.stringify({videoFlag: false}),
              headers: new Headers({
                'Content-Type': 'application/json'
              })
          })
          .then(response => response.json())
          .then(data => {
            if(data.message === 'Robot not connected.') {
              // Tell user robot is not connected
              showError(data.message);
            } else {
              // Show the live video
              liveRobotVideo.src =  data.imageData;
            }
          })
          .catch(err => showError('Error. Please make sure Robot is connected.'));
        }
      }, 1000);
    } else {
      // Everything becomes null and unsubscribe from /usb_cam/Image_raw/compressed topic
      clearInterval(fetchVideoIntervalID);
      fetch(`https://${apiUrl}/ros/stopVideo`, {
        method: 'post',
        headers: new Headers({
          'Content-Type': 'application/json'
        })
      })
      .then(response => response.json())
      .then(data => {
        // Tell user we stopped
        showInfo('Stopped receiving live video');
        // Make the video (image) element invisible
        liveRobotVideo.style.display = 'none';
      })
      .catch(err => showError('Error. Please make sure Robot is connected.'));
      liveRobotVideo.setAttribute('src', null);
      console.log('Video not displayed anymore.');
    }
}

// Check if robot connected when user refreshes
window.addEventListener('DOMContentLoaded', (e) => {
  /*------ Communicate with server -----*/
  fetch(`https://${apiUrl}/ros/checkConnected`, {
    method: 'post',
    headers: new Headers({
      'Content-Type': 'application/json'
    })
  })
  .then(response => response.json())
  .then(data => {
    if(data.success===true) {
      buttonConnected(data.name);
    } else {
      buttonConnect();
    }
  })
  .catch(err => {
    console.log(`Error: ${err}`);
    buttonConnect();
  });
});

/*----- Button state function -----*/
const buttonConnected = (name) => {
  // change the name of the robot (try catch for receving from server)
  try {
    document.getElementById('robotName').innerText = name.charAt(0).toUpperCase() + data.name.slice(1); // Name of the robot on the interface
  } catch(err) {
    document.getElementById('robotName').innerText = name;
  }
  // change the button state to 'connected'
  btnConnect.classList.remove('btn-primary');
  btnConnect.classList.add('btn-success');
  btnConnect.innerText = 'Connected';
}

const buttonConnecting = () => {
  /*----- 'Connecting' button -----*/
  btnConnect.classList.remove('btn-success');
  btnConnect.classList.add('btn-primary');
  btnConnect.innerText = "Connecting...";
}

const buttonConnect = () => {
  document.getElementById('robotName').innerText = 'None'; // Double check
  btnConnect.classList.remove('btn-success');
  btnConnect.classList.add('btn-primary');
  btnConnect.innerText = 'Connect to Robot';
}
