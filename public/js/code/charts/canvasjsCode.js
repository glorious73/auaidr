// Show different charts in this file
/* BMS chart: Monitor the status of 12 batteries in real-time
              and visualize them with the BMS chart.
*/
// Variables
let bmsFlag       = true; // initially
let perecntFlag   = true; // initially
let fetchBMSIntervalID = 0;
let voltageChartContainer;
let chart; // scope resolution
let percentageChartContainer;
let percentageChart; // scope resolution
const canvasApiUrl = apiUrl;
/*--------------- Battery Percentage ---------------*/
window.onload = function () {
  // grab the element
  percentageChartContainer = document.querySelector('#percentageChartContainer');
  // make the chart
  percentageChart = new CanvasJS.Chart("percentageChartContainer", {
    explodeOnClick: true,
  	animationEnabled: true,
    backgroundColor: "transparent",
  	title:{
  		text: "",
  		horizontalAlign: "center"
  	},
  	data: [{
  		type: "doughnut",
  		startAngle: 90,
  		toolTipContent: "<b>{label}:</b> {y} (#percent%)",
  		dataPoints: [
  			{ y: 0, label: "Charged" },
  			{ y: 100, label: "Discharged"}
  		]
  	}]
  });
  percentageChart.render();
  setInterval(monitorPercentage, 5000); // 10 seconds not to overwhelm server
}

function monitorPercentage() {
    fetch(`https://${canvasApiUrl}/ros/monitorPercentage`, {
        method: 'post',
        headers: new Headers({
          'Content-Type': 'application/json'
        })
    })
    .then(response => response.json())
    .then(data => {
      if(data.monitoredPercent === true) {
        // 1. Make the chart visible
        percentageChart.options.data[0].visible = true;
        // 2. Receive new percentage and update chart
        percentageChart.options.data[0].dataPoints[0].y = parseFloat(data.percentage);
        percentageChart.options.data[0].dataPoints[1].y = parseFloat(100-data.percentage);
        percentageChart.render();
      }
    })
    .catch(err => {
      console.log('Not connected to robot --> No percentage received.');
    });
}
/*--------------- /BatteryPercentage ---------------*/
/*--------------- Battery Voltages ---------------*/
// 1. Initialize the data points representing batteries
let voltageDataPoints = [
  {label: 'S1', y: 0},
  {label: 'S2', y: 0},
  {label: 'S3', y: 0},
  {label: 'S4', y: 0},
  {label: 'S5', y: 0},
  {label: 'S6', y: 0},
  {label: 'S7', y: 0},
  {label: 'S8', y: 0},
  {label: 'S9', y: 0},
  {label: 'S10', y: 0},
  {label: 'S11', y: 0},
  {label: 'S12', y: 0},
  {label: 'S13', y: 0}
];
// 2. Get the variable from the HTML chartContainer
voltageChartContainer = document.querySelector('#voltageChartContainer');
if(voltageChartContainer) {
  chart = new CanvasJS.Chart('voltageChartContainer', {
    animationEnabled: true,
    theme: 'light2',
    title: {
      text: 'BMS'
    },
    axisY: {
		  title: "Voltage",
		  suffix: "V",
      interval: 0.5,
      viewportMinimum: 0,
      viewportMaximum: 5,
      minimum: 0,
      maximum: 5,
		  includeZero: false
  	},
	  axisX: {
	     title: "Battery"
	  },
    data: [
      {
        type: 'column',
        yValueFormatString: "#,##0.0#\"V\"",
        dataPoints: voltageDataPoints
      }
    ]
  });
  // chart.render();
}
function createChartAgain() {
  voltageChartContainer = document.querySelector('#voltageChartContainer');
  if(voltageChartContainer) {
    chart = new CanvasJS.Chart('voltageChartContainer', {
      animationEnabled: true,
      theme: 'light2',
      title: {
        text: 'BMS'
      },
      axisY: {
  		  title: "Voltage",
  		  suffix: "V",
        interval: 0.5,
        viewportMinimum: 0,
        viewportMaximum: 5,
        minimum: 0,
        maximum: 5,
  		  includeZero: false
    	},
  	  axisX: {
  	     title: "Battery"
  	  },
      data: [
        {
          type: 'column',
          yValueFormatString: "#,##0.0#\"V\"",
          dataPoints: voltageDataPoints
        }
      ]
    });
    // chart.render();
  }
}
function monitorBMSVoltage() {
    bmsFlag = bmsFlag===false ? true : false; // switch implementation
    // Send an asynchronous request to the server to get the voltage values
    if(bmsFlag===false) {
      fetchBMSIntervalID = setInterval(() => {
        if(bmsFlag===false) {
          fetch(`https://${canvasApiUrl}/ros/monitorBMSVoltage`, {
              method: 'post',
              body: JSON.stringify({bmsFlag: false}),
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
              // 1. Make the chart visible
              createChartAgain();
              chart.options.data[0].visible = true;
              // 2. Receive new values and assign to bmsArray
              for (let i = 0; i < data.bmsArray.length; i++) {
                // 3. Update voltageDataPoints and chart
                chart.options.data[0].dataPoints[i].y = parseFloat(data.bmsArray[i]);
                chart.render();
              }
            }
          })
          .catch(err => {
            showError('Error: Please make sure robot is connected.');
            console.log(err);

          });
        }
      }, 2000);
    } else {
      // Unsubscribe from /bms topic and 'zero' the chart
      clearInterval(fetchBMSIntervalID);
      fetch(`https://${canvasApiUrl}/ros/stopMonitorBMSVoltage`, {
        method: 'post',
        headers: new Headers({
          'Content-Type': 'application/json'
        })
      })
      .then(response => response.json())
      .then(data => {
        // Tell user we stopped
        showInfo('Stopped monitoring BMS');
        // hide the chart
        voltageChartContainer.innerHTML = '';
      })
      .catch(err => {
        showError(err);
      });

    }
}

function monitorCurrent() {
  showInfo('Not implemented yet');
}

function monitorTemperature() {
  showInfo('Not implemented yet');
}
