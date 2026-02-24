import { NT4_Client } from "../lib/NT4.js";

const toRobotPrefix = "/DriverDashboard/ToRobot/";
const fromRobotPrefix = "/DriverDashboard/FromRobot/";

const shouldRespectDriverInputName = "ShouldRespectDriverInput";
const hopperHeightName = "HopperHeight";

const shouldAvoidTrenchName = "ShouldAvoidTrench";
const shouldAvoidBumpName = "ShouldAvoidBump";
const shouldAutoIntakeName = "ShouldAutoIntake";
const shouldStopToShootName = "ShouldStopToShoot";
const intakeDeployedName = "IntakeDeployed";

let shouldRespectDriverInput = true;
let hopperHeight = 0;

let shouldAvoidTrench = true;
let shouldAvoidBump = true;
let shouldAutoIntake = true;
let shouldStopToShoot = false;
let intakeDeployed = false;

const ntClient = new NT4_Client(
  window.location.hostname,
  "DriverDashboard",
  (topic) => {},
  (topic) => {},
  (topic, timestamp, value) => {
	if (topic === fromRobotPrefix + intakeDeployedName) {
		intakeDeployed = value;
	} else {
		return;
	}
	updateUI();
  },
  () => {},
  () => {}
);

window.addEventListener("load", () => {
  ntClient.subscribe(
    [
      "/AdvantageKit/Timestamp" //Required due to a bug in networktables
    ]
  );

  ntClient.publishTopic(toRobotPrefix + shouldKeepOutName, "boolean");
  ntClient.publishTopic(toRobotPrefix + keepOutTopLeftName + "X", "double");
  ntClient.publishTopic(toRobotPrefix + keepOutTopLeftName + "Y", "double");
  ntClient.publishTopic(toRobotPrefix + keepOutBottomRightName + "X", "double");
  ntClient.publishTopic(toRobotPrefix + keepOutBottomRightName + "Y", "double");
  ntClient.publishTopic(toRobotPrefix + shouldClimbName, "boolean");
  ntClient.publishTopic(toRobotPrefix + shouldNeutralZoneName, "boolean");
  ntClient.publishTopic(toRobotPrefix + shouldAvoidBumpName, "boolean");
  ntClient.publishTopic(toRobotPrefix + shouldNotAutoIntakeName, "boolean");
  ntClient.publishTopic(toRobotPrefix + shouldStopToShootName, "boolean");
  ntClient.connect();

  updateUI();
});

const fieldDOM = document.getElementById("field");
const keepOutBoxDOM = document.getElementById("keepout");
const shouldKeepOutDOM = document.getElementById("respect_keepout");
const shouldClimbDOM = document.getElementById("climb");
const shouldNeutralZoneDOM = document.getElementById("neutralzone");
const shouldAvoidBumpDOM = document.getElementById("bump");
const shouldAutoIntakeDOM = document.getElementById("autointake");
const shouldStopToShootDOM = document.getElementById("sotm");

window.addEventListener("load", () => {
  bind(shouldKeepOutDOM, () => {
    ntClient.addSample(toRobotPrefix + shouldKeepOutName, shouldKeepOutDOM.checked);
    console.log(shouldKeepOutDOM.checked);
  });
  bind(shouldClimbDOM, () => {
    ntClient.addSample(toRobotPrefix + shouldClimbName, shouldClimbDOM.checked);
    console.log(shouldClimbDOM.checked);
  });
  bind(shouldNeutralZoneDOM, () => {
	ntClient.addSample(toRobotPrefix + shouldNeutralZoneName, shouldNeutralZoneDOM.checked);
	console.log(shouldNeutralZoneDOM.checked);
  });
  bind(shouldAvoidBumpDOM, () => {
	ntClient.addSample(toRobotPrefix + shouldAvoidBumpName, shouldAvoidBumpDOM.checked);
	console.log(shouldAvoidBumpDOM.checked);
  });
  bind(shouldNotAutoIntakeDOM, () => {
	ntClient.addSample(toRobotPrefix + shouldNotAutoIntakeName, shouldNotAutoIntakeDOM.checked);
	console.log(shouldNotAutoIntakeDOM.checked);
  });
  bind(shouldStopToShootDOM, () => {
	ntClient.addSample(toRobotPrefix + shouldStopToShootName, shouldStopToShootDOM.checked);
	console.log(shouldStopToShootDOM.checked);
  });
});

let startX, startY, startW, startH, startL, startT;
let mode = null;

keepOutBoxDOM.addEventListener("mousedown", e => {
  e.preventDefault();

  const handleClasses = ["n"];
  const handleClicked = handleClasses.find(c => e.target.classList.contains(c));
  mode = handleClicked;

  startX = e.clientX;
  startY = e.clientY;
  startW = keepOutBoxDOM.offsetWidth;
  startH = keepOutBoxDOM.offsetHeight;
  startL = keepOutBoxDOM.offsetLeft;
  startT = keepOutBoxDOM.offsetTop;

  document.addEventListener("mousemove", onMove);
  document.addEventListener("mouseup", stopDrag);
});

function onMove(e) {
  const dx = e.clientX - startX;
  const dy = e.clientY - startY;

  let newL = startL;
  let newT = startT;
  let newW = startW;
  let newH = startH;

  if (mode === "move") {
    newL = startL + dx;
    newT = startT + dy;
  } else {
    if (mode.includes("e")) newW = startW + dx;
    if (mode.includes("s")) newH = startH + dy;
    if (mode.includes("w")) { newW = startW - dx; newL = startL + dx; }
    if (mode.includes("n")) { newH = startH - dy; newT = startT + dy; }

    const minSize = 20;
    newW = Math.max(minSize, newW);
    newH = Math.max(minSize, newH);
  }

  keepOutBoxDOM.style.top = newT + "px";
  keepOutBoxDOM.style.width = newW + "px";
  keepOutBoxDOM.style.height = newH + "px";

//   updateUI();
  fieldDOM.style.backgroundImage = "../media/retracted.png";
  const corners = getBoxRelativePositions();
  ntClient.addSample(toRobotPrefix + hopperHeightName, corners.topLeft.y);
// console.log("Top-left corner (fraction):", corners.topLeft);
// console.log("Bottom-right corner (fraction):", corners.bottomRight);

}

function stopDrag() {
  document.removeEventListener("mousemove", onMove);
  document.removeEventListener("mouseup", stopDrag);
}

function getBoxRelativePositions() {
  const bounds = document.getElementById("fieldbounds");
  const box = document.getElementById("keepout");

  const boundsRect = bounds.getBoundingClientRect();
  const boxRect = box.getBoundingClientRect();

  const boundsWidthPx = boundsRect.width;
  const boundsHeightPx = boundsRect.height;

  const HOPPER_WIDTH_IN = 8.001;

  const pixelsPerMeter = boundsWidthPx / HOPPER_WIDTH_IN;

  const topLeftX_m = (boxRect.left - boundsRect.left) / pixelsPerMeter;
  const topLeftY_m = (boundsHeightPx - (boxRect.top - boundsRect.top)) / pixelsPerMeter;

  const bottomRightX_m = (boxRect.right - boundsRect.left) / pixelsPerMeter;
  const bottomRightY_m = (boundsHeightPx - (boxRect.bottom - boundsRect.top)) / pixelsPerMeter;

  return {
    topLeft: { x: topLeftX_m, y: topLeftY_m },
    bottomRight: { x: bottomRightX_m, y: bottomRightY_m }
  };
}

function bind(element, callback) {
   let lastActivation = 0;

  element.addEventListener("click", (event) => {
    const now = new Date().getTime();
    if (now - lastActivation > 250) {
      callback();
      lastActivation = now;
    }
    // DO NOT preventDefault! Let checkbox animate normally
  });

  // Optional: handle touch for mobile
  element.addEventListener("touchstart", (event) => {
    const now = new Date().getTime();
    if (now - lastActivation > 250) {
      callback();
      lastActivation = now;
    }
  });
}

function unpackInt(n, totalBits, size) {
  let values = [];
  for (let i = 0; i < totalBits / size; i++) {
    values.unshift(n & ((1 << size) - 1));
    n >>= size;
  }
  return values;
}

function packInt(values, size) {
  let n = 0;
  for (let i = 0; i < values.length; i++) {
    n = (n << size) | values[i];
  }
  return n;
}

function updateUI() {
	shouldAutoIntakeDOM.checked = shouldAutoIntake;

	fieldDOM.style.backgroundImage = intakeDeployed ? "../media/deployed.png" : "../media/retracted.png";
	keepOutBoxDOM.style.width = intakeDeployed ? "69vh" : "37.5vh";
}