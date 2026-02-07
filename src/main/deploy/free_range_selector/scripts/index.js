import { NT4_Client } from "../lib/NT4.js";

const toRobotPrefix = "/FreeRangeSelector/ToRobot/";

const shouldKeepOutName = "KeepOut/ShouldNotKeepOut";
const keepOutTopLeftName = "KeepOut/TopLeft";
const keepOutBottomRightName = "KeepOut/BottomRight";

const shouldClimbName = "ShouldClimb";
const shouldNeutralZoneName = "ShouldNeutralZone";
const shouldAvoidBumpName = "ShouldAvoidBump";
const shouldNotAutoIntakeName = "ShouldNotAutoIntake";
const shouldStopToShootName = "ShouldStopToShoot";

let shouldKeepOut = true;
let topLeftX = 0;
let topLeftY = 0;
let bottomRightX = 0;
let bottomRightY = 0;

let shouldClimb = false;

const ntClient = new NT4_Client(
  window.location.hostname,
  "FreeRangeSelector",
  (topic) => {},
  (topic) => {},
  (topic, timestamp, value) => {},
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
});

const keepOutBoxDOM = document.getElementById("keepout");
const shouldKeepOutDOM = document.getElementById("respect_keepout");
const shouldClimbDOM = document.getElementById("climb");
const shouldNeutralZoneDOM = document.getElementById("neutralzone");
const shouldAvoidBumpDOM = document.getElementById("bump");
const shouldNotAutoIntakeDOM = document.getElementById("autointake");
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

  const handleClasses = ["nw","ne","sw","se","n","s","e","w"];
  const handleClicked = handleClasses.find(c => e.target.classList.contains(c));
  mode = handleClicked ? handleClicked : "move";

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

  keepOutBoxDOM.style.left = newL + "px";
  keepOutBoxDOM.style.top = newT + "px";
  keepOutBoxDOM.style.width = newW + "px";
  keepOutBoxDOM.style.height = newH + "px";

  const corners = getBoxRelativePositions();
  ntClient.addSample(toRobotPrefix + keepOutTopLeftName + "X", corners.topLeft.x);
  ntClient.addSample(toRobotPrefix + keepOutTopLeftName + "Y", corners.topLeft.y);
  ntClient.addSample(toRobotPrefix + keepOutBottomRightName + "X", corners.bottomRight.x);
  ntClient.addSample(toRobotPrefix + keepOutBottomRightName + "Y", corners.bottomRight.y);
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

  const FIELD_HEIGHT_M = 8.001;

  const pixelsPerMeter = boundsHeightPx / FIELD_HEIGHT_M;

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