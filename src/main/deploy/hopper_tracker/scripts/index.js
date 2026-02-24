import { NT4_Client } from "../lib/NT4.js";

const toRobotPrefix = "/DriverDashboard/ToRobot/";
const toDashboardPrefix = "/DriverDashboard/FromRobot/";

const hopperHeightName = "HopperHeight";
const intakeDeployedName = "IntakeDeployed";

let hopperHeight = 0;

let intakeDeployed = true;

const ntClient = new NT4_Client(
  window.location.hostname,
  "DriverDashboard",
  (topic) => {},
  (topic) => {},
  (topic, timestamp, value) => {
	  if (topic === toDashboardPrefix + intakeDeployedName) {
		  intakeDeployed = value;
	  } else if (topic === toDashboardPrefix + hopperHeightName) {
      hopperHeight = value;
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
      toDashboardPrefix + hopperHeightName,
      toDashboardPrefix + intakeDeployedName
    ]
  );

  ntClient.publishTopic(toRobotPrefix + hopperHeight, "double");
  ntClient.connect();

  updateUI();
});

const fieldDOM = document.getElementById("field");
const hopperBoxDOM = document.getElementById("hopper");

let startX, startY, startW, startH, startL, startT;
let mode = null;

hopperBoxDOM.addEventListener("mousedown", e => {
  e.preventDefault();

  const handleClasses = ["n"];
  const handleClicked = handleClasses.find(c => e.target.classList.contains(c));
  mode = handleClicked;

  startX = e.clientX;
  startY = e.clientY;
  startW = hopperBoxDOM.offsetWidth;
  startH = hopperBoxDOM.offsetHeight;
  startL = hopperBoxDOM.offsetLeft;
  startT = hopperBoxDOM.offsetTop;

  document.addEventListener("mousemove", onMove);
  document.addEventListener("mouseup", stopDrag);
});

function onMove(e) {
  if (mode !== "n") return; // Only allow north handle resizing

  const dy = e.clientY - startY;

  let newH = startH - dy;
  let newT = startT + dy;

  const minSize = 20;

  // If shrinking past minimum, clamp height and adjust top correctly
  if (newH < minSize) {
    newH = minSize;
    newT = startT + (startH - minSize);
  }

  hopperBoxDOM.style.top = newT + "px";
  hopperBoxDOM.style.height = newH + "px";

  const corners = getBoxRelativePositions();
  ntClient.addSample(toRobotPrefix + hopperHeightName, corners.topLeft.y);

  updateUI();
}

function stopDrag() {
  document.removeEventListener("mousemove", onMove);
  document.removeEventListener("mouseup", stopDrag);
}

function getBoxRelativePositions() {
  const bounds = document.getElementById("hopperbounds");
  const box = document.getElementById("hopper");

  const boundsRect = bounds.getBoundingClientRect();
  const boxRect = box.getBoundingClientRect();

  const boundsWidthPx = boundsRect.width;
  const boundsHeightPx = boundsRect.height;

  const HOPPER_WIDTH_IN = 24.75;

  const pixelsPerIn = boundsWidthPx / HOPPER_WIDTH_IN;

  const topLeftX_m = (boxRect.left - boundsRect.left) / pixelsPerIn;
  const topLeftY_m = (boundsHeightPx - (boxRect.top - boundsRect.top)) / pixelsPerIn;

  const bottomRightX_m = (boxRect.right - boundsRect.left) / pixelsPerIn;
  const bottomRightY_m = (boundsHeightPx - (boxRect.bottom - boundsRect.top)) / pixelsPerIn;

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
  });

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
  const bounds = document.getElementById("hopperbounds");
  const boundsRect = bounds.getBoundingClientRect();

  const boundsWidthPx = boundsRect.width;
  const boundsHeightPx = boundsRect.height;

  const HOPPER_WIDTH_IN = 24.75;
  const pixelsPerIn = boundsWidthPx / HOPPER_WIDTH_IN;

  // ---- Apply intake width ----
  hopperBoxDOM.style.width = intakeDeployed ? "69vh" : "37.5vh";

  // Wait one frame for width to apply
  requestAnimationFrame(() => {

    // Convert real-world hopperHeight (meters) to pixels
    const newHeightPx = hopperHeight * pixelsPerIn;

    const minSize = 20;
    const clampedHeight = Math.max(minSize, newHeightPx);

    // Keep bottom anchored (resize from north)
    const currentBottom =
      hopperBoxDOM.offsetTop + hopperBoxDOM.offsetHeight;

    hopperBoxDOM.style.height = clampedHeight + "px";
    hopperBoxDOM.style.top =
      (currentBottom - clampedHeight) + "px";
  });

  // Background image
  fieldDOM.style.backgroundImage = intakeDeployed
    ? 'url("./media/deployed.png")'
    : 'url("./media/retracted.png")';
}