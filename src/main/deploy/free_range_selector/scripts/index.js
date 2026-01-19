const overlay = document.getElementById("keepout");

let startX, startY, startW, startH, startL, startT;
let mode = null;

overlay.addEventListener("mousedown", e => {
  e.preventDefault();

  // If clicked on a handle → resize; else → move
  const handleClasses = ["nw","ne","sw","se","n","s","e","w"];
  const handleClicked = handleClasses.find(c => e.target.classList.contains(c));
  mode = handleClicked ? handleClicked : "move";

  startX = e.clientX;
  startY = e.clientY;
  startW = overlay.offsetWidth;
  startH = overlay.offsetHeight;
  startL = overlay.offsetLeft;
  startT = overlay.offsetTop;

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
    // Move the entire box
    newL = startL + dx;
    newT = startT + dy;
  } else {
    // Resize based on handle
    if (mode.includes("e")) newW = startW + dx;
    if (mode.includes("s")) newH = startH + dy;
    if (mode.includes("w")) { newW = startW - dx; newL = startL + dx; }
    if (mode.includes("n")) { newH = startH - dy; newT = startT + dy; }

    // Minimum size
    const minSize = 20;
    newW = Math.max(minSize, newW);
    newH = Math.max(minSize, newH);
  }

  overlay.style.left = newL + "px";
  overlay.style.top = newT + "px";
  overlay.style.width = newW + "px";
  overlay.style.height = newH + "px";

  const corners = getBoxRelativePositions();
console.log("Top-left corner (fraction):", corners.topLeft);
console.log("Bottom-right corner (fraction):", corners.bottomRight);

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

  const FIELD_HEIGHT_M = 8.001; // 26'3" in meters

  // Pixels-to-meters ratio based on height
  const pixelsPerMeter = boundsHeightPx / FIELD_HEIGHT_M;

  // Top-left corner relative to bottom-left
  const topLeftX_m = (boxRect.left - boundsRect.left) / pixelsPerMeter;
  const topLeftY_m = (boundsHeightPx - (boxRect.top - boundsRect.top)) / pixelsPerMeter;

  // Bottom-right corner
  const bottomRightX_m = (boxRect.right - boundsRect.left) / pixelsPerMeter;
  const bottomRightY_m = (boundsHeightPx - (boxRect.bottom - boundsRect.top)) / pixelsPerMeter;

  return {
    topLeft: { x: topLeftX_m, y: topLeftY_m },
    bottomRight: { x: bottomRightX_m, y: bottomRightY_m }
  };
}