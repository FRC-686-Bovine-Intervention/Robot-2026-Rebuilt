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
  const field = document.getElementById("fieldbounds");
  const box = document.getElementById("keepout");

  const fieldHeightPx = field.clientHeight;
  const fieldWidthPx = field.clientWidth;

  const fieldHeightFt = 26.25; // 26' 3" in feet

  // Top-left corner of the box (relative to bottom-left)
  const topLeftX = box.offsetLeft / fieldWidthPx;  // fraction across width
  const topLeftY = (fieldHeightPx - box.offsetTop) / fieldHeightPx; // fraction up from bottom

  // Bottom-right corner of the box
  const bottomRightX = (box.offsetLeft + box.offsetWidth) / fieldWidthPx;
  const bottomRightY = (fieldHeightPx - (box.offsetTop + box.offsetHeight)) / fieldHeightPx;

  // Convert to feet
  const topLeftX_ft = topLeftX * fieldWidthPx;  // optional: scale to width in whatever units
  const topLeftY_ft = topLeftY * fieldHeightFt;

  const bottomRightX_ft = bottomRightX * fieldWidthPx;
  const bottomRightY_ft = bottomRightY * fieldHeightFt;

  return {
    topLeft: { x: topLeftX, y: topLeftY, yFeet: topLeftY_ft },
    bottomRight: { x: bottomRightX, y: bottomRightY, yFeet: bottomRightY_ft }
  };
}