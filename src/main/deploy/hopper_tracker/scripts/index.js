const rightPanel   = document.getElementById('rightPanel');
const imageFrame   = document.getElementById('imageFrame');
const measureBox   = document.getElementById('measureBox');
const measureLabel = document.getElementById('measureLabel');
const yReadout     = document.getElementById('yReadout');
const ratioReadout = document.getElementById('ratioReadout');
const holdBtn      = document.getElementById('holdBtn');
const btnState     = document.getElementById('btnState');
const dragHandle   = document.getElementById('dragHandle');
const ASPECT = 4 / 3;
function resizeFrame() {
  const pw = rightPanel.clientWidth;
  const ph = rightPanel.clientHeight;
  let fw, fh;
  if (pw / ph > ASPECT) {
    fh = ph * 0.88;
    fw = fh * ASPECT;
  } else {
    fw = pw * 0.88;
    fh = fw / ASPECT;
  }
  imageFrame.style.width  = fw + 'px';
  imageFrame.style.height = fh + 'px';
}
resizeFrame();
window.addEventListener('resize', resizeFrame);
function applyRatio(ratio) {
  ratio = Math.min(1, Math.max(0, ratio));
  const frameH = imageFrame.getBoundingClientRect().height;
  const pct = (ratio * 100).toFixed(1);
  measureBox.style.height = (ratio * 100) + '%';
  measureLabel.textContent = pct + '%';
  yReadout.textContent = Math.round((1 - ratio) * frameH) + 'px';
  ratioReadout.textContent = ratio.toFixed(3);
}
let isDragging = false;
rightPanel.addEventListener('click', (e) => {
  if (isDragging) return;
  const frameRect = imageFrame.getBoundingClientRect();
  const relY = e.clientY - frameRect.top;
  const ratio = 1 - (relY / frameRect.height);
  applyRatio(ratio);
});
function onDragStart(e) {
  e.preventDefault();
  isDragging = true;
  measureBox.classList.add('dragging');
  dragHandle.classList.add('active');
  rightPanel.classList.add('dragging');
  const moveEvent = e.type === 'touchstart' ? 'touchmove' : 'mousemove';
  const endEvent  = e.type === 'touchstart' ? 'touchend'  : 'mouseup';
  function onMove(ev) {
    const clientY = ev.touches ? ev.touches[0].clientY : ev.clientY;
    const frameRect = imageFrame.getBoundingClientRect();
    const relY = clientY - frameRect.top;
    const ratio = 1 - (relY / frameRect.height);
    applyRatio(ratio);
  }
  function onEnd() {
    setTimeout(() => { isDragging = false; }, 10);
    measureBox.classList.remove('dragging');
    dragHandle.classList.remove('active');
    rightPanel.classList.remove('dragging');
    document.removeEventListener(moveEvent, onMove);
    document.removeEventListener(endEvent, onEnd);
  }
  document.addEventListener(moveEvent, onMove);
  document.addEventListener(endEvent, onEnd);
}
dragHandle.addEventListener('mousedown', onDragStart);
dragHandle.addEventListener('touchstart', onDragStart, { passive: false });
// Hold button
let held = false;
function setHeld(val) {
  held = val;
  holdBtn.classList.toggle('held', val);
  btnState.textContent = val ? 'Paused' : 'Active';
  btnState.classList.toggle('active', val);
}
holdBtn.addEventListener('mousedown', () => setHeld(true));
document.addEventListener('mouseup', () => { if (held) setHeld(false); });
holdBtn.addEventListener('mouseleave', (e) => {
  if (!(e.buttons & 1)) setHeld(false);
});
holdBtn.addEventListener('touchstart', (e) => { e.preventDefault(); setHeld(true); });
holdBtn.addEventListener('touchend', () => setHeld(false));