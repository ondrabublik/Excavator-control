#ifndef WEBPAGE_H
#define WEBPAGE_H

const char webpage[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
<meta name="viewport" content="width=device-width, initial-scale=1">

<style>
* {
  -webkit-user-select: none;
  -ms-user-select: none;
  user-select: none;
}

body {
  font-family: Arial;
  text-align: center;
  background: #f2f2f2;
}

.container {
  display: grid;
  grid-template-columns: 1fr 1fr;
  gap: 10px;
  max-width: 400px;
  margin: auto;
}

button {
  padding: 15px;
  font-size: 18px;
  border: 3px solid black;
  background: white;
  cursor: pointer;
  touch-action: none;
}

.orange { color: orange; }
.green  { color: green; }
.black  { color: black; }

.arrows {
  display: grid;
  grid-template-columns: 1fr 1fr 1fr;
  margin-top: 15px;
}

.arrow-btn {
  font-size: 30px;
  color: red;
}

.lights {
  display: grid;
  grid-template-columns: 1fr 1fr 1fr;
  margin-top: 15px;
}
</style>

<script>
const INTERVAL_MS = 100;

let timer = null;

function sendCmd(cmd) {
  fetch("/cmd?c=" + cmd);
}

// ===== kontinuální =====
function startContinuous(cmd) {
  if (timer) return;
  sendCmd(cmd);
  timer = setInterval(() => sendCmd(cmd), INTERVAL_MS);
}

function stopContinuous() {
  if (!timer) return;
  clearInterval(timer);
  timer = null;
  fetch("/cmd?c=stop");
}

// ===== jednorázové =====
function momentary(cmd) {
  sendCmd(cmd);
}
</script>
</head>

<body>
<h2>Excavator Control</h2>

<div class="container">
  <button class="orange"
    onmousedown="startContinuous('rope_out')" onmouseup="stopContinuous()"
    ontouchstart="startContinuous('rope_out')" ontouchend="stopContinuous()">rope out</button>

  <button class="orange"
    onmousedown="startContinuous('comb_out')" onmouseup="stopContinuous()"
    ontouchstart="startContinuous('comb_out')" ontouchend="stopContinuous()">comb out</button>

  <button class="orange"
    onmousedown="startContinuous('rope_in')" onmouseup="stopContinuous()"
    ontouchstart="startContinuous('rope_in')" ontouchend="stopContinuous()">rope in</button>

  <button class="orange"
    onmousedown="startContinuous('comb_in')" onmouseup="stopContinuous()"
    ontouchstart="startContinuous('comb_in')" ontouchend="stopContinuous()">comb in</button>

  <button class="green"
    onmousedown="startContinuous('turn_left')" onmouseup="stopContinuous()"
    ontouchstart="startContinuous('turn_left')" ontouchend="stopContinuous()">turn left</button>

  <button class="green"
    onmousedown="startContinuous('turn_right')" onmouseup="stopContinuous()"
    ontouchstart="startContinuous('turn_right')" ontouchend="stopContinuous()">turn right</button>

  <button class="black" onclick="momentary('spoon_open')">spoon open</button>
  <button class="black" onclick="momentary('spoon_close')">spoon close</button>
</div>

<div class="arrows">
  <div></div>
  <button class="arrow-btn"
    onmousedown="startContinuous('forward')" onmouseup="stopContinuous()"
    ontouchstart="startContinuous('forward')" ontouchend="stopContinuous()">&#8593;</button>
  <div></div>

  <button class="arrow-btn"
    onmousedown="startContinuous('left')" onmouseup="stopContinuous()"
    ontouchstart="startContinuous('left')" ontouchend="stopContinuous()">&#8592;</button>
  <div></div>

  <button class="arrow-btn"
    onmousedown="startContinuous('right')" onmouseup="stopContinuous()"
    ontouchstart="startContinuous('right')" ontouchend="stopContinuous()">&#8594;</button>

  <div></div>
  <button class="arrow-btn"
    onmousedown="startContinuous('back')" onmouseup="stopContinuous()"
    ontouchstart="startContinuous('back')" ontouchend="stopContinuous()">&#8595;</button>
  <div></div>
</div>

<div class="lights">
  <button onclick="momentary('L1')">L1</button>
  <button onclick="momentary('L2')">L2</button>
  <button onclick="momentary('L3')">L3</button>
</div>

</body>
</html>
)rawliteral";

#endif
