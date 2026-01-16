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
  font-family: Arial, sans-serif;
  text-align: center;
  background: white;
  padding: 15px;
  margin: 0;
  color: #333;
}

h2 {
  color: #333;
  margin-top: 10px;
  margin-bottom: 5px;
  font-size: 28px;
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
  border: none;
  border-radius: 10px;
  background: white;
  cursor: pointer;
  touch-action: none;
  font-weight: bold;
  box-shadow: 0 2px 8px rgba(0,0,0,0.15);
  transition: all 0.2s ease;
}

button:active {
  transform: scale(0.98);
  box-shadow: 0 1px 4px rgba(0,0,0,0.1);
}

.orange { color: orange; }
.blue { color: blue; }
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
const INTERVAL_MS = 280;

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
    ontouchstart="startContinuous('rope_out')" ontouchend="stopContinuous()"
    onmousedown="startContinuous('rope_out')" onmouseup="stopContinuous()" onmouseleave="stopContinuous()">rope out</button>

  <button class="blue"
    ontouchstart="startContinuous('comb_out')" ontouchend="stopContinuous()"
    onmousedown="startContinuous('comb_out')" onmouseup="stopContinuous()" onmouseleave="stopContinuous()">comb out</button>

  <button class="orange"
    ontouchstart="startContinuous('rope_in')" ontouchend="stopContinuous()"
    onmousedown="startContinuous('rope_in')" onmouseup="stopContinuous()" onmouseleave="stopContinuous()">rope in</button>

  <button class="blue"
    ontouchstart="startContinuous('comb_in')" ontouchend="stopContinuous()"
    onmousedown="startContinuous('comb_in')" onmouseup="stopContinuous()" onmouseleave="stopContinuous()">comb in</button>

  <button class="green"
    ontouchstart="startContinuous('turn_left')" ontouchend="stopContinuous()"
    onmousedown="startContinuous('turn_left')" onmouseup="stopContinuous()" onmouseleave="stopContinuous()">turn left</button>

  <button class="green"
    ontouchstart="startContinuous('turn_right')" ontouchend="stopContinuous()"
    onmousedown="startContinuous('turn_right')" onmouseup="stopContinuous()" onmouseleave="stopContinuous()">turn right</button>

  <button class="black" onclick="momentary('spoon_open')">spoon open</button>
  <button class="black" onclick="momentary('spoon_close')">spoon close</button>
</div>

<div class="arrows">
  <div></div>
  <button class="arrow-btn"
    ontouchstart="startContinuous('forward')" ontouchend="stopContinuous()"
    onmousedown="startContinuous('forward')" onmouseup="stopContinuous()" onmouseleave="stopContinuous()">&#8593;</button>
  <div></div>

  <button class="arrow-btn"
    ontouchstart="startContinuous('left')" ontouchend="stopContinuous()"
    onmousedown="startContinuous('left')" onmouseup="stopContinuous()" onmouseleave="stopContinuous()">&#8592;</button>
  <div></div>

  <button class="arrow-btn"
    ontouchstart="startContinuous('right')" ontouchend="stopContinuous()"
    onmousedown="startContinuous('right')" onmouseup="stopContinuous()" onmouseleave="stopContinuous()">&#8594;</button>

  <div></div>
  <button class="arrow-btn"
    ontouchstart="startContinuous('back')" ontouchend="stopContinuous()"
    onmousedown="startContinuous('back')" onmouseup="stopContinuous()" onmouseleave="stopContinuous()">&#8595;</button>
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
