/*
 * Gamepad API Test
 * Written in 2013 by Ted Mielczarek <ted@mielczarek.org>
 *
 * To the extent possible under law, the author(s) have dedicated all copyright and related and neighboring rights to this software to the public domain worldwide. This software is distributed without any warranty.
 *
 * You should have received a copy of the CC0 Public Domain Dedication along with this software. If not, see <http://creativecommons.org/publicdomain/zero/1.0/>.
 */
var haveEvents = 'GamepadEvent' in window;
var haveWebkitEvents = 'WebKitGamepadEvent' in window;
var controllers = {};
var rAF = window.mozRequestAnimationFrame ||
  window.webkitRequestAnimationFrame ||
  window.requestAnimationFrame;

function connecthandler(e) {
  addgamepad(e.gamepad);
}
function addgamepad(gamepad) {
  controllers[gamepad.index] = gamepad; 
  xBoxLeft = gamepad.buttons[14];
  xBoxRight = gamepad.buttons[15];
  xBoxUp = gamepad.buttons[12];
  xBoxDown = gamepad.buttons[13];
  xBoxA = gamepad.buttons[0];
  
//   var d = document.createElement("div");
//   d.setAttribute("id", "controller" + gamepad.index);
//   var t = document.createElement("h1");
//   t.appendChild(document.createTextNode("gamepad: " + gamepad.id));
//   d.appendChild(t);
//   var b = document.createElement("div");
//   b.className = "buttons";
//   for (var i=0; i<gamepad.buttons.length; i++) {
//     var e = document.createElement("span");
//     e.className = "button";
//     //e.id = "b" + i;
//     e.innerHTML = i;
//     b.appendChild(e);
//   }
//   d.appendChild(b);
//   var a = document.createElement("div");
//   a.className = "axes";
//   for (i=0; i<gamepad.axes.length; i++) {
//     e = document.createElement("meter");
//     e.className = "axis";
//     //e.id = "a" + i;
//     e.setAttribute("min", "-1");
//     e.setAttribute("max", "1");
//     e.setAttribute("value", "0");
//     e.innerHTML = i;
//     a.appendChild(e);
//   }
//   d.appendChild(a);
//   document.getElementById("start").style.display = "none";
//   document.body.appendChild(d);
  rAF(updateStatus);
}

function disconnecthandler(e) {
  removegamepad(e.gamepad);
}

function removegamepad(gamepad) {
//   var d = document.getElementById("controller" + gamepad.index);
//   document.body.removeChild(d);
  delete controllers[gamepad.index];
}

function updateStatus() {
  scangamepads();
  xBoxLeft = gamepad.buttons[14].pressed;
  xBoxRight = gamepad.buttons[15].pressed;
  xBoxUp = gamepad.buttons[12].pressed;
  xBoxDown = gamepad.buttons[13].pressed;
  xBoxA = gamepad.buttons[0].pressed;
//   if (xBoxLeft.pressed) {
//     console.log("LEFT PRESSED");
//   }
// //   if (xBoxA.pressed) {
// //     console.log("A PRESSED");
// //   }
//   setInterval(function(){
//     isPressed = xBoxA.pressed;
//     // document.getElementById("button").innerHTML = isPressed;
//     if (isPressed) {
//         console.log("A PRESSED");
//     }
    
//   }, 1000)
//   setTimeout(() => rAF(updateStatus), 1000)
//   for (j in controllers) {
//     var controller = controllers[j];
    // console.log(controller);
    // var d = document.getElementById("controller" + j);
    // var buttons = d.getElementsByClassName("button");
    // for (var i=0; i<controller.buttons.length; i++) {
    //   var b = buttons[i];
    //   var val = controller.buttons[i];
    //   console.log(val);
    //   var pressed = val == 1.0;
    //   var touched = false;
    //   if (typeof(val) == "object") {
    //     pressed = val.pressed;
    //     if ('touched' in val) {
    //       touched = val.touched;
    //     }
    //     val = val.value;
    //   }
    //   var pct = Math.round(val * 100) + "%";
    //   b.style.backgroundSize = pct + " " + pct;
    //   b.className = "button";
    //   if (pressed) {
    //     b.className += " pressed";
    //   }
    //   if (touched) {
    //     b.className += " touched";
    //   }
    // }

    // var axes = d.getElementsByClassName("axis");
    // for (var i=0; i<controller.axes.length; i++) {
    //   var a = axes[i];
    //   a.innerHTML = i + ": " + controller.axes[i].toFixed(4);
    //   a.setAttribute("value", controller.axes[i]);
    // }
//   }
  rAF(updateStatus);
}

function scangamepads() {
  var gamepads = navigator.getGamepads ? navigator.getGamepads() : (navigator.webkitGetGamepads ? navigator.webkitGetGamepads() : []);
  for (var i = 0; i < gamepads.length; i++) {
    if (gamepads[i] && (gamepads[i].index in controllers)) {
      controllers[gamepads[i].index] = gamepads[i];
    }
  }
  
}

if (haveEvents) {
  window.addEventListener("gamepadconnected", connecthandler);
  window.addEventListener("gamepaddisconnected", disconnecthandler);
} else if (haveWebkitEvents) {
  window.addEventListener("webkitgamepadconnected", connecthandler);
  window.addEventListener("webkitgamepaddisconnected", disconnecthandler);
} else {
  setInterval(scangamepads, 500);
}