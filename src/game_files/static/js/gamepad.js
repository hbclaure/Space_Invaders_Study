var controllers = {};
window.addEventListener("gamepadconnected", function(e) {
  controllers[e.gamepad.index] = e.gamepad

  if (e.gamepad.index == 0) {
    gamepad1 = true;

    setInterval(function(){
      var gp1 = navigator.getGamepads()[e.gamepad.index];
      console.log("This is controller 1", gp1)

      p1xBoxA = gp1.buttons[0].pressed;
      p1xBoxLeft = gp1.buttons[14].pressed;
      p1xBoxRight = gp1.buttons[15].pressed;
      p1xBoxUp = gp1.buttons[2].pressed;
      p1xBoxDown = gp1.buttons[1].pressed;
      p1xBoxY = gp1.buttons[3].pressed;
    }, 100)
  } else if (e.gamepad.index == 1) {
    gamepad2 = true;

    setInterval(function(){
      var gp2 = navigator.getGamepads()[e.gamepad.index];

      p2xBoxA = gp2.buttons[0].pressed;
      p2xBoxLeft = gp2.buttons[14].pressed;
      p2xBoxRight = gp2.buttons[15].pressed;
      p2xBoxUp = gp2.buttons[6].pressed;
      p2xBoxDown = gp2.buttons[1].pressed;
      p2xBoxY = gp2.buttons[3].pressed;
    }, 100)
  }
});

window.addEventListener("gamepaddisconnected", function(e) {
  if (e.gamepad.index == 0) {
    gamepad1 = false;
  } else if (e.gamepad.index == 1) {
    gamepad2 = false;
  }
  delete controllers[e.gamepad.index];
})
