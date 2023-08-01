
window.addEventListener("gamepaddisconnected", function(e) {
    if (e.gamepad.index == 0) {
      gamepad1 = false;
    } else if (e.gamepad.index == 1) {
      gamepad2 = false;
    }
    // gamepad1 = false;
    delete controllers[e.gamepad.index];
  })
  
  // window.addEventListener("gamepadbuttondown", function(e){
  //   // Button down
  //   console.log(
  
  //      "Button down",
  //      e.button, // Index of button in buttons array
  //      e.gamepad
  
  //   );
  // });
  
  // window.addEventListener("gamepadbuttonup", function(e){
  //   // Button up
  //   console.log(
  
  //      "Button up",
  //      e.button, // Index of button in buttons array
  //      e.gamepad
  
  //   );
  // });