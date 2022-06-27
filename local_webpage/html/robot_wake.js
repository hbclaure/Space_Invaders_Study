function robotWake(){
    console.log("hmm")
    let robot = new WebSocket("ws://127.0.0.1:8282/robot_wake")
    robot.send("wake")
};
