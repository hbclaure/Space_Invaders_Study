function goPython(){
    console.log("hmm")
    let robot = new WebSocket("ws://127.0.0.1:8282/robot")
    robot.send("wake")
};
