function robotSleep(){
    let robot = new WebSocket("ws://127.0.0.1:8282/robot_sleep")
    robot.send("sleep")
};