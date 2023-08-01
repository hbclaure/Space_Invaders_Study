function robotIntro(){
    let robot = new WebSocket("ws://127.0.0.1:8282/robot_introduction")
    console.log('intro')
    robot.onopen = () => robot.send("intro")
};
