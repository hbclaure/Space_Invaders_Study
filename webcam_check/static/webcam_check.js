proto = window.location.protocol === 'https:' ? 'wss' : 'ws';

var sockets = {
    image: new WebSocket(proto + "://"+window.location.host+"/image"),
  }

// handle commands from the server
var box = {
    x1: 0,
    y1: 0,
    width: 0,
    height: 0,
    enough: false,
};
sockets.image.onmessage = function(event) {
    //console.log("receive image msg")
    var msg = JSON.parse(event.data);
    box.x1 = msg.x1;
    box.y1 = msg.y1;
    box.width = msg.width;
    box.height = msg.height;
    enough_faces = msg.enough;
    console.log(msg.too_many)
    if(msg.too_many && !too_many_faces){
        console.log("set true")
        too_many_faces = msg.too_many;
        startTime = new Date().getTime();
    } else if(!msg.too_many && new Date().getTime() - startTime >= 5000){
        console.log("set false")
        too_many_faces = msg.too_many;
    }
    var ctx = canvas.getContext('2d');
    ctx.beginPath();
    ctx.lineWidth = "6";
    ctx.strokeStyle = "magenta"; 
    ctx.rect(box.x1, box.y1, box.width, box.height);
    ctx.stroke();
};

// set image width; height will be matched accordingly
var width = 500;
var height = 0;

var recording = null;

var streaming = false;

var video = null;
var canvas = null;
var photo = null;
var image = null;
var canvas2 = null;

var player_id;                          //!< unique ID
var game_num;                            //!< game number

var enough_faces = false;
var too_many_faces = false;
var startTime;

/**
Function to find the game id and game mode (which are passed as GET parameters)
Modified from code found at: https://stackoverflow.com/questions/5448545/how-to-retrieve-get-parameters-from-javascript
**/
function findGetParameter(parameterName) {
    var result = null,
        tmp = [];
    var items = location.search.substr(1).split("&");
    for (var index = 0; index < items.length; index++) {
        tmp = items[index].split("=");
        if (tmp[0] === parameterName) result = tmp[1];
    }
    return result;
}

player_id = findGetParameter('id') ? findGetParameter('id') : 'UNDEFINED';

function startup() {
    video = document.getElementById('video');
    canvas = document.getElementById('canvas');
    photo = document.getElementById('photo');
    canvas2 = document.getElementById('canvas2');

    navigator.mediaDevices.getUserMedia({ video: true, audio: false })
    .then(function(stream) {

        video.srcObject = stream;
        video.play();
    })
    .catch(function(err) {
        console.log("An error occurred: " + err);
    });
    
    video.addEventListener('canplay', function(ev){
        if (!streaming) {

            // maintain aspect ratio
            height = video.videoHeight / (video.videoWidth/width);
            ratio = video.videoWidth/width

            ratio_data = {
                ratio: ratio,
            };
            //console.log('image logged')

            video.setAttribute('width', width);
            video.setAttribute('height', height);
            canvas.setAttribute('width', width);
            canvas.setAttribute('height', height);
            canvas2.setAttribute('width', width);
            canvas2.setAttribute('height', height);
            streaming = true;
        }
    }, false);
    //while(sockets.image.readyState != 1){
    //    console.log(sockets.image.readyState)
    //}
    console.log(sockets.image.readyState)
    if(sockets.image.readyState == 1){
        //sockets.image.send(JSON.stringify({player_id:player_id}));
        //console.log("sent a message");
        //save_image_loop();
        socket_start();
    } else if(sockets.image.readyState == 0){
        sockets.image.onopen = () => socket_start();
    }
}
function socket_start() {
    player_id = findGetParameter('id') ? findGetParameter('id') : 'UNDEFINED';
    sockets.image.send(JSON.stringify({player_id:player_id}));
    //console.log("sent a message");
    save_image_loop();
}
function save_image_loop() {
    recording = setInterval(function(){
        logpicture();
    }, 66);
}

// log user video frame
function logpicture() {
    var context = canvas.getContext('2d');
    var ctx2 = canvas2.getContext('2d');
    if (width && height) {
        canvas.width = width;
        canvas.height = height;
        context.drawImage(video, 0, 0, width, height);
        //console.log("sending image");
        sockets.image.send(JSON.stringify({'img':canvas.toDataURL('image/jpeg')}))
        ctx2.clearRect(0,0, ctx2.canvas.width, ctx2.canvas.height);
        ctx2.beginPath();
        ctx2.fillStyle="black";
        ctx2.font = "10pt sans-serif";
        ctx2.fillText("WE ARE CHECKING THE QUALITY OF YOUR WEBCAM IMAGE.", 20, 50);
        ctx2.fillText("YOU SHOULD SEE A BOX AROUND YOUR FACE.", 20, 75);
        ctx2.fillText("PLEASE ENSURE YOU ARE FACING THE CAMERA SO THAT ", 20, 100);
        ctx2.fillText("THE IMAGE CAPTURES YOUR FACE FROM THE FRONT.", 20, 125);
        if(too_many_faces){
            ctx2.beginPath();
            ctx2.fillStyle="red";
            ctx2.font = "10pt sans-serif";
            ctx2.fillText("TOO MANY FACES DETECTED.", 20, 175);
            ctx2.fillText("PLEASE ENSURE NO ONE ELSE IS IN FRAME.", 20, 200);
            ctx2.fillText("PLEASE REMOVE OBJECTS OTHER THAN", 20, 225);
            ctx2.fillText("YOUR FACE THAT SHOW UP IN BOXES.", 20, 250);
        }else if(enough_faces){
            ctx2.beginPath();
            ctx2.fillStyle="magenta";
            ctx2.font = "20pt sans-serif";
            ctx2.fillText("ENOUGH FACES FOUND.", 20, 175);
            ctx2.fillText("COLOR IS: MAGENTA", 20, 200);
        }
        

        
        
    }
}

window.addEventListener('load', startup, false);