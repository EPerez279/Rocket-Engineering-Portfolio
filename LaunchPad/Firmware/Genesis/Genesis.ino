#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>
#include <ESP32Servo.h>

// Browser: http://192.168.4.1
const char* ssid     = "NanoESP32_LED";
const char* password = "12345678";  // WPA2 needs 8+ chars

// Pins
constexpr uint8_t PIN_SERVO_1 = 9;
constexpr uint8_t PIN_SERVO_2 = 10;
constexpr uint8_t PIN_SERVO_3 = 11;
constexpr uint8_t PIN_SERVO_4 = 7;

constexpr uint8_t PIN_IGN     = 8;

// Servos
Servo SERVO_1, SERVO_2, SERVO_3, SERVO_4;

// Web server
WebServer server(80);

// Simple state machine for safety + non-blocking timing
enum class LaunchState : uint8_t { IDLE, ARMED, IGNITING, RELEASED };
LaunchState launchState = LaunchState::IDLE;

constexpr uint16_t SERVO_LOCK_ANGLE   = 0;
constexpr uint16_t SERVO_UNLOCK_ANGLE = 160;

constexpr uint32_t IGNITION_PULSE_MS  = 2000;
uint32_t ignitionStartMs = 0;

// ---------- HTML ----------
String pageHtml() {
  String html;
  html.reserve(9000); // helps avoid heap fragmentation

  html += R"rawliteral(
<!DOCTYPE html><html>
<head>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Nano ESP32 LED</title>
  <link href="https://cdn.jsdelivr.net/npm/bootstrap@5.3.8/dist/css/bootstrap.min.css" rel="stylesheet">
  <style>
    body {font-family:Arial;margin:40px;}
    input {
      font-size:22px;padding:14px 22px;margin:10px;
      position: fixed;
      right: 100px;
      bottom: 150px;
      border-radius: 10px;
      background-color: red;
      color: #ffc6d0;
      cursor: pointer;
    }
    input:hover {
      background-color: rgb(200, 0, 0);
    }
    input:active {
      background-color: darkred;
    }
    .info {
      border-style: solid;
      border-color: black;
      border-radius: 5px;
      padding: 5px 10px;
      background-color: #f3f3f3;
    }
    canvas { position: fixed; bottom: 256px; border-style: solid; border-color: black; border-radius: 5px;}
    #timers { position: fixed; left: 50px; }
    #data { position: fixed; right: 50px; }
    #status-servos { position: fixed; top: 250px; left: 50px; }
    canvas { position: fixed; bottom: 50px; left: 50px; }

    @keyframes servo1Animation { from {transform: rotate(0deg);} to {transform: rotate(-90deg);} }
    @keyframes servo3Animation { from {transform: rotate(0deg);} to {transform: rotate(90deg);} }

   
    #servo1 { clip-path: polygon(0% 0%, 100% 0%, 100% 40%, 50% 40%, 50% 70%, 0% 70%); }
    #servo3 { clip-path: polygon(100% 0%, 0% 0%, 0% 40%, 50% 40%, 50% 70%, 100% 70%); }
  </style>
  <!--<script type="importmap">
  {
    "imports": {
      "three": "https://cdn.jsdelivr.net/npm/three@0.183.2/build/three.module.js",
      "three/addons/": "https://cdn.jsdelivr.net/npm/three@0.183.2/examples/jsm/"
    }
  }
  </script>-->
  <script type="module">

    //import * as THREE from 'three';
    //import { STLLoader } from 'three/addons/loaders/STLLoader.js';
    //import {OrbitControls} from 'three/addons/controls/OrbitControls.js';

    let timer = 8.0;
    let rocketLiftoff = false;
    let timerHandle = null;
    let animateServos = false;
    let animateRocket = false;
    let rocketArmed = false;

    function dec_timer() {
      timer -= 0.1;
      const text = document.getElementById("countDown");
      if (timer >= 0) text.innerHTML = "T-" + timer.toFixed(1);
      else text.innerHTML = "T+" + (-timer).toFixed(1);

      if (!rocketLiftoff && timer <= 0) {
        rocketLiftoff = true;
        on_liftoff();
      }
    }

    function change_status(status) {
      document.getElementById("status").innerHTML = "Status: " + status;
    }

    async function on_liftoff() {
      // update UI immediately
      const newStatus = "<b style='color:green'>UNLOCKED</b>";
      document.getElementById("status-servo1").innerHTML = "Servo 1: " + newStatus;
      document.getElementById("status-servo2").innerHTML = "Servo 2: " + newStatus;
      document.getElementById("status-servo3").innerHTML = "Servo 3: " + newStatus;
      document.getElementById("status-servo4").innerHTML = "Servo 4: " + newStatus;

      document.getElementById("servo1").style.animationName = "servo1Animation";
      document.getElementById("servo3").style.animationName = "servo3Animation";

      animateServos = true;
      animateRocket = true;

      // call ESP32 endpoint (note leading slash)
      try {
        const res = await fetch("/liftoff");
        const txt = await res.text();
        console.log("liftoff:", res.status, txt);
        change_status('<b style="color:green">IN LIFTOFF</b>');
      } catch (e) {
        console.error("liftoff failed:", e);
        change_status('<b style="color:red">LIFFOFF FAILED</b>');
      }
    }

    function update_localTime() {
      const now = new Date();
      document.getElementById("localTime").innerHTML = "Local Time: " + now.toLocaleTimeString();
    }

    // NOTE: NOAA-from-browser is often unreliable due to User-Agent/CORS.
    // If you keep weather, do it from ESP32 and serve it to this page.

    function on_launch() {
      if (timerHandle) clearInterval(timerHandle);
      timer = 8.0;
      rocketLiftoff = false;
      timerHandle = setInterval(dec_timer, 100);
    }

    setInterval(update_localTime, 1000);

    async function on_arm_press() {


      if (rocketArmed) {
        rocketArmed = false;
        rocketLiftoff = false;
        clearInterval(timerHandle);
        change_status('<b style="color:grey">DISARMING SUCCESSFUL</b>');
        animateServos = true;
        animateRocket = false;

        return;
      }
     
      window.addEventListener("load", on_launch);
     

      try {
        const res = await fetch("/arm");
        const txt = await res.text();
        console.log("arm:", res.status, txt);
        change_status('<b style="color:orange">ARMED</b>');
        rocketArmed = true;
        animateServos = true;
        animateRocket = false;
        on_launch();
      } catch (e) {
        console.error("arming failed:", e);
        change_status('<b style="color:red">ARMING FAILED</b>');
      }
    }

    /*function setup_scene() {
      const canvas = document.querySelector( '#gl-canvas' );

      const scene = new THREE.Scene();
      const camera = new THREE.PerspectiveCamera(75,1, 0.1, 1000);

      camera.position.set(0,0,15);

      const controls = new OrbitControls(camera, canvas);
      controls.target.set(0, 0, 0);
      controls.update();

      const renderer = new THREE.WebGLRenderer({canvas});
      renderer.setClearColor(0xFFFFFF);

      const geometry = new THREE.BoxGeometry(2,1,1);
      const material = new THREE.MeshPhongMaterial({color:0xffffaa});
      const mesh = new THREE.Mesh(geometry, material);

      const mesh2 = new THREE.Mesh(new THREE.BoxGeometry(0.8,1,1), material);
      mesh2.position.x = 0.6;
      mesh2.position.y = 1;
      mesh.add(mesh2);

      const distance = 3

      const mesh11 = new THREE.Mesh(geometry, material);
      mesh11.add(mesh2.clone());
      mesh11.position.x = distance;
      mesh11.position.z = -distance;
      mesh11.rotation.y = Math.PI/4;

      const mesh12 = new THREE.Mesh(geometry, material);
      mesh12.add(mesh2.clone());
      mesh12.position.x = -distance;
      mesh12.position.z = -distance;
      mesh12.rotation.y = Math.PI/4*3;

      const mesh13 = new THREE.Mesh(geometry, material);
      mesh13.add(mesh2.clone());
      mesh13.position.x = -distance;
      mesh13.position.z = distance;
      mesh13.rotation.y = Math.PI/4*5;

      const mesh14 = new THREE.Mesh(geometry, material);
      mesh14.add(mesh2.clone());
      mesh14.position.x = distance;
      mesh14.position.z = distance;
      mesh14.rotation.y = Math.PI/4*7;

      const materialRocket = new THREE.MeshPhongMaterial({color:0xAAAAAA});
      const materialRocketCone = new THREE.MeshPhongMaterial({color:333333});
      const materialRocketThruster = new THREE.MeshPhongMaterial({color:0xDD4444});
      const meshRocket = new THREE.Mesh(new THREE.CylinderGeometry(2,2,12,32), materialRocket);
      const meshRocketCone = new THREE.Mesh(new THREE.CylinderGeometry(0,2,3,32), materialRocketCone);
      const meshRocketThruster = new THREE.Mesh(new THREE.CylinderGeometry(0.8,0.8,4,32), materialRocketThruster);
      meshRocketCone.position.y = 7.5;
      meshRocket.add(meshRocketCone);
      meshRocketThruster.position.y = -5;
      meshRocket.add(meshRocketThruster);
      meshRocket.position.y = 6;

      const loader = new STLLoader();
      //const geometry2 = await loader.loadAsync( './models/stl/slotted_disk.stl' )
      scene.add(mesh11);
      scene.add(mesh12);
      scene.add(mesh13);
      scene.add(mesh14);

      scene.add(meshRocket);
      //const geometry2 = await loader.loadAsync( './models/stl/slotted_disk.stl' )
      {
        loader.load('Sphericon.stl', function (geometry3) {
         
         
          const mesh3 = new THREE.Mesh(geometry3, material);
          mesh3.scale.x = 0.001;
          mesh3.scale.y = 0.001;
          mesh3.scale.z = 0.001;
          //console.log(mesh3);
          //mesh.add(mesh3);
        }, undefined, function ( error ) {

        console.error( error );

        } );
      }


      const light = new THREE.DirectionalLight(0xFFFFFF, 0.7);
      light.position.set(0,10,0);
      light.target.position.set(0,0,0);
      scene.add(light);
      scene.add(light.target);

      const light2 = new THREE.AmbientLight(0xFFFFFF, 0.3);
      scene.add(light2);


     
      var animationTimer = 0.0; // Goes from 0.0 to 1.0;
      var rocketAnimationTimer = 0.0;

      function updateScene() {

        if (animateServos) {
          const animationSpeed = 0.015;

          if (rocketLiftoff || !rocketArmed) {
            animationTimer -= animationSpeed;
          } else {
            animationTimer += animationSpeed;
          }

          if (animationTimer >= 1) {
            animateServos = false;
            animationTimer = 1.0;
          }
          if (animationTimer <= 0) {
            animateServos = false;
            animationTimer = 0.0;
          }
          let t = animationTimer;

          mesh11.rotation.z = Math.PI/2*t;
          mesh12.rotation.z = Math.PI/2*t;
          mesh13.rotation.z = Math.PI/2*t;
          mesh14.rotation.z = Math.PI/2*t;
         
        }

        if (animateRocket) {
          rocketAnimationTimer += 0.08;
          meshRocket.position.y = rocketAnimationTimer * rocketAnimationTimer * 0.5 + 6;
        } else {
          rocketAnimationTimer = 0.0;
          meshRocket.position.y = 6;
        }

        renderer.render(scene, camera);
      }

      renderer.setAnimationLoop(updateScene);
    }*/

    //setup_scene();

    const arm_button = document.getElementById("button");

    arm_button.addEventListener("click", on_arm_press);

  </script>
</head>
<body>
  <h2>Nano ESP32 LED Control</h2>
  <p id="status">Status: <b style="color:grey">None</b></p>

  <div id="timers" class="info">
    <p id="localTime">Local Time:</p>
    <p id="countDown">T-0</p>
  </div>

  <div id="data" class="info">
    <p id="alt-data">Altitude: <b style="color:grey">UNAVAILABLE</b></p>
    <p id="pressure-data">Pressure: <b style="color:grey">UNAVAILABLE</b></p>
    <p id="humidity-data">Humidity: <b style="color:grey">UNAVAILABLE</b></p>
    <p id="temp-data">Temperature: <b style="color:grey">UNAVAILABLE</b></p>
  </div>

  <p>
    <input type="button" id="button" onclick="//" value="ARM / START COUNTDOWN">
  </p>

  <div id="status-servos" class="info">
    <p id="status-servo1">Servo 1: <b style="color:red">LOCKED</b></p>
    <p id="status-servo2">Servo 2: <b style="color:red">LOCKED</b></p>
    <p id="status-servo3">Servo 3: <b style="color:red">LOCKED</b></p>
    <p id="status-servo4">Servo 4: <b style="color:red">LOCKED</b></p>
  </div>

  <canvas width="400", height="400" id="gl-canvas" class="servo"></canvas>
  <div id="servos">
   
    <div id="servo1"></div>
    <div id="servo3"></div>
  </div>

  <p style="position:fixed; bottom:20px; left:50px;">
    <a href="/reset" class="btn btn-secondary">RESET</a>
  </p>

</body></html>
)rawliteral";

  return html;
}

// ---------- Helpers ----------
void lockServos() {
  SERVO_1.write(SERVO_LOCK_ANGLE);
  SERVO_2.write(SERVO_LOCK_ANGLE);
  SERVO_3.write(SERVO_LOCK_ANGLE);
  SERVO_4.write(SERVO_LOCK_ANGLE);
}

void unlockServos() {

  delay(800); // to account for 0.8s ignition delay
 
  SERVO_1.write(SERVO_UNLOCK_ANGLE);
  SERVO_2.write(SERVO_UNLOCK_ANGLE);
  SERVO_3.write(SERVO_UNLOCK_ANGLE);
  SERVO_4.write(SERVO_UNLOCK_ANGLE);
}

// ---------- Routes ----------
void handleRoot() {
  server.send(200, "text/html", pageHtml());
}

void handleArm() {
  // Only arm if idle
  if (launchState == LaunchState::IDLE) {
    launchState = LaunchState::ARMED;
  }
  server.sendHeader("Location", "/");
  server.send(303);
}

void handleReset() {
  launchState = LaunchState::IDLE;
  digitalWrite(PIN_IGN, LOW);
  lockServos();
  server.sendHeader("Location", "/");
  server.send(303);
}

void handleLiftoff() {
  // Safety: only allow liftoff if ARMED
  if (launchState != LaunchState::ARMED) {
    server.send(409, "text/plain", "Not armed");
    return;
  }

  // Start ignition pulse (non-blocking)
  launchState = LaunchState::IGNITING;
  ignitionStartMs = millis();

  unlockServos();
  delay(2000);
 
  digitalWrite(PIN_IGN, HIGH);

  // Unlock immediately (or you can delay unlock until pulse ends if you prefer)
 
  launchState = LaunchState::RELEASED;

  server.send(200, "text/plain", "Liftoff started");
}

// Optional JSON status endpoint
void handleStatus() {
  String s = "{";
  s += "\"state\":";
  switch (launchState) {
    case LaunchState::IDLE:     s += "\"IDLE\""; break;
    case LaunchState::ARMED:    s += "\"ARMED\""; break;
    case LaunchState::IGNITING: s += "\"IGNITING\""; break;
    case LaunchState::RELEASED: s += "\"RELEASED\""; break;
  }
  s += ",\"ign\":"; s += (digitalRead(PIN_IGN) ? "true" : "false");
  s += "}";
  server.send(200, "application/json", s);
}

void setup() {
  Serial.begin(115200);
  delay(200);

  SERVO_1.attach(PIN_SERVO_1);
  SERVO_2.attach(PIN_SERVO_2);
  SERVO_3.attach(PIN_SERVO_3);
  SERVO_4.attach(PIN_SERVO_4);
  lockServos();

  pinMode(PIN_IGN, OUTPUT);
  digitalWrite(PIN_IGN, LOW);

  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid, password);

  Serial.print("AP SSID: "); Serial.println(ssid);
  Serial.print("AP IP: ");   Serial.println(WiFi.softAPIP()); // usually 192.168.4.1

  server.on("/", handleRoot);
  server.on("/arm", handleArm);
  server.on("/reset", handleReset);
  server.on("/liftoff", handleLiftoff);
  server.on("/status", handleStatus);
 
  server.begin();
  Serial.println("Web server started");
}

void loop() {
  server.handleClient();

  // Non-blocking ignition pulse end
  if (digitalRead(PIN_IGN) && (millis() - ignitionStartMs >= IGNITION_PULSE_MS)) {
    //digitalWrite(PIN_IGN, LOW);
  }
}