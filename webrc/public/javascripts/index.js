const href = window.location.href;
const socket = new WebSocket('ws://192.168.43.173:40510/');

var joystickLeft = nipplejs.create({
  zone: document.getElementById("jLeft"),
  mode: "static",
  restJoystick: true,
  size:180,
  position: { left: '50%', top: '50%' },
  color: "red"
});


var joystickCenter = nipplejs.create({
  zone: document.getElementById("jCenter"),
  mode: "static",
  size:180,
  restJoystick: true,
  position: { left: '50%', top: '50%' },
  lockY: true,
  color: "blue"
});


var joystickRight = nipplejs.create({
  zone: document.getElementById("jRight"),
  mode: "static",
  size:180,
  restJoystick: true,
  position: { left: '50%', top: '50%' },
  lockX: true,
  color: "green"
});


joystickLeft.on('move end', function (evt, nip) {
  let distance = 0;
  let angle = 0;

  if (evt.type == 'move') {
    distance = nip.distance;
    angle = nip.angle.radian;
  }

  const x = distance * Math.cos(angle) / 90;
  const y = distance * Math.sin(angle) / 90;

  sendSignal('vh', {x: y, y: x});
});


joystickCenter.on('move end', function (evt, nip) {
  let distance = 0;
  let angle = 0;

  if (evt.type == 'move') {
    distance = nip.distance;
    angle = nip.angle.radian;
  }

  sendSignal('vd', -distance * Math.sin(angle) / 90);
});


joystickRight.on('move end', function (evt, nip) {
  let distance = 0;
  let angle = 0;

  if (evt.type == 'move') {
    distance = nip.distance;
    angle = nip.angle.radian;
  }

  sendSignal('orientation', -distance * Math.cos(angle) / 90);
});


function sendSignal(type, signal) {
  socket.send(JSON.stringify({
    type: type,
    signal: signal
  }));
}