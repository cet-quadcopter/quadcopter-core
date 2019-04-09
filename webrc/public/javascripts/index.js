var joystickLeft = nipplejs.create({
  zone: document.getElementById("jLeft"),
  mode: "static",
  size:180,
  position: { left: '50%', top: '50%' },
  color: "red"
});

var joystickCenter = nipplejs.create({
  zone: document.getElementById("jCenter"),
  mode: "static",
  size:180,
  position: { left: '50%', top: '50%' },
  lockY: true,
  color: "blue"
});

var joystickRight = nipplejs.create({
  zone: document.getElementById("jRight"),
  mode: "static",
  size:180,
  position: { left: '50%', top: '50%' },
  lockX: true,
  color: "green"
});