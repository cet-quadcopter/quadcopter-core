var rosnode = require('rosnodejs');
var express = require('express');
var path = require('path');
var logger = require('morgan');

var indexRouter = require('./routes/index');

var app = express();

app.use(logger('dev'));
app.use(express.json());
app.use(express.urlencoded({ extended: false }));
app.use(express.static(path.join(__dirname, 'public')));

app.use('/', indexRouter);

rosnode.initNode('webrc');

var drone_msgs = rosnode.require('drone_std_msgs');
const pub = rosnode.nh.advertise('/controls/velocity', 'drone_std_msgs/InputVelocity');

var WebSocketServer = require('ws').Server,
wss = new WebSocketServer({port: 40510});

var state = {
  v_n: 0.0,
  v_e: 0.0,
  v_d: 0.0,
  omega_d: 0.0
}
wss.on('connection', function (ws) {
  ws.on('message', function (message) {
    var signal = JSON.parse(message);
    if (signal.type == 'vh') {
      state.v_n = signal.signal.x * 3;
      state.v_e = signal.signal.y * 3;
    } else if (signal.type == 'vd') {
      state.v_d = signal.signal * 3;
    } else if (signal.type == 'orientation') {
      state.omega_d = signal.signal * 0.2;
    }

    var msg = new drone_msgs.msg.InputVelocity(state);
    pub.publish(msg);
  });
});

module.exports = app;
