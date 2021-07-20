if (!AutowareStateSubscriber) {
  var AutowareStateSubscriber = {
    ros: null,
    name: "",
    init: function () {
      this.ros = new ROSLIB.Ros();
      this.ros.connect("ws://" + location.hostname + ":9090");

      var sub = new ROSLIB.Topic({
        ros: this.ros,
        name: "/autoware/state",
        messageType: "autoware_system_msgs/AutowareState",
      });

      sub.subscribe(function (message) {
        document.getElementById("autoware_state").innerHTML = message.state;
        document.getElementById("autoware_state_msg").innerHTML = message.msg;
      });
    },
  };
  AutowareStateSubscriber.init();

  window.onload = function () {};
  window.onunload = function () {
    AutowareStateSubscriber.ros.close();
  };
}
