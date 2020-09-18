if (!LocalizationDiagnosticSubscriber) {
    var LocalizationDiagnosticSubscriber = {
        ros: null,
        name: "",
        init: function() {
            this.ros = new ROSLIB.Ros();
            this.ros.on('error', function(error) {
                document.getElementById('localization_diagnostic_info').innerHTML = "Error";
            });
            this.ros.on('connection', function(error) {
                document.getElementById('localization_diagnostic_info').innerHTML = "Connect";
            });
            this.ros.on('close', function(error) {
                document.getElementById('localization_diagnostic_info').innerHTML = "Close";
            });
            this.ros.connect('ws://' + location.hostname + ':9090');

            var sub = new ROSLIB.Topic({
                ros: this.ros,
                name: '/localization/diagnostics',
                messageType: 'diagnostic_msgs/DiagnosticArray'
            });
            sub.subscribe(function(message) {
                var div = document.getElementById("localization_diagnostic_stamp");
                if (div.hasChildNodes()) {
                    div.removeChild(div.firstChild);
                }
                var res = message.header.stamp.secs + "." + message.header.stamp.nsecs;
                var el = document.createElement("span");
                el.innerHTML = res;
                document.getElementById("localization_diagnostic_stamp").appendChild(el);

                var div = document.getElementById("localization_diagnostic_status");
                if (div.hasChildNodes()) {
                    div.removeChild(div.firstChild);
                }
                message.status.forEach(function (element) {
                    if (element.level == 0) {
                        var res = "OK";
                    }
                    else if (element.level == 1) {
                        var res = "WARN";
                    }
                    else if (element.level == 2) {
                        var res = "<font color='red'>Error</font>";
                    }
                    var el = document.createElement("span");
                    el.innerHTML = res;
                    document.getElementById("localization_diagnostic_status").appendChild(el);
                });

            });
        }
    }
    LocalizationDiagnosticSubscriber.init();

    window.onload = function() {};
    window.onunload = function() {
        LocalizationDiagnosticSubscriber.ros.close();
    };
}