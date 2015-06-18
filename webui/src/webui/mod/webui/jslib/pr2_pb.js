var PowerboardBreakerWidget = Class.create({
    initialize: function(domobj) {
		this.domobj = domobj;
		this.topics = [domobj.getAttribute("topic")];
		this.breaker_num = parseInt(domobj.getAttribute("breaker_num"));
	},

	init: function() {
	},

	receive: function(topic, msg) {
		msg = msg.power_board_state;
		var state = msg.circuit_state[this.breaker_num];
		var voltage = msg.circuit_voltage[this.breaker_num];
		
		var state_str = "Unknown";
		if(state == 0) state_str = "No Power";
		if(state == 1) state_str = "Standby";
		if(state == 2) state_str = "Pumping";
		if(state == 3) state_str = "On";
		if(state == 4) state_str = "Disabled";
		
		this.domobj.className = "status powerboard_breaker_" + state;
		var cb;
		if(voltage != null) {
			voltage = voltage.toFixed(2);
			cb = state_str + " @ " + voltage + "V";
		} else {
			cb = state_str;
		}
		
		this.domobj.innerHTML = cb;
    },
});

var PowerboardRunStopWidget = Class.create({
    initialize: function(domobj) {
        this.domobj = domobj;
        this.topics = [domobj.getAttribute("topic")];
        this.brake = domobj.getAttribute("brake");

        this.estopDict = {false: "Stopped", true: "Running"};
    },

    init: function() {
    },

    receive: function(topic, msg) {
        msg = msg.power_board_state;
	    var estop_button_status = null;
	    var estop_wireless_status = null;

        var status = null;
        if (this.brake == "button") {
            status = msg.run_stop;
        } else if (this.brake == "wireless") {
            status = msg.wireless_stop;
        }

	    this.domobj.className = "status powerboard_runstop_" + status;
	
	    this.domobj.innerHTML = this.estopDict[status];
    },
});

var PowerboardMotorsWidget = Class.create({
	initialize: function(domobj) {
		this.domobj = domobj;
		this.topics = [domobj.getAttribute("topic")];
		this.key = domobj.getAttribute("key");
	},

    init: function() {
	},

    receive: function(topic, msg) {
		var value = msg.motors_halted.data;
		if (value) {
			this.domobj.innerHTML = "Halted";
    		this.domobj.className = "status powerboard_runstop_false";
		} else {
    		this.domobj.innerHTML = "Running";
    		this.domobj.className = "status powerboard_runstop_true";
		}
	},
});

var PowerboardRunStopWidget_Single = Class.create({
    initialize: function(domobj) {
		this.domobj = domobj;
		this.topics = [domobj.getAttribute("topic")];
		this.key = domobj.getAttribute("key");
	},

    init: function() {
	},

    receive: function(topic, msg) {
		var pbmsg = msg.status[0]
		if(pbmsg.name == "Power board 0") {
			var estop_button_status = null;
			var estop_wireless_status = null;
			
			for(var j=0; j<pbmsg.values.length; j++) {
				if(pbmsg.values[j].label == "RunStop Button Status") {
					var status = parseFloat(pbmsg.values[j].value);
					if(status > 0.5) {
						estop_wireless_status = "Run";
					} else {
						estop_wireless_status = "Stop";
					}
  				}
				if(pbmsg.values[j].label == "RunStop Status") {
					var status = parseFloat(pbmsg.values[j].value);
					if(status > 0.5) {
						estop_button_status = "Run";
					} else {
						if(estop_wireless_status == "Stop") {
							estop_button_status = "Unknown";
						} else {
							estop_button_status = "Stop";
						}
					}
  				}
			}

			var estop_status = null;
			if(estop_button_status != "Run" || estop_wireless_status != "Run") {
				estop_status = "Stop";
			} else {
				estop_status = "Run";
			}


			if (this.key == "estop_status") {
				this.domobj.innerHTML = estop_status;
				this.domobj.className = "powerboard_runstop_" + estop_status;
			} else if (this.key == "estop_wireless_status") {
				this.domobj.innerHTML = estop_wireless_status;
				this.domobj.className = "powerboard_runstop_" + estop_wireless_status;
			} else if (this.key == "estop_button_status") {
				this.domobj.innerHTML = estop_button_status;
				this.domobj.className = "powerboard_runstop_" + estop_button_status;
			}
		}
	}
});


