var BatteryMonitor = Class.create(Widget, {
  initialize: function($super, domobj) {
    $super(domobj);
    this.topics = ['/dashboard_agg']
    this.img = new Element('img', {'src' : HDF.CGI.BaseURI + 'webui/templates/images/toolbar/battery_gray.png', 'width': 41, 'height': 16});
    this.canvas = new Element('canvas', {'width': this.img.width, 'height': this.img.height});
    this.domobj.appendChild(this.canvas);
    this.span = new Element('span', {'style':"font-size:11px;position:relative;top:-3"});
    this.domobj.appendChild(this.span);
    this.img.onload = this.draw.bind(this);
    this.jquery.menu();
    this.recharge_active = false;
    this.dialog_active = false;
  },

  check: function() {
    if (!this.valid) {
      this.jquery.find("img, canvas, span").css({"-moz-opacity": 0.4});   
      this.span.innerHTML = " n/a";
      this.jquery.find(".menu_pane").html("<b>Battery</b><br />no valid data")      
    }
  },

  draw: function() {
    if (this.img.complete) {
      var ctx = this.canvas.getContext('2d');  
      try {
        ctx.drawImage(this.img,0,0);  
      } catch(e) {
        this.span.innerHTML = ' ' + this.percent + '%';
        return;
      }
      var width = 32 * this.percent / 100;
      ctx.beginPath();  
      ctx.moveTo(38-width, 1);  
      ctx.lineTo(38, 1);  
      ctx.lineTo(40, 4);  
      ctx.lineTo(40,11);  
      ctx.lineTo(38,15);  
      ctx.lineTo(38-width,15);  
      ctx.lineTo(38-width,1);  
      if (this.percent > 30)
        ctx.fillStyle = "rgb(0, 200, 0)";
      else
        ctx.fillStyle = "rgb(200, 0, 0)";
      ctx.fill();  
    }
    this.span.innerHTML = ' ' + this.percent + '%';
  },

  init: function() {
    this.valid = false;
  },

  dialog_check: function(self) {
    if(!self.dialog_active && self.valid && self.percent < 30.0 && !self.recharge_active && !(jQuery.cookie("dismissed_warning" + HDF.CGI.Login)) && !self.ac_present){
      self.dialog_active = true;
      dialogMessage("Battery Status", "The robot's battery is at " + self.percent + "% charge. It is recommended that you plug the robot in at anything below 30%.",
      {
        buttons: {
          "Run Recharge Application": function() {
          jQuery.cookie("dismissed_warning" + HDF.CGI.Login, true, {path: HDF.CGI.BaseURI});
	  self.dialog_active = false;
          document.location.href = HDF.CGI.BaseURI + "webui/appinfo.py?taskid=pr2_recharge_application/pr2_recharge_application";
          },
          "Live in Danger": function() {
          jQuery.cookie("dismissed_warning" + HDF.CGI.Login, true, {path: HDF.CGI.BaseURI});
	  self.dialog_active = false;
          jQuery(this).dialog("close");
          }
        }
      });
    }
  },

  receive: function(topic, msg) {
    if(topic == "/dashboard_agg"){
      this.valid = msg.power_state_valid;
      if(this.valid && msg.power_state.relative_capacity != null) {
        this.percent = parseInt(msg.power_state.relative_capacity);
	this.ac_present = parseInt(msg.power_state.AC_present);

        this.draw();
        writeMenuTable(jQuery(this.domobj), msg.power_state);
        this.jquery.find("img, canvas, span").css({"-moz-opacity": 1});   
      }

      var self = this;
      gPump.service_call2("list_tasks", {}, 
      function(task_list) {
        var recharge_active = false;
        for(var i=0; i < task_list.tasks.length; ++i){
          if(task_list.tasks[i] == "PR2 Recharge"){
	    recharge_active = true;
            jQuery.cookie("dismissed_warning" + HDF.CGI.Login, null, {path: HDF.CGI.BaseURI});
          }
        }
        self.recharge_active = recharge_active;
	self.dialog_check(self);
      });
    }
  }
});

var MiniboxDcDc = Class.create(Widget, {
  initialize: function($super, domobj) {
    $super(domobj);
    this.img = new Element('img', {'src' : HDF.CGI.BaseURI + 'webui/templates/images/toolbar/battery_gray.png', 'width': 41, 'height': 16});
    this.canvas = new Element('canvas', {'width': this.img.width, 'height': this.img.height});
    this.domobj.appendChild(this.canvas);
    this.span = new Element('span', {'style':"font-size:11px;position:relative;top:-3"});
    this.domobj.appendChild(this.span);
    this.img.onload = this.draw.bind(this);
    this.jquery.menu();
  },

  check: function() {
    if (!this.valid) {
      this.jquery.find("img, canvas, span").css({"-moz-opacity": 0.4});   
      this.span.innerHTML = " n/a";
      this.jquery.find(".menu_pane").html("<b>Battery</b><br />no valid data")      
    }
  },

  draw: function() {
    if (this.img.complete) {
      var ctx = this.canvas.getContext('2d');  
      try {
        ctx.drawImage(this.img,0,0);  
      } catch(e) {
        this.span.innerHTML = ' ' + this.percent + '%';
        return;
      }
      var width = 32 * this.percent / 100;
      ctx.beginPath();  
      ctx.moveTo(38-width, 1);  
      ctx.lineTo(38, 1);  
      ctx.lineTo(40, 4);  
      ctx.lineTo(40,11);  
      ctx.lineTo(38,15);  
      ctx.lineTo(38-width,15);  
      ctx.lineTo(38-width,1);  
      if (this.percent > 5)
        ctx.fillStyle = "rgb(0, 200, 0)";
      else
        ctx.fillStyle = "rgb(200, 0, 0)";
      ctx.fill();  
    }
    this.span.innerHTML = ' ' + this.percent + '%';
  },

  init: function() {
    this.valid = false;
  },

  receive: function(topic, msg) {
    this.valid = true;
    if(msg.relative_capacity != null) {
      this.percent = parseInt(msg.relative_capacity);
      this.draw();
      writeMenuTable(jQuery(this.domobj), msg);
      this.jquery.find("img, canvas, span").css({"-moz-opacity": 1});   
    }
  }
});

var TexasChargeMonitor = Class.create(Widget, {
        initialize: function($super, domobj) {
            $super(domobj);
            this.jquery.append('<div class="tool"></div>');
            this.jquery.menu();
        },

        init: function() {
        },

	    check: function() {
        },

        receive: function(topic, msg) {
            var ac = msg.AC_present;
            var src = '';
            var title = '';

            if (ac) {
                src = 'ac_power.png';
                title = 'The robot is plugged in.';
                this.jquery.find(".tool").css({backgroundPosition: '0 0'});
            } else {
		        src = 'battery_power.png';
                title = 'The robot is running on battery power.';
                this.jquery.find(".tool").css({backgroundPosition: '0 40'});
            }
            this.jquery.find(".tool").html('<img alt="' + title + '" src="' + HDF.CGI.BaseURI + 'webui/templates/images/toolbar/' + src + '"> ');
            this.jquery.find(".menu_pane").html("<b>Power</b><br />" + title);
        }
});


var state_map = {
  0: "<span class='powerboard_runstop_false'>No Power</span>",
  1: "<span class='powerboard_runstop_standby'>Standby</span>",
  2: "<span class='powerboard_runstop_false'>Pumping</span>",
  3: "<span class='powerboard_runstop_true'>On</span>",
  4: "<span class='powerboard_runstop_false'>Disabled</span>"
}

var CircuitMonitor = Class.create(Widget, {
  initialize: function($super, domobj) {
    $super(domobj);
    var tool = 
      ['<div class="tool">',
        '<div id="runstop"></div>',
      '</div>'];
    this.jquery.append(tool.join(''));
    this.jquery.menu();
  },

  init: function() {
    this.valid = false;
  },

  check: function() {
    if (!this.valid) {
      this.jquery.find("#runstop").css({backgroundPosition: '0 0'});
      this.jquery.find(".menu_pane").html("<b>Power Controls</b><br />no valid data");
    }
  },

  receive: function(topic, msg) {
    this.valid = msg.power_board_state_valid;
    // 1. set circuit breakers
    var states = msg.power_board_state.circuit_state;
    var circuit_state = "green";
    str = "<b>Circuit Breakers</b><br />";
    for (var i = 0; i < 3; ++i) {
      var circuit_id = "#circuit_" + i;
      var x_pos = '' + 90 - i*30;
      var y_pos = '0';

      if (states[i] == 3) {
      } else {
	  if (states[i] == 1) {
	      if (circuit_state != "red") {
		  circuit_state = "yellow";
	      }
	  } else {
	      circuit_state = "red";
	  }
      }
    }

    str += "Left:&nbsp;" + state_map[states[0]] + ' ';
    str += "Base:&nbsp;" + state_map[states[1]] + ' ';
    str += "Right:&nbsp;" + state_map[states[2]];

    if (circuit_state != "green") {
      str += "<br /><br />One or more of the circuit breakers is not on. ";
      str += "Go to the" +
        "<a href='" + HDF.CGI.BaseURI + "webui/powerboard.py'>Power Controls</a>" +
        "page to reset the circuits.";
    }

    // 2. set run stop
    str += "<br /><br /><b>Run-stop</b><br />";
    str += "Robot Run-stop: " + (msg.power_board_state.run_stop == "1" ? "<span class='powerboard_runstop_true'>Running</span>" : "<span class='powerboard_runstop_false'>Stopped</span>") + "<br />";
    //str += "Wireless Brake: " + (msg.power_board_state.wireless_stop == "1" ? "<span class='powerboard_runstop_true'>OFF</span>" : "<span class='powerboard_runstop_false'>ON</span>");
    if (msg.power_board_state.run_stop == "1" && msg.power_board_state.wireless_stop == "1" && circuit_state != "red") {
	    if (circuit_state == "green") {
	        this.jquery.find("#runstop").css({backgroundPosition: '0 90'});
	        //jQuery("#all_nav").css("border", "none");
	    } else if (circuit_state == "yellow") {
	        this.jquery.find("#runstop").css({backgroundPosition: '0 60'});
	        //jQuery("#all_nav").css("border", "1px solid #ff4");
	    }
    } else {
      this.jquery.find("#runstop").css({backgroundPosition: '0 30'});
      str += "<br />When the run-stop is stopped, the robot can not run. ";
      str += "Go to the" +
        "<a href='" + HDF.CGI.BaseURI + "webui/powerboard.py'>Power Controls</a>" +
        "page to reset the run-stop.";
      //jQuery("#all_nav").css("border", "1px solid #f44");
    }
    this.jquery.find(".menu_pane").html(str);
  }
});

var HomeStatus = Class.create(Widget, {
  initialize: function($super, domobj) {
    $super(domobj);
  },

  init: function() {
    this.valid = false;
  },

  check: function() {
    if (!this.valid) {
      //this.jquery.html("");
      //this.jquery.css({border: '1px solid #f44', background: '#B44', color: '#000', padding: '3px'});
    }
  },

  receive: function(topic, msg) {
    this.valid = msg.power_board_state_valid;
    // 1. set circuit breakers
    var states = msg.power_board_state.circuit_state;
    var circuit_state = "green";

    for (var i = 0; i < 3; ++i) {
      if (states[i] == 3) {
      } else {
    	  if (states[i] == 1) {
	          if (circuit_state != "red") {
		          circuit_state = "yellow";
	          }
	      } else {
	          circuit_state = "red";
	      }
      }
    }
    
    if (msg.motors_halted && msg.motors_halted.data) {
        // motors are halted
        circuit_state = "red";        
    }

    var status = "";

    if (msg.power_board_state.run_stop == "1" && msg.power_board_state.wireless_stop == "1" && circuit_state != "red") {
	    if (circuit_state == "green") {
	        status = "The robot systems are online.";
	        this.jquery.hide();
	    } else if (circuit_state == "yellow") {
	        status = "The robot systems are in standby mode.  The circuit breakers must be reset in order to use the robot.  Go to the <a href='powerboard.py'>Power Controls</a> page to reset the circuit breakers.";
	        this.jquery.css({border: '1px solid #ff4', background: '#BB4'});
	        this.jquery.show();
	    }
    } else {
	    status = "The robot systems are disabled.  Go to the <a href='powerboard.py'>Power Controls</a> page for more information.";
	    this.jquery.css({border: '1px solid #f44', background: '#B44'});
        this.jquery.show();
    }
    this.jquery.html(status);
  }
});

var ChargeMonitor = Class.create(Widget, {
  initialize: function($super, domobj) {
    $super(domobj);
    this.jquery.append('<div class="tool"></div>');
    this.jquery.menu();
  },

  init: function() {
    this.valid = false;
  },

  check: function() {
    if (!this.valid) {
        this.jquery.find(".tool").css({backgroundPosition: '0 20'});
        //this.jquery.find(".tool").css({"-moz-opacity": 0.4});
        this.jquery.find(".menu_pane").html("<b>Power</b><br />no valid data");
    }
  },

  receive: function(topic, msg) {
    this.valid = msg.power_state_valid;
    var ac = msg.power_state.AC_present;
    var src = '';
    var title = '';

    if (this.valid) {
        if (ac > 0) {
            src = 'ac_power.png';
            title = 'The robot is plugged in.';
            this.jquery.find(".tool").css({backgroundPosition: '0 0'});
        } else {
            src = 'battery_power.png';
            title = 'The robot is running on battery power.';
            this.jquery.find(".tool").css({backgroundPosition: '0 40'});
        }
        this.jquery.find(".tool").html('<img alt="' + title + '" src="' + HDF.CGI.BaseURI + 'webui/templates/images/toolbar/' + src + '"> ');
        this.jquery.find(".menu_pane").html("<b>Power</b><br />" + title);
        //this.jquery.find(".tool").css({"-moz-opacity": 1});
    }
  }
});

/******************************/

var MotorsMonitor = Class.create(Widget, {
  initialize: function($super, domobj) {
    $super(domobj);
    this.jquery.append('<div class="tool"></div>');
    this.jquery.menu();
  },
  
  init: function() {
    this.valid = false;
  },

  check: function() {
    if (!this.valid) {
      this.jquery.find(".tool").css({backgroundPosition: '0 0'});
      this.jquery.find(".menu_pane").html("<b>Motors</b><br />no valid data")
    }
  },

  receive: function(topic, msg) {
    // response is for "motors halted"
    this.valid = msg.motors_halted_valid;
    if (msg.motors_halted.data) {
      this.jquery.find(".tool").css({backgroundPosition: '0 30'});
      this.jquery.find(".menu_pane").html("<b>Motors</b><br />The robot motors have been halted." +
        "When the motors are halted, the robot can not run.<br /><br />" + 
        "Go to the" +
        "<a href='" + HDF.CGI.BaseURI + "webui/powerboard.py'>Power Controls</a>" +
        "page to reset the motors.");
    } else {
      this.jquery.find(".tool").css({backgroundPosition: '0 90'});
      this.jquery.find(".menu_pane").html("<b>Motors</b><br />The robot motors are running.");
    }
  }
});

/******************************/

var statusColor = function(level) {
  if (level == 0) {
    return '#4f4';
  } else if (level == 1) {
    return '#ff4';
  } else {
    return '#f44';
  }
}

var wikiLink = function(level, pieces, message) {
    if (level > 0) {
        var wiki_root = "http://www.ros.org/wiki/robot_monitor/Troubleshooting/";
        return " (<a target='_blank' href='" + [wiki_root, pieces[1], pieces[2], message].join("/") + "'>help wiki</a>)";
    } else {
        return '';
    }
}

var RobotMonitor = Class.create(Widget, {
  initialize: function($super, domobj) {
    $super(domobj);
    this.jquery.append("<ul id='monitor_list'></ul>");
    this.jquery.append("<p id='monitor_message'></p>")
    this.list_object = this.jquery.find("#monitor_list");
  },
  
  init: function() {
    var self = this;
    setTimeout(function() {
      if (!self.lastUpdate) {
        self.lastUpdate = 1; // so the status check will be triggered
        self.check();
      }
    }, 5000);
  },

  check: function() {
    // check lastUpdate so we don't go to offline before getting our first message
    if (this.inactive() && this.lastUpdate) {
      this.list_object.html("");
      this.jquery.find("#monitor_message").html("Robot monitor is offline.");
      this.jquery.find("#loading").hide();
    }
  },

  receive: function(topic, msg) {
    for (var i=0; i<msg.status.length; i++) {
      // first piece is empty, before leading slash
      var pieces = msg.status[i].name.split('/');
      var var_name = stringToVariable(pieces[1]);
      var color = statusColor(msg.status[i].level);
      if (pieces.length == 2) {
        var var_name = stringToVariable(pieces[1]);

        if (this.list_object.find('#' + var_name).length > 0) {
          this.list_object.find('#' + var_name + ' > span.status').html(msg.status[i].message);
          this.list_object.find('#' + var_name + ' > span.status').css({color: color});        
        } else {
          var link = "<a href='monitor.py?system=" + pieces[1] + "'>" + pieces[1] + "</a>";
          this.list_object.append(["<li id='", var_name, "'><span>", link, "</span>: <span class='status'>", msg.status[i].message, " (", msg.status[i].level, ")</span><ul></ul></li>"].join(''));
        }
      } else if (pieces.length == 3) {
        var var_sub_name = stringToVariable(pieces[2]);
                  
        if (this.list_object.find('#' + var_name + ' #' + var_sub_name).length > 0) {
          this.list_object.find('#' + var_name + ' #' + var_sub_name + ' > span.status').html(msg.status[i].message);
          this.list_object.find('#' + var_name + ' #' + var_sub_name + ' > span.status').css({color: color});
        } else {
          var link = "<a href='monitor_device.py?system=" + pieces[1] + "&device=" + pieces[2] + "'>" + pieces[2] + "</a>";
          this.list_object.find('#' + var_name + ' ul').append(["<li id='", var_sub_name, 
            "'><span>", link, "</span>: <span class='status' style='color: " + color + "'>", 
            msg.status[i].message, "</span>", 
            wikiLink(msg.status[i].level, pieces, msg.status[i].message), 
            "</li>"].join(''));
        }      
      }
    }
    this.jquery.find("#monitor_message").html("");
    this.jquery.find("#loading").hide();
    this.ping();
  }
});

var RobotDeviceMonitor = Class.create(Widget, {
  initialize: function($super, domobj) {
    $super(domobj);
    this.jquery.append("<p id='monitor_message'></p>")
    this.jquery.append("<table></table>");
    this.table_object = this.jquery.find("table");
  },
  
  init: function() {
    var self = this;
    setTimeout(function() {
      if (!self.lastUpdate) {
        self.lastUpdate = 1; // so the status check will be triggered
        self.check();
      }
    }, 5000);
  },

  check: function() {
    // check lastUpdate so we don't go to offline before getting our first message
    if (this.inactive() && this.lastUpdate) {
      this.table_object.html("");
      this.jquery.find("#monitor_message").html("Robot monitor is offline.");
      this.jquery.find("#loading").hide();
    }
  },

  receive: function(topic, msg) {
    for (var i=0; i<msg.status.length; i++) {
      // first piece is empty, before leading slash
      var pieces = msg.status[i].name.split('/');
      var var_name = stringToVariable(pieces[1]);
      var color = statusColor(msg.status[i].level);
      if (pieces.length == 3) {
        var var_sub_name = stringToVariable(pieces[2]);
        var values = "<tr><th colspan='2' style='text-align: left'>" + pieces[2] + ": <span style='color: " + color + "'>" + 
                     msg.status[i].message + "</span>" + wikiLink(msg.status[i].level, pieces, msg.status[i].message) + "</th></tr>";
        for (var j=0; j<msg.status[i].values.length; j++) {
          values = values + "<tr><td>" + msg.status[i].values[j].key + "</td><td>" + msg.status[i].values[j].value + "</td></tr>";
        }
        this.table_object.html(values);
      }
    }
    this.jquery.find("#monitor_message").html("");
    this.jquery.find("#loading").hide();
    this.ping();
  }
});

var RobotStatusMonitor = Class.create(Widget, {
  initialize: function($super, domobj) {
    $super(domobj);
  },
  
  init: function() {
  },

  check: function() {
    if (this.inactive()) {
      this.jquery.html("N/A")
      this.jquery.css({color: "#fff"})
    }
  },

  receive: function(topic, msg) {
    switch (msg.level) {
      case 0:
        this.jquery.html("OK")
        this.jquery.css({color: "#4f4"})
        break;
      case 1:
        this.jquery.html("WARN")
        this.jquery.css({color: "#ff4"})
        break;
      case 2:
        this.jquery.html("ERROR");
        this.jquery.css({color: "#f44"})
        break;
      this.jquery.html("unknown");
    }
    this.ping();
  }
});


