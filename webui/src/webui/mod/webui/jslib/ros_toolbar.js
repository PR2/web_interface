
// *******************************************

var OnOffButtonWidget = Class.create(Widget, {
  initialize: function($super, domobj) {
    $super(domobj);
    this.state = false;
    this.key = domobj.getAttribute("key");
    this.selector = domobj.getAttribute("selector");
    this.selectorValue = domobj.getAttribute("selectorValue");
    this.onValue = domobj.getAttribute("onValue");
  },

  init: function() {
    var obj = this;
    this.domobj.onmousedown = function() {obj.onmousedown();};

    if(this.state == true) {
      this.domobj.setAttribute("class", "buttonOn");
    } else {
      this.domobj.setAttribute("class", "buttonOff");
    }
  },

  receive: function(topic, msg) {
    if(this.selector && this.selectorValue) {
      if(msg[this.selector] != this.selectorValue) return;
    }
    if(msg[this.key] == null) return;
  
    var state = msg[this.key];
    if(state == this.onValue) this.state = true;
    else this.state = false;

    var button = this.domobj;
    if(button != null) {
      if(this.state == true) {
        button.setAttribute("class", "buttonOn");
      } else {
        button.setAttribute("class", "buttonOff");
      }
    }
  },

  onmousedown: function(evt) {
    var newstate = !this.state;
    clearSelection();
  }
});

// *******************************************

var TextWidget = Class.create(Widget, {
  initialize: function($super, domobj) {
    $super(domobj);
    this.key = domobj.getAttribute("key");
    this.selector = domobj.getAttribute("selector");
    this.selectorValue = domobj.getAttribute("selectorValue");
  },

  init: function() {
  },

  receive: function(topic, msg) {
    if(this.selector && this.selectorValue) {
      if(this.selector && this.selectorValue) {
        if(msg[this.selector] != this.selectorValue) 
	  return;
      }
    }
    var v = 'msg.' + this.key;
    var val = eval(v);
    if(val != null) {
      this.domobj.innerHTML = val;
    }
  }
});

// *******************************************

var PercentTextWidget = Class.create(Widget, {
  initialize: function($super, domobj) {
    $super(domobj);
    this.numerator = domobj.getAttribute("num");
    this.denominator = domobj.getAttribute("den");
  },

  init: function() {
  },

  receive: function(topic, msg) {
    if(msg[this.numerator] != null) {
      var percent = parseFloat(msg[this.numerator]) / parseFloat(msg[this.denominator]);
      this.domobj.innerHTML = (100. * percent).toFixed(2) + "%";
    }
  }
});

// *******************************************

var NumberWidget = Class.create(Widget, {
  initialize: function($super, domobj) {
    $super(domobj);
    this.key = domobj.getAttribute("key");
    this.precision = domobj.getAttribute("precision");
    if(this.precision == null) {
      this.precision = 2;
    } else {
      this.precision = parseInt(this.precision);
    }
  },

  init: function() {
  },

  receive: function(topic, msg) {
    var v = 'msg.' + this.key;
    var val = eval(v);
    if(val != null) {
      this.domobj.innerHTML = (val).toFixed(this.precision);
    }
  }
});


// *******************************************

var ScrollingTextWidget = Class.create(Widget, {
  initialize: function($super, domobj) {
    $super(domobj);
    this.maxlines = domobj.getAttribute("maxlines");
    if(!this.maxlines) this.maxlines = 40;
    else this.maxlines = parseInt(this.maxlines);

    this.textdiv = null;
  },

  init: function() {
      //this.domobj.style.position = "absolute";
      this.domobj.style.overflowY = "scroll";

      this.textdiv = document.createElement("div");
      this.domobj.appendChild(this.textdiv);
  },

  new_message: function(msg) {
     var d = document.createElement("div");
     d.innerHTML = Object.toJSON(msg) + "<br>";
     this.textdiv.appendChild(d);      
  },

  receive: function(topic, msg) {
    this.new_message(msg);
    if(this.textdiv.childNodes.length > this.maxlines) {
      this.textdiv.removeChild(this.textdiv.childNodes[0]);
    } else {
      this.domobj.scrollTop = this.domobj.scrollHeight;
    }
  }
});

// *******************************************

var msg_and_border_width = 0;

var SliderTextWidget = Class.create(Widget, {
  initialize: function($super, domobj) {
    $super(domobj);
    this.maxlines = 100;
    this.msg_width = 400;
    this.slider_padding = 10;
    this.border_width = 1;
    msg_and_border_width = this.msg_width + this.border_width * 2;
    this.textdiv = null;
  },

  init: function() {
      this.jquery.prepend("<p id='no_data' style='font-size: 200%; text-align: center; color: #fff'>(No data)</p>");
      this.domobj.style.overflowX = "hidden";
      this.domobj.style.overflowY = "hidden";
      this.domobj.style.width = "" + (this.msg_width + this.border_width * 2) * 3 + "px";
      this.domobj.style.position = "relative";
      
      this.jquery.append('<div id="slider_contents" style="width: 50000px; position: relative;"></div>');
      this.jquery.after('<div id="slider_data">Head: Now</div>');
      this.jquery.after('<div id="slider" style="margin: 10px; width: ' + ((this.msg_width + this.border_width * 2) * 3 - this.slider_padding * 2) + 'px;"></div>');
      this.jquery.after('<div style="width: ' + ((this.msg_width + this.border_width * 2) * 3) + 'px; text-align: center;"><div style="float: left; color: #4f4;">New ...</div><div style="float: right">... Old</div>Drag the slider to see message history</div>')
      this.textdiv = this.jquery.find("#slider_contents")[0];
      
      this.jquery.append('<div style="width:' + (this.msg_width + this.border_width * 2) + 'px; height: 100%; position: absolute; left: ' + (this.msg_width + this.border_width * 2) + '; opacity: 0.5; background: black"></div>')      
      this.jquery.append('<div style="width:' + (this.msg_width + this.border_width * 2) + 'px; height: 100%; position: absolute; left: ' + ((this.msg_width + this.border_width * 2) * 2) + '; opacity: 0.7; background: black"></div>')
      
      jQuery("#slider").slider({
        slide: function(event, ui) {
          var str = "Head: Now";
          if (ui.value > 0) { 
            str = str + " - " + ui.value + " messages"; 
          }
          jQuery("#slider_data").html(str);
          jQuery("#slider_contents").css({left: -msg_and_border_width*ui.value});
        }
      });  

  },

  list_node: function(key, value, prefix) {
    //ros_debug("key value " + key + value);
    if (typeof(value) == "object") {
      var buf = ""; //"<ul>";
      buf = buf + key + "<br />";
      //if (value.length > 0) {
      for (var sub_key in value) {
        if (typeof(value[sub_key]) != "function") {
          buf = buf + this.list_node(sub_key, value[sub_key], prefix + "&nbsp;&nbsp;");
        }
      }
      //}
      //buf = buf + "</ul>";
      return buf;
    } else if (typeof(value) == "array") {
      return "ARRAY";   
    } else {
      return prefix + key + ": " + value + "<br />";
    }
  },
  
  new_message: function(msg) {
    var d = document.createElement("div");
    d.style.background = "#333";
    d.style.borderWidth = "1px";
    d.style.borderColor = "#4f4";
    d.style.borderStyle = "solid";
    d.style.cssFloat = "left";
    d.style.width = "" + this.msg_width + "px";
    d.innerHTML = Object.toJSON(msg);
     
    var s = "";//"<ul>";
    for(var key in msg) {
      if (key != "header") {
        s = s + this.list_node(key, msg[key], "");
      } else {
        s = s + "seq #: " + msg[key].seq + "<br />";
      }
    }
    //s = s + "</ul>";
    d.innerHTML = "<div style='padding: 4px'>" + s + "</div>";
    
    this.jquery.find("#no_data").remove();
    jQuery(this.textdiv).prepend(d);
  },

  receive: function(topic, msg) {
    this.new_message(msg);
    if(this.textdiv.childNodes.length > this.maxlines) {
      var num_nodes = this.textdiv.childNodes.length;
      this.textdiv.removeChild(this.textdiv.childNodes[num_nodes-1]);
    } else {
      this.domobj.scrollTop = this.domobj.scrollHeight;
    }
  }
});

var TerminalTextWidget = Class.create(ScrollingTextWidget, {
  initialize: function($super, domobj) {
    this.key = domobj.getAttribute("key");
    $super(domobj);
  },
  new_message: function($super, msg) {
    if(msg[this.key] != null) {
      var s = msg[this.key].toString();
      var lines = s.split(/\n/);
      for(var i=0; i<lines.length; i++) {
      var line = lines[i];
      var d = document.createElement("div");
      d.appendChild(document.createTextNode(line));
      this.textdiv.appendChild(d);      
      }
    }
  }
});

// *******************************************

var ListWidget = Class.create(Widget, {
  initialize: function($super, domobj) {
    $super(domobj);
    this.key = domobj.getAttribute("key");
    this.itemhtml = null;
  },

  init: function() {
    this.itemhtml = this.domobj.innerHTML;
    this.domobj.innerHTML = "";
  },

  receive: function(topic, msg) {
    if(msg[this.key] != null) {
      var lst = msg[this.key];
      var s = "";
      for(var i=0; i<lst.length; i++) {
        if (this.key == "nodes") {
          lst[i] = "<a href='" + HDF.CGI.BaseURI + "webui/node.py?node=" + lst[i] + "'>" + lst[i] + "</a>";
        }
	      if(this.itemhtml) {
	        var t = this.itemhtml.replace(/__item__/g, lst[i]);
	        s = s + t;
	      } else {
	        s = s + "<li>" + lst[i];
	      }
      }
      this.domobj.innerHTML = s;
    }
  }
});

// *******************************************

var MessageWidget = Class.create(Widget, {
  init: function() {
    this.domobj.innerHTML = "";
  },

  receive: function(topic, msg) {
    var s = "<ul>";
    for(var key in msg) {
      s = s + "<li>" + key + ": " + msg[key];
    }
    s = s + "</ul>";
    this.domobj.innerHTML = s;
  }
});

var RosOut_Widget = Class.create(Widget, {
  initialize: function($super, domobj) {
    $super(domobj);
    this.topics = ['/rosout'];
    this.maxlines = domobj.getAttribute("maxlines");
    this.even = true;
    if(!this.maxlines) this.maxlines = 100;
    else this.maxlines = parseInt(this.maxlines);

    this.msg_levels = {
      16: "Fatal",
      8: "Error",
      4: "Warn",
      2: "Info",
      1: "Debug"
    }
    
    this.tbl = null;
  },

  init: function() {
    this.tbl = document.createElement("table");
    this.tbl.width = "100%";
    this.tbl.className = "rosout";
    this.tbl.cellSpacing = "0";
    this.domobj.appendChild(this.tbl);
  },

  new_message: function(msg) {
    jQuery(this.tbl).append("<tr class='rosout " + (this.even ? "even" : "odd") + "'><td class='rosout severity'>" + this.msg_levels[msg.level] + "</td><td class='rosout message'>" + msg.msg + "</td><td class='rosout node'><a href='node.py?node=" + msg.name + "'>" + msg.name + "</a></td></tr>");
    this.even = !this.even;
  },

  receive: function(topic, msg) {
    this.new_message(msg);
    if(this.tbl.rows.length > this.maxlines) {
      this.tbl.deleteRow(0);
    } else {
      this.domobj.scrollTop = this.domobj.scrollHeight;
    }
  }
});


function toDiffTime(theDiff) {
  var years = Math.floor (theDiff/31536000);
  theDiff = theDiff - (years * 31536000);

  var aMonth = (3600*24*30.5);

  var months = Math.floor (theDiff/aMonth);
  theDiff = theDiff - (months * aMonth);

  var weeks = Math.floor (theDiff/604800);
  var days = Math.floor ((((theDiff/60)/60)/24)%7);
  var hours = Math.floor (((theDiff/60)/60)%24);
  var minutes = Math.floor ((theDiff/60)%60);

  if(years >= 2) {
    if(years == 1) year_str = "year";
    else year_str = "yrs";
  } else {
    months = months + 12*years;
  }

  if(months == 1) month_str = "month";
  else month_str = "mths";

  if(weeks == 1) week_str = "week";
  else week_str = "wks";
  if(days == 1) day_str = "day";
  else day_str = "days";
  if(hours == 1) hour_str = "hr";
  else hour_str = "hrs";
  if(minutes == 1) minute_str = "min";
  else minute_str = "mins";

  var s = "";
  if(years >= 2) {
    s = s + years + " " + year_str + " ";
    s = s + months + " " + month_str + " ";
  } else if(weeks > 0 && months > 0) {
    s = s + months + " " + month_str + " ";
    s = s + weeks + "&nbsp;" + week_str + " ";
  } else if(days > 0) {
    s = s + days + "&nbsp;" + day_str;
  } else if(hours > 0) {
    s = s + hours + "&nbsp;" + hour_str;
  } else if(minutes > 0) {
    s = s + minutes + "&nbsp;" + minute_str;
  } else {
    s = s + "Active";
  }
  return s;
}

var UserListWidget = Class.create(Widget, {
  initialize: function($super, domobj) {
    $super(domobj);
    this.itemhtml = null;

    this.hosts = new Hash();
  },

  init: function() {
    this.itemhtml = this.domobj.innerHTML;
    this.domobj.innerHTML = "";
  },

  receive: function(topic, msg) {
      this.hosts.set(msg.hostname, msg);

      var users = new Hash();
      this.hosts.each(function(pair) {
        var msg = pair.value;
	var lst = msg.users;
        for(var i=0; i<lst.length; i++) {
	  var user = users.get(lst[i].username);
	  if(user == null) {
	    var user = {};
	    users.set(lst[i].username, user);
	    user.username = lst[i].username;
	    user.idletime = lst[i].idletime;
	  } else {
	    user.idletime = Math.max(user.idletime, lst[i].idletime);
	  }
	}
      }); 
      var s = "" + "<table>";
      users.each(function(pair) {
        var dt = msg.current_time - pair.value.idletime;
	s = s + "<tr><td>"+pair.value.username + "</td></td>" + toDiffTime(dt) + "</td></tr>\n";
      });
      s = s + "</table>\n";
      
      this.domobj.innerHTML = s;
  }
});

// *******************************************

var LaunchButtonWidget2 = Class.create(Widget, {
  initialize: function($super, domobj) {
    $super(domobj);
    this.taskid = domobj.getAttribute("taskid");
    this.state = null;
    this.topics = ["/app_status"];

    this.button = null;
    this.statusdiv = null;
  },

  init: function() {
    var obj = this;

    this.domobj.innerHTML = '<input class=app_button type=button value="">\n<span class=app_status>&nbsp;</span>';
    this.button = this.domobj.childNodes[0];
    this.statusdiv = this.domobj.childNodes[2];

    this.button.onmousedown = function() {obj.onmousedown();};

    this.set_state();
  },

  receive: function(topic, msg) {
    if(topic == "/app_update") this.receive_app_update(msg);
    else if(topic == "/app_status") this.receive_app_status(msg);
  },

  receive_app_status: function(msg) {
    var active = false;
    for(var i=0; i<msg.active.length; i++) {
      if(msg.active[i].taskid == this.taskid) {
	this.receive_app_update(msg.active[i]);
	active = true;
      }
    }
    if(active == false) {
      this.state = false;
      this.set_state();
    }
  },

  receive_app_update: function(msg) {
    if(msg.taskid != this.taskid) return;

    var prev_state = this.state;

    this.statusdiv.innerHTML = msg.status;

    var state = msg.status;
    if(state == "running") this.state = true;
    if(state == "stopped") this.state = false;

    if(prev_state != this.state) {
      this.set_state();
    }
  },

  set_state: function() {
    if(this.domobj == null) return;

    if(this.state == true) {
      this.button.disabled = 0;
      this.button.value = "Stop";
    } else if (this.state == false) {
      this.button.disabled = 0;
      this.button.value = "Launch";
    } else {
      this.button.disabled = 1;
      this.button.value = "";
    }
  },

  start_task_callback: function(status) {
    if (status != "done") {
      dialogMessage("Oops!", "There was a problem starting this app: " + status,
        {
                buttons:
                {
				    Ok: function() {
					    jQuery(this).dialog('close');
				    }
			    }
			});
    }
  },

  stop_task_callback: function(status) {
    if (status != "done") {
      dialogMessage("Oops!", "There was a problem stopping this app: " + status,
         {
                buttons:
                {
				    Ok: function() {
					    jQuery(this).dialog('close');
				    }
			    }
			});
    }
  },

  onmousedown: function(evt) {    
    var newstate = !this.state;

    if(newstate) {
      this.pump.service_call("start_task", {'taskid':this.taskid, 'username':'anonymous'}, this.start_task_callback);
    } else {
      this.pump.service_call("stop_task", {'taskid':this.taskid, 'username':'anonymous'}, this.stop_task_callback);
    }
  }
});

// *******************************************

Array.prototype.avg = function() {
    var av = 0;
    var cnt = 0;
    var len = this.length;
    for (var i = 0; i < len; i++) {
        var e = +this[i];
        if(!e && this[i] !== 0 && this[i] !== '0') e--;
        if (this[i] == e) {av += e; cnt++;}
    }
    return av/cnt;
}

AP_MAP = {
    "00:24:6c:82:48:10": "ar03",
    "00:24:6c:82:48:18": "ar03",
    "00:24:6c:82:3a:00": "ar04",
    "00:24:6c:82:3a:08": "ar04",
    "00:24:6c:82:2f:30": "ar05",
    "00:24:6c:82:2f:38": "ar05",
    "00:24:6c:81:b9:f0": "ar06",
    "00:24:6c:81:b9:f8": "ar06",
    "00:24:6c:81:bf:80": "ar07",
    "00:24:6c:81:bf:88": "ar07",
    "00:24:6c:82:54:b0": "ar08",
    "00:24:6c:82:54:b8": "ar08",
    "00:24:6c:82:45:e0": "ar09",
    "00:24:6c:82:45:e8": "ar09",
    "00:24:6c:81:d5:e0": "ar11",
    "00:24:6c:81:d5:e8": "ar11",
};

var WirelessSignalMonitor = Class.create(Widget, {
  init: function() {
    this.valid = false;
    var tool = 
      ['<div id="wireless_mask">',
        '<div id="wireless_strength_0_20" style="height: 4px; top: 20; left: 0;"></div>',
        '<div id="wireless_strength_20_40" style="height: 8px; top: 16; left: 6;"></div>',
        '<div id="wireless_strength_40_60" style="height: 12px; top: 12; left: 12;"></div>',
        '<div id="wireless_strength_60_80" style="height: 16px; top: 8; left: 18;"></div>',
        '<div id="wireless_strength_80_100" style="height: 20px; top: 4; left: 24;"></div>',
        '</div>',
      '<div id="wireless_strength">n/a</div>'];
    this.jquery.append(tool.join(''));
    this.jquery.menu();
  },

  check: function() {
    if (!this.valid) {
      this.jquery.find("#wireless_mask").css({"-moz-opacity": 0.2});   
      this.jquery.find("#wireless_strength").html('n/a');  
      this.jquery.find(".menu_pane").html("<b>Wireless</b><br />no data")
    }
  },

  receive: function(topic, msg) {
    var percent = msg.access_point.quality;
    this.valid = msg.access_point_valid;
    this.jquery.find("#wireless_mask").css({"-moz-opacity": 1});   
    this.jquery.find("#wireless_mask").css({width: percent*28/100});   
    this.jquery.find("#wireless_strength").html(percent + '%');  
    
    // look up the access point from the table
    var ap = AP_MAP[msg.access_point["macaddr"].toLowerCase()]
    msg.access_point["access_point"] = ap ? ap : "unknown";
    
    writeMenuTable(jQuery(this.domobj), msg.access_point);
    this.ping();
  },
});

// *******************************************

var redirectROSPage = function(url) {
    //dialogMessage("Contacting ROS", "Working...");
    document.location.href = url;
}

var LaunchAppFromInfoPageWidget = Class.create(Widget, {
  initialize: function($super, domobj) {
    $super(domobj);
    this.taskid = domobj.getAttribute("taskid");
    this.app_name = domobj.getAttribute("app_name");
    this.state = null;
    this.topics = ["/app_status"];

    this.button = null;
    this.statusdiv = null;
    this.jquery = null;
    this.app_started_on_page = false;
  },

  init: function() {
    var obj = this;

    //this.domobj.innerHTML = '<input class=app_button type=button value="">\n<div class=app_status>&nbsp;</div>';
    
    this.jquery = jQuery(this.domobj);
   
    this.button = this.domobj.childNodes[0];
    this.statusdiv = this.domobj.childNodes[2];

    var self = this;
    this.jquery.find("input.button, .start_app, .stop_app").click(function() {
      self.app_started_on_page = true;
      obj.onmousedown();
      return false;
    });
    //this.button.onmousedown = function() {obj.onmousedown();};

    this.set_state();
  },

  receive: function(topic, msg) {
    if(topic == "/app_update") this.receive_app_update(msg);
    else if(topic == "/app_status") this.receive_app_status(msg);
  },

  receive_app_status: function(msg) {
    var active = false;
    for(var i=0; i<msg.active.length; i++) {
      if(msg.active[i].taskid == this.taskid) {
	      this.receive_app_update(msg.active[i]);
      	active = true;
      }
    }
    if(active == false) {
      this.state = false;
      this.set_state();
    }
  },

  receive_app_update: function(msg) {
    if(msg.taskid != this.taskid) return;

    var prev_state = this.state;

    this.jquery.find(".status").html(msg.status);

    var state = msg.status;
    if (state == "running") {
      this.state = true;
      this.jquery.find(".spinner").hide();
      this.jquery.find(".status").hide();

      var app_name = this.app_name;
      var taskid = this.taskid;
      var started_on_page = this.app_started_on_page;
      // show a dialog telling the user the app has started
      jQuery.get(HDF.CGI.BaseURI + "active/active.py?Action.NoticeDismissed=1&notice=app_running", function(ret_val) {
	      if (ret_val.indexOf("YES") > -1) {
	          if (started_on_page) {
        		  redirectROSPage(HDF.CGI.BaseURI + "app/" + taskid + '/');
	          }
	      } else if (started_on_page) {
		      var settings = {
		          buttons: {
			          "Don't show this message again": function() { 
			              jQuery.get(HDF.CGI.BaseURI + "active/active.py?Action.DismissNotice=1&notice=app_running",
					         function(ret_val) {
					             if (ret_val.indexOf("OK") > -1) {
						            jQuery("#dialog").dialog("close");
						            redirectROSPage(HDF.CGI.BaseURI + "app/" + taskid + '/');
					             } else {
						            alert("Uh oh!  There was a problem saving your preference.");
					             }
					         });
			      },
			          "Ok": function() { 
			            jQuery(this).dialog("close");
                        redirectROSPage(HDF.CGI.BaseURI + "app/" + taskid + '/');
			          },
		          }
		      };
		      var content = "<p>The app " + app_name + " is now running.</p>";
		      content += "<p>Apps that are running have a <span style='color: #4f4'>green</span> border around their icons.</p>";
		      content += "<p>You can see which apps are running by looking for their icon thumbnails in the dashboard at the top of the page.</p>";
		      dialogMessage("App Status", content, settings);
	      }
	  });
      
      // go to the app?
      //this.jquery.find(".go_to_app").show();
      this.jquery.find(".stop_app").show();
      this.jquery.find(".start_app").hide();

    } else {
      if (state == "stopped") {
        this.state = false;
        this.jquery.find(".spinner").hide();
        this.jquery.find(".status").hide();
        if (this.app_started_on_page) {
            redirectROSPage(HDF.CGI.BaseURI + "webui/appinfo.py?taskid=" + this.taskid);
        }
      } else if (state == "error") {
        this.state = true;
        this.jquery.find(".spinner").hide();
        this.jquery.find(".app_thumb").css({borderColor: "#f44"});
        this.jquery.find(".status").show();
      } else if (state == "starting" || state == "stopping") {
        this.jquery.find(".spinner").show(); 
        this.jquery.find(".app_thumb").css({borderColor: "#fe9"});
        this.jquery.find(".status").show();
      } else {
        this.jquery.find(".spinner").hide();
        this.jquery.find(".status").show();
      }
      //this.jquery.find(".go_to_app").hide();
      this.jquery.find(".stop_app").hide();
      this.jquery.find(".start_app").show();
    }
    
    if(prev_state != this.state) {
      this.set_state();
    }
  },
  
  start_task_callback: function(status) {
    if (status != "done") {
      dialogMessage("Oops!", "There was a problem starting this app: " + status,
         {
                buttons:
                {
				    Ok: function() {
					    jQuery(this).dialog('close');
				    }
			    }
			});
    }
  },

  stop_task_callback: function(status) {
    if (status != "done") {
      dialogMessage("Oops!", "There was a problem stopping this app: " + status,
         {
                buttons:
                {
				    Ok: function() {
					    jQuery(this).dialog('close');
				    }
			    }
			});
    }
  },

  set_state: function() {
    if(this.domobj == null) return;

    if(this.state == true) {
      this.jquery.find("input.button").addClass("running").removeAttr("disabled");
      this.jquery.find(".app_thumb").css({borderColor: "#44ff44"});
    } else if (this.state == false) {
      this.jquery.find("input.button").removeClass("running").removeAttr("disabled");
      this.jquery.find(".app_thumb").css({borderColor: "#333"});
      //this.jquery.find(".go_to_app").hide();
      this.jquery.find(".start_app").show();
    }
  },

  onmousedown: function(evt) {    
    var newstate = !this.state;

    if(newstate) {
      this.pump.service_call("start_task", {'taskid':this.taskid, 'username':'anonymous'}, this.start_task_callback);
    } else {
      this.pump.service_call("stop_task", {'taskid':this.taskid, 'username':'anonymous'}, this.stop_task_callback);
    }
  }
});

// *******************************************


var ActiveTasks = Class.create(Widget, {
  init: function() {
    this.domobj.innerHTML = "";
    this.role = [this.domobj.getAttribute("role")];
  },

  receive: function(topic, msg) {
    var s = "";
    for(var i=0; i<msg.active.length; i++) {
      if (this.role == "admin") {
        s = s + "<a href='" + HDF.CGI.BaseURI + "app/" + msg.active[i].taskid + "/'><img title='" + msg.active[i].name + " is running' style='width:26px; height:26px; border: 2px solid #4f4;' src='" + HDF.CGI.BaseURI + "app/" + msg.active[i].taskid + "/" + msg.active[i].icon + "' /></a>";	  
	  } else {
        s = s + "<img title='" + msg.active[i].name + " is running' style='width:26px; height:26px; border: 2px solid #4f4;' src='" + HDF.CGI.BaseURI + "app/" + msg.active[i].taskid + "/" + msg.active[i].icon + "' />";	  
	  }
    }
    this.domobj.innerHTML = s;
  }
});

// ********************************************

var ActiveUIApps = Class.create(Widget, {
  init: function() {
    this.domobj.innerHTML = "";
  },

  receive: function(topic, msg) {
    var html = ["No user interface applications are running."];
    var ui_msg = [];    
    for (var i=0; i<msg.active.length; i++) {
      if (msg.active[i].provides == "joy") {
        ui_msg.push(msg.active[i]);
      }
    }
    
    if (ui_msg.length == 1) {
      var url = HDF.CGI.BaseURI + "app/" + ui_msg[0].taskid + '/';
      html = ["<p id='loading'>Taking you to the teleop page...</p>"];
      document.location.href = url;
    } else if (ui_msg.length > 0) {
      var html = ["<p>The following user interface applications are running:</p><table class='running_ui_apps'>"];
      for(var i=0; i<ui_msg.length; i++) {
        html.push("<tr><td><a href='" + HDF.CGI.BaseURI + "app/" + ui_msg[i].taskid + "/'>");
        html.push("<img title='" + ui_msg[i].name + "' style='border: 0px;' src='" + HDF.CGI.BaseURI + "app/" + ui_msg[i].taskid + "/" + ui_msg[i].icon + "' />");
        html.push("</a></td>");
        html.push("<td class='ui_app_title'><a href='" + HDF.CGI.BaseURI + "app/" + ui_msg[i].taskid + "/'>" + ui_msg[i].name + "</a></td></tr>");
      }
      html.push("</table>");    
    }
    this.domobj.innerHTML = html.join('');    
  }
});

// ********************************************

var PresenceWidget = Class.create({
  initialize: function(domobj) {
    this.pump = null;
    this.domobj = domobj;
    this.topics = [domobj.getAttribute("topic")];
    this.key = domobj.getAttribute("key");
    this.itemhtml = null;

    this.users = new Hash();
  },

  init: function() {
    this.itemhtml = this.domobj.innerHTML;
    this.domobj.innerHTML = "";
  },

  receive: function(topic, msg) {
    var now = new Date().getTime();
    var user = msg.data;
    this.users.set(user, now);

    var users = [];
    this.users.each(function(pair) {
      if(now - pair.value < 20000) {
        if (HDF.CGI.ActiveUser && HDF.CGI.ActiveUser == pair.key) {
          users.push("<span title='" + pair.key + " is the pilot on this robot' style='font-weight: bold; color: white;'>" + pair.key + "</span>");
        } else {
          users.push("<span title='" + pair.key + " is a passenger on this robot'>" + pair.key + "</span>");
        }
      }
    });
    this.domobj.innerHTML = "Online: " + users.join(', ') + "<br />";
  }
});


// ********************************************

var UsersOnlineWidget = Class.create(Widget, {
  initialize: function($super, domobj) {
    $super(domobj);
  },

  init: function() {
    this.itemhtml = this.domobj.innerHTML;
    this.domobj.innerHTML = "";
  },

  receive: function(topic, msg) {
    var online = msg.data.split(':')[0];
    var active = msg.data.split(':')[1];
    var current_html = this.domobj.innerHTML;
    
    var users = [];
    online.split(',').each(function(user) {
        if (active == user) {
          users.push("<span title='" + user + " is the pilot on this robot' style='font-weight: bold; color: white;'>" + user + " (pilot)</span>");
        } else {
          users.push("<span title='" + user + " is a passenger on this robot'>" + user + "</span>");
        }
    });
    
    var new_html = "Online: " + users.join(', ') + "<br />";
    
    this.domobj.innerHTML = new_html;
    
    // detect a change
    if (current_html != "" && current_html != new_html) {
        // see if this user was the pilot
        if (current_html.indexOf(HDF.CGI.Login + " (pilot)") > -1 && new_html.indexOf(HDF.CGI.Login + " (pilot)") == -1) {
            dialogMessage("Robot Status", "The user <strong class='user'>" + active + "</strong> has become the pilot of this robot.  You are now logged in as a passenger.", 
            {
                buttons: {
                    "Ok": function() {
		                document.location.href = HDF.CGI.BaseURI + "active/active.py?Action.MakeInactive=1&request=" + HDF.CGI.RequestURI;
                    },
                }            
            });
        }
    }
  }
});

var latency_measurements = [];
  

var update_latency = function() {
    if (latency_measurements.length > 0) {
        var avg = latency_measurements.slice(latency_measurements.length - 3, latency_measurements.length).avg().toFixed(1);
        jQuery("#nav_element_latency").html(avg + " ms");
    }
}

var time_request = function() {
    var request_start = new Date();
    new Ajax.Request(gPump.urlprefix + "/ping/", {
        method: 'get',
        onSuccess: function(transport) {
            var request_time = (new Date()) - request_start;
            latency_measurements.push(request_time);
            if (latency_measurements.length > 100) {
                latency_measurements = latency_measurements.slice(10);
            }
            update_latency();
	    gPump.publish("/webui/events", "web_msgs/WebEvent", ["browser", "ping", "" + request_time]);
        }
    });
    setTimeout(function() {
        time_request();
    }, 5000);
}


// ********************************************

/*
PILOT_REQUEST_TIME = 30;

var PilotResponseWidget = Class.create(Widget, {
	initialize: function($super, domobj) {
	    $super(domobj);
	    this.wait_time = PILOT_REQUEST_TIME;
	    this.updateTimer();
	},

	init: function() {
	},

	updateTimer: function() {
	    if (this.wait_time == 0) {
		jQuery("#loading").hide();
		jQuery("#response").html("The pilot did not respond in time, so you will become the pilot.  Continuing login...").show();
		setTimeout(function() {
			document.location.href = HDF.CGI.BaseURI + "active/active.py?Action.MakeActive=1&override=1&request=" + HDF.CGI.RedirectURI;
		    }, 5000);
	    } else {
		jQuery("#loading").html("Please wait up to " + this.wait_time + " seconds for a response...");
		this.wait_time -= 1;
		var that = this;
		setTimeout(function() {
			that.updateTimer();
		    }, 1000);
	    }
	},

	receive: function(topic, msg) {
	    var data = msg.data.split(':')[0];
	    var timestamp = msg.data.split(':')[1];
	    
	    if (user == HDF.CGI.Login) {
		return;
	    }
	    
	    timestamps = jQuery.cookie("pilot_response_timestamps");
	    if (!timestamps) { 
		timestamps = "";
	    }
	    
	    if (timestamps.indexOf(timestamp) == -1) {
		var user = data.split(',')[0];
		var resp = data.split(',')[1];
		if (user == HDF.CGI.Login && resp == "yes") {
		    jQuery("#loading").hide();
		    jQuery("#response").html("Your request was granted.  Continuing login...").show();	    
		    document.location.href = HDF.CGI.BaseURI + "active/active.py?Action.MakeActive=1&request=" + HDF.CGI.RedirectURI;
		} else if (user == HDF.CGI.Login && resp == "no") {
		    jQuery("#loading").hide();
		    jQuery("#response").html("Your request to pilot the robot was denied.  You will be logged in as a passenger.").show();
		    setTimeout(function() {
			    document.location.href = HDF.CGI.BaseURI + "active/active.py?Action.MakeInactive=1&request=" + HDF.CGI.RedirectURI;
			}, 5000);
		}
	    }

	    var appended = timestamps.split(',');
	    appended.push(timestamp);
	    if (appended.length > 100) {
		appended = appended.slice(50); // slice a bunch so you don't have to do it as often
	    }
	    jQuery.cookie("pilot_response_timestamps", appended.join(','));
	}
});


var PilotControlWidget = Class.create(Widget, {
  initialize: function($super, domobj) {
    $super(domobj);
  },

  init: function() {
  },

  receive: function(topic, msg) {
    var user = msg.data.split(':')[0];
    var timestamp = msg.data.split(':')[1];

	if (user == HDF.CGI.Login) {
		return;
	}

    timestamps = jQuery.cookie("pilot_request_timestamps");
    if (!timestamps) { 
	timestamps = "";
    }
    if (timestamps.indexOf(timestamp) == -1) {
        dialogMessage("Robot Status", "The user " + user + " wants to become the pilot of this robot.  Do you want to let them?" +
					  "<p style='font-size: 80%'>Note: You have <span id='wait_time'>30</span> seconds to say no;  If you say yes or do not respond, your current page will be reloaded and you will become a passenger</p>", 
        {
            buttons: {
                "Yes": function() {
		    gPump.publish("/pilot_response", "std_msgs/String", [user + ",yes:" + (new Date()).getTime()]);
		    document.location.href = HDF.CGI.BaseURI + "active/active.py?Action.MakeInactive=1&request=" + HDF.CGI.RequestURI;
                },
                "No": function() {
		    gPump.publish("/pilot_response", "std_msgs/String", [user + ",no:" + (new Date()).getTime()]);
                    jQuery(this).dialog("close");
                }                
            }
        
        });
	setTimeout(function() {
		document.location.href = HDF.CGI.BaseURI + "active/active.py?Action.MakeInactive=1&request=" + HDF.CGI.RequestURI;
	    }, PILOT_REQUEST_TIME*1000);
    }

    var appended = timestamps.split(',');
    appended.push(timestamp);
    if (appended.length > 100) {
        appended = appended.slice(50); // slice a bunch so you don't have to do it as often
    }
    jQuery.cookie("pilot_request_timestamps", appended.join(','));
  }
});
*/


