
// *******************************************************
function getDataFromServer(id, url, callback) { 
  var oScript = document.getElementById(id); 
  var head = document.getElementsByTagName("head").item(0); 
  if (oScript) { 
    // Destroy object 
    head.removeChild(oScript); 
    delete oScript;
  }
  // Create object 
  oScript = document.createElement("script"); 
  oScript.setAttribute("type","text/javascript");
  oScript.setAttribute("charset","utf-8");
  oScript.setAttribute("src",url)
  oScript.setAttribute("id",id); 
  head.appendChild(oScript); 
} 

// *******************************************************


var MessagePump = Class.create({
  initialize: function(ros_bridge_uri) {
    this.urlprefix = ros_bridge_uri;
    var parts = ros_bridge_uri.split('/')
    this.hostport_prefix = parts.slice(0, 3).join('/')

    this.lastTime = 0;
    this.topicListeners = new Hash();
    this.topicBatchListeners = new Hash();
    this.widgets = [];
    this.widget_hash = {};
    this.http_request = [];
    this.ajax_waiting = new Date();

    this.totalReceivedBytes = 0;
    this.WIDGET_TIMEOUT = 10000; // in milliseconds
    this.WIDGET_LOOP = 5000; // in milliseconds
    this.BRIDGE_TIMEOUT = 30000; // in milliseconds
    this.OFFLINE_COOKIE = "ros_offline";
  },

  registerWidget: function(widget, type) {
    widget.pump = this;

    this.widgets.push(widget);
    if (type) {
        this.widget_hash[type] = widget;
    }

    for(var i=0; i<widget.topics.length; i++) {
        var topic = widget.topics[i];
        this.registerListener(widget, topic);
    }
  },
  
  registerListener: function(listener, topic) {
    var listeners = this.topicListeners.get(topic);
    if(listeners == null) {
      listeners = [];
      this.topicListeners.set(topic, listeners);
    }
    listeners.push(listener);
  },
  
  registerBatchListener: function(listener, topic) {
    var listeners = this.topicBatchListeners.get(topic);
    if (listeners == null) {
        listeners = [];
        this.topicBatchListeners.set(topic, listeners);
    }
    listeners.push(listener);
  },

  service_call2: function(service_name, parameters, callback) {
    var uri = this.urlprefix + '/service/' + service_name + '?callback=g_response&json=' + Object.toJSON([parameters]);

        var that = this;
        new Ajax.Request(uri, {
            method: 'get',
            onSuccess: function(transport) {
                eval(transport.responseText);
                var json = g_response;
                if (callback) {
                    callback(json);
                }
            },
            onFailure: function(transport) {
                log('failure');
            }
        });
    },
      
  evalMessages: function(json_result) {
    this.lastTime = json_result.since;

    // Handle batch listeners (those that want an aggregate of messages).
    // Handy in the case that client-side processing is costly, and we can ignore earlier messages.
    var topicMsgs = new Hash();
    for(var i=0; i<json_result.msgs.length; i++) {
      var msgEnv = json_result.msgs[i];
      var topic = msgEnv.topic;

      if (this.topicBatchListeners.get(topic) != null)
      {
        var msgs = topicMsgs.get(topic);
        if (msgs == null) {
          msgs = [];
          topicMsgs.set(topic, msgs);
        }
        msgs.push(msgEnv.msg);
      }
    }
    var batchTopics = topicMsgs.keys();
    for(var i=0; i < batchTopics.length; i++) {
        var topic = batchTopics[i];

        var listeners = this.topicBatchListeners.get(topic);
        var msgs = topicMsgs.get(topic);
        for (var j=0;j<listeners.length;j++) {
          try {
            listeners[j].receiveMany(topic, msgs);
          } catch (e) {
            ros_debug("Error with receiver: " + e);
          }            
        }
    }
    
    for(var i=0; i<json_result.msgs.length; i++) {
      var msgEnv = json_result.msgs[i];

      var listeners = this.topicListeners.get(msgEnv.topic);
      if(listeners) {
        for(var j=0; j<listeners.length; j++) {
          try {
            listeners[j].receive(msgEnv.topic, msgEnv.msg);
          } catch (e) {
            ros_debug("Error with receiver: " + e);
          }
        }
      } else {
        if(msgEnv.topic != "/topics") {
          // ros_debug("No listeners for " + msgEnv.topic);
        }
      }
    }
  },

 setupWidgets:  function() {
    var widget = null;
    var allHTMLTags=document.getElementsByTagName("*");
    for (i=0; i<allHTMLTags.length; i++) {
	var domobj = allHTMLTags[i];
      
	var objtype = domobj.getAttribute("objtype");
	if(objtype) {
	    var clss = window[objtype];
	    if(clss) {
		widget = new clss(domobj);
		this.registerWidget(widget, domobj.getAttribute("objtype"));
	    }
	}
    }

    for(var i=0; i<this.widgets.length; i++) {
	this.widgets[i].init();
    }

    var urlprefix = this.urlprefix;
  },

  service_call: function(service_name, parameterList, callback) {
    // check to see if this user is the pilot
    // exceptions (service calls we allow from passengers): status_update
    if (jQuery.cookie('inactive') == '1' && service_name != "status_update") {
        var message = "You can only perform this operation (service call: <em>" + service_name + "</em>) if you are logged in as the robot pilot.  ";
        if (HDF.CGI.Login) {
            message += "You are currently logged in as a passenger.";
        } else {
            message += "You are currently not logged in.";
        }

        dialogMessage("Oops!", message,
        	{buttons: {"Ok": function() { 
			    jQuery(this).dialog("close");
            }}}
        );
          
        return;
    }
    
    var parameters = {args: parameterList};

    var uri = this.urlprefix + "/service/" + service_name;
    uri = uri + "?callback=gPump.receive_service&json=" + Object.toJSON(parameterList);
    var parameters = {json: Object.toJSON(parameterList)};

    new Ajax.Request(uri, {
      parameters: parameters, 
      method: 'get',
      onSuccess: function(transport) {
        gPump.ajax_waiting = null;
        jQuery.cookie(gPump.OFFLINE_COOKIE, null, {path: HDF.CGI.BaseURI});
        jQuery("#bridge_loading").hide();
	//jQuery("#nav_element_bridge").hide();
        eval(transport.responseText);
        if (callback) {
          callback(gPump.receive_service.status);        
        }
      },
      onFailure: function(transport) {
        gPump.ajax_waiting = null;
        jQuery.cookie(gPump.OFFLINE_COOKIE, null, {path: HDF.CGI.BaseURI});
        jQuery("#bridge_loading").hide();
        //dialogMessage("Uh oh!", "This service call returned an error.  Please check the web service logs for more information.");
        //gPump.publish("/webui/events", "web_msgs/WebEvent", ["browser", "warning", "service"]); 
	//jQuery("#nav_element_bridge").show();
      },
      onCreate: function(transport, exception) {
        gPump.ajax_waiting = new Date();
        jQuery("#bridge_offline").hide();
        jQuery("#bridge_loading").show();
        setTimeout(function() {
          if (gPump.ajax_waiting) {
            if ((new Date()) - gPump.ajax_waiting > gPump.BRIDGE_TIMEOUT) {
              gPump.ajax_waiting = null;
              jQuery("#bridge_loading").hide();
              //if (!jQuery.cookie(gPump.OFFLINE_COOKIE)) {
              //  dialogMessage("Uh oh!", "Unable to contact the ROS web service.  Please be sure it's running.");
              //  jQuery.cookie(gPump.OFFLINE_COOKIE, true, {path: HDF.CGI.BaseURI});              
              //} else {
              
	      jQuery("#bridge_offline").show();
	      
	      //}
            }
          }
        }, gPump.BRIDGE_TIMEOUT + 500);
      }
    });
  },

  // response:
  // gPump.receive_service={"status": "no parent task group texas_teleop running."}; var _xsajax$transport_status=200;

  publish: function(topic, topic_type, parameterList) {
    var parameters = {args: parameterList};

    var uri = this.urlprefix + "/publish" + topic
    uri = uri + "?topic_type=" + topic_type + "&callback=gPump.publish_callback&json=" + Object.toJSON(parameterList);
    getDataFromServer("_ros_publish_pump", uri);
  },

  receive_server: function(msg) {
    alert('receive_server');
  },

  pump: function() {
    var uri = this.urlprefix + "/receive?callback=gMessage&since=" + this.lastTime;
    var topics = new Hash();
    this.pumping = true;
    this.topicListeners.each(function(pair) {
        topics.set(pair.key, true);
    });
    this.topicBatchListeners.each(function(pair) {
        topics.set(pair.key, true);
    });
    topics.each(function(pair) {
      uri = uri + "&topic=" + pair.key;
    });
    var obj = this;
    new Ajax.Request(uri, {
      method: 'GET',
      //requestHeaders: {Accept: 'application/json'},
      onSuccess: function(transport) {
	try {
 	  //var response_json = transport.responseText.evalJSON();
	  eval(transport.responseText);
	  var json = gMessage;
	  obj.evalMessages(json);

	  // clear the script tags from head        
          jQuery("script[custom=ajax]").remove();
          
	  obj.pumping = true;
          obj.pump();
	} catch(e) {
          setTimeout(function() {
            obj.pump();
	  }, 5000);
	}
      },
      onFailure: function(transport) {
	setTimeout(function() {
	  if (transport.status == 403 || transport.status == 404) {	    
	    pumpStopped(transport.status);
	  }
	  obj.pump();
	}, 5000);
      }
    });
  },
  
  checkWidgets: function() {
    for(var i=0; i<this.widgets.length; i++) {
      var widget = this.widgets[i];
      if (widget) {
        try {
          widget.check();
        } catch (exception) {
          //ros_debug("error checking widget: " + exception);
        }
      }
    }
    setTimeout(function() {
      gPump.checkWidgets();
    }, this.WIDGET_LOOP);
  },
});

function pumpStopped(status) {}

function startWebTracking(pump) {
    jQuery("a").click(function() {
        var link = jQuery(this);
        var data = link.attr('href') + '|' + link.html();
        pump.publish("/webui/events", "web_msgs/WebEvent", ["user", "navigate", data]);
        return true;
    });

    jQuery("input").click(function() {
        var input = jQuery(this);
        var data = input.attr('id');
        pump.publish("/webui/events", "web_msgs/WebEvent", ["user", "input", data]);
        return true;
    });

    pump.publish("/webui/events", "web_msgs/WebEvent", ["browser", "location", document.location.toString()]);
}

var gPump = null;

function ros_handleOnLoad(ros_bridge_uri) 
{
  gPump = new MessagePump(ros_bridge_uri);
  gPump.setupWidgets();

  if(window.gMessage) {
    gPump.evalMessages(window.gMessage);
  }
  gPump.pump();
  gPump.service_call("status_update", []);
  
  gPump.checkWidgets();

  startWebTracking(gPump);
}

// *******************************************

/* parent class for widget objects */

var Widget = Class.create({
  initialize: function(domobj) {
    this.pump = null;
    this.domobj = domobj;
    this.jquery = jQuery(this.domobj);
    this.topics = [domobj.getAttribute("topic")];
    this.lastUpdate = null;
  },

  ping: function() {
    this.lastUpdate = new Date();
  },  
  
  inactive: function() {
    return new Date() - this.lastUpdate > gPump.WIDGET_TIMEOUT;
  },
});

/* utilities used by widgets */

var writeListNode = function(key, value, table, jquery_obj, indent) {
  if (key == "header") {
    return;
  }
  tr = document.createElement("tr");
  if (value.constructor == Object) {
    th = document.createElement("tr");
    th.setStyle("font-weight: bold");
    td = document.createElement("td");
    td.appendChild(document.createTextNode(indent + key));
    th.appendChild(td);
    table.appendChild(th);    
    
    for (var v in value) {
      writeListNode(v, value[v], table, jquery_obj, " - ");
    }
  } else {
    td1 = document.createElement("td");
    td1.appendChild(document.createTextNode(indent + key));
    td2 = document.createElement("td");
    td2.appendChild(document.createTextNode(String(value).replace(/,/g, ', ')));
    tr.appendChild(td1);
    tr.appendChild(td2);
    table.appendChild(tr);  
  }
}

var writeMenuTable = function(jquery_obj, msg, target) {
  if (!target) { target = ".menu_pane"; }
  var title = jquery_obj.attr('title');
  var table = document.createElement("table");
  if (title) {
    th = document.createElement("tr");
    th.setStyle("font-weight: bold");
    td = document.createElement("td")
    td.colSpan = "2";
    td.appendChild(document.createTextNode(title));
    th.appendChild(td);
    table.appendChild(th);
  }
  for (var v in msg) {
    writeListNode(v, msg[v], table, jquery_obj, "");
  }
  jquery_obj.find(target)[0].innerHTML = "";
  jquery_obj.find(target).html(table);
}

function clearDebug() {
  jQuery('#ErrorDiv').html('');
}

function ros_debug(str) {
  jQuery('#ErrorDiv').append(str + '<br />');
}

function clearSelection() {
  if (document.selection) {
    document.selection.clear();
  } else {
    if (window.getSelection) {
      sel = window.getSelection();
      sel.removeAllRanges();
    }
  }
}

var stringToVariable = function(str) {
  return str.replace(/[^0-9A-Za-z]/g,'_');
}

var dialogMessage = function(title, message, options) {
  var settings = jQuery.extend({
    modal: true,
    width: (HDF.CGI.ScreenType && HDF.CGI.ScreenType == 'style_phone.css') ? 300: 400,
    close: function(event, ui) {
      jQuery(this).remove();
    }
  }, options);

  jQuery("body").append("<div id='dialog' style='display: none' title='" + title + "'>" + message + "</div>");
  jQuery("#dialog").dialog(settings);
};

function Set_Cookie( name, value, expires, path, domain, secure ) {
    // set time, it's in milliseconds
    var today = new Date();
    today.setTime( today.getTime() );

    if ( expires ) {
      expires = expires * 1000 * 60 * 60 * 24;
    }
    var expires_date = new Date( today.getTime() + (expires) );

    document.cookie = name + "=" +escape( value ) +
    ( ( expires ) ? ";expires=" + expires_date.toGMTString() : "" ) +
    ( ( path ) ? ";path=" + path : "" ) +
    ( ( domain ) ? ";domain=" + domain : "" ) +
    ( ( secure ) ? ";secure" : "" );
}


/* jQuery plugins */

jQuery.fn.app_thumb = function(options) {
  var settings = jQuery.extend({
    text: 'extra text',
    path: HDF.CGI.BaseURI,
  }, options);

  
  jQuery(this).hover(
    function() {
        jQuery(this).attr("originalColor", jQuery(this).css("borderColor"));
	jQuery(this).css({borderColor: '#ccc'});
    },
    function() {
	jQuery(this).css({borderColor: jQuery(this).attr("originalColor")});
    }
  );
  
    
  jQuery(this).find('.app_image, .app_name').click(
    function() {
      var taskid = jQuery(this).parent().parent().attr('taskid');
      var status = jQuery(this).parent().find(".status").html();
      if (status == "running") {
        window.location.href = settings.path + "app/" + taskid + "/";
      } else {
        window.location.href = settings.path + "webui/appinfo.py?taskid=" + taskid;      
      }
    }
  );

  jQuery(this).find("#favorite_image").click(function() {
    var taskid = jQuery(this).parent().parent().attr('taskid');
    var image = jQuery(this);
    jQuery.post(settings.path + "webui/appinfo.py?Action.Favorites=1&taskid=" + taskid + "&set_favorite=" + (image.hasClass("selected") ? 0 : 1), {}, function(data) {
      var ret_val = data.split('\n')[0];
      if (ret_val == "1") {
        // added to favorites
        image.addClass("selected");
      } else {
        image.removeClass("selected");
        jQuery("#cat_Favorites div[taskid=" + taskid + "]").remove();
        jQuery("div.app_info[taskid=" + taskid +"]").find("#favorite_image").removeClass("selected");
      }
    });
  });

};

jQuery.fn.menu = function(options) {
  jQuery(this).append('<div class="menu_pane"><div class="data">(no data)</div></div>');

  jQuery(this).hover(
    function() {
      jQuery(this).find('.menu_pane').show();
    },
    function() {
      jQuery(this).find('.menu_pane').hide();
    }
  );
};

var Querystring = Class.create({
    initialize: function(qs) {
        this.params = {};
    
        if (qs == null)
            qs = location.search.substring(1, location.search.length);
        if (qs.length == 0)
            return;
    
        qs = qs.replace(/\+/g, ' ');
        var args = qs.split('&');
        for (var i = 0; i < args.length; i++) {
            var pair = args[i].split('=');
            var name = decodeURIComponent(pair[0]);
    
            var value = (pair.length == 2) ? decodeURIComponent(pair[1]) : name;
    
            this.params[name] = value;
        }
    },
    
    get: function(key, dflt) {
        var value = this.params[key];
        return (value != null) ? value: dflt;
    },
    
    contains: function(key) {
        var value = this.params[key];
        return (value != null);
    }
});

