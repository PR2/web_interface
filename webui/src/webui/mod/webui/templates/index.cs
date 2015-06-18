<?cs include:"ros_app_include_header.cs" ?>
  <div class="welcome">
    Welcome to the robot <?cs var:CGI.Robot?>!
  </div>
  
  <?cs if:CGI.robot_type == "texas_alpha" ?>
    <div objtype=ActiveUIApps topic="/app_status">active UI apps</div>
  <?cs /if ?>

  <?cs if:CGI.robot_type == "pr2" ?>
    <div id="not_calibrated">
        The robot is on but not yet calibrated.
	<a onclick='jQuery("#explain_calibration").fadeIn();'>What does this mean?</a>
    </div>
    <div id="explain_calibration">
        <div class="close_button"><a onclick="jQuery('#explain_calibration').fadeOut();">&laquo; close</a></div>
    	<div><strong>Robot Calibration</strong></div>
	<p>
            When the robot is powered on, it starts only its basic systems.  Before it can run applications, it must must also be calibrated, and
	    start its power and motor systems.
	</p>
	<p>
	    When the robot calibrates itself, it will move its arms and head.  Please be sure the robot is in a position where it can safely move its arms.
        </p>
    </div>
    <div objtype=RobotStartup topic="/dashboard_agg"></div>
    <div id="start_robot">
	<input type="button" value="Click Here to Start and Calibrate the Robot" id="start_button" />
	<div id="statuses">
        <h1>Robot Launch Progress</h1>
	<div id="runstop_status" style="display: none">
	     <div class="status_title">Resetting Robot Run-stop</div>
	     <p>
	         The first step in starting the robot is resetting the robot's "run-stop."
	     </p>
	</div>
	<div id="reset_status">
	     <div class="status_title">Robot Software System</div>
	     <div class="progressbar"></div>
	</div>
	<div id="circuit_status">
	     <div class="status_title">Circuits and Power</div>
	     <div class="progressbar"></div>
	</div>
	<div id="motor_status">
	     <div class="status_title">Robot Motors</div>
	     <div class="progressbar"></div>
	</div>
	<div id="reload_status">
	     <div class="status_title">Reloading interface...</div>
	</div>
	</div>
    </div>

    <script>

    var robot_started = false;

    var RobotStartup = Class.create(Widget, {
	    initialize: function($super, domobj) {
            $super(domobj);
            this.domobj.innerHTML = "Checking robot status...";
            var self = this;
            setTimeout(function() {
                if (!robot_started) {
	                jQuery("#not_calibrated").show();
                    jQuery("#start_robot").show();
	                self.domobj.innerHTML = "";
	            }
            }, 3000);
        },

	    init: function() {},

	    receive: function(topic, msg) {
            var self = this;
            if (!robot_started) {
	            if (msg.motors_halted_valid && msg.power_board_state_valid && msg.power_state_valid &&
	                msg.motors_halted.data == false &&
	                msg.power_board_state.run_stop && msg.power_board_state.wireless_stop &&
	                msg.power_board_state.circuit_state.join(',') == "3,3,3"
	            ) {
	                self.domobj.innerHTML = "The robot is calibrated and running.";
                } else {
                    self.domobj.innerHTML = ""; //"The robot is on but not yet calibrated.";
                }
                robot_started = true;
            }
        }
    });

    jQuery(document).ready(function() {
        var startCircuits = function() {	

        	// reset circuits
	        jQuery("#circuit_status .status_title").css({color: '#fff'});
	        gPump.service_call('power_board/control', [0, 0, 'start', 0]);
	        gPump.service_call('power_board/control', [0, 1, 'start', 0]);
	        gPump.service_call('power_board/control', [0, 2, 'start', 0]);

            jQuery("#circuit_status .progressbar").progressbar();
            updateBar("#circuit_status .progressbar", startMotors, 2.5, 0);
	    }

        var startMotors = function() {
            // reset motors
            jQuery("#motor_status .status_title").css({color: '#fff'});
            jQuery("#motor_status .progressbar").progressbar();
            updateBar("#motor_status .progressbar", reloadPage, 2.5, 0);
            gPump.service_call('pr2_etherCAT/reset_motors', []);
        }
	
        var reloadPage = function() {
        	jQuery("#reload_status").fadeIn();
        	window.location.reload();
        }

        var updateBar = function(selector, callback, inc, complete) {
            jQuery(selector).progressbar('option', 'value', complete);
            if (complete >= 100) {
	            callback();
            } else {
                setTimeout(function() {
                    var random_inc = inc * Math.random() * 2;
                    updateBar(selector, callback, inc, complete + random_inc);
                }, 100);
            };
        };


        jQuery("#start_button").click(function() {
            jQuery("#start_button").hide();
            
            // do a robot reset
        	jQuery("#statuses").fadeIn();
	        jQuery("#reset_status .status_title").css({color: '#fff'});

            jQuery("#reset_status .progressbar").progressbar();
            updateBar("#reset_status .progressbar", startCircuits, 0.2, 0);

	        jQuery.get("<?cs var:CGI.BaseURI ?>/webui/reset.py?Action.AjaxReset=1",
	            function(response) {
        	});

            return false;
        });
    });
    </script>

  <?cs /if ?>


<?cs include:"ros_app_include_footer.cs" ?>

