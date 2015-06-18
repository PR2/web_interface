<?cs include:"ros_app_include_header.cs" ?>
<?cs include:"status_header.cs" ?>

<h3>Robot Power Controls</h3>

<p>
    This page provides controls for the robot power systems, motors, and electrical circuits.    
</p>

<p>
    If all the status values below are <span class="powerboard_runstop_true">green</span>, the robot is active.
    If one or more values are <span class="powerboard_runstop_standby">yellow</span> (standby) or <span class="powerboard_runstop_false">red</span> (disabled),
    you can use the controls on the right to get the robot back to 
    an active state.
</p>

<table class="powerboard">
  <tr>
    <th>Component</th>
    <th>Status</th>
    <?cs if:Cookie.inactive != 1 ?>
    <th>Controls</th>
    <?cs /if ?>
  </tr>
  <tr>
    <td class="component">Run-stop</td>
    <td class="status powerboard_runstop_stall" objtype=PowerboardRunStopWidget topic="/dashboard_agg" brake="button">(no data)</td>
    <?cs if:Cookie.inactive != 1 ?>
        <td><a onclick="jQuery('#rs_info').fadeIn();">&raquo; How do I reset the Run-stop?</a></td>
    <?cs /if ?>
  </tr>
  <tr id='rs_info'>
    <td colspan="3">
      <div class="close_button"><a onclick="jQuery('#rs_info').fadeOut();">&laquo; close</a></div>
      <div><strong>Resetting the Run-stop</strong></div>
      <p>There are two parts of the run-stop that need to be checked, and reset if necessary.<p>
      <table style="border: 0;">
      <tr>
        <td style="text-align: center;"><img src='templates/images/button_stop.jpg' alt='Robot button run-stop' /></td>
        <td>There is a round red button on the back of the robot.  Twist it clockwise to reset the stop.  You will feel it pop out slightly when it resets.</td>
      </tr>
      <tr>
        <td style="text-align: center;"><img src='templates/images/run_stop.png' alt='Robot wireless run-stop' /></td>
        <td>There is a small yellow box (known as the <em>wireless stop</em>) with two buttons on it, START and STOP.  
            Press START to reset the run-stop.  When it is running, you will see the top yellow light flashing.
            <br /><span style="font-size:80%">(Note: If the battery light (the bottom light) on the box
            is flashing, the batteries inside the box need to be replaced.)</span>
        </td>
      </tr>
      </table>  
    </td>
  </tr>
  <tr>
    <td class="component">Circuits</td>
  </tr>
  <tr>
    <td class="subcomponent">Left Arm</td>
    <td class="status" objtype=PowerboardBreakerWidget topic="/dashboard_agg" breaker_num=0>No Status</td>
    <?cs if:Cookie.inactive != 1 ?>
    <td>
      <input class=pbbutton type=button value=Run onclick="tryCircuitRun(0);">
      <input class=pbbutton type=button value=Standby onclick="gPump.service_call('power_board/control', [0, 0, 'stop', 0]);">
      <input class=pbbutton type=button value=Disable onclick="gPump.service_call('power_board/control', [0, 0, 'disable', 0]);">
      <input class=pbbutton type=button value="Enable" onclick="gPump.service_call('power_board/control', [0, 0, 'reset', 0]);">
    </td>
    <?cs /if ?>
  </tr>
  <tr>
    <td class="subcomponent">Base/Body</td>
    <td class="status" objtype=PowerboardBreakerWidget topic="/dashboard_agg" breaker_num=1>No Status</td>
    <?cs if:Cookie.inactive != 1 ?>
    <td>
      <input class=pbbutton type=button value=Run onclick="tryCircuitRun(1);">
      <input class=pbbutton type=button value=Standby onclick="gPump.service_call('power_board/control', [0, 1, 'stop', 0]);">
      <input class=pbbutton type=button value=Disable onclick="gPump.service_call('power_board/control', [0, 1, 'disable', 0]);">
      <input class=pbbutton type=button value="Enable" onclick="gPump.service_call('power_board/control', [0, 1, 'reset', 0]);">
    </td>
    <?cs /if ?>
  </tr>
  <tr>
    <td class="subcomponent">Right Arm</td>
    <td class="status" objtype=PowerboardBreakerWidget topic="/dashboard_agg" breaker_num=2>No Status</td>
    <?cs if:Cookie.inactive != 1 ?>
    <td>
      <input class=pbbutton type=button value=Run onclick="tryCircuitRun(2);">
      <input class=pbbutton type=button value=Standby onclick="gPump.service_call('power_board/control', [0, 2, 'stop', 0]);">
      <input class=pbbutton type=button value=Disable onclick="gPump.service_call('power_board/control', [0, 2, 'disable', 0]);">
      <input class=pbbutton type=button value="Enable" onclick="gPump.service_call('power_board/control', [0, 2, 'reset', 0]);">
    </td>
    <?cs /if ?>
  </tr>
  <tr id='cb_info'>
    <td colspan="3">
      <div class="close_button"><a onclick="jQuery('#cb_info').fadeOut();">&laquo; close</a></div>
      <div><strong>Resetting the Circuit Breakers</strong></div>
      <p>To run the robot, all circuits must be running.<p>
      <p>To take a circuit from standby to running, click the 'Run' button for that circuit, above.</p>
      <p>To enable a disabled circuit, click the 'Enable' button for that circuit, above.</p>
    </td>
  </tr>
  <tr>
    <td class="component">Motors</td>
    <td class="status" objtype=PowerboardMotorsWidget topic="/dashboard_agg" key="motors_halted.data">n/a</td>
    <?cs if:Cookie.inactive != 1 ?>
    <td>
      <input class=pbbutton type=button value="Reset" onclick="tryMotorReset();">
      <input class=pbbutton type=button value="Halt" onclick="gPump.service_call('pr2_etherCAT/halt_motors', []);">
    </td>   
    <?cs /if ?>
  </tr>
  <tr>
    <td class="component">Power Board</td>
    <td class="powerboard_runstop_true"><span objtype=TextWidget topic="/dashboard_agg" key="power_board_state.serial_num"></span></td>
    <?cs if:Cookie.inactive != 1 ?>
    <td>-</td>
    <?cs /if ?>
  </tr>
  <tr>
    <td class="component">Input Voltage</td>
    <td class="powerboard_runstop_true"><span objtype=TextWidget topic="/dashboard_agg" key="power_board_state.input_voltage"></span></td>
    <?cs if:Cookie.inactive != 1 ?>
    <td>-</td>
    <?cs /if ?>
  </tr>
</table>


<script>
  var tryMotorReset = function() {
    var warnings = [];
    var rs_warning = false;
    var cb_warning = false;
    jQuery(".status[objtype='PowerboardRunStopWidget']").each(function(elmt) {
        if (jQuery(this).html().indexOf('Running') == -1) {
            rs_warning = true;
            warnings.push("The robot's run-stop has been stopped.  Please reset the run-stop before you reset the circuits or motors.");
        }
    });
    jQuery(".status[objtype='PowerboardBreakerWidget']").each(function(elmt) {
        if (jQuery(this).html().indexOf('On') == -1) {
            cb_warning = true;
            warnings.push("One or more circuit breakers are not on.  Please run all the circuits before you reset the motors.");
        }
    });
    if (warnings.length > 0) {
        var settings = {buttons: {
                "Ok": function() { jQuery(this).dialog("close"); },
        		"Show me how...": function() { 
        		    jQuery(this).dialog("close"); 
        	        if (rs_warning) { jQuery('#rs_info').fadeIn(); }
        		    if (cb_warning) { jQuery('#cb_info').fadeIn(); }
        		}
            }
        };
        dialogMessage("Oops!", "<p>" + warnings.uniq().join("</p><p>") + "</p>", settings);
    } else {
        gPump.service_call('pr2_etherCAT/reset_motors', []);
    }
  }

  var tryCircuitRun = function(circuit_id) {
    var warnings = [];
    jQuery(".status[objtype='PowerboardRunStopWidget']").each(function(elmt) {
        if (jQuery(this).html().indexOf('Running') == -1) {
            warnings.push("The robot's run-stop has been stopped.  Please reset the run-stop before you reset the circuits or motors.");
        }
    });
    if (warnings.length > 0) {
        var settings = {buttons: {
                "Ok": function() { jQuery(this).dialog("close"); },
        		"Show me how...": function() { jQuery(this).dialog("close"); jQuery('#rs_info').fadeIn(); }
            }
        };
        dialogMessage("Oops!", "<p>" + warnings.uniq().join("</p><p>") + "</p>", settings);
    } else {
        gPump.service_call('power_board/control', [0, circuit_id, 'start', 0]);
    }
  }
</script>

<?cs include:"ros_app_include_footer.cs" ?>

