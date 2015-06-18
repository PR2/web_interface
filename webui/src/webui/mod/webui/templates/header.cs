
<div id="navigation" class="ui-tabs ui-widget ui-widget-content ui-corner-all" >
    <ul id="right_nav" class="ui-tabs-nav ui-helper-reset ui-helper-clearfix ui-widget-header ui-corner-all">
    <li>
      <div id="nav_element_bridge" title="Web Bridge" class="nav_element"><span style="text-decoration:line-through">ROS</span></div>
    </li>
    <?cs if:CGI.robot_type == "pr2" ?>
      <li>
        <div id="nav_element_motors" title="Motors" class="nav_element"  objtype=MotorsMonitor topic="/dashboard_agg"></div>
      </li>
      <li>
        <div id="nav_element_circuits" title="Circuits" class="nav_element"  objtype=CircuitMonitor topic="/dashboard_agg"></div>
      </li>
      <li>
        <div id="nav_element_charge" class="nav_element" objtype=ChargeMonitor topic="/dashboard_agg"></div>
      </li>
      <li>
        <div id="nav_element_battery" title="Battery" class="nav_element"  objtype=BatteryMonitor topic="/dashboard_agg" style="white-space:nowrap; font-weight: normal"></div>
      </li>
      <li>
        <div id="nav_element_status" title="Robot Status" class="nav_element"  objtype=RobotStatusMonitor topic="/diagnostics_agg:Status">
          N/A
        </div>
      </li>
    <?cs else ?>
      <li>
        <div id="nav_element_battery" title="Battery" class="nav_element"  objtype=MiniboxDcDc topic="/texas_power/state" style="white-space:nowrap; font-weight: normal"></div>
      </li>
      <li>
        <div id="nav_element_charge" class="nav_element" objtype=TexasChargeMonitor topic="/texas_power/state"></div>
      </li>
    <?cs /if ?>
    <li>
      <div id="nav_element_wireless" title="Wireless" class="nav_element"  objtype=WirelessSignalMonitor topic="/dashboard_agg"></div>
    </li>
    
    <?cs if:CGI.cur.device_style != "style_phone.css" ?>
        <li>
          <div id="nav_element_latency" title="Round-trip Latency to Robot" class="nav_element"></div>
        </li>
    <?cs /if ?>
    
    <?cs if:CGI.Login ?>
      <li id="username"><div class="nav_element">Robot Menu &raquo;</div></li>
    <?cs else ?>
      <li><a href="<?cs var:CGI.BaseURI ?>login/signin0.py?q=1" class=tablink>Login</a></li>
    <?cs /if ?>
  </ul>

	<ul id="all_nav" class="ui-tabs-nav ui-helper-reset ui-helper-clearfix ui-widget-header ui-corner-all">
	
    <?cs if:CGI.cur.device_style != "style_phone.css" ?>
	
		<li class="ui-state-active" style="border-bottom-width: 1 !important; margin: 0px 0px 2px">
        <a href="<?cs var:CGI.home_page ?>">Robot: <?cs var:CGI.Robot ?> (<?cs var:CGI.robot_type ?>)</a></li>
		<li class="" style="color: #999">
            <div class="nav_element" objtype=UsersOnlineWidget topic="/users_online" key="data"></div>  
        </li>
        <li>
            <div class="nav_element" objtype=ActiveTasks topic="/app_status" role="<?cs var:CGI.Role ?>" style="padding: 1px 0 0 0; margin-right: 30px" ></div>
        </li>
    <?cs /if ?>
	</ul>
</div>

  <?cs if:CGI.robot_type == "pr2" ?>
    <div id="home_status" objtype=HomeStatus topic="/dashboard_agg">
        The robot is not yet running its web interface service.  You can bring
        up the web interface by <a href="processes.py?reset=1">resetting</a> the robot.
        (If there are processes running on the robot, you will be asked to verify 
	that it is ok to stop these processes.)
    </div>
  <?cs /if ?>

<div class="loading" id="bridge_loading">
  Connecting to robot computers...
</div>

<div id="bridge_offline">
  Unable to connect to robot computers
</div>

<div id="tabs" class="ui-tabs ui-widget">
	<ul class="ui-tabs-nav ui-helper-reset ui-helper-clearfix ui-widget-header ui-corner-all">
		<li class="ui-state-default ui-corner-top <?cs if:CGI.cur.tabs.0 == "apps" ?>ui-tabs-selected ui-state-active<?cs /if ?>"><a href="<?cs var:CGI.BaseURI?>webui/apps.py">Apps</a></li>
		<li class="ui-state-default ui-corner-top <?cs if:CGI.cur.tabs.0 == "status" ?>ui-tabs-selected ui-state-active<?cs /if ?>"><a href="<?cs var:CGI.BaseURI?>webui/status.py">Diagnostics</a></li>
		<?cs if:CGI.Role=="admin"?><li class="ui-state-default ui-corner-top <?cs if:CGI.cur.tabs.0 == "admin" ?>ui-tabs-selected ui-state-active<?cs /if ?>"><a href="<?cs var:CGI.BaseURI?>webui/admin.py">Admin</a></li><?cs /if ?>
	</ul>
</div>

<script>
  jQuery(document).ready(function() {
    jQuery('#nav_element_circuits, #nav_element_motors').click(function() {
      document.location.href = "<?cs var:CGI.BaseURI ?>webui/powerboard.py";
    });
    jQuery('#nav_element_status').click(function() {
      document.location.href = "<?cs var:CGI.BaseURI ?>webui/monitor.py";    
    });
    jQuery('#username').menu();
    jQuery('#username .menu_pane').css({width: "inherit"});
    jQuery('#username .menu_pane').html(
      <?cs if:CGI.robot_type == "pr2" ?>
        "<a href='<?cs var:CGI.BaseURI ?>login/signin.py?Action.Logout=1' class=tablink>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;Logout</a>"
      <?cs else ?>
        "<a style='border-bottom: 1px solid #fff; margin-bottom: 0.5em;' href='<?cs var:CGI.BaseURI ?>login/signin.py?Action.Logout=1' class=tablink>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&laquo;&nbsp;Back&nbsp;to&nbsp;Lobby</a>"
      <?cs /if ?>
      <?cs if:Cookie.inactive != 1 ?>
        + "<a href='<?cs var:CGI.BaseURI ?>webui/processes.py?reset=1' class=tablink onclick='dialogMessage(\"System Status\", \"<br /><span id=reset_loading>Preparing to reset...</span>\"); return true;'>Reset&nbsp;Robot</a>"
      <?cs /if ?>
      <?cs if:CGI.robot_type == "pr2" ?>
        + "<a href='<?cs var:CGI.BaseURI ?>active/active.py?request=<?cs var:url_escape(CGI.RequestURI) ?>'>Change&nbsp;my&nbsp;Status</a>"
        + "<a href='<?cs var:CGI.BaseURI ?>login/changePassword.py?request=<?cs var:url_escape(CGI.RequestURI) ?>'>Change&nbsp;my&nbsp;Password</a>"
      <?cs /if ?>
    );
    jQuery('#username').click(function() { jQuery('#username .menu_pane').show() });

    jQuery('#nav_element_bridge').menu();
    jQuery('#nav_element_bridge').find(".menu_pane").html("<b>Robot Operating System</b><br />Unable to connect to robot operating system.")
  });
</script>

<div id="content"> <!-- to be closed in rosfooter -->
