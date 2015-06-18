<?cs include:"ros_app_include_header.cs" ?>
<?cs include:"status_header.cs" ?>

<script type="text/javascript" src="<?cs var:CGI.BaseURI?>webui/jslib/jquery.treeview.js"></script> 
<link rel="stylesheet" href="<?cs var:CGI.BaseURI?>webui/templates/css/jquery.treeview.css" type="text/css" media="all" />

<h3>Status:</h3>

<div style="float: right; text-align: right">
  Color codes
  <div style="color:#4f4">OK</div>
  <div style="color:#ff4">WARN</div>
  <div style="color:#f44">ERROR</div>
</div>

<a href="monitor.py">All Systems</a>
<br />
<ul>
  <li><a href="monitor.py?system=<?cs var:Query.system ?>"><?cs var:Query.system ?></a></li>
</ul>

<div style="margin-left: 60px;" objtype="RobotDeviceMonitor" topic="/diagnostics_agg:filter:/<?cs var:Query.system ?>/<?cs var:Query.device ?>">
  <p id="loading">Loading <?cs var:Query.device ?> status...</p>
</div>

<?cs include:"ros_app_include_footer.cs" ?>
