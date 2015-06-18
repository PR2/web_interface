<?cs include:"ros_app_include_header.cs" ?>
<?cs include:"status_header.cs" ?>

<script type="text/javascript" src="<?cs var:CGI.BaseURI?>webui/jslib/jquery.treeview.js"></script> 
<link rel="stylesheet" href="<?cs var:CGI.BaseURI?>webui/templates/css/jquery.treeview.css" type="text/css" media="all" />

<h3>Status</h3>

<div style="float: right; text-align: right">
  Color codes
  <div style="color:#4f4">OK</div>
  <div style="color:#ff4">WARN</div>
  <div style="color:#f44">ERROR</div>
</div>

<a href="monitor.py">All Systems</a>

<div objtype="RobotMonitor" topic="/diagnostics_agg:<?cs if:Query.system ?>filter:/<?cs var:Query.system ?><?cs else ?>Summary<?cs /if ?>">
  <p id="loading">Loading <?cs alt:Query.system ?><?cs /alt ?> status...</p>
</div>


<?cs include:"ros_app_include_footer.cs" ?>
