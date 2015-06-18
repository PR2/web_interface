<title>Web User Interface: <?cs var:CGI.Robot?></title>

<script type="text/javascript" src="<?cs var:CGI.BaseURI?>webui/jslib/jquery-1.3.2.min.js"></script> 
<script type="text/javascript" src="<?cs var:CGI.BaseURI?>webui/jslib/jquery-ui-1.7.2.custom.min.js"></script> 
<script type="text/javascript" src="<?cs var:CGI.BaseURI?>webui/jslib/jquery.cookie.js"></script> 
<script type="text/javascript" src="<?cs var:CGI.BaseURI?>webui/jslib/prototype.js"></script>
<script type="text/javascript" src="<?cs var:CGI.BaseURI?>webui/jslib/ros.js"></script> 
<script type="text/javascript" src="<?cs var:CGI.BaseURI?>webui/jslib/ros_toolbar.js"></script> 
<script type="text/javascript" src="<?cs var:CGI.BaseURI?>webui/jslib/pr2_graph.js"></script> 
<script type="text/javascript" src="<?cs var:CGI.BaseURI?>webui/jslib/pr2_pb.js"></script> 
<script type="text/javascript" src="<?cs var:CGI.BaseURI?>webui/jslib/pr2_widgets.js"></script> 
<link rel="stylesheet" href="<?cs var:CGI.BaseURI?>webui/templates/css/jquery-ui-1.7.2.custom.css" type="text/css" media="all" />
<link rel="stylesheet" type="text/css" href="<?cs var:CGI.BaseURI?>webui/templates/css/style_desktop.css">
<?cs if:CGI.cur.device_style ?>
  <link rel="stylesheet" type="text/css" href="<?cs var:CGI.BaseURI?>webui/templates/css/<?cs var:CGI.cur.device_style?>">
<?cs /if ?>

<script type="text/javascript">
/* <![CDATA[ */
  HDF = {};
  HDF.CGI = {};
  HDF.CGI.BaseURI = '<?cs var:js_escape(CGI.BaseURI)?>';
  HDF.CGI.username = '<?cs var:js_escape(CGI.username)?>';
  HDF.CGI.Login = '<?cs var:js_escape(CGI.Login)?>';
  HDF.CGI.BaseURI = '<?cs var:js_escape(CGI.BaseURI)?>';
  HDF.CGI.ActiveUser = '<?cs var:js_escape(CGI.active_user)?>';
  HDF.CGI.ScreenType = '<?cs var:CGI.cur.device_style?>';
  HDF.CGI.RemoteAddress = '<?cs var:CGI.RemoteAddress ?>';
  HDF.CGI.RequestURI = '<?cs var:CGI.RequestURI ?>';
  HDF.CGI.RedirectURI = '<?cs var:Query.request?>';
  HDF.CGI.Referrer = '<?cs var:HTTP.Referer?>';
  
  var publish_presence = function() {
    if(HDF.CGI.Login) {
      gPump.publish("/presence", "std_msgs/String", [HDF.CGI.Login]);
    }
    setTimeout('publish_presence()', 10000);
  }

  jQuery(document).ready(function(){
    ros_handleOnLoad('<?cs var:CGI.ros_bridge_uri ?>');
    setTimeout('publish_presence()', 100);
    <?cs if:CGI.cur.device_style != "style_phone.css" ?>
        time_request();
    <?cs /if ?>
  });
/* ]]> */
</script>
