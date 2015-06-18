<html>
<head>
  <title>Activate</title>
  <!-- Changed by: Josh Tyler, 27-Oct-2009 -->

  <script language="JavaScript">
    function handleOnload() {
      var top_parent = window;
      while (top_parent.parent && (top_parent.parent != top_parent)) {
        top_parent = top_parent.parent;
      }
      if (top_parent != window) {
        top_parent.location.href="<?cs var:js_escape(CGI.RequestURI) ?>";
      }
    }
  </script>

  <link rel="stylesheet" href="<?cs var:CGI.BaseURI ?>webui/templates/css/jquery-ui-1.7.2.custom.css" type="text/css" media="all" />
  <link rel="stylesheet" type="text/css" href="<?cs var:CGI.BaseURI?>/webui/templates/css/style_desktop.css">
  <?cs if:CGI.cur.device_style ?>
    <link rel="stylesheet" type="text/css" href="<?cs var:CGI.BaseURI?>/webui/templates/css/<?cs var:CGI.cur.device_style?>">
  <?cs /if ?>
</head>
<body>

<div id="login" class="">
  <div class="ui-state-highlight ui-corner-all">
    <form action="" method=post>
      <input type=hidden name=request value="<?cs var:url_escape(CGI.cur.request)?>">
      <p class="dialog_title">Set user status for <span style="color: #FFB73D"><?cs var: CGI.Login ?></span> on <?cs var: CGI.ServerName ?> </p>
      <p>
        Currently piloting <?cs var: CGI.ServerName ?>:

        <?cs if:CGI.active_user ?>
          <strong class="user"><?cs var:CGI.active_user ?></strong>
        <?cs else ?>
          <strong class="nobody">Nobody</strong>
        <?cs /if ?>
      </p>
      <p style="text-align: center">
        <input type=hidden name="uri" value="<?cs var:Query.url ?>">
        <input type=submit value="Make Me the Pilot" name="Action.MakeActive">
        <input type=submit value="Make Me a Passenger" name="Action.MakeInactive">
      </p>
    </form>
  </div>
</div>

<script type="text/javascript" src="<?cs var:CGI.BaseURI?>/webui/jslib/jquery-1.3.2.min.js"></script> 
<script type="text/javascript">
  jQuery(document).ready(function() {
    handleOnload();
  });
</script>

</body>
</html>
