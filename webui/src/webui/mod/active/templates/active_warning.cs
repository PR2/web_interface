<html>
<head>
<title>Activate</title>
<!-- Changed by: Josh Tyler, 27-Oct-2009 -->

<script language="JavaScript">
<!--
function handleOnload() {
  var top_parent = window;
  while (top_parent.parent && (top_parent.parent != top_parent)) {
    top_parent = top_parent.parent;
  }
  if (top_parent != window) {
    top_parent.location.href="<?cs var:js_escape(CGI.RequestURI) ?>";
  }
}
//-->
</script>

<link rel="stylesheet" href="<?cs var:CGI.BaseURI ?>webui/templates/css/jquery-ui-1.7.2.custom.css" type="text/css" media="all" />
<link rel="stylesheet" type="text/css" href="<?cs var:CGI.BaseURI?>/webui/templates/css/style_desktop.css">
<?cs if:CGI.cur.device_style ?>
  <link rel="stylesheet" type="text/css" href="<?cs var:CGI.BaseURI?>/webui/templates/css/<?cs var:CGI.cur.device_style?>">
<?cs /if ?>

</head>
<body onload="handleOnload()">

<div id="login" class="">
  <div class="ui-state-highlight ui-corner-all">
    <form action="<?cs var:CGI.BaseURI ?>active/active.py" method=post>
      <input type=hidden name=request value="<?cs var:Query.request ?>">
      <p class="dialog_title">Continue Login to <?cs var: CGI.ServerName ?></p>
      <table>
        <tr>
          <td class="ui-widget" style="padding-right: 20px;">
			      <div style="padding: 0; height: 16px; width: 16px; margin: 0; background-color: #FFC73D;" class="ui-state-error ui-corner-all"> 
				      <span style="float: left; margin-right: 0.3em;" class="ui-icon ui-icon-alert"/>
			      </div>
          </td>
          <td>
            <p>
              Are you sure you want to continue?
              <strong class="user"><?cs var: CGI.active_user ?></strong> will immediately
              be changed to a passenger user.

            </p>
          </td>
        </tr>
      </table>
      <p style="text-align: center">
        <input type=hidden name="uri" value="<?cs var:Query.url ?>"/>
        <input type=hidden name="override" value="1"/>

        <input type=submit value="Yes, continue" name="Action.MakeActive"/>
        <input type=submit value="No, go back" name="Action.Display"/>
      </p>
      <p class="note">
        <input type=checkbox name="dismiss_active_warning" value="1"/>
        don't show this message again
      </p>
    </form>
  </div>
</div>

</body>
</html>

