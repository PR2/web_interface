<html>
<head>
<title>Sign-In</title>

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
  document.login_form.username.focus();
}
//-->
</script>

<link rel="stylesheet" href="<?cs var:CGI.BaseURI?>/webui/templates/css/jquery-ui-1.7.2.custom.css" type="text/css" media="all" />
<link rel="stylesheet" type="text/css" href="<?cs var:CGI.BaseURI?>/webui/templates/css/style_desktop.css">
<?cs if:CGI.cur.device_style ?>
  <link rel="stylesheet" type="text/css" href="<?cs var:CGI.BaseURI?>/webui/templates/css/<?cs var:CGI.cur.device_style?>">
<?cs /if ?>

</head>
<body onload="handleOnload()">



<input type=hidden name=request value="<?cs var:CGI.cur.request?>">

<?cs if:Query.reason == "timeout"?>
  <b>Your login has timed-out due to inactivity, please sign in
  again</b><p>
<?cs /if ?>


<div id="login" class="">
  <div class="ui-state-highlight ui-corner-all login_container">
    <p class="dialog_title">Login to <?cs var: CGI.ServerName ?></p>

    <form name="login_form" action="<?cs var:CGI.BaseURI?>login/signin0.py" method=post>
    <table cellspacing=0 cellpadding=2 style="margin: auto">
    <tr align=center><td colspan=2><?cs if:Query.err?><font color=red><?cs var:html_escape(Query.err)?><?cs /if ?></td></tr>
    <tr>
      <td align=right>Username:</td>
      <td><input name=username type=text size=20 value="<?cs var:url_escape(CGI.username) ?>"></td>
    </tr>
    <tr>
      <td align=right>Password:</td>
      <td><input name=password type=password size=20></td>
    </tr>
    <tr><td><Td colspan=1>
    <!--<font size=-2><input type=checkbox value="1" name="persist" <?cs if:Cookie.MB_persist==1?>CHECKED<?cs /if ?>> Don't ask for my password for 2 weeks.</font>-->
    </td></tr>
    <tr>
      <td colspan=2 align=center>
        <br>
        <input type=submit value="Login" name="Action.Login">
      </td>
    </tr>


    <tr><td align=center colspan=2>
    <?cs if:0 ?>
      <a href="<?cs var:CGI.BaseURI?>/login/forgotpw.py">Forgot Password</a>
    <?cs /if ?>
    </td></tr>

    </table>
    </form>
  </div>
</div>

</body>
</html>
