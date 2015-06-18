<html>
<head>
<title>Change Password</title>

<link rel="stylesheet" href="<?cs var:CGI.BaseURI ?>webui/templates/css/jquery-ui-1.7.2.custom.css" type="text/css" media="all" />
<link rel="stylesheet" type="text/css" href="<?cs var:CGI.BaseURI?>/webui/templates/css/style_desktop.css">
<?cs if:CGI.cur.device_style ?>
  <link rel="stylesheet" type="text/css" href="<?cs var:CGI.BaseURI?>/webui/templates/css/<?cs var:CGI.cur.device_style?>">
<?cs /if ?>

</head>
<body>


<div id="login" class="">
  <div class="ui-state-highlight ui-corner-all" style="padding: 0pt 0.7em; margin: 200px auto; width: 400px; background: gray">
    <p class="dialog_title">Change password for <span style="color: #FFB73D"><?cs var: CGI.Login ?></span> on <?cs var: CGI.ServerName ?></p>

    <form action="changePassword.py" method="post" autocomplete="off">
    <input type=hidden name="request" value="<?cs var:CGI.cur.request?>">
    <input type=hidden name="login" value="<?cs var:CGI.Login?>">

    <table cellspacing=0 cellpadding=2 style="margin: auto">
    <tr align=center><td colspan=2><?cs if:Query.err?><font color=red><?cs var:html_escape(Query.err)?><?cs /if ?></td></tr>
    <tr><td nowrap align=right>Old Password:</td>
    <td><input name=pw0 type=password size=20></tr>
    <tr><td nowrap align=right>New Password:</td>
    <td><input name=pw1 type=password size=20></tr>
    <tr><td nowrap align=right>Confirm Password:</td>
    <td><input name=pw2 type=password size=20></tr>
    <tr><td><Td colspan=1>
    </td></tr>
    <tr><Td colspan=2 align=center>
    <input type=hidden name="Action.changePassword" value="1">
    <input type=submit value="Change Password" name="Action.changePassword">
    </td></tr>


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
