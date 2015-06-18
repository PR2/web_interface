<html>
<head>
<title>Mail - Forgot Password</title>
</head>
<body>

<table width=100% height=70%>
<form action="forgotpw.py" method=post>
<tr><td align=center valign=middle>
<table width=150 cellspacing=0 cellpadding=2 style="border:1px solid #777799;" >
<tr><td colspan=2 style="color:white;background:#777799;" align=center>
Forgot Password
</td></tr>
<tr><td align=right>Login:</td>
<td><b><?cs var:CGI.Login ?></b></td></tr>

<input type=hidden name="login" value="<?cs var:CGI.Login ?>">

<tr><td nowrap align=right>New Password:</td>
<td><input name=pw1 type=password size=20></tr>

<tr><td nowrap align=right>Confirm Password:</td>
<td><input name=pw2 type=password size=20></tr>

<tr><Td colspan=2 align=center>
<input type=submit value="Set Password" name="Action.SetPassword">
</td></tr>
</form>
</table>

</body>
</html>
