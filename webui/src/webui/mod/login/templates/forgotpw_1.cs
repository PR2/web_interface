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
<td><input name=login type=text size=20 value="<?cs var:CGI.login ?>"></td></tr>
<tr><Td colspan=2 align=center>
<input type=hidden name="Action.ResetPw" value="1">
<input type=submit value="Reset Password" name="Action.ResetPw">
</td></tr>
</form>
</table>

</body>
</html>
