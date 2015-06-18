<table class=head_buttons width=100%>
<tr>
<td class=pagetitle><?cs var:CGI.ServerName?>: <?cs var:CGI.cur.pageName ?></td>
<td class=head_buttons width=1% onclick="javascript:location.href='apps.py'">Apps</a></td>
<td class=head_buttons width=1% onclick="javascript:location.href='move.py'">Move</td>
<td class=head_buttons width=1% onclick="javascript:location.href='topics.py'">Topics</a></td>
<td class=head_buttons width=1% onclick="javascript:location.href='nodes.py'">Nodes</td>
<td class=head_buttons width=1% onclick="javascript:location.href='powerboard.py'">Power</td>
<td class=head_buttons width=1% onclick="javascript:location.href='status.py'">Status</a></td>
<td class=head_buttons width=1% onclick="javascript:location.href='admin.py'">Admin</a></td>

<?cs if:CGI.Login=="hassan"?><td width=1%><a href="<?cs var:CGI.BaseURI ?>/webui/tables.py/" class=tablink>Tables</a></td><?cs /if ?>

<?cs if:CGI.Login ?>
<td align=right><a href="<?cs var:CGI.BaseURI ?>/login/signin.py?signout=1" class=tablink>Logout</a>(<?cs var:CGI.Login ?>)</td>
<?cs else ?>
<td align=right><a href="<?cs var:CGI.BaseURI ?>/login/signin0.py?q=1" class=tablink>Login</a></td>
<?cs /if ?>
</tr>
</table>

<table style="border: 0px; width: 150px; float: right">
<tr><td>

<div objtype=BatteryGauge topic="/battery_state" key=energy_remaining key2=energy_capacity width=150 height=150></div>

</td></tr>
<tr><td>
<table style="border: 2px solid #fff; width: 100%">
<tr><td colspan=2 style="background-color: #aaa; color: #000">Powerboard:</td></tr>
<?cs include:"powerboard_status2.cs" ?>
</td></tr>
</table>
</tr>
<tr><td>
<table style="border: 2px solid #fff; width: 100%;">
<tr><td style="background-color: #aaa; color: #000">Users:</td></tr>
<tr>
<td objtype="ListWidget" topic="/users" key="users">
  __item__<br>
</td>
</tr>
</table>
</td></tr>
<tr><td>
</td></tr>
</table>

