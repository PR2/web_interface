<html>
  <head>
    <?cs include:"includes.cs" ?>
</head>

<body>

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
</table>

<div class=pagetitle><?cs var:CGI.ServerName?>: <?cs var:CGI.cur.pageName ?></div>


<table>
<tr valign=top>
<td valign=top>
<h3>Published topics:</h3>
<div objtype="ListWidget" topic="/topics" key="pubtopics">
  <div><a href="topic.py?topic=__item__">__item__</a>
</div>
</td>
<td>
<div objtype=PowerboardGraph2Widget topic="/diagnostics/Power board 0" width=300></div>
</td>
</tr>
</table>


</body>
</html>
