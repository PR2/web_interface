<?cs include:"ros_app_include_header.cs" ?>
<?cs include:"status_header.cs" ?>

<h3>Service: <?cs var:CGI.cur.service ?></h3>
<ul>
<?cs each:_arg=CGI.cur.args ?>
  <li><?cs var:_arg ?>
<?cs /each ?>
</ul>

<br>

<?cs include:"ros_app_include_footer.cs" ?>

