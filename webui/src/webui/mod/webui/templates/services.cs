<?cs include:"ros_app_include_header.cs" ?>
<?cs include:"status_header.cs" ?>

<h3>Services:</h3>
<ul>
<?cs each:_service=CGI.cur.services ?>
  <li><a href="service.py?service=<?cs var:_service ?>"><?cs var:_service ?></a>
<?cs /each ?>
</ul>

<br>

<?cs include:"ros_app_include_footer.cs" ?>

