<?cs include:"ros_app_include_header.cs" ?>
<?cs include:"status_header.cs" ?>

<h3>Node: <?cs var:CGI.cur.node ?></h3>

<?cs if:CGI.cur.error ?>
  <span class="error_message">Uh oh!</span>  <?cs var:CGI.cur.error ?>
<?cs else ?>
  <h4>Publications</h4>
  <?cs each:_topic=CGI.cur.publications ?>
    <li><a href="topic.py?topic=<?cs var:_topic ?>"><?cs var:_topic ?></a></li>  
  <?cs /each ?>

  <h4>Subscriptions</h4>
  <?cs each:_topic=CGI.cur.subscriptions ?>
    <li><a href="topic.py?topic=<?cs var:_topic ?>"><?cs var:_topic ?></a></li>  
  <?cs /each ?>

  <h4>Services</h4>
  <?cs each:_service=CGI.cur.services ?>
    <li><a href="service.py?service=<?cs var:_service ?>"><?cs var:_service ?></a></li>  
  <?cs /each ?>
<?cs /if ?>

<p>
  <a href="nodes.py">&laquo; All nodes</a>
</p>

<?cs include:"ros_app_include_footer.cs" ?>
