<?cs include:"ros_app_include_header.cs" ?>
<?cs include:"status_header.cs" ?>

<h3>Topic: <?cs var:CGI.cur.topic?></h3>

<?cs if:CGI.cur.error ?>
  <span class="error_message">Uh oh!</span>  <?cs var:CGI.cur.error ?>
<?cs else ?>
  <h4>Message History</h4>
  <div objtype=SliderTextWidget topic="<?cs var:CGI.cur.topic ?>"></div><br>

  <h4>Publishers</h4>
  <?cs each:_node=CGI.cur.publishers ?>
    <li><a href="node.py?node=<?cs var:_node ?>"><?cs var:_node ?></a></li>  
  <?cs /each ?>

  <h4>Subscribers</h4>
  <?cs each:_node=CGI.cur.subscribers ?>
    <li><a href="node.py?node=<?cs var:_node ?>"><?cs var:_node ?></a></li>  
  <?cs /each ?>
<?cs /if ?>

<p>
  <a href="topics.py">&laquo; All topics</a>
</p>

<?cs include:"ros_app_include_footer.cs" ?>
