    
    
    <?cs if:CGI.taskid ?>
        <div class="app_info" style="margin-top: 40px;" objtype="LaunchAppFromInfoPageWidget" taskid="<?cs var:CGI.taskid ?>" app_name="<?cs var:CGI.cur.app.name ?>">
          <p class="start_app">
            <a href="">&raquo; Start this application</a>
          </p>
          <p class="go_to_app">
            <p class="stop_app">
              <a href="">&raquo; Stop this application</a>
            </p>
          </p>
          <p>
            
          </p>
          <div class="dialog" style="display: none">
            The <?cs var:CGI.cur.app.name ?> app is now running.
          </div>
        </div>

      <p class="divclear">
        &laquo; 
        <a href="<?cs var:CGI.BaseURI?>webui/apps.py">All Apps</a> | 
      	<a href="<?cs var:CGI.BaseURI?>webui/appinfo.py?taskid=<?cs var:CGI.taskid ?>">Application Info Page</a>
      </p>
    <?cs /if ?>
    <div id=ErrorDiv></div>
    <?cs include:"rosfooter.cs"?>
  </body>
</html>
