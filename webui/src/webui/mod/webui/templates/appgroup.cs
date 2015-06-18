<?cs include:"ros_app_include_header.cs" ?>
    <script type="text/javascript">
      jQuery(document).ready(function(){
        jQuery(".app_thumb").app_thumb({path: "<?cs var:CGI.BaseURI?>"});
      });
    </script>
    
    <h3>App Category: <?cs var:CGI.cur.app_group.category ?></h3>
    
    <div>
      <div style="position: relative; top: 0; z-index: 0">
      <?cs each:_app=CGI.cur.app_group.apps ?>
        <div class="app_info" objtype="LaunchAppFromInfoPageWidget" taskid="<?cs var:_app.taskid ?>">
          <div class="app_thumb">
            <div class="status" style="position: absolute; top: 0; width: 100px; background: black; z-index: 3; font-size: 80%; -moz-opacity:0.8; text-align: center;"></div>
            <div class="app_image" taskid="<?cs var:_app.taskid ?>" style="background-image: url(<?cs var:CGI.BaseURI?>/app/<?cs var:_app.taskid ?>/<?cs var:_app.icon?>)"></div>
            <div class=app_name><?cs var:_app.name ?></div>
            <input style="display: none" class="button" type="button" value="..." />
          </div>
        </div>
      <?cs /each ?>
      <div class="divclear"></div>
      </div>
    </div>

    <p>
    <form method="post" action="">
      <input type="hidden" name="Action.AddApp" value="1" />
      Add an app to this category:
      <select name="taskid">
        <?cs each:_app=CGI.cur.available_apps ?>
          <option value="<?cs var:_app.taskid ?>"><?cs var:_app.name ?></option>
        <?cs /each ?>
      </select>
      <input type="submit" value="Add">
    </form>
    </p>
<?cs include:"ros_app_include_footer.cs" ?>

