<?cs include:"ros_app_include_header.cs" ?>

<div class="app_info" objtype="LaunchAppFromInfoPageWidget" taskid="<?cs var:CGI.cur.app.taskid ?>" app_name="<?cs var:CGI.cur.app.name ?>">
  <table>
    <tr>
      <td taskid="<?cs var:CGI.cur.app.taskid ?>" >
        <div class="app_thumb">
          <div class="status" style="position: absolute; top: 0; width: 100px; background: black; z-index: 3; font-size: 80%; -moz-opacity:0.8; text-align: center;"></div>
          <div class="app_image" style="background-image: url(<?cs var:CGI.BaseURI?>/app/<?cs var:CGI.cur.app.taskid ?>/<?cs var:CGI.cur.app.icon?>)"></div>
          <div class=app_name><?cs var:CGI.cur.app.name ?></div>
          <div id="favorite_image" title="Add/remove this app from Favorites" <?cs if:CGI.cur.app.favorite ?>class="selected"<?cs /if ?>></div>
          <input type="checkbox" title="Add this app to Favorites" id="favorite_checkbox" name="favorite_checkbox" <?cs if:CGI.cur.app.favorite ?>checked<?cs /if ?>/>
          <input style="display: none" class="button" type="button" value="" title="Start/stop this application" />
        </div>
      </td>
      <td class="app_title">
        <h2><?cs var:CGI.cur.app.name ?></h2>
      </td>
    </tr>
  </table>
  <p class="divclear app_description">
    <?cs var:CGI.cur.app.description ?>
  </p>
  <p class="start_app">
    <a href="">&raquo; Start this application</a>
  </p>
  <p class="go_to_app">
    <a href="<?cs var:CGI.BaseURI ?>/app/<?cs var:CGI.cur.app.taskid ?>/">&raquo; Go to application</a>
    <p class="stop_app">
      <a href="">&raquo; Stop this application</a>
    </p>
  </p>
  <p>
    &laquo; <a href="apps.py">All Apps</a>
  </p>
  <div class="dialog" style="display: none">
    The <?cs var:CGI.cur.app.name ?> app is now running.
  </div>
</div>

<script type="text/javascript">
  jQuery(document).ready(function(){    
    jQuery(".app_thumb").app_thumb({path: "<?cs var:CGI.BaseURI?>"});
  });
</script>

<?cs include:"ros_app_include_footer.cs"?>

