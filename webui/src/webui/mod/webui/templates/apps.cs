<?cs include:"ros_app_include_header.cs"?>

<div id="category_list">
<ul>
<?cs each:_category=CGI.cur.categories ?>
  <li><a href="" id="<?cs var:_category ?>"><?cs var:_category ?> (<?cs var:subcount(_category.apps) ?>)</a></li>
<?cs /each ?>
</ul>
</div>

<div id="all_apps">
<div style="position: relative; top: 0; z-index: 0">
<?cs each:_category=CGI.cur.categories ?>
  <div class="category_group" id="cat_<?cs var:_category ?>" style="display: none">
  <?cs if:_category=="Favorites" ?>
    <?cs if:subcount(_category.apps) == 0 ?>
      <p style="margin-left: 20px;">You have no favorite apps.  Add an app to your favorites by selecting the star in the top right corner of the app icon.</p>
    <?cs /if ?>
  <?cs /if ?>
  <?cs each:_taskid=_category.apps ?>
  <?cs with:_app=CGI.cur.available_apps[_taskid] ?>
  <div class="app_info" objtype="LaunchAppFromInfoPageWidget" taskid="<?cs var:_app.taskid ?>">
    <div class="app_thumb">
      <div class="status" style="position: absolute; top: 0; width: 100px; background: black; z-index: 3; font-size: 80%; -moz-opacity:0.8; text-align: center;"></div>
      <div class="app_image" name="app/<?cs var:_app.package ?>" style="background-image: url(<?cs var:CGI.BaseURI?>/app/<?cs var:_app.taskid ?>/<?cs var:_app.icon?>)"></div>
      <div class=app_name><?cs var:_app.name ?></div>
      <div id="favorite_image" title="Add/remove this app from Favorites" <?cs if:_app.favorite ?>class="selected"<?cs /if ?>></div>
      <input type="checkbox" title="Add this app to Favorites" id="favorite_checkbox" name="favorite_checkbox" <?cs if:_app.favorite ?>checked<?cs /if ?>/>
      <input style="display: none" class="button" type="button" value="" title="Start/stop this application" />
    </div>
  </div>
  <?cs /with ?>
  <?cs /each ?>
  </div>
<?cs /each ?>
<div class="divclear"></div>
</div>
</div>

<div id=ErrorDiv></div>


<script type="text/javascript">
  jQuery(".app_thumb").app_thumb({path: "<?cs var:CGI.BaseURI?>"});
  
  jQuery("#category_list li a").click(function() {
    var a = jQuery(this);
    // for Favorites, reload the page (so we can get the latest apps), for
    // all others, just show/hide the appropriate category
    if (a.attr("id") != "Favorites") {    
      // hide all categories and show the one that was clicked
      jQuery(".category_group").hide();
      jQuery("#category_list li a").removeClass("selected");

      a.addClass("selected");
      var cat = jQuery("#cat_" + a.attr("id"));
      cat.show();
      return false;
    }
  });
  
  jQuery(document).ready(function() {
    jQuery("#cat_Favorites").show();
    jQuery("#Favorites").addClass("selected");
  });
</script>

<style>

</style>

<?cs include:"ros_app_include_footer.cs"?>
