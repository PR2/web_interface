<?cs include:"ros_app_include_header.cs" ?>
<?cs include:"status_header.cs" ?>

 
 
<script type="text/javascript" src="<?cs var:CGI.BaseURI?>webui/jslib/jquery.treeview.js"></script> 
<script type="text/javascript" src="<?cs var:CGI.BaseURI?>webui/jslib/jquery.editinplace.js"></script> 
<link rel="stylesheet" href="<?cs var:CGI.BaseURI?>webui/templates/css/jquery.treeview.css" type="text/css" media="all" />

<style type="text/css">
  .treeview .hover {
    color: #26B3F7;
  }
  
  #parameters_tree {
    display: none;
  }
</style>

<h3>Parameters</h3>

<?cs def:tree(params, prefix) ?>
  <?cs each:_node=params?>
     <?cs set:new_prefix = prefix + '/' + name(_node) ?>
     <?cs if:subcount(_node) ?>
        <li><span class="folder"><?cs name:_node?></span>
        <ul><?cs call:tree(_node, new_prefix) ?></ul></li>
     <?cs else ?>
        <li><span class="file"><?cs name:_node?> = <span id="<?cs var:new_prefix?>" class="value"><?cs var:_node?></span></span></li>
     <?cs /if ?>
  <?cs /each?>
<?cs /def ?>

<ul id="parameters_tree">
<?cs call:tree(CGI.cur.params, '')?>
</ul>

<p id="loading">
  Loading...
</p>

<?cs if:Cookie.inactive != 1 ?>
    <p>
      <form action="">
      <input type="hidden" name="Action.New" value="1" />
      <table>
        <tr>
          <td>Parameter: </td>
          <td><input name="key" />&nbsp;&nbsp;</td>
          <td>Value: </td>
          <td><input name="value" />&nbsp;&nbsp;</td>
          <td><input type="submit" value="Set new parameter"></td>
        </tr>
      </table>
      </form>
    </p>
<?cs /if ?>

<p class="app_description">
  This is a list of parameters from the master.  Refresh the page to update.
  <br />
<?cs if:Cookie.inactive != 1 ?>
  Parameter values are editable.  Click a parameter value to edit or delete it.
<?cs /if ?>
</p>

<script>
  jQuery(document).ready(function() {
    jQuery("#parameters_tree").treeview({collapsed: true});
    jQuery("#example").treeview();

<?cs if:Cookie.inactive != 1 ?>
      jQuery("#parameters_tree .file .value").editInPlace({
      url: "params.py?Action.Edit=1",
      bg_over: "#666",
      on_blur: "cancel",
      params: "test=foo",
      value_required: true,
      method: "GET",
      show_buttons: true
    });
<?cs /if ?>
    jQuery("#loading").hide();
    jQuery("#parameters_tree").show();
  });
</script>

<?cs include:"ros_app_include_footer.cs" ?>
