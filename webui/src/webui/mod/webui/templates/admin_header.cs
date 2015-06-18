
<div id="sub_tabs" class="ui-tabs ui-widget">
  <ul class="ui-tabs-nav ui-helper-reset ui-helper-clearfix">
    <li class="ui-state-default <?cs if:CGI.cur.tabs.1 == "users" ?>ui-state-active<?cs /if ?>"><a href="<?cs var:CGI.BaseURI?>webui/users.py">Users</a></li>
    <li class="ui-state-default <?cs if:CGI.cur.tabs.1 == "maintenance" ?>ui-state-active<?cs /if ?>"><a href="<?cs var:CGI.BaseURI?>webui/maintenance.py">Maintenance</a></li>
    <li class="ui-state-default <?cs if:CGI.cur.tabs.1 == "tables" ?>ui-state-active<?cs /if ?>"><a href="<?cs var:CGI.BaseURI?>webui/tables.py">Tables</a></li>
  </ul>
</div>


