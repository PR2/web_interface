
<div id="sub_tabs" class="ui-tabs ui-widget">
  <ul class="ui-tabs-nav ui-helper-reset ui-helper-clearfix">
    <li class="ui-state-default <?cs if:CGI.cur.tabs.1 == "log" ?>ui-state-active<?cs /if ?>"><a href="<?cs var:CGI.BaseURI?>webui/status.py">Log</a></li>
    <li class="ui-state-default <?cs if:CGI.cur.tabs.1 == "topics" ?>ui-state-active<?cs /if ?>"><a href="<?cs var:CGI.BaseURI?>webui/topics.py">Topics</a></li>
    <li class="ui-state-default <?cs if:CGI.cur.tabs.1 == "nodes" ?>ui-state-active<?cs /if ?>"><a href="<?cs var:CGI.BaseURI?>webui/nodes.py">Nodes</a></li>
    <li class="ui-state-default <?cs if:CGI.cur.tabs.1 == "params" ?>ui-state-active<?cs /if ?>"><a href="<?cs var:CGI.BaseURI?>webui/params.py">Parameters</a></li>
    <li class="ui-state-default <?cs if:CGI.cur.tabs.1 == "services" ?>ui-state-active<?cs /if ?>"><a href="<?cs var:CGI.BaseURI?>webui/services.py">Services</a></li>
    <li class="ui-state-default <?cs if:CGI.cur.tabs.1 == "powerboard" ?>ui-state-active<?cs /if ?>"><a href="<?cs var:CGI.BaseURI?>webui/powerboard.py">Power</a></li>
    <li class="ui-state-default <?cs if:CGI.cur.tabs.1 == "monitor" ?>ui-state-active<?cs /if ?>"><a href="<?cs var:CGI.BaseURI?>webui/monitor.py">Monitor</a></li>
    <li class="ui-state-default <?cs if:CGI.cur.tabs.1 == "processes" ?>ui-state-active<?cs /if ?>"><a href="<?cs var:CGI.BaseURI?>webui/processes.py">Processes</a></li>
  </ul>
</div>


