<?cs include:"ros_app_include_header.cs" ?>
<?cs include:"admin_header.cs" ?>

<table class="user_table">
  <tr><th>Name</th><th>Role</th><th>Actions</th></tr>
  <?cs each:user=CGI.cur.users ?>
  <tr>
    <td><?cs var:user.username ?></td>
    <td><?cs var:user.role ?></td>
    <td>
      <a href="edituser.py?username=<?cs var:user.username ?>"><span title="edit this user" class="ui-icon ui-icon-pencil"/></a>
      <a onclick="if (confirm('Are you sure you want to delete this user?')) { return true; } else { return false; }" 
      href="users.py?Action.DeleteUser=1&username=<?cs var:user.username ?>">
        <span title="delete this user" class="ui-icon ui-icon-minus"/>
      </a>
     </td>
  </tr>
  <?cs /each ?>
  <tr><th colspan="3"><a href="<?cs var:CGI.BaseURI?>webui/newuser.py">&raquo; Create a new user</a></th></tr>
</table>

<?cs include:"ros_app_include_footer.cs" ?>
