<?cs include:"ros_app_include_header.cs" ?>

<?cs include:"admin_header.cs" ?>

<?cs each:err=CGI.cur.error_message ?>
  <div class="error_message">Error: <?cs var:err ?></div>
<?cs /each ?>

<form action="" method="post" autocomplete="off">
<table class="user_table">
  <tr><th colspan="2">Create a new user:</th></tr>
  <tr>
    <td>Name:</td>
    <td><input type="text" name="username" value="<?cs var:Query.username ?>"></td>
  </tr>
  <tr>
    <td>Role:</td>
    <td>
      <select name="role">
        <option value="">basic</option>
        <option value="admin" <?cs if:Query.role == "admin" ?>selected<?cs /if ?> >admin</option>
      </select>
    </td>
  </tr>
  <tr>
    <td>Password:</td>
    <td><input type="password" name="password"></td>
  </tr>
  <tr>
    <td></td>
    <td><input type="submit" name="Action.CreateUser" value="Save user" /></td>
  </tr>
</table>
</form>

<?cs include:"ros_app_include_footer.cs" ?>

