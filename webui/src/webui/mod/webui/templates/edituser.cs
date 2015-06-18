<?cs include:"ros_app_include_header.cs" ?>
<?cs include:"admin_header.cs" ?>

<?cs each:err=CGI.cur.error_message ?>
  <div class="error_message">Error: <?cs var:err ?></div>
<?cs /each ?>

<form action="" method="post" autocomplete="off">
<input type="hidden" name="uid" value="<?cs var:CGI.cur.user.uid ?>" />
<table>
  <tr><th colspan="2">Editing user: <?cs var:CGI.cur.user.username ?></th></tr>
  <tr>
    <td>Role:</td>
    <td>
      <select name="role">
        <option value="">basic</option>
        <option value="admin" <?cs if:CGI.cur.user.role == "admin" ?>selected<?cs /if ?> >admin</option>
      </select>
    </td>
  </tr>
  <tr>
    <td></td>
    <td><input type="submit" name="Action.SaveUser" value="Save user" /></td>
  </tr>
</table>
</form>

<?cs include:"ros_app_include_footer.cs" ?>

