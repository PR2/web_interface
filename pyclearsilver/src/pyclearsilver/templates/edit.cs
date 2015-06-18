<?cs include:"header.cs" ?>

<h1><?cs var:CGI.cur.title?></h1>

<form method=post>
<input type=hidden name=key value="<?cs var:CGI.cur.row.key?>">

<table border=0>
  <?cs each:_col=CGI.cur.row.values?>
    <?cs with:_field=CGI.cur.table.fields[name(_col)] ?>
    <?cs if:_field.options.autoincrement || _field.options.autoguid ?>
      <tr><td><b><?cs name:_col ?>:</b></td><td><?cs var:_col ?></td>
     <input type=hidden name="values.<?cs name:_col ?>" value="<?cs var:_col?>">
      </tr>
    <?cs else ?>
<?cs if:_field.col_type == "kBigString" ?>
      <tr><td><b><?cs name:_col ?> (<?cs var:_field.col_type ?>):</b></td></tr>
      <tr>
     <td><textarea name="values.<?cs name:_col ?>" cols=40 rows=5><?cs var:_col ?></textarea></td>
      </tr>
<?cs else ?>
      <tr><td><b><?cs name:_col ?> (<?cs var:_field.col_type ?>):</b></td>
  <?cs if:_field.col_type == "kInteger" ?>
     <td><input type=text name="values.<?cs name:_col ?>" value="<?cs var:_col ?>" size=10></td>
  <?cs else ?>
     <td><input type=text name="values.<?cs name:_col ?>" value="<?cs var:_col ?>" size=40></td>
<?cs /if ?>
<?cs /if ?>
    <?cs /if ?>
    <?cs /with ?>
  <?cs /each ?>
<tr><td>
   <input type=submit name="Action.<?cs var:CGI.cur.action?>" value="Submit">
</td>
</tr>
</table>
</form>

