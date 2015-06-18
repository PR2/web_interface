<?cs include:"header.cs" ?>

<h1><?cs var:CGI.cur.title?></h1>

<table border=1>
<tr>
  <?cs each:_col=CGI.cur.table.fields ?>
    <th><?cs var:_col.name ?> </th>
  <?cs /each ?>
</tr>
  <?cs each:_col=CGI.cur.row.values ?>
    <tr>
      <td><?cs name:_col ?> </td>
      <td><?cs var:_col ?> </td>
    </tr>
  <?cs /each ?>
</table>

