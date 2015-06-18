<html>
  <head>
<style>
<?cs include:"style.css" ?>
</style>
<script type="text/javascript">
<!--
<?cs include:"sorttable.js" ?>
-->
</script>
    <title>invent</title>
  </head>
<body>
<?cs include:"header.cs" ?>

<h1><?cs var:CGI.cur.title?></h1>

<a href="new">[New Row]</a>

<table class=sortable>
<tr>
  <?cs each:_col=CGI.cur.table.fields ?>
    <th><?cs var:_col.name ?> </th>
  <?cs /each ?>
</tr>
<?cs set:_count=1?>
<?cs each:_row=CGI.cur.table.rows?>
<?cs set:_count=_count+1?>
<?cs if:(_count % 2) == 0 ?>
<tr class=grey>
<?cs else ?>
<tr>
<?cs /if ?>
  <?cs each:_col=_row.values ?>
    <td><?cs var:_col ?> </td>
  <?cs /each ?>
  <td><a href="show?<?cs var:_row.key?>">[Show]</a></td>
  <td><a href="edit?<?cs var:_row.key?>">[Edit]</a></td>
  <td><a href="delete?<?cs var:_row.key?>">[Del]</a></td>
</tr>
<?cs /each ?>
</table>

<a href="new">[New Row]</a>

