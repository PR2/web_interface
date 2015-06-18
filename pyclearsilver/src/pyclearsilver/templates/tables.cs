<?cs include:"header.cs" ?>

<h1><?cs var:CGI.cur.title?></h1>

<table border=1>
<?cs each:_row=CGI.cur.tables?>
<tr>
   <td><a href="<?cs var:_row ?>/"><?cs var:_row ?></a></td>
</tr>
<?cs /each ?>
</table>

