<?cs include:"ros_app_include_header.cs" ?>
<?cs include:"status_header.cs" ?>
  <h3>Robot <?cs if:Cookie.inactive != 1 ?>Reset<?cs else ?>Processes<?cs /if ?></h3>
  
  <p>
  <?cs if:Cookie.inactive != 1 ?>
    If you want to reset the robot, please take a moment to look at the list
    of currently running processes (output of ckill list) and make sure
    it's safe to proceed:
  <?cs else ?>
    The following is a list of processes running on the robot:
  <?cs /if ?>
  </p>
  
  <?cs if:CGI.cur.error ?>
    <p class="error_message"><?cs var:CGI.cur.error ?></p>
  <?cs else ?>
    <pre>
    <?cs each:_line=CGI.cur.lines ?>
      <?cs var:_line ?>
    <?cs /each ?>
    </pre>

    <?cs if:Cookie.inactive != 1 ?>
        <input type="button" class="reset_link" value="Reset the Robot" />

        <script>
          jQuery('.reset_link').click(function() {
            document.location.href = "<?cs var:CGI.BaseURI ?>/webui/reset.py?Action.DoReset=1";
          });
        </script>
    <?cs /if ?>
  <?cs /if ?>

<?cs include:"ros_app_include_footer.cs" ?>
