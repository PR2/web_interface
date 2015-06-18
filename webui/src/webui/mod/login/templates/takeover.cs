<html>
    <head>
        <?cs include:"includes.cs" ?>
    </head>

    <body>
      <p>
	The user <?cs var:Query.username ?> would like to take this Texai.
      </p>
      <p>
	If you do not answer within <span id="timer">5000</span> seconds, you will
	be logged out automatically.
      </p>
      <p>
	<input type="button" value="Allow" onclick="allow();" />
	<input type="button" value="Deny" onclick="deny();" />
      </p>
      

    </body>

    <script>
      function updateTimer() {
        var val = parseInt(jQuery("#timer").html());
        if (val > 0) {
          jQuery("#timer").html(val - 1);
          setTimeout(function() { updateTimer() }, 1000);
        } else {
          allow();
        }
      }

      function allow() {
        document.location.href = "?Action.Response=allow";
      }

      function deny() {
        document.location.href = "?Action.Response=deny";
      }

      jQuery(document).ready(function() { updateTimer() });
    </script>

</html>
