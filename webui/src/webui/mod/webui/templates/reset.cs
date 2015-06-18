<html>
  <head>
    <?cs include:"includes.cs" ?>
    <title>Robot Reset</title>

    <style>
      #tabs, #sub_tabs, .nav_element { display: none }
    </style>

    <script>
function ros_handleOnLoad(ros_bridge_uri)  
{ 
  gPump = new MessagePump(ros_bridge_uri); 
  gPump.setupWidgets(); 
 
  if(window.gMessage) { 
    gPump.evalMessages(window.gMessage); 
  } 
  gPump.pump(); 
   
  gPump.checkWidgets(); 
} 
 
    </script>
  </head>

  <body>
    <?cs include:"header.cs" ?>
    <?cs include:"status_header.cs" ?>

    <p style="text-align: center; font-size: 150%">
      Please wait while the robot finishes resetting itself... 
    </p>

    <div id="progressbar"></div>
    <script>
      jQuery("#progressbar").progressbar();
      
      updateBar = function(complete) {
        jQuery("#progressbar").progressbar('option', 'value', complete);
        if (complete == 100) {
          document.location.href = "<?cs var:CGI.home_page ?>";
        } else {
          setTimeout(function() {
            updateBar(complete + 1);
          }, 15);
        };
      };
      
      jQuery(document).ready(function() {
        updateBar(0);
      });
    </script>

    <?cs include:"rosfooter.cs"?>
  </body>
</html>
