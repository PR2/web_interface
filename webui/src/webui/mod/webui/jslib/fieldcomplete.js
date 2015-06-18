// Auto-complete field support
// Copyright 2003,2004 by David W. Jeske
// 
// Use as you wish, this code is in the public domain.

// http://www.quirksmode.org/js/events_compinfo.html

function htmlescape(str) {
  str = str.replace("&","&amp;");
  str = str.replace("<","&lt;");
  str = str.replace(">","&gt;");
  return str;
  
}

function xylocationof(offsetobj) {
  var offset = 0;
  var loffset = 0;
  var loc = new Object();

  loc.top = 0;
  loc.left = 0;

  if (offsetobj==null || offsetobj.clientTop == null) {
    if (xIE4Up) {
      return loc;
    } else {
      // mozilla
      while (offsetobj) {
        offset += offsetobj.offsetTop;
        loffset += offsetobj.offsetLeft;

        offsetobj = offsetobj.offsetParent;
      }
     loc.top = offset;
     loc.left = loffset;

     return loc;
    }
  } else {
    while (offsetobj) {
      offset += offsetobj.offsetTop + offsetobj.clientTop;
      loffset += offsetobj.offsetLeft + offsetobj.clientLeft;
      offsetobj = offsetobj.offsetParent;
    }
  }

  loc.top = offset;
  loc.left = loffset;
  return loc;
}


// --- autocomplete ----

var active_completion_index = -1;
var active_start = -1;
var active_end = -1;
var prekey = new Object();
var save_prekey = new Object();
var matches = new Array();
var fieldcompleteinfo = new Object();
var fieldinfo = new Object();
var last_completed_value = null;

var KEY_UP     = 38;
var KEY_DOWN   = 40;
var KEY_PGUP   = 33;
var KEY_PGDOWN = 34;
var KEY_HOME   = 36;
var KEY_END    = 35;

function fc_handleKeyDown(e) {
  var event = new xEvent(e);
  var key = event.keyCode;

  var field = e.target ? e.target : e.srcElement;
  var value = field.value;

  if (e) {
    if (key == 13) {
      if (active_completion_index != -1) {
        doCompleteCur(field);
      } 
      // always prevent CR from propagating.
      return false;
    } 

    if (key == KEY_UP) {
      for (var i=0;i<matches.length;i++) {
        if (matches[i] == active_completion_index && i>0) {
          active_completion_index = matches[i-1];
          prekey[field] = save_prekey[field];
          break;
        }
      }
      return false;
    } 

    if (key == KEY_DOWN) {
      for (var i=0;i<matches.length;i++) {
        if (matches[i] == active_completion_index && (i+1)<matches.length) {
          active_completion_index = matches[i+1];
          prekey[field] = save_prekey[field];
          break;
        }
      }
      return false;
    }
    
    // ignore keys we don't handle yet
    if (key == KEY_PGDOWN || key == KEY_PGUP) {
      prekey[field] = save_prekey[field];
      return false;
    }

    // don't reset the pre-key until after we are done catching the 
    // keys above. This make sure we keep the same completion render
    prekey[field] = value;
    save_prekey[field] = value;
  }
}

// is this a valid completion separator

function isCmplSep(achar) {
  if (achar == 44 || achar == 188 || achar == 10 || achar == 13) {
    return 1;
  } else { 
    return 0;
  }
}

function doCompleteCur(field) {
  if (active_completion_index != -1) {
    doComplete(field.name,active_start,active_end,active_completion_index);
  }
}

function doComplete(fieldname,start,end,index) {
  var field = xGetElementById(fieldname);
  var value = field.value;
  var completeinfo = fieldcompleteinfo[field][index];
  //field.value = value.substring(0,start) + completeinfo + value.substring(end,value.length);
  var i = completeinfo.indexOf("--");
  field.value = completeinfo;
  if(i != -1) {
    field.value = completeinfo.substring(0,i);
  }
  last_completed_value = completeinfo;
  handleBlurForField(fieldname);
}

function buildCompleteDialog(partial,start,end,field) {
    // alert("completion of: " + partial);
    // debug re-render
    var now = new Date();
    var timestamp = "" + now.getSeconds() + ":" + now.getMilliseconds();
    // var debug = xGetElementById("debug");
    // if (debug) { debug.value = timestamp + "- " + partial; }

    if (partial == last_completed_value) {
      return;
    } else {
      last_completed_value = null;
    }

    // check for completions in our list..
    var dialog = document.getElementById('cmpldlg');
    var non_exact_match_count = 0;
    var match_count = 0;
    var total_match_count = 0;
    matches = new Array();
    var nearest_active_index = -1;
    var cmpl = fieldcompleteinfo[field];
    var partial_lower = partial.toLowerCase();

    if (partial != "") {
      for (var i=0;i<cmpl.length;i++) {
        var str_lower = cmpl[i].toLowerCase();
        var pos = str_lower.indexOf(partial_lower);
        if (pos >= 0) {
         total_match_count++;

         if (match_count == 10) {
           continue;
         }
         match_count++;      

         matches[matches.length] = i;

         if (i == active_completion_index) {
           nearest_active_index = i;
         } else {
           if (nearest_active_index == -1) {
             if (i > active_completion_index) {
               nearest_active_index = i;
             }
           }
         }


         if (partial_lower != str_lower) {
           non_exact_match_count++;
         }
        }
      }

      if (match_count) {
        active_start = start;
        active_end = end;
        if (active_completion_index == -1) {
          active_completion_index = matches[0];
        } else {
          active_completion_index = nearest_active_index;
        }

        // render completion information
        var completion_html = "<table class=cmpl width=100% cellspacing=0 cellpadding=2>";
        for (var x=0;x<matches.length;x++) {
           var i = matches[x];
           var str_lower = cmpl[i].toLowerCase();
           var pos = str_lower.indexOf(partial_lower);

           if (i == active_completion_index) {
             completion_html += "<tr bgcolor=#CCCCCC>";
           } else {
             completion_html += "<tr>";
           }
 
           completion_html += "<td nowrap style=\"cursor:pointer;\" onmousedown=\"doComplete('" + 
               field.name + "'," + start + "," + end + "," + i  + ")\">";
           completion_html += htmlescape(cmpl[i].substring(0,pos)) + 
               "<b style=\"color:#770000\">" + htmlescape(partial) + "</b>" + 
               htmlescape(cmpl[i].substring(0+pos+partial.length,cmpl[i].length));
           completion_html += "</td></tr>";
        }
        completion_html += "</table>";

        completion_html += "<Table width=100%><tr><Td colspan=2 bgcolor=#EEEECC align=center><font size=-1 color=#555555>";
        if (total_match_count != match_count) {
           completion_html += match_count + "/" + total_match_count + " ";
        }
        completion_html += "[Press ENTER to accept completion] ";

        if (0) {
          // trying to add completion management hook
          var myfieldinfo = fieldinfo[field];
          if (1 || myfieldinfo.manageUrl) {
             completion_html += "[<a href=\"#\" onmousedown=\"alert('" + field.name + "');\">edit</a>]";
          }
        }

        completion_html += "</td></tr>";
        // debug re-render
        // completion_html += "<tr><td>" + now.getSeconds() + ":" + now.getMilliseconds() + "</td></tr>";
        completion_html += "</table>";


//	alert(completion_html);

        // position properly
        if (xIE4Up) {
          var loc = xylocationof(field);
          dialog.style.top = loc.top + field.clientHeight;
          dialog.style.left = loc.left;
        } else {
          var loc = xylocationof(field);
//	  window.status = "loc.top=" + loc.top + " loc.left=" + loc.left + " " + field.height;
          dialog.style.top = loc.top + field.offsetHeight; 
          dialog.style.left = loc.left;
        }

        // fill data
        dialog.style.width='';
        dialog.style.height='';
        xInnerHtml(dialog, completion_html);
        dialog.style.display = "block";
        var max_width = document.body.clientWidth - (loc.left + 50);
        var max_height = document.body.clientHeight - (loc.top + field.clientHeight + 20);
        if (dialog.clientWidth > max_width) {
          dialog.style.width=max_width;
        }
        if (dialog.clientHeight > max_height) {
          dialog.style.height=max_height;
        }
        
      } else {
        handleBlurForField(field);
      }
    } else {
      handleBlurForField(field);
    }
}


function fc_handleBlur(e) {
  if (!e) { e = window.event; }

  var field = e.srcElement;
  handleBlurForField(field);
}

function handleBlurForField(field) {
  var dialog = document.getElementById('cmpldlg');
  dialog.style.display = "none";
  prekey[field] = null;
  active_completion_index = -1;
}

function fc_handleKeyUp(e) {
  if (!e) { e = window.event; }

  var field = e.target ? e.target : e.srcElement;
  fc_TriggerCompletion(field);
}

function fc_CompletionListChanged(field) {
  prekey[field] = save_prekey[field];
  fc_TriggerCompletion(field);
}

function fc_TriggerCompletion(field) {


  var value = field.value;
  var newkeyat = -1;

  var debug = document.getElementById("debug");
  if (debug) {
    debug.value = "keyup = " + e.keyCode + ", newkeyat=" + newkeyat + ", myprekey=" + prekey[field];
  }


  if (prekey[field] != null) {
    var myprekey = prekey[field];
    prekey[field] = null;

    if (myprekey == value) {
      // handleBlur(e);
      // the blur was causing a problem when I hit modifier
      // keys like shift.. wonder why - jeske
      return;
    }

    // find extra char location

    for(var i=0;i < value.length ; i++) {
      if (i >= myprekey.length ) {
        newkeyat = i;
        // alert("lpos : " + i);
        break;
      }
      if (value.charCodeAt(i) != myprekey.charCodeAt(i)) {
        newkeyat = i;
        // alert("pos : " + i);
        break;
      }
    }

    if (newkeyat==-1 && (value.length < myprekey.length)) {
      newkeyat = value.length-1;
    }
  }

  // alert(newkeyat);

  if (newkeyat != -1) {
    // backup until we hit the beginning of the string or a comma

    var start = 0;
    for(var i=newkeyat-1;i>0;i--) {
      if (isCmplSep(value.charCodeAt(i))) {
        start = i+1; break;
      }
    }

    // try to do completion on the phrase
    var end = newkeyat + 1;
    var partial = strip(value.substring(start,end));

    if (0 && isCmplSep(partial.charCodeAt(partial.length-1))) {
      doComplete(field.name,start,end-1,active_completion_index);
    } else {


      // show completion hover window...

      buildCompleteDialog(partial,start,end,field);
    }
  }

  // split on " ," and test address format... ??
}

function strip(astr) {
  while (astr.charCodeAt(0) == 32) {
   astr = astr.substring(1,astr.length);
  }
  while (astr.charCodeAt(astr.length - 1) == 32) {
   astr = astr.substring(0,astr.length-1);
  }
  return astr;
}


function setupField(field,cmplarr,manageUrl) {
  if (xIE4Up) {
    field.attachEvent("onkeydown", fc_handleKeyDown);
    field.attachEvent("onkeyup", fc_handleKeyUp);
    field.attachEvent("onblur", fc_handleBlur);
    fieldcompleteinfo[field] = cmplarr;

    if (0) {
      a_fieldinfo = new Object();
      a_fieldinfo.manageUrl = manageUrl;
      a_fieldinfo.completeArr = cmplarr;
      fieldinfo[field] = a_fieldinfo;
    }
  } else {
    field.onkeydown = fc_handleKeyDown;
    field.onkeyup = fc_handleKeyUp;
    field.blur = fc_handleBlur;

    //    field.addEventListener("keydown",fc_handleKeyDown,true);
    //   field.addEventListener("keyup",fc_handleKeyUp,true);
    //    field.addEventListener("blur", fc_handleBlur,false);

    fieldcompleteinfo[field] = cmplarr;
  }
}

