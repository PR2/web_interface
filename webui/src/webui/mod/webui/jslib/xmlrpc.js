
function CreateXmlHttpReq(handler) {
    var xmlhttp = null;

    if (xIE4Up) {
        // Guaranteed to be ie5 or ie6
        var control = (xIE5) ? "Microsoft.XMLHTTP" : "Msxml2.XMLHTTP";
        try {
            xmlhttp = new ActiveXObject(control);
	    if(handler) {
	      xmlhttp.onreadystatechange = handler;
	      handler.req = xmlhttp;
	    }
        } catch(e) {
            // TODO: better help message
            alert("You need to enable active scripting and activeX controls");
            DumpException(e);
        }
    } else {
        // Mozilla
        xmlhttp = new XMLHttpRequest();
	if(handler) {
	  xmlhttp.onload = handler;
	  xmlhttp.onerror = handler;
	  handler.req = xmlhttp;
	}
    }
    return xmlhttp;
}

var F=navigator.userAgent.indexOf("Safari")>=0;
function _sendx(url,callback){
  var req=L();
  if(!req||F&&!callback){
    (new Image()).src=url;
  } else {
    req.open("GET",url,true);
    if(callback){
      req.onreadystatechange=function(){
	if(req.readyState==4){
	  callback(req.responseText);
	}
      }
    }
    req.send(null);
  }
}

function L() {
  var a=null;

  try{
    a=new ActiveXObject("Msxml2.XMLHTTP");
  } catch(b) {
    try{
      a=new ActiveXObject("Microsoft.XMLHTTP");
    } catch(c){
      a=null;
    }
		
  }
  if(!a && typeof XMLHttpRequest!="undefined"){
    a=new XMLHttpRequest();
  }
  return a;
}
