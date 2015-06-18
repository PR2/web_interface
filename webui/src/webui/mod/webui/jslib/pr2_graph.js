
var PowerboardGraphWidget = Class.create({
  initialize: function(domobj) {
    this.domobj = domobj;
    this.topics = [domobj.getAttribute("topic")];

  },

  init: function() {
    this.data = new google.visualization.DataTable();
    this.data.addColumn('string', 'Time');
    this.data.addColumn('number', 'Breaker 1 Voltage');
    this.data.addColumn('number', 'Breaker 2 Voltage');
    this.data.addColumn('number', 'Breaker 3 Voltage');

    var width = this.domobj.getAttribute("width");
    if(!width) width=400;
    else width = parseInt(width);

    var height = this.domobj.getAttribute("height");
    if(!height) height=240;
    else height = parseInt(height);

    this.options = {width: width, height: height, legend: 'bottom', 
		    pointSize: 0,
		    max: 80,
		    min: 0,
		    backgroundColor: 'black',
		    titleColor: 'white',
		    legendTextColor: 'white',
		    legendBackgroundColor: 'black',
		    titleY: 'Volts',
		    title: 'Power Board 1'};
    this.chart = new google.visualization.LineChart(this.domobj);

    this.chart.draw(this.data, this.options);
  },

  receive: function(topic, msg) {
    var pbmsg = msg.status[0]
    if(pbmsg.name == "Power board 0") {
      if(this.data.getNumberOfRows() > 60) {
        this.data.removeRow(0);
      }
      this.data.addRow();
      var i = this.data.getNumberOfRows()-1;
      this.data.setValue(i, 1, pbmsg.values[4].value);
      this.data.setValue(i, 2, pbmsg.values[5].value);
      this.data.setValue(i, 3, pbmsg.values[6].value);
        
      this.chart.draw(this.data, this.options);
    }
  }
});

var PowerboardGraph2Widget = Class.create({
  initialize: function(domobj) {
    this.domobj = domobj;
    this.topics = [domobj.getAttribute("topic")];

  },

  init: function() {
    this.data = new google.visualization.DataTable();
    this.data.addColumn('string', 'Time');
    this.data.addColumn('number', 'Input Current');
	
    var width = this.domobj.getAttribute("width");
    if(!width) width=400;
    else width = parseInt(width);

    var height = this.domobj.getAttribute("height");
    if(!height) height=240;
    else height = parseInt(height);

    this.options = {width: width, height: height, legend: 'bottom', 
		    pointSize: 0,
		    max: 10,
		    min: 0,
		    backgroundColor: 'black',
		    titleColor: 'white',
		    legendTextColor: 'white',
		    legendBackgroundColor: 'black',
		    titleY: 'Amps',
		    title: 'Power Board 1'};
    this.chart = new google.visualization.LineChart(this.domobj);

    this.chart.draw(this.data, this.options);
  },

  receive: function(topic, msg) {
    var pbmsg = msg.status[0]
    if(pbmsg.name == "Power board 0") {
      if(this.data.getNumberOfRows() > 60) {
        this.data.removeRow(0);
      }
      this.data.addRow();
      var i = this.data.getNumberOfRows()-1;
      this.data.setValue(i, 1, pbmsg.values[0].value);
        
      this.chart.draw(this.data, this.options);
    }
  }
});


var PowerboardGraph3Widget = Class.create({
  initialize: function(domobj) {
    this.domobj = domobj;
    this.topics = [domobj.getAttribute("topic")];

  },

  init: function() {
    this.data = new google.visualization.DataTable();
    this.data.addColumn('string', 'Time');
    this.data.addColumn('number', 'Power Consumption');
	
	
    var width = this.domobj.getAttribute("width");
    if(!width) width=400;
    else width = parseInt(width);

    var height = this.domobj.getAttribute("height");
    if(!height) height=240;
    else height = parseInt(height);

    this.options = {width: width, 
		    height: height, 
		    pointSize: 0,
		    max: 300,
		    min: 0,
		    backgroundColor: 'black',
		    titleColor: 'white',
		    legendTextColor: 'white',
		    legendBackgroundColor: 'black',
		    legend: 'bottom', 
		    titleY: 'Watts',
		    title: 'Battery State'};
    this.chart = new google.visualization.LineChart(this.domobj);

    this.chart.draw(this.data, this.options);
  },

  receive: function(topic, msg) {
    if(this.data.getNumberOfRows() > 60) {
      this.data.removeRow(0);
    }
    this.data.addRow();
    var i = this.data.getNumberOfRows()-1;
    this.data.setValue(i, 1, -msg.power_consumption);
        
    this.chart.draw(this.data, this.options);
  }
});

var PowerboardWidget = Class.create({
  initialize: function(domobj) {
    this.domobj = domobj;
    this.topics = [domobj.getAttribute("topic")];
  },

  init: function() {
  },

  receive: function(topic, msg) {
    var pbmsg = msg.status[0]
    if(pbmsg.name == "Power board 0") {
        var cb0 = pbmsg.strings[1].value + " @ " + pbmsg.values[4].value.toFixed(2) + "V";
        var cb1 = pbmsg.strings[2].value + " @ " + pbmsg.values[5].value.toFixed(2) + "V";
        var cb2 = pbmsg.strings[3].value + " @ " + pbmsg.values[6].value.toFixed(2) + "V";
        this.domobj.innerHTML = cb0 + " " + cb1 + " " + cb2;
    }
  }
});


var BatteryGauge = Class.create({
  initialize: function(domobj) {
    this.domobj = domobj;
    this.topics = [domobj.getAttribute("topic")];

    this.key = domobj.getAttribute("key");
    this.key2 = domobj.getAttribute("key2");
  },

  init: function() {
    this.data = new google.visualization.DataTable();
    this.data.addColumn('string', 'Label');
    this.data.addColumn('number', 'Value');
    this.data.addRows(1);
    this.data.setValue(0, 0, "%Charge")
    this.data.setValue(0, 1, 0)

    var width = this.domobj.getAttribute("width");
    if(!width) width=120;
    else width = parseInt(width);

    var height = this.domobj.getAttribute("height");
    if(!height) height=120;
    else height = parseInt(height);

    this.options = {width: width, height: height,
                    redFrom: 0, redTo: 15,
		    yellowFrom: 15, yellowTo: 30,
		    minorTicks: 5}
    this.chart = new google.visualization.Gauge(this.domobj);

    this.chart.draw(this.data, this.options);
  },

  receive: function(topic, msg) {
    if(msg[this.key] != null) {
      var percent = 100. * parseFloat(msg[this.key]) / parseFloat(msg[this.key2]);
      this.data.setValue(0, 1, percent|0);
      this.chart.draw(this.data, this.options);
    }
  }
});

