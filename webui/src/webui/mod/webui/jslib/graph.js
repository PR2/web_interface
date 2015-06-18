/* ---------------------------------------------------------------
   Javascript CSS Bar Graph (v1.0)
   Copyright (C) 2007 Nicholas Cook
   
   Permission is hereby granted, free of charge, to any person
   obtaining a copy of this software and associated documentation
   files (the "Software"), to deal in the Software without
   restriction, including without limitation the rights to use,
   copy, modify, merge, publish, distribute, sublicense, and/or sell
   copies of the Software, and to permit persons to whom the
   Software is furnished to do so, subject to the following
   conditions:

   The above copyright notice and this permission notice shall be
   included in all copies or substantial portions of the Software.

   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
   EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
   OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
   NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
   HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
   WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
   FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
   OTHER DEALINGS IN THE SOFTWARE.
   --------------------------------------------------------------- */

var JSGraph = Class.create({
  defaultBackgrounds: ['#415180', '#5A4180', '#417680', '#804141', '#806641', '#418042'],
  
  initialize: function(id, width, height, w_pad) {
    this.el = $(id);
    this.width  = (typeof(width) != 'undefined') ? width : 500;
    this.height = (typeof(height) != 'undefined') ? height : 200;
    this.pad    = (typeof(w_pad) != 'undefined') ? w_pad : 30;
    this.items  = [];
    
    this.backgroundImage    = null;
    this.el.style.width = this.addPx(this.width);
  },
  
  addItem: function() {
    var obj = [];

    for(var i = 0; i < arguments.length; i++)
    {
      if(typeof(arguments[i]) == 'object')
      {
        for(var j = 0; j < arguments[i].length; j++)
        {
          obj.push(arguments[i][j]);
          if(j == arguments[i].length - 1) { this.items.push(obj); obj = []; }
        }
      }
      else
        obj.push(arguments[i]);
    }

    if(obj.length > 0) this.items.push(obj);
  },
  
  setBackground: function() {
    for(var i = 0; i < arguments.length; i++)
    {
      var arg = this.formatImage(arguments[i])
      
      if ( arguments.length == 1 && typeof(arg) == 'object' )
        this.backgroundImage = arg;
      else
        this.defaultBackgrounds[i] = arg;
    }
  },
  
  buildBarGraph: function() {
    var columns               = this.items.length;
    var li_width              = Math.floor((this.width - (this.pad * columns)) / columns);
    
    // create the ULs
    var bar_ul                = document.createElement('ul');
    var legend_ul             = document.createElement('ul');
    bar_ul.className          = 'bar';
    legend_ul.className       = 'legend';
    
    for( var col = 0; col < columns; col++ )
    {
      // get values
      var title               = this.items[col][0];
      var votes               = this.items[col][1];
      var value               = this.items[col][2];
      var backColor           = (typeof(this.items[col][3]) != 'undefined') ? this.formatImage(this.items[col][3]) : this.defaultBackgrounds[col];
      
      // create the bar LI and style
      var li                  = document.createElement('li');
      var div                 = document.createElement('div');
      var li_height           = Math.round(value * .01 * this.height, 0);
      li.style.width          = this.addPx(li_width);
      div.style.marginTop     = this.addPx(this.height - li_height);
      div.style.height        = this.addPx(li_height);
  
      // set background image or color
      if(this.backgroundImage){
        div.style.background  = 'url(' + this.backgroundImage.filename + '-' + parseInt(col + 1) + this.backgroundImage.ext + ') repeat-x';
      } else if(typeof(backColor) == 'object') {
        div.style.background  = 'url(' + backColor.filename + backColor.ext + ') repeat-x';
      } else {
        div.style.background  = backColor;
      }
        
      // no votes? 
      if ( value == 0 )
  		{
  		  div.className         = 'no-results';
  		  div.style.marginTop   = this.addPx(this.height - 22);
  		  div.style.height      = this.addPx(22);
  		  div.style.background  = 'none';
  		  votes                 = '0 votes';
  		}
  		// failsafe: if height of div is too small to display label
  		// display it above
  		if ( li_height < 25 && value != 0 )
  		{
  		  div.style.marginTop   = 0;
  		  var span = document.createElement('span');
  		  span.appendChild(document.createTextNode(votes));
  		  span.style.marginTop  = this.addPx(this.height - li_height - 15);
  		  li.appendChild(span);
  	  }
  	  
  	  // append vote count
  	  if (li_height == 0 || li_height > 24) div.appendChild(document.createTextNode(votes));
  		li.appendChild(div);
  		bar_ul.appendChild(li); 

  		// create and append legend LI
  		var legend_li           = document.createElement('li');
  		legend_li.style.width	  = this.addPx(li_width)
  	  legend_li.appendChild(document.createTextNode(title));
  	  legend_ul.appendChild(legend_li);
    }
    
    // append survey to DIV
  	this.el.appendChild(bar_ul);
  	this.el.appendChild(legend_ul);
  },
  
  addPx: function(line) { return line + 'px' },

  formatImage: function(string) { 
    var matches = string.match(/([a-zA-Z0-9\/\-_]+)|\.(png|gif|jpg|jpeg)$/g);
    if( typeof(matches[1]) != 'undefined')
      return { filename: matches[0], ext: matches[1] }
    else
      return (matches) ? matches[0] : string;
  }
});
