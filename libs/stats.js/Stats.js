// stats.js r10 - http://github.com/mrdoob/stats.js
(function(f){function d(a){this.i=a.i||a.name.toUpperCase();this.scale=a.scale;this.c=0;this.h=Infinity;this.g=-Infinity;this.b=document.createElement("div");this.b.id=a.name;this.b.style.cssText="padding:0 0 3px 3px;text-align:left";this.b.style.backgroundColor=a.e;this.text=document.createElement("div");this.text.id=a.name+"Text";this.text.style.cssText="font-family:Helvetica,Arial,sans-serif;font-size:9px;font-weight:bold;line-height:15px";this.text.style.color=a.text;this.text.innerHTML=a.name.toUpperCase();
this.b.appendChild(this.text);this.a=document.createElement("div");this.a.id=a.name+"Graph";this.a.style.cssText="position:relative;width:74px;height:30px";this.a.style.backgroundColor=a.f;for(this.b.appendChild(this.a);74>this.a.children.length;){var b=document.createElement("span");b.style.cssText="width:1px;height:30px;float:left";b.style.backgroundColor=a.d;this.a.appendChild(b)}}d.prototype.set=function(a){this.c=a;this.h=Math.min(this.h,this.c);this.g=Math.max(this.g,this.c);this.text.textContent=
this.c+" "+this.i+" ("+this.h+"-"+this.g+")";var a=this.a.appendChild(this.a.firstChild),b=Math.min(30,30-30*(this.c/this.scale));a.style.height=b+"px"};f.Stats=function(){function a(a){c.appendChild(a.b);e.push(a);return a}var b=Date.now(),g=b,h=0,i=0,e=[],c=document.createElement("div");c.id="stats";c.addEventListener("mousedown",function(a){a.preventDefault();this.setMode(++i%e.length)}.bind(this),!1);c.style.cssText="width:80px;opacity:0.9;cursor:pointer";var f=a(new d({name:"fps",scale:100,e:"#002",
text:"#0ff",f:"#0ff",d:"#113"})),j=a(new d({name:"ms",scale:200,e:"#020",text:"#0f0",f:"#0f0",d:"#131"})),k=a(new d({name:"mb",scale:50,e:"#200",text:"#f00",f:"#f00",d:"#311"}));this.domElement=c;this.setMode=function(a){i=a;for(a=0;a<e.length;++a)e[a].b.style.display=a===i?"block":"none"};this.setMode(0);this.begin=function(){b=Date.now()};this.end=function(){var a=Date.now();j.set(a-b);h++;a>g+1E3&&(f.set(Math.round(1E3*h/(a-g))),g=a,h=0);k.set(Math.round(9.5367431640625E-7*performance.memory.usedJSHeapSize));
return a};this.update=function(){b=this.end()}}})(this);
