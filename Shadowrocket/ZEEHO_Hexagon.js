// ZEEHO Hexagon — Shadowrocket only.
// Request capture + scheduled monthly cache + response statistics + visibility gate.
// Uses Shadowrocket native APIs only: $persistentStore / $httpClient / $notification / $done.
(function () {
  function storeGet(key){
    try { return $persistentStore.read(key); } catch(e) { return null; }
  }
  function storeSet(value,key){
    try { return $persistentStore.write(String(value),key); } catch(e) { return false; }
  }
  function httpFetch(req){
    return new Promise(function(resolve,reject){
      try {
        var method=String(req.method||'GET').toUpperCase();
        var opt={url:req.url,headers:req.headers||{}};
        if(req.body!==undefined)opt.body=req.body;
        var cb=function(error,response,data){
          if(error)return reject(Error('transport'));
          response=response||{};
          resolve({
            statusCode:Number(response.statusCode||response.status||0),
            headers:response.headers||{},
            body:data===undefined?(response.body||''):data
          });
        };
        if(method==='POST')$httpClient.post(opt,cb);
        else if(method==='PUT'&&typeof $httpClient.put==='function')$httpClient.put(opt,cb);
        else if(method==='DELETE'&&typeof $httpClient.delete==='function')$httpClient.delete(opt,cb);
        else $httpClient.get(opt,cb);
      } catch(e) { reject(Error('transport')); }
    });
  }
  function postNotice(title,subtitle,message){
    try { $notification.post(title,subtitle,message); } catch(e) {}
  }
  var C = {threshold:0.85,minDrop:0,minSpeed:20,maxGap:6,mergeSeconds:20};
  var ended=false;
  function log(x){try{console.log('ZEEHO_Hexagon '+JSON.stringify(x));}catch(e){}}
  function finish(obj){if(ended)return;ended=true;$done(obj?{body:JSON.stringify(obj)}:{});}
  function num(x){return typeof x==='number'&&isFinite(x)?x:typeof x==='string'&&x.trim()!==''&&isFinite(Number(x))?Number(x):null;}
  function date(x){
    var m=String(x||'').match(/^(\d{4})[.\/-](\d{2})[.\/-](\d{2})(?:$|[T\s])/);
    if(!m)return null;
    var y=Number(m[1]),month=Number(m[2]),day=Number(m[3]),d=new Date(Date.UTC(y,month-1,day));
    return y>=1000&&d.getUTCFullYear()===y&&d.getUTCMonth()===month-1&&d.getUTCDate()===day?m.slice(1).join('-'):null;
  }
  function validMonth(x){return typeof x==='string'&&/^\d{4}-(?:0[1-9]|1[0-2])$/.test(x);}
  function record(x){return !!x&&typeof x==='object'&&!Array.isArray(x);}
  function integer(x,min,max){return typeof x==='number'&&isFinite(x)&&Math.floor(x)===x&&x>=min&&x<=max;}
  function count(x){var n=num(x);return n!==null&&integer(n,0,9007199254740991)?n:null;}
  function emptyCache(size){return {pages:{},pageSize:size||20};}

  function estimate(text){
    if(typeof text!=='string'||!text)return null;
    var tokens=text.split('&'), points=[];
    for(var i=0;i<tokens.length;i++){
      if(!tokens[i])continue;
      var a=tokens[i].split(','),s=num(a[2]),t=num(a[3]);
      if(a.length!==5||s===null||t===null||s<0||s>200||t<1000000000000)return null;
      points.push({s:s,t:t});
    }
    if(points.length<2)return null;
    var count=0,last=-Infinity,valid=0;
    for(var j=1;j<points.length;j++){
      var prev=points[j-1],cur=points[j],dt=(cur.t-prev.t)/1000;
      if(dt<=0)return null;
      if(dt>C.maxGap)continue;
      valid++;
      var drop=prev.s-cur.s;
      if(prev.s>=C.minSpeed&&drop>=C.minDrop&&drop/3.6/dt>=C.threshold){
        if(cur.t-last>=C.mergeSeconds*1000)count++;
        last=cur.t;
      }
    }
    return valid?{count:count,points:points.length}:null;
  }
  // Experimental GPS turn events; inferred steady-state lean, not measured vehicle roll.
  var BEND={minSpeed:20,minLean:10,maxLean:45,minHeading:8,maxHeading:100,minSegment:10,minRatio:0.55,maxRatio:1.45,maxGap:6,mergeGap:8,minHits:1,minDuration:0,minTurn:15};
  var BEND_REV='gps-turn:'+vehicleKey(JSON.stringify(BEND));
  function estimateBend(text){
    if(typeof text!=='string'||!text)return null;
    var tokens=text.split('&'),p=[],rad=Math.PI/180;
    for(var i=0;i<tokens.length;i++){
      if(!tokens[i])continue;
      var a=tokens[i].split(','),x=num(a[0]),y=num(a[1]),s=num(a[2]),t=num(a[3]);
      if(a.length!==5||x===null||y===null||s===null||t===null||Math.abs(x)>180||Math.abs(y)>90||s<0||s>200||t<1000000000000)return null;
      if(p.length&&t<=p[p.length-1].t)return null;
      p.push({x:x*rad,y:y*rad,s:s,t:t});
    }
    if(p.length<3)return null;
    var seg=[];
    for(var j=1;j<p.length;j++){
      var a=p[j-1],b=p[j],dx=b.x-a.x,dy=b.y-a.y,dt=(b.t-a.t)/1000;
      var hav=Math.sin(dy/2)*Math.sin(dy/2)+Math.cos(a.y)*Math.cos(b.y)*Math.sin(dx/2)*Math.sin(dx/2);
      var length=12742000*Math.asin(Math.sqrt(Math.max(0,Math.min(1,hav))));
      var heading=Math.atan2(Math.sin(dx)*Math.cos(b.y),Math.cos(a.y)*Math.sin(b.y)-Math.sin(a.y)*Math.cos(b.y)*Math.cos(dx));
      seg.push({dt:dt,length:length,h:heading,s:(a.s+b.s)/2/3.6});
    }
    var count=0,event=null,valid=0,maxLean=0;
    function close(){if(event&&event.hits>=BEND.minHits&&event.last-event.first>=BEND.minDuration*1000&&event.turn>=BEND.minTurn)count++;event=null;}
    for(var k=1;k<seg.length;k++){
      var prev=seg[k-1],cur=seg[k],center=p[k],v=(prev.s+cur.s)/2;
      if(prev.dt>BEND.maxGap||cur.dt>BEND.maxGap){close();continue;}
      if(v*3.6<BEND.minSpeed){valid++;close();continue;}
      var ratio1=prev.s>0?prev.length/prev.dt/prev.s:0,ratio2=cur.s>0?cur.length/cur.dt/cur.s:0;
      if(prev.length<BEND.minSegment||cur.length<BEND.minSegment||ratio1<BEND.minRatio||ratio1>BEND.maxRatio||ratio2<BEND.minRatio||ratio2>BEND.maxRatio){close();continue;}
      valid++;
      var angle=Math.atan2(Math.sin(cur.h-prev.h),Math.cos(cur.h-prev.h)),degrees=Math.abs(angle)/rad;
      var omega=Math.abs(angle)/((prev.dt+cur.dt)/2),lean=Math.atan(v*omega/9.80665)/rad,sign=angle>=0?1:-1;
      if(degrees<BEND.minHeading||degrees>BEND.maxHeading||lean<BEND.minLean||lean>BEND.maxLean){if(event&&center.t-event.last>BEND.mergeGap*1000)close();continue;}
      if(event&&(event.sign!==sign||center.t-event.last>BEND.mergeGap*1000))close();
      if(!event)event={sign:sign,first:center.t,last:center.t,hits:0,turn:0};
      event.last=center.t;event.hits++;event.turn+=degrees;maxLean=Math.max(maxLean,lean);
    }
    close();
    return valid?{count:count,points:p.length,maxInferredLean:Math.round(maxLean*10)/10}:null;
  }

  function rides(obj){
    if(!obj||String(obj.code)!=='10000'||!Array.isArray(obj.data))throw Error('invalid ride response');
    var out=[];
    obj.data.forEach(function(day){if(!day||!Array.isArray(day.historyList))throw Error('invalid day');day.historyList.forEach(function(r){if(!record(r))throw Error('invalid ride');out.push(r);});});
    return out;
  }
  function replaceZero(o,key,value){
    if(!Object.prototype.hasOwnProperty.call(o,key)||num(o[key])!==0)return false;
    o[key]=typeof o[key]==='string'?String(value):value;return true;
  }
  function aggregate(o,key,value){var original=num(o[key]);if(original===null||value<original)return false;if(value===original)return false;o[key]=typeof o[key]==='string'?String(value):value;return true;}
  function query(url){
    var out=Object.create(null),at=url.indexOf('?'),text=at<0?'':url.slice(at+1).split('#')[0];
    text.split('&').forEach(function(pair){
      if(!pair)return;
      var eq=pair.indexOf('='),key=decodeURIComponent((eq<0?pair:pair.slice(0,eq)).replace(/\+/g,' '));
      var value=decodeURIComponent((eq<0?'':pair.slice(eq+1)).replace(/\+/g,' '));
      if(!key||Object.prototype.hasOwnProperty.call(out,key))throw Error('invalid query');
      out[key]=value;
    });
    return out;
  }

  function vehicleKey(text){var h=2166136261;for(var i=0;i<text.length;i++){h^=text.charCodeAt(i);h=Math.imul(h,16777619);}return (h>>>0).toString(16);}
  function getCache(key){
    var value=null;
    try{value=JSON.parse(storeGet('ZEEHO_Hexagon_'+key)||'null');}catch(e){}
    function normalize(part,allowDays,selectedDay){
      if(!record(part)||!record(part.pages)||!integer(part.pageSize,1,500))return emptyCache();
      var clean=emptyCache(part.pageSize);
      Object.keys(part.pages).forEach(function(page){
        var rows=part.pages[page];
        if(!/^[1-9]\d*$/.test(page)||!integer(Number(page),1,100)||!Array.isArray(rows)||rows.length>part.pageSize)return;
        if(!rows.every(function(r){return record(r)&&typeof r.id==='string'&&r.id!==''&&date(r.day)===r.day&&r.day.slice(0,7)===key.slice(-7)&&(!selectedDay||r.day===selectedDay);}))return;
        clean.pages[page]=rows.map(function(r){
          return {id:r.id,day:r.day,count:count(r.count),bend:r.bendRev===BEND_REV?count(r.bend):null,bendRev:r.bendRev===BEND_REV?BEND_REV:undefined};
        });
      });
      if(allowDays&&record(part.singleDays)){
        clean.singleDays={};
        Object.keys(part.singleDays).forEach(function(day){if(date(day)===day)clean.singleDays[day]=normalize(part.singleDays[day],false,day);});
      }
      return clean;
    }
    return normalize(value,true);
  }
  function samePageIds(a,b){
    if(!Array.isArray(a)||a.length!==b.length)return false;
    for(var i=0;i<a.length;i++)if(a[i].id!==b[i].id)return false;
    return true;
  }
  function savePage(cache,page,size,rows,reset){
    if(cache.pageSize!==size){cache.pages={};cache.pageSize=size;}
    var pageKey=String(page),old=cache.pages[pageKey],changed=!samePageIds(old,rows);
    if(reset)cache.pages={};
    else if(changed||rows.length<size)Object.keys(cache.pages).forEach(function(k){if(Number(k)>=page)delete cache.pages[k];});
    cache.pages[pageKey]=rows;
    return cache;
  }

  function monthRegistry(vehicle,month,cache){
    var key='ZEEHO_Hexagon_MONTHS_'+vehicle,months=[];
    try{var saved=JSON.parse(storeGet(key)||'[]');if(Array.isArray(saved))months=saved.filter(function(m){return validMonth(m);});}catch(e){}
    var hasData=Object.keys(cache.pages||{}).length>0||Object.keys(cache.singleDays||{}).length>0;
    if(hasData&&months.indexOf(month)<0){months.push(month);months.sort();try{storeSet(JSON.stringify(months),key);}catch(e){}}
    return months;
  }
  function cumulativeBend(vehicle,months,currentMonth,currentCache){
    var seen=Object.create(null),total=0,rides=0;
    function rows(part){
      Object.keys(part.pages||{}).forEach(function(page){
        part.pages[page].forEach(function(r){
          var value=num(r.bend);
          if(!r.id||seen[r.id]||r.bendRev!==BEND_REV||value===null||value<0||Math.floor(value)!==value)return;
          seen[r.id]=true;total+=value;rides++;
        });
      });
      Object.keys(part.singleDays||{}).forEach(function(day){rows(part.singleDays[day]);});
    }
    months.forEach(function(m){rows(m===currentMonth?currentCache:getCache(vehicle+':'+m));});
    return {total:total,rides:rides,months:months.length};
  }
  function metricState(){return {seen:Object.create(null),days:Object.create(null),oldest:null,valid:true,previousDay:null};}
  function metricAdd(state,r,field){
    var metric=count(r[field]);
    if(!r.id||!r.day||metric===null||metric<0){state.valid=false;return;}
    if(state.previousDay&&r.day>state.previousDay)state.valid=false;
    state.previousDay=r.day;
    if(state.seen[r.id]){state.valid=false;return;}
    state.seen[r.id]=true;
    state.days[r.day]=(state.days[r.day]||0)+metric;
    if(!state.oldest||r.day<state.oldest)state.oldest=r.day;
  }
  function metricFinish(state,complete,pages){
    var completeDays={},total=0,keys=Object.keys(state.days);
    for(var i=0;i<keys.length;i++){
      var d=keys[i];total+=state.days[d];
      if(state.valid&&(complete||d!==state.oldest))completeDays[d]=state.days[d];
    }
    return {days:completeDays,total:total,complete:complete&&state.valid,pages:pages,rides:Object.keys(state.seen).length};
  }
  function summarizePair(cache){
    var brake=metricState(),bend=metricState(),complete=false,page=1,hasPage1=Object.prototype.hasOwnProperty.call(cache.pages,'1');
    while(Object.prototype.hasOwnProperty.call(cache.pages,String(page))){
      var rows=cache.pages[String(page)];
      for(var i=0;i<rows.length;i++){
        metricAdd(brake,rows[i],'count');
        metricAdd(bend,rows[i],'bend');
      }
      if(rows.length<cache.pageSize){complete=true;break;}
      page++;
    }
    var pages=!hasPage1?0:complete?page:page-1;
    return {brake:metricFinish(brake,complete,pages),bend:metricFinish(bend,complete,pages)};
  }


  function rideMetrics(r,month,selectedDay){
    var d=date(r.startTime)||date(r.dayTime),id=r.rideId===null||r.rideId===undefined?'':String(r.rideId);
    if(!id||!d||d.slice(0,7)!==month||(selectedDay&&d!==selectedDay))throw Error('unexpected ride data');
    var n=count(r.brakesTimes),eligible=d>'2026-07-02'&&n===0,e=eligible?estimate(r.trajectory):null;
    var value=n!==null&&(n>0||d<='2026-07-02')?n:e?e.count:null;
    var nativeBend=count(r.bendingTimes),be=nativeBend===0?estimateBend(r.trajectory):null;
    return {row:{id:id,day:d,count:value,bend:nativeBend!==null&&nativeBend>0?nativeBend:be?be.count:null,bendRev:BEND_REV},brakes:e,bend:be};
  }

function bytes(s){var a=[],u=unescape(encodeURIComponent(s));for(var i=0;i<u.length;i++)a.push(u.charCodeAt(i));return a;}
function rol(x,n){return (x<<n)|(x>>>(32-n));}
function hexWord(x,little){var s='';for(var i=0;i<4;i++){var v=(x>>>(little?i*8:(3-i)*8))&255;s+=('0'+v.toString(16)).slice(-2);}return s;}
function padded(s,little){var a=bytes(s),n=a.length;a.push(128);while(a.length%64!==56)a.push(0);var lo=(n*8)>>>0,hi=Math.floor(n/536870912);for(var i=0;i<8;i++){var v=i<4?(little?lo:hi):(little?hi:lo);a.push((v>>>(little?(i%4)*8:(3-i%4)*8))&255);}return a;}
function sha1(s){var a=padded(s,false),h=[0x67452301,0xefcdab89,0x98badcfe,0x10325476,0xc3d2e1f0];for(var p=0;p<a.length;p+=64){var w=[];for(var i=0;i<16;i++)w[i]=(a[p+4*i]<<24)|(a[p+4*i+1]<<16)|(a[p+4*i+2]<<8)|a[p+4*i+3];for(i=16;i<80;i++)w[i]=rol(w[i-3]^w[i-8]^w[i-14]^w[i-16],1);var A=h[0],B=h[1],C=h[2],D=h[3],E=h[4];for(i=0;i<80;i++){var f,k;if(i<20){f=(B&C)|(~B&D);k=0x5a827999;}else if(i<40){f=B^C^D;k=0x6ed9eba1;}else if(i<60){f=(B&C)|(B&D)|(C&D);k=0x8f1bbcdc;}else{f=B^C^D;k=0xca62c1d6;}var t=(rol(A,5)+f+E+k+w[i])|0;E=D;D=C;C=rol(B,30);B=A;A=t;}h[0]=(h[0]+A)|0;h[1]=(h[1]+B)|0;h[2]=(h[2]+C)|0;h[3]=(h[3]+D)|0;h[4]=(h[4]+E)|0;}return h.map(function(x){return hexWord(x,false);}).join('');}
function md5(s){var a=padded(s,true),h=[0x67452301,0xefcdab89,0x98badcfe,0x10325476],S=[7,12,17,22,5,9,14,20,4,11,16,23,6,10,15,21],K=[];for(var i=0;i<64;i++)K[i]=Math.floor(Math.abs(Math.sin(i+1))*4294967296)|0;for(var p=0;p<a.length;p+=64){var w=[];for(i=0;i<16;i++)w[i]=a[p+4*i]|(a[p+4*i+1]<<8)|(a[p+4*i+2]<<16)|(a[p+4*i+3]<<24);var A=h[0],B=h[1],C=h[2],D=h[3];for(i=0;i<64;i++){var f,g,r=Math.floor(i/16);if(r===0){f=(B&C)|(~B&D);g=i;}else if(r===1){f=(D&B)|(~D&C);g=(5*i+1)%16;}else if(r===2){f=B^C^D;g=(3*i+5)%16;}else{f=C^(B|~D);g=7*i%16;}var t=D;D=C;C=B;B=(B+rol((A+f+K[i]+w[g])|0,S[r*4+i%4]))|0;A=t;}h[0]=(h[0]+A)|0;h[1]=(h[1]+B)|0;h[2]=(h[2]+C)|0;h[3]=(h[3]+D)|0;}return h.map(function(x){return hexWord(x,true);}).join('');}

var signingBranches=["a03b705c3ad450e07a408769e56eff94c612726f", "c5e0da7f4da28df805694ec3dd1fc6792e9df99d"];


  var AUTO={months:24,authMinutes:30,timeoutSeconds:6,maxPage:100};
  var ACTIVE='ZEEHO_Hexagon_AUTO_ACTIVE',LOCK='ZEEHO_Hexagon_AUTO_LOCK';
  function read(k){try{return JSON.parse(storeGet(k)||'null');}catch(e){return null;}}
  function write(k,v){if(!storeSet(JSON.stringify(v),k))throw Error('storage');}
  function uid(){return String(Date.now())+'-'+Math.random().toString(36).slice(2);}
  function nowMonth(){var d=new Date(Date.now()+8*3600000);return d.getUTCFullYear()+'-'+('0'+(d.getUTCMonth()+1)).slice(-2);}
  function monthsBack(){var out=[],a=nowMonth().split('-').map(Number);for(var i=0;i<AUTO.months;i++){var d=new Date(Date.UTC(a[0],a[1]-1-i,1));out.push(d.getUTCFullYear()+'-'+('0'+(d.getUTCMonth()+1)).slice(-2));}return out;}
  function headerNames(h){var names=Object.create(null);Object.keys(h||{}).forEach(function(k){names[k.toLowerCase()]=k;});return names;}
  function notify(message){try{postNotice('ZEEHO 自动缓存','',message);}catch(e){}}
  function validState(s){
    return record(s)&&typeof s.id==='string'&&typeof s.vehicle==='string'&&Array.isArray(s.months)&&s.months.length===AUTO.months&&s.months.every(validMonth)&&integer(s.index,0,s.months.length)&&integer(s.page,1,AUTO.maxPage+1)&&typeof s.started==='number'&&isFinite(s.started);
  }
  function sameState(a,b){return !!a&&!!b&&a.id===b.id&&a.generation===b.generation;}
  function sameAuth(a,b){return !!a&&!!b&&a.authRevision===b.authRevision&&a.authAt===b.authAt;}
  function request(req){
    // Bound the task independently because client timeout handling differs.
    return new Promise(function(resolve,reject){
      var settled=false,timer=setTimeout(function(){complete(Error('timeout'));},AUTO.timeoutSeconds*1000);
      function complete(error,result){if(settled)return;settled=true;clearTimeout(timer);if(error)reject(error);else resolve(result);}
      try{httpFetch(req).then(function(r){complete(null,r);},function(){complete(Error('transport'));});}
      catch(e){complete(Error('transport'));}
    });
  }
  async function runAuto(){
    var state=null,lockId=null,capturing=typeof $request!=='undefined'&&!!$request;
    try{
      if(capturing){
        var url=String($request.url||''),h=$request.headers||{},names=headerNames(h);
        if(h[names['x-zeeho-auto-cache']]||String($request.method||'GET').toUpperCase()!=='GET')return;
        var q=query(url),param=h[names['cfmoto-x-param']],sig=h[names['cfmoto-x-sign']],branch=-1;
        if(!q.vinNo||!param||!sig||String(h[names.signature]||'').toLowerCase()!==String(sig).toLowerCase()){
          log({auto:'capture skipped',reason:'missing signing headers'});return;
        }
        var originalQuery=decodeURIComponent(url.slice(url.indexOf('?')+1));
        for(var i=0;i<signingBranches.length;i++)if(md5(sha1(originalQuery+param+signingBranches[i]))===String(sig).toLowerCase()){branch=i;break;}
        if(branch<0){log({auto:'capture skipped',reason:'native signature mismatch'});return;}
        var id=md5(String(h[names.authorization]||'')+'|'+q.vinNo);
        state=read(ACTIVE);
        if(!validState(state)||state.id!==id||state.months[0]!==nowMonth()||(state.done&&Date.now()-state.started>24*3600000)){
          state={id:id,generation:uid(),vehicle:vehicleKey(q.vinNo),months:monthsBack(),index:0,page:1,started:Date.now(),done:false};
        }
        if(!state.generation)state.generation=uid();
        // A completed queue must not retain a newly captured credential indefinitely.
        if(state.done){state.headers=null;write(ACTIVE,state);return;}
        var clean={};Object.keys(h).forEach(function(k){if(!/^(host|connection|content-length|accept-encoding)$/i.test(k))clean[k]=h[k];});
        state.headers=clean;state.url=url;state.branch=branch;state.authAt=Date.now();state.authRevision=uid();state.paused=false;state.error=null;
        write(ACTIVE,state);log({auto:'queue ready',months:state.months.length,monthIndex:state.index,page:state.page,signatureVerified:true});return;
      }
      state=read(ACTIVE);
      if(!state)return;
      if(!validState(state)){
        write(ACTIVE,null);notify('缓存队列数据无效，重新打开历史轨迹可重建队列。');return;
      }
      if(state.done||state.paused){if(state.headers){state.headers=null;write(ACTIVE,state);}return;}
      if(typeof state.authAt!=='number'||!isFinite(state.authAt)||Date.now()<state.authAt||Date.now()-state.authAt>AUTO.authMinutes*60000){
        state.paused=true;state.headers=null;write(ACTIVE,state);notify('认证已过期，重新打开一次历史轨迹可继续缓存。');return;
      }
      if(!record(state.headers)||typeof state.url!=='string'||!integer(state.branch,0,signingBranches.length-1))throw Error('invalid state');
      var lock=read(LOCK);
      if(record(lock)&&typeof lock.time==='number'&&Date.now()-lock.time>=0&&Date.now()-lock.time<12000)return;
      lockId=uid();write(LOCK,{id:lockId,time:Date.now()});
      while(state.index<state.months.length){
        var month=state.months[state.index],key=state.vehicle+':'+month,cache=getCache(key);
        if(state.page===1&&month!==nowMonth()){var cachedSummary=summarizePair(cache);if(cachedSummary.brake.complete&&cachedSummary.bend.complete){monthRegistry(state.vehicle,month,cache);state.index++;continue;}}
        break;
      }
      if(state.index>=state.months.length){state.done=true;state.headers=null;write(ACTIVE,state);notify('按月缓存完成，重新进入骑行分析或我的骑行刷新。');return;}
      if(state.page>AUTO.maxPage)throw Error('page limit');
      var q=query(state.url);q.page=String(state.page);q.pageSize='20';q.startTime=month;
      delete q.startDateTime;delete q.endDateTime;delete q.month;
      var queryText=Object.keys(q).map(function(k){return encodeURIComponent(k)+'='+encodeURIComponent(q[k]);}).join('&');
      var headers={},sourceHeaders=state.headers;Object.keys(sourceHeaders).forEach(function(k){headers[k]=sourceHeaders[k];});var names=headerNames(headers);
      if(!names['cfmoto-x-param']||!names['cfmoto-x-sign']||!names.signature)throw Error('invalid state');
      var signature=md5(sha1(decodeURIComponent(queryText)+headers[names['cfmoto-x-param']]+signingBranches[state.branch]));
      headers[names['cfmoto-x-sign']]=signature;headers[names.signature]=signature;headers['X-Zeeho-Auto-Cache']='1';
      var response=await request({url:'https://tapi.zeehoev.com/v1.0/app/cfmotoserverapp/ridetrack_v2?'+queryText,method:'GET',headers:headers});
      // A newer capture owns the queue after an in-flight request finishes.
      var latest=read(ACTIVE);
      if(!latest||latest.id!==state.id||latest.generation!==state.generation||!sameAuth(latest,state))return;
      if(!response||Number(response.statusCode)!==200)throw Error('HTTP '+Number(response&&response.statusCode));
      var obj=JSON.parse(response.body);
      if(!record(obj)||String(obj.code)!=='10000')throw Error('business '+String(obj&&obj.code).replace(/[^0-9]/g,''));
      var records=rides(obj);
      if(records.length>20)throw Error('unexpected ride data');
      var saved=records.map(function(r){return rideMetrics(r,month,null).row;});
      var ids=JSON.stringify(saved.map(function(r){return r.id;}));
      if(state.page>1&&records.length&&ids===state.previousIds)throw Error('repeated page');
      cache=getCache(key);
      // Never mix page offsets from different page sizes.
      if(state.page>1&&cache.pageSize!==20){state.page=1;state.previousIds=null;write(ACTIVE,state);return;}
      savePage(cache,state.page,20,saved,state.page===1);
      write('ZEEHO_Hexagon_'+key,cache);monthRegistry(state.vehicle,month,cache);
      log({auto:'page cached',month:month,page:state.page,rides:saved.length});
      if(records.length<20){state.index++;state.page=1;state.previousIds=null;}else{state.page++;state.previousIds=ids;}
      if(state.index>=state.months.length){state.done=true;state.headers=null;notify('按月缓存完成，返回统计页面刷新。');}
      var owner=read(ACTIVE);if(sameState(owner,state)&&sameAuth(owner,state))write(ACTIVE,state);
    }catch(e){
      if(capturing){log({auto:'capture skipped',reason:'invalid request or storage'});}
      else{
        var current=read(ACTIVE);
        if(state&&sameState(current,state)&&sameAuth(current,state)){
          current.paused=true;current.headers=null;
          current.error=/^(HTTP \d+|business \d+|page limit|unexpected ride data|repeated page|storage|timeout|invalid state)$/.test(String(e.message))?e.message:'request failed';
          try{write(ACTIVE,current);}catch(ignore){}
          log({auto:'paused',reason:current.error});notify('自动缓存暂停：'+current.error+'。重新打开历史轨迹刷新认证后再试。');
        }
      }
    }finally{
      // $done may terminate the VM immediately: release the lock first.
      if(lockId){try{var lock=read(LOCK);if(lock&&lock.id===lockId)write(LOCK,null);}catch(e){log({auto:'lock cleanup failed'});}}
      finish();
    }
  }

  function runCache(){
    try{
      var url=$request.url,match=url.match(/^https:\/\/tapi\.zeehoev\.com\/v1\.0\/app\/cfmotoserverapp\/(ridetrack_v2|myRideInfo|analyse\/analyse)(?:\?|$)/);
      if(!match){finish();return;}
      var api=match[1],obj=JSON.parse($response.body),q=query(url);
      if(!record(obj)||String(obj.code)!=='10000'||!q.vinNo){finish();return;}
      var selectedDay=date(q.startDateTime),month=selectedDay?selectedDay.slice(0,7):String(q.startTime||q.month||'').replace('.','-');
      if(!validMonth(month)||(q.startDateTime&&!selectedDay)||(q.endDateTime&&(!selectedDay||date(q.endDateTime)!==selectedDay))){finish();return;}
      var vehicle=vehicleKey(q.vinNo),key=vehicle+':'+month,cache=getCache(key),changed=0;

      if(api==='ridetrack_v2'){
        var page=Number(q.page||1),size=Number(q.pageSize||20),rows=rides(obj),saved=[];
        if(!integer(page,1,100)||!integer(size,1,500)||rows.length>size){finish();return;}
        for(var i=0;i<rows.length;i++){
          var r=rows[i],metric=rideMetrics(r,month,selectedDay);
          if(metric.brakes&&replaceZero(r,'brakesTimes',metric.brakes.count))changed++;
          if(metric.bend&&replaceZero(r,'bendingTimes',metric.bend.count))changed++;
          saved.push(metric.row);
        }
        if(selectedDay){
          cache.singleDays=cache.singleDays||{};
          var dayCache=cache.singleDays[selectedDay]||emptyCache(size);
          cache.singleDays[selectedDay]=savePage(dayCache,page,size,saved,false);
        }else savePage(cache,page,size,saved,false);
        try{
          if(!storeSet(JSON.stringify(cache),'ZEEHO_Hexagon_'+key))throw Error('cache write returned false');
        }catch(e){log({api:api,status:'cache write failed',page:page});}
        monthRegistry(vehicle,month,cache);
        log({api:api,source:'trajectory estimate/cache',changed:changed,cachedPage:page,cachedRides:saved.length,selectedDay:selectedDay||null});
        finish(changed?obj:null);
        return;
      }

      var summaries=summarizePair(cache),sum=summaries.brake,bendSum=summaries.bend;
      var knownMonths=monthRegistry(vehicle,month,cache);
      if(api==='myRideInfo'){
        if(!record(obj.data)||!Array.isArray(obj.data.rideRecordList)){finish();return;}
        var cumulative=cumulativeBend(vehicle,knownMonths,month,cache);
        if(aggregate(obj.data,'bendingTimesTotal',cumulative.total))changed++;
        var daySummaryCache=Object.create(null);
        function dayPair(d){
          if(!cache.singleDays||!cache.singleDays[d])return null;
          if(!daySummaryCache[d])daySummaryCache[d]=summarizePair(cache.singleDays[d]);
          return daySummaryCache[d];
        }
        for(var j=0;j<obj.data.rideRecordList.length;j++){
          var rr=obj.data.rideRecordList[j];if(!record(rr))continue;
          var d=date(rr.date),pair=d?dayPair(d):null;
          if(d&&d>'2026-07-02'){
            var brakeValue=Object.prototype.hasOwnProperty.call(sum.days,d)?sum.days[d]:null;
            if(pair&&pair.brake.complete)brakeValue=pair.brake.total;
            if(brakeValue!==null&&aggregate(rr,'brakesTimesDay',brakeValue))changed++;
          }
          var bendValue=d&&Object.prototype.hasOwnProperty.call(bendSum.days,d)?bendSum.days[d]:null;
          if(pair&&pair.bend.complete)bendValue=pair.bend.total;
          if(bendValue!==null&&replaceZero(rr,'bendingTimesDay',bendValue))changed++;
        }
        log({api:api,status:'cumulative bend from cached rides',total:cumulative.total,rides:cumulative.rides,months:cumulative.months,changed:changed,coverage:'loaded history only; may be incomplete'});
        finish(changed?obj:null);
        return;
      }

      if(api==='analyse/analyse'){
        if(!record(obj.data)){finish();return;}
        if(bendSum.complete&&replaceZero(obj.data,'bendingTimes',bendSum.total))changed++;
        if(month>='2026-07'&&sum.complete&&aggregate(obj.data,'brakesTimes',sum.total))changed++;
        var shown=num(obj.data.brakesTimes),ratio=num(obj.data.brakesTimesPercent);
        if(month>'2026-06'&&shown!==null&&shown>=63&&ratio!==null&&ratio>=0&&ratio<=0.01){
          obj.data.brakesTimesPercent=typeof obj.data.brakesTimesPercent==='string'?'1.0':1;
          changed++;
          log({api:api,status:'radar ratio synchronized',count:shown,oldRatio:ratio,newRatio:1,basis:'observed official saturated range >=63'});
        }
        var bendShown=num(obj.data.bendingTimes);
        if(bendShown!==null&&bendShown>=0&&Math.floor(bendShown)===bendShown){
          var bendRatio=bendShown===0?0:Math.min(10,Math.ceil(bendShown/10))/10;
          if(num(obj.data.bendingTimesPercent)!==bendRatio){
            obj.data.bendingTimesPercent=typeof obj.data.bendingTimesPercent==='string'?bendRatio.toFixed(1):bendRatio;
            changed++;
          }
        }
        log({api:api,source:'trajectory estimate/cache',changed:changed,bendMonthComplete:bendSum.complete,bendMonthTotal:bendSum.complete?bendSum.total:null,cachedPages:sum.pages,cachedRides:sum.rides,monthComplete:sum.complete,monthTotal:sum.complete?sum.total:null});
        finish(changed?obj:null);
        return;
      }
      finish();
    }catch(e){log({status:'original preserved',reason:'invalid response, query or cache'});finish();}
  }
  function runGate() {
// ZEEHO iOS 3.0.5 / Shadowrocket
// V21 gate: exact V5 logic, kept isolated from trajectory processing.
(function () {
  try {
    if (!$response || !$response.body) { finish(); return; }
    var obj = JSON.parse($response.body);
    var changed = 0;

    function walk(v) {
      if (!v || typeof v !== "object") return;
      if (Array.isArray(v)) {
        for (var i=0;i<v.length;i++) walk(v[i]);
        return;
      }

      if (Object.prototype.hasOwnProperty.call(v, "vehicleKinds")) {
        var k=v.vehicleKinds;
        if (k===1 || k===4) { v.vehicleKinds=2; changed++; }
        else if (k==="1" || k==="4") { v.vehicleKinds="2"; changed++; }

        if (v.cyclingEventStatisticFlag !== true) {
          v.cyclingEventStatisticFlag=true;
          changed++;
        }
      }

      for (var key in v) {
        if (Object.prototype.hasOwnProperty.call(v,key)) walk(v[key]);
      }
    }

    walk(obj);
    if (changed>0) finish(obj);
    else finish();
  } catch(e) { finish(); }
})();

  }
  // Dispatch after shared constants have been initialized.
  try {
    var req=typeof $request!=='undefined'&&$request?$request:null;
    var response=typeof $response!=='undefined'&&$response?$response:null;
    var url=req?String(req.url||''):'';
    var match=url.match(/^https:\/\/tapi\.zeehoev\.com\/v1\.0\/app\/cfmotoserverapp\/(ridetrack_v2|myRideInfo|analyse\/analyse|vehicle\/list|vehicleHomePageV2\/[^?#]*)(?:\?[^#]*)?$/);
    if(response){
      if(!match){finish();return;}
      if(match[1]==='vehicle/list'||match[1].indexOf('vehicleHomePageV2/')===0)runGate();
      else runCache();
    }else if(req){
      if(match&&match[1]==='ridetrack_v2')runAuto();else finish();
    }else if(typeof $request==='undefined'&&typeof $response==='undefined')runAuto();
    else finish();
  }catch(e){log({status:'original preserved',reason:'invalid execution context'});finish();}
})();
