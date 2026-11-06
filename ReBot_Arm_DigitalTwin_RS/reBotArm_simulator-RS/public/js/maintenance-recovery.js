'use strict';
(function(root,factory){
  if(typeof module==='object'&&module.exports)module.exports=factory;
  else root.ReBotMaintenance=factory(root.fetch.bind(root));
})(typeof window==='undefined'?globalThis:window,function(fetch,wait=ms=>new Promise(resolve=>setTimeout(resolve,ms))){
  async function api(method,action,body) {
    const response=await fetch('/api/maintenance/'+action,{method,cache:'no-store',
      headers:{'Content-Type':'application/json'},...(body?{body:JSON.stringify(body)}:{}),signal:AbortSignal.timeout(10000)});
    const result=await response.json();
    if(!response.ok||!result.ok)throw new Error(result.error||'维护服务请求失败');
    return result.job;
  }
  async function follow(job,onStatus) {
    const id=job.id;
    onStatus(job);
    while(job.state==='running') {
      await wait(500);job=await api('GET','status');
      if(job.id!==id)throw new Error('维护任务已变化，当前操作结果未确认');
      onStatus(job);
    }
    return job;
  }
  return {
    restore:async onStatus=>follow(await api('POST','restore',{confirmed:true}),onStatus),
    // Reloading a page can observe an existing job, but never initiates one.
    resume:async onStatus=>{const job=await api('GET','status');if(job.state==='running')return follow(job,onStatus);return job;}
  };
});
