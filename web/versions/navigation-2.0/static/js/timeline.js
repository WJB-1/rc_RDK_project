// 以追加方式显示仿真和导航事件，避免页面自行推断状态。
function renderTimeline(events){const box=document.getElementById('timeline');if(!box)return;box.replaceChildren();(events||[]).slice(-100).reverse().forEach(item=>{const row=document.createElement('div');row.className='event';row.innerHTML='<time>'+(item.timestamp||'--')+'</time><span>'+String(item.message||item.kind||item.type||'事件')+'</span>';box.append(row)})}
