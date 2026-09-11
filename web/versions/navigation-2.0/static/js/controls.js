// 仅发送后端白名单中的仿真控制命令。
function bindControls(send){document.querySelectorAll('[data-command]').forEach(button=>button.addEventListener('click',()=>{button.disabled=true;Promise.resolve(send(button.dataset.command)).finally(()=>{button.disabled=false})}))}
