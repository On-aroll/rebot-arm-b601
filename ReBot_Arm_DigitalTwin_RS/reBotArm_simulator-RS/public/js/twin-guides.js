(function (root) {
  'use strict';
  function padIndicators(pad) {
    if (!pad || pad.mapping !== 'standard') return {buttons:[],sticks:[0,0,0,0]};
    return {buttons:Array.from({length:17},(_,i) => Number(pad.buttons[i]?.value) > .5),
      sticks:Array.from({length:4},(_,i) => Number.isFinite(pad.axes[i]) ? Math.max(-1,Math.min(1,pad.axes[i])) : 0)};
  }
  function showPad(pad) {
    const indicators = padIndicators(pad);
    document.querySelectorAll('[data-pad-button]').forEach(node => {
      const active = node.dataset.padButton.split(',').some(i => indicators.buttons[Number(i)]);
      node.classList.toggle('is-active',active);
    });
    document.querySelectorAll('[data-pad-stick]').forEach(node => {
      const offset=Number(node.dataset.padStick)*2;
      node.setAttribute('transform',`translate(${indicators.sticks[offset]*9} ${indicators.sticks[offset+1]*9})`);
      node.classList.toggle('is-active',Math.hypot(...indicators.sticks.slice(offset,offset+2))>.16);
    });
    const status=document.getElementById('twin-gamepad-compat');
    if (status) status.textContent = !pad ? '未检测到手柄：连接后按任意键唤醒。无需连接 ROS，即可检查图中按键高亮。'
      : pad.mapping === 'standard' ? `映射检查通过：standard · ${pad.axes.length} 轴 / ${pad.buttons.length} 键。高亮只表示按键输入；运动还需使能、接管并按住 RB。`
      : `当前映射为 ${pad.mapping || '空（非标准）'}，已禁止手柄运动。请切换 XInput 模式或 USB / 接收器连接，再检查。`;
  }
  const api={padIndicators,showPad};
  if (typeof module !== 'undefined' && module.exports) module.exports=api;
  else {
    root.ReBotTwinGuides=api;
    const figure=document.getElementById('pad-guide'),dialog=document.getElementById('pad-guide-dialog');
    const zoom=document.getElementById('pad-guide-zoom');
    if (figure && dialog && zoom) {
      const home=figure.parentNode,after=figure.nextSibling;
      zoom.addEventListener('click',()=>{dialog.append(figure);dialog.showModal();});
      document.getElementById('pad-guide-close').addEventListener('click',()=>dialog.close());
      dialog.addEventListener('close',()=>home.insertBefore(figure,after));
    }
  }
})(typeof window === 'undefined' ? globalThis : window);
