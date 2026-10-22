(function () {
  const button = document.getElementById('quick-start-button');
  const status = document.getElementById('quick-start-status');
  if (!button || !status) return;

  let state = 'idle';
  const translate = (key) => window.rebotI18n ? window.rebotI18n.t(key) : key;
  const render = (detailKey) => {
    status.textContent = translate(`quickStart.${state}`);
    status.classList.toggle('online', state === 'ready' || state === 'reused');
    status.classList.toggle('error', state === 'failed');
    if (detailKey) button.title = translate(detailKey);
  };

  function setState(next, detailKey) {
    state = next;
    render(detailKey);
  }

  async function startSimulation() {
    if (button.disabled) return;
    button.disabled = true;
    button.textContent = translate('quickStart.buttonBusy');
    setState('starting');
    try {
      const response = await fetch('/api/quick-start', {
        method: 'POST',
        headers: { Accept: 'application/json' }
      });
      const result = await response.json().catch(() => ({}));
      if (!response.ok || !result.ok) {
        throw new Error(result.error || `HTTP ${response.status}`);
      }
      setState(result.reused ? 'reused' : 'ready', result.reused ? 'quickStart.detailReused' : 'quickStart.detail');
    } catch (error) {
      console.error('[quick-start]', error);
      setState('failed', 'quickStart.detailFailed');
    } finally {
      button.disabled = false;
      button.textContent = translate('quickStart.button');
    }
  }

  button.addEventListener('click', startSimulation);
  if (window.rebotI18n && window.rebotI18n.onLangChange) {
    window.rebotI18n.onLangChange(() => {
      render();
      button.textContent = button.disabled ? translate('quickStart.buttonBusy') : translate('quickStart.button');
    });
  }
  render();
})();
