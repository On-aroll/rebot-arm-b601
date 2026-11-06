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

  async function refreshStatus() {
    try {
      const response = await fetch('/api/startup/status', { cache: 'no-store' });
      if (!response.ok) return;
      const result = await response.json();
      const simulation = result.services?.simulation;
      if (!simulation) return;
      if (simulation.ready) {
        setState('reused', 'quickStart.detailReused');
      } else if (simulation.running) {
        setState('starting');
      } else if (['ready', 'reused', 'starting'].includes(state)) {
        setState('idle');
      }
    } catch (_) {
      // Keep the last known status when the local API is temporarily unavailable.
    }
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
  void refreshStatus();
  const statusTimer = window.setInterval(refreshStatus, 3000);
  window.addEventListener('pagehide', () => window.clearInterval(statusTimer));
  if (window.rebotI18n && window.rebotI18n.onLangChange) {
    window.rebotI18n.onLangChange(() => {
      render();
      button.textContent = button.disabled ? translate('quickStart.buttonBusy') : translate('quickStart.button');
    });
  }
  render();
})();
