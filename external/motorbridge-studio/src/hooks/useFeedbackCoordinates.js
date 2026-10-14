import React from 'react';

// A computed base-frame point from measured joints, never a target or a direct
// Cartesian measurement. Discard replies for earlier poses and stale sources.
export function useFeedbackCoordinates(qDeg, fresh) {
  const key = fresh && Array.isArray(qDeg) && qDeg.length === 6 && qDeg.every(Number.isFinite) ? JSON.stringify(qDeg) : '';
  const [result, setResult] = React.useState(null);
  React.useEffect(() => {
    if (!key) return undefined;
    let alive = true;
    const controller = new AbortController();
    fetch('/kinematics/fk', { method: 'POST', headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ q_deg: JSON.parse(key) }), signal: controller.signal })
      .then(async response => { const data = await response.json(); if (!response.ok || !data.ok || !Array.isArray(data.xyz_mm) || data.xyz_mm.length !== 3 || !data.xyz_mm.every(Number.isFinite)) throw new Error(data.error || '坐标计算失败'); return data; })
      .then(data => { if (alive) setResult({ key, xyz: data.xyz_mm }); })
      .catch(() => { if (alive) setResult(null); });
    return () => { alive = false; controller.abort(); };
  }, [key]);
  return key && result?.key === key ? result.xyz : null;
}
