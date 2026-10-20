/* One outstanding request, no polling backlog; names are inserted as text. */
const rows = document.getElementById('motorRows');
const fmt = value => Number.isFinite(value) ? value.toFixed(3) : '—';
function render(data) {
  document.getElementById('subtitle').textContent = `ROS_DOMAIN_ID=${data.domain_id} · ${data.source_topic}`;
  document.getElementById('connectionText').textContent = data.connected ? 'Feedback live' : 'No fresh feedback';
  document.getElementById('connection').classList.toggle('ok', data.connected);
  const online = data.motors.filter(m => m.online).length;
  document.getElementById('summary').textContent = `${online}/${data.motors.length} online`;
  document.getElementById('safety').textContent = data.estop === null ? 'Safety status: unknown' :
    data.estop ? 'Safety estop: asserted' : 'Safety estop: released (not proof of motor enable)';
  rows.replaceChildren(...data.motors.map(m => {
    const tr = document.createElement('tr');
    tr.className = m.online ? '' : 'offline';
    [m.joint_name, m.motor_id ?? '—', m.model ?? '—', fmt(m.position_rad),
      fmt(m.velocity_radps), fmt(m.effort_nm), fmt(m.temperature_c),
      fmt(m.source_age_ms), m.online ? 'Live' : 'Offline / stale'].forEach(value => {
      const td = document.createElement('td'); td.textContent = value; tr.append(td);
    });
    return tr;
  }));
}
async function poll() {
  const controller = new AbortController();
  const timer = setTimeout(() => controller.abort(), 2000);
  try {
    const response = await fetch('/api/status', {cache: 'no-store', signal: controller.signal});
    if (!response.ok) throw new Error(`HTTP ${response.status}`);
    render(await response.json());
  } catch (error) {
    document.getElementById('connectionText').textContent = `UI disconnected: ${error.message}`;
    document.getElementById('connection').classList.remove('ok');
    document.getElementById('safety').textContent = 'Safety status: unknown (UI disconnected)';
    for (const row of rows.children) {
      row.className = 'offline'; row.lastElementChild.textContent = 'UI disconnected';
    }
  } finally {
    clearTimeout(timer); setTimeout(poll, 100);
  }
}
poll();
