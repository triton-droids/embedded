/* Feedback only: action reviews never publish or POST a motor command.
 * A future control_core integration must own admission and verified progress;
 * receiving telemetry or opening a review is never execution acknowledgement.
 */
const el = id => document.getElementById(id);
const rows = el('motorRows');
const fmt = value => Number.isFinite(value) ? value.toFixed(3) : '—';
let motors = [];
let selectedJoint = '';
let transportLive = false;
let reviewAction = null;
let configSignature = '';
const configValue = value => value === null || value === undefined ? 'Not configured' :
  typeof value === 'object' ? JSON.stringify(value) : String(value);
function renderConfig(config) {
  const signature = JSON.stringify(config ?? null);
  if (signature === configSignature) return;
  configSignature = signature;
  const status = el('configStatus');
  status.classList.toggle('config-error', Boolean(config?.error));
  const loaded = config?.loaded_at ? new Date(config.loaded_at * 1000).toLocaleString() : 'never';
  status.textContent = !config?.path ? 'Configuration: no motor_config_file selected; showing discovered feedback only.' :
    `Config: ${config.path} · Check interval: ${config.reload_interval_s} s · Last loaded: ${loaded}` +
    (config.error ? ` · Reload failed: ${config.error}. ${config.parameters ? 'Showing last valid configuration.' : 'No valid configuration loaded.'}` : '');
  const params = config?.parameters;
  const motors = params?.motors ?? {};
  const fields = [...new Set(Object.values(motors).flatMap(motor => Object.keys(motor)))];
  const header = document.createElement('tr');
  for (const name of ['Joint', ...fields]) {
    const th = document.createElement('th'); th.scope = 'col'; th.textContent = name; header.append(th);
  }
  el('configHead').replaceChildren(header);
  el('configRows').replaceChildren(...Object.entries(motors).map(([name, cfg]) => {
    const tr = document.createElement('tr');
    [name, ...fields.map(field => configValue(cfg[field]))].forEach((value, index) => {
      const td = document.createElement('td'); td.textContent = value;
      if (index > 0 && cfg[fields[index - 1]] && typeof cfg[fields[index - 1]] === 'object') td.className = 'structured';
      tr.append(td);
    });
    return tr;
  }));
  const entries = [];
  function flatten(value, prefix) {
    if (value && typeof value === 'object' && !Array.isArray(value) && Object.keys(value).length) {
      for (const [key, item] of Object.entries(value)) flatten(item, prefix ? `${prefix}.${key}` : key);
    } else entries.push([prefix, configValue(value)]);
  }
  for (const [key, value] of Object.entries(params ?? {})) if (key !== 'motors') flatten(value, key);
  el('parameterRows').replaceChildren(...entries.map(entry => {
    const tr = document.createElement('tr');
    for (const value of entry) { const td = document.createElement('td'); td.textContent = value; tr.append(td); }
    return tr;
  }));
}
const actions = {
  home: {
    title: 'Home',
    description: 'Move the selected joint to its calibrated zero. This is physical motion; it does not redefine zero.',
    steps: ['control_core takes exclusive command ownership and pauses conflicting motion.',
      'Validate fresh feedback, calibration, limits and safety conditions.',
      'Generate a bounded trajectory from the measured position to calibrated zero.',
      'Verify arrival before reporting completion.']
  },
  zero: {
    title: 'Set mechanical zero',
    description: 'Set the current physical position as the motor encoder zero. This is not a move-to-zero command or a software-only offset.',
    steps: ['Pause all body motion owned by control_core and decelerate within configured limits.',
      'Confirm fresh feedback, supported firmware/mode and a settled motor before the zero write.',
      'Write the selected motor zero and verify fresh raw position feedback; an acknowledgement alone is insufficient.',
      'Rebind coordinate-dependent targets, offsets and limits; discard pre-zero commands.',
      'Resume previously authorized control only after verification. On failure, remain paused and report the reason.']
  }
};
function selectedMotor() { return motors.find(m => m.joint_name === selectedJoint); }
function updateSelection() {
  const motor = selectedMotor();
  el('jointSelect').value = motor ? selectedJoint : '';
  el('jointIdentity').textContent = motor ?
    `${motor.can_interface ?? 'Unknown CAN'} · ID ${motor.motor_id ?? '—'} · ${motor.model ?? 'Unknown model'}` : 'No joint selected';
  el('jointPosition').textContent = fmt(motor?.position_rad);
  el('jointVelocity').textContent = fmt(motor?.velocity_radps);
  el('jointTorque').textContent = fmt(motor?.effort_nm);
  el('jointFreshness').textContent = !motor ? 'Unknown' : !transportLive ? 'UI disconnected' :
    motor.online ? `Live · ${fmt(motor.source_age_ms)} ms` : 'Offline / stale';
  // These buttons open local reviews only; execution stays disabled unconditionally.
  el('homeButton').disabled = !motor;
  el('zeroButton').disabled = !motor;
  for (const row of rows.children) row.classList.toggle('selected', row.dataset.joint === selectedJoint);
  if (reviewAction) {
    el('reviewFeedback').textContent = `Current ROS feedback: position ${fmt(motor?.position_rad)} rad · velocity ${fmt(motor?.velocity_radps)} rad/s · ${el('jointFreshness').textContent}. Raw zero verification is unavailable.`;
  }
}
function selectJoint(name) { selectedJoint = name; updateSelection(); }
function review(action) {
  const motor = selectedMotor();
  if (!motor) return;
  reviewAction = action;
  const config = actions[action];
  el('reviewTitle').textContent = config.title;
  el('reviewJoint').textContent = `${motor.joint_name} · ${motor.can_interface ?? 'Unknown CAN'} · ID ${motor.motor_id ?? '—'}`;
  el('reviewDescription').textContent = config.description;
  el('reviewSteps').replaceChildren(...config.steps.map(step => {
    const item = document.createElement('li'); item.textContent = step; return item;
  }));
  el('actionStatus').textContent = `${config.title} reviewed for ${motor.joint_name}. No action sent.`;
  updateSelection();
  el('actionDialog').showModal();
}
el('jointSelect').addEventListener('change', event => selectJoint(event.target.value));
el('homeButton').addEventListener('click', () => review('home'));
el('zeroButton').addEventListener('click', () => review('zero'));
el('actionDialog').addEventListener('close', () => { reviewAction = null; });
function render(data) {
  renderConfig(data.config);
  motors = data.motors;
  transportLive = true;
  el('subtitle').textContent = `ROS_DOMAIN_ID=${data.domain_id} · ${data.source_topic}`;
  el('connectionText').textContent = data.connected ? 'Feedback live' : 'No fresh feedback';
  el('connection').classList.toggle('ok', data.connected);
  el('summary').textContent = `${motors.filter(m => m.online).length}/${motors.length} online · ${data.config?.parameters ? 'All configured motors' : 'Discovered motors'}`;
  const lastEstop = data.last_reported_estop == null ? 'unknown' : data.last_reported_estop ? 'asserted' : 'released';
  const supervisor = data.supervisor_online ?
    `online (diagnostic level ${data.supervisor_level}: ${data.supervisor_message})` : 'unknown / stale';
  el('safety').textContent = `Supervisor: ${supervisor} · Last reported estop: ${lastEstop} · ` +
    (data.estop == null ? 'Current estop: unknown' : data.estop ? 'Current estop: asserted' :
      'Current estop: released (not proof of motor enable)');
  // Preserve the selector DOM and keyboard focus across 10 Hz feedback updates.
  const names = motors.map(m => m.joint_name);
  const options = Array.from(el('jointSelect').options).slice(1).map(option => option.value);
  if (JSON.stringify(names) !== JSON.stringify(options)) {
    const placeholder = document.createElement('option'); placeholder.value = ''; placeholder.textContent = 'Select a joint…';
    el('jointSelect').replaceChildren(placeholder, ...names.map(name => {
      const option = document.createElement('option'); option.value = name; option.textContent = name; return option;
    }));
  }
  if (!selectedMotor()) selectedJoint = '';
  // Keep row buttons attached while values refresh, preserving keyboard focus.
  if (JSON.stringify(names) !== JSON.stringify(Array.from(rows.children).map(row => row.dataset.joint))) {
    rows.replaceChildren(...motors.map(m => {
      const tr = document.createElement('tr'); tr.dataset.joint = m.joint_name;
      const jointCell = document.createElement('td');
      const button = document.createElement('button'); button.className = 'joint-button'; button.textContent = m.joint_name;
      button.addEventListener('click', () => selectJoint(m.joint_name)); jointCell.append(button); tr.append(jointCell);
      for (let i = 0; i < 9; i++) tr.append(document.createElement('td'));
      return tr;
    }));
  }
  motors.forEach((m, index) => {
    const tr = rows.children[index];
    tr.className = m.online ? '' : 'offline';
    [m.can_interface ?? '—', m.motor_id ?? '—', m.model ?? '—', fmt(m.position_rad),
      fmt(m.velocity_radps), fmt(m.effort_nm), fmt(m.temperature_c),
      fmt(m.source_age_ms), m.online ? 'Live' : 'Offline / stale'].forEach((value, cellIndex) => {
      tr.children[cellIndex + 1].textContent = value;
    });
  });
  updateSelection();
}
async function poll() {
  const controller = new AbortController();
  const timer = setTimeout(() => controller.abort(), 2000);
  try {
    const response = await fetch('/api/status', {cache: 'no-store', signal: controller.signal});
    if (!response.ok) throw new Error(`HTTP ${response.status}`);
    render(await response.json());
  } catch (error) {
    transportLive = false;
    el('connectionText').textContent = `UI disconnected: ${error.message}`;
    el('connection').classList.remove('ok');
    el('safety').textContent = 'Safety status: unknown (UI disconnected)';
    el('configStatus').textContent = 'Configuration refresh unavailable (UI disconnected). Displayed configuration is cached.';
    el('configStatus').classList.add('config-error');
    configSignature = ''; // Restore configuration health after reconnecting, even if unchanged.
    for (const row of rows.children) {
      row.classList.add('offline'); row.lastElementChild.textContent = 'UI disconnected';
    }
    updateSelection();
  } finally {
    clearTimeout(timer); setTimeout(poll, 100);
  }
}
poll();
