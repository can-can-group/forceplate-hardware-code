/**
 * CMJ web client – Web Bluetooth + live chart + /api/cmj for analysis.
 * BLE: 1 byte sample_count, then sample_count × 16 bytes (8× int16 LE: L1..L4, R5..R8).
 */

(function () {
  'use strict';

  const SERVICE_UUID = '12345678-1234-1234-1234-123456789abc';
  const DATA_CHAR_UUID = '87654321-4321-4321-4321-cba987654321';
  const COMMAND_CHAR_UUID = '11111111-2222-3333-4444-555555555555';
  const SAMPLE_RATE_HZ = 1000;
  const PLOT_WINDOW_SEC = 10;
  const MAX_BUFFER_POINTS = PLOT_WINDOW_SEC * SAMPLE_RATE_HZ;
  /** Downsample to this many points for the chart (keeps UI smooth at 1000 Hz). */
  const CHART_POINTS = 600;
  /** Chart refresh interval (ms). */
  const CHART_INTERVAL_MS = 40;

  let device = null;
  let server = null;
  let dataChar = null;
  let commandChar = null;
  let buffer = [];
  let chart = null;
  let chartIntervalId = null;

  const el = (id) => document.getElementById(id);
  const statusEl = el('status');
  const samplesEl = el('samples');
  const resultsEl = el('results');
  const logEl = el('log');
  const localCalOut = el('localCalOut');
  const remoteCalOut = el('remoteCalOut');
  const btnConnect = el('btnConnect');
  const btnDisconnect = el('btnDisconnect');
  const btnStartRec = el('btnStartRec');
  const btnStopRec = el('btnStopRec');
  const btnRunCmj = el('btnRunCmj');
  const btnClear = el('btnClear');
  const btnSave = el('btnSave');
  const btnLoad = el('btnLoad');
  const loadSelect = el('loadSelect');

  function setStatus(text) {
    statusEl.textContent = text;
  }

  function log(msg) {
    const pre = logEl;
    pre.textContent += msg + '\n';
    pre.scrollTop = pre.scrollHeight;
  }

  function parseJsonResponse(res) {
    return res.text().then(function (text) {
      try {
        return JSON.parse(text);
      } catch (_) {
        return { _raw: text, _status: res.status };
      }
    });
  }

  function parsePacket(data) {
    if (!data || data.byteLength < 1) return { sample_count: 0, samples: [] };
    const view = new DataView(data.buffer, data.byteOffset, data.byteLength);
    const sampleCount = view.getUint8(0);
    if (sampleCount > 10 || sampleCount === 0) return { sample_count: 0, samples: [] };
    const expectedLen = 1 + sampleCount * 16;
    if (data.byteLength < expectedLen) return { sample_count: 0, samples: [] };
    const samples = [];
    for (let i = 0; i < sampleCount; i++) {
      const off = 1 + i * 16;
      const row = [];
      for (let ch = 0; ch < 8; ch++) {
        row.push(view.getInt16(off + ch * 2, true));
      }
      samples.push(row);
    }
    return { sample_count: sampleCount, samples };
  }

  function onNotification(event) {
    const value = event.target.value;
    if (!value) return;
    const parsed = parsePacket(value);
    for (const row of parsed.samples) {
      buffer.push(row.slice());
    }
  }

  function updateSamplesLabel() {
    samplesEl.textContent = 'Samples: ' + buffer.length;
  }

  function bufferToCmjInput() {
    const n = buffer.length;
    const left = [], right = [], total = [], index = [];
    for (let i = 0; i < n; i++) {
      const s = buffer[i];
      const l = s[0] + s[1] + s[2] + s[3];
      const r = s[4] + s[5] + s[6] + s[7];
      left.push(l);
      right.push(r);
      total.push(l + r);
      index.push(i / SAMPLE_RATE_HZ);
    }
    return { left_force: left, right_force: right, total_force: total, index };
  }

  /** Downsample array to at most maxPoints by taking evenly spaced indices. */
  function downsample(arr, maxPoints) {
    const n = arr.length;
    if (n <= maxPoints) return arr;
    const step = (n - 1) / (maxPoints - 1);
    const out = [];
    for (let i = 0; i < maxPoints; i++) {
      const idx = Math.round(i * step);
      out.push(arr[idx]);
    }
    return out;
  }

  function updateChartFromBuffer() {
    if (!chart) return;
    const n = buffer.length;
    samplesEl.textContent = 'Samples: ' + n;
    if (n === 0) {
      chart.data.labels = [];
      chart.data.datasets[0].data = [];
      chart.data.datasets[1].data = [];
      chart.data.datasets[2].data = [];
      chart.update('none');
      return;
    }
    const start = Math.max(0, n - MAX_BUFFER_POINTS);
    const slice = buffer.slice(start);
    const len = slice.length;
    const t = [];
    const left = [];
    const right = [];
    const total = [];
    for (let i = 0; i < len; i++) {
      const s = slice[i];
      const l = s[0] + s[1] + s[2] + s[3];
      const r = s[4] + s[5] + s[6] + s[7];
      t.push((start + i) / SAMPLE_RATE_HZ);
      left.push(l);
      right.push(r);
      total.push(l + r);
    }
    const maxPts = CHART_POINTS;
    chart.data.labels = downsample(t, maxPts);
    chart.data.datasets[0].data = downsample(left, maxPts);
    chart.data.datasets[1].data = downsample(right, maxPts);
    chart.data.datasets[2].data = downsample(total, maxPts);
    chart.update('none');
  }

  function onCmdNotification(event) {
    const value = event.target.value;
    if (!value || value.byteLength === 0) return;
    const text = new TextDecoder().decode(value);
    let json;
    try {
      json = JSON.parse(text);
    } catch (_) {
      log('Cmd response (not JSON): ' + text);
      return;
    }
    const target = (json.target || '').toUpperCase();
    const cmd = json.cmd || '';
    const out = formatCmdResponse(json);
    if (target === 'LOCAL') {
      localCalOut.textContent = out;
    } else if (target === 'REMOTE') {
      remoteCalOut.textContent = out;
    } else {
      log(out);
    }
  }

  function formatCmdResponse(json) {
    const cmd = json.cmd || '';
    if (json.ok === false && json.err) {
      return cmd + ': ' + json.err + (json.ms != null ? ' (' + json.ms + ' ms)' : '');
    }
    if (cmd === 'SHOW' && Array.isArray(json.lc)) {
      let s = 'SHOW (off, slope a, points n):\n';
      json.lc.forEach(function (lc, i) {
        s += '  LC' + (i + 1) + ': off=' + lc.off + ', a=' + (lc.a != null ? lc.a.toFixed(6) : '?') + ', n=' + (lc.n != null ? lc.n : '?') + '\n';
      });
      if (json.ms != null) s += 'ms: ' + json.ms;
      return s;
    }
    if (cmd === 'READ' && Array.isArray(json.v)) {
      const tot = json.v.length >= 5 ? json.v[4] : json.v.reduce(function (a, b) { return a + b; }, 0);
      return 'READ (10g): ' + json.v.join(', ') + '\nTotal: ' + tot + ' ('.concat((tot / 100).toFixed(2), ' kg)');
    }
    if (cmd === 'PTS' && Array.isArray(json.n)) {
      return 'Points: ' + json.n.join(', ');
    }
    return (cmd || 'OK') + (json.ok === true ? ' ok' : '') + (json.ms != null ? ' (' + json.ms + ' ms)' : '');
  }

  function setCalButtonsEnabled(enabled) {
    ['localTare', 'localShow', 'localRead', 'localClear', 'localPts', 'localAdd', 'localAddChBtn',
     'remoteTare', 'remoteShow', 'remoteRead', 'remoteClear', 'remotePts', 'remoteAdd', 'remoteAddChBtn'].forEach(function (id) {
      const b = el(id);
      if (b) b.disabled = !enabled;
    });
  }

  async function connect() {
    if (!navigator.bluetooth) {
      setStatus('Web Bluetooth not supported (use Chrome, HTTPS or localhost).');
      log('Web Bluetooth not supported.');
      return;
    }
    try {
      setStatus('Requesting device...');
      device = await navigator.bluetooth.requestDevice({
        filters: [{ namePrefix: 'LoadCell' }],
        optionalServices: [SERVICE_UUID]
      });
      setStatus('Connecting...');
      server = await device.gatt.connect();
      const service = await server.getPrimaryService(SERVICE_UUID);
      dataChar = await service.getCharacteristic(DATA_CHAR_UUID);
      commandChar = await service.getCharacteristic(COMMAND_CHAR_UUID);
      await dataChar.startNotifications();
      dataChar.addEventListener('characteristicvaluechanged', onNotification);
      await commandChar.startNotifications();
      commandChar.addEventListener('characteristicvaluechanged', onCmdNotification);
      setStatus('Connected: ' + (device.name || device.id));
      log('Connected.');
      btnConnect.disabled = true;
      btnDisconnect.disabled = false;
      btnStartRec.disabled = false;
      setCalButtonsEnabled(true);
      device.addEventListener('gattserverdisconnected', onDisconnected);
    } catch (err) {
      setStatus('Error: ' + err.message);
      log('Connect error: ' + err.message);
    }
  }

  function onDisconnected() {
    setStatus('Disconnected.');
    log('Device disconnected.');
    btnConnect.disabled = false;
    btnDisconnect.disabled = true;
    btnStartRec.disabled = true;
    btnStopRec.disabled = true;
    setCalButtonsEnabled(false);
    device = null;
    server = null;
    dataChar = null;
    commandChar = null;
  }

  async function disconnect() {
    if (!device || !device.gatt.connected) return;
    try {
      if (dataChar) {
        try { await dataChar.stopNotifications(); } catch (_) {}
      }
      if (commandChar) {
        try { await commandChar.stopNotifications(); } catch (_) {}
      }
      device.gatt.disconnect();
    } catch (_) {}
    onDisconnected();
  }

  async function sendCommand(cmd) {
    if (!commandChar) return;
    const enc = new TextEncoder().encode(cmd);
    await commandChar.writeValue(enc);
    log('>> ' + cmd);
  }

  async function startRecording() {
    buffer = [];
    await sendCommand('ALL_START');
    setStatus('Recording...');
    btnStartRec.disabled = true;
    btnStopRec.disabled = false;
    updateSamplesLabel();
  }

  async function stopRecording() {
    await sendCommand('ALL_STOP');
    setStatus('Stopped. Run CMJ or start a new recording.');
    btnStartRec.disabled = false;
    btnStopRec.disabled = true;
    updateSamplesLabel();
  }

  async function runCmj() {
    if (buffer.length === 0) {
      resultsEl.textContent = 'No data. Record a jump first (Start recording → jump → Stop recording).';
      return;
    }
    const input = bufferToCmjInput();
    const apiBase = window.location.origin;
    try {
      resultsEl.textContent = 'Running CMJ analysis...';
      const res = await fetch(apiBase + '/api/cmj', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(input)
      });
      const result = await parseJsonResponse(res);
      if (result._raw !== undefined) {
        resultsEl.textContent = 'CMJ error: ' + res.status + ' – ' + (result._raw || '').substring(0, 300);
        log('CMJ error: ' + res.status);
        return;
      }
      if (!res.ok) {
        resultsEl.textContent = 'CMJ error: ' + (result.error || res.statusText);
        return;
      }
      let out = '';
      if (result.points) {
        out += 'Points:\n';
        result.points.forEach(function (p) {
          out += '  ' + p.point_type + ': t=' + p.time + ', F=' + p.total_force + '\n';
        });
        out += '\n';
      }
      if (result.areas) {
        out += 'Phases:\n';
        result.areas.forEach(function (a) {
          out += '  ' + a.phase + ': ' + a.start_time + ' – ' + a.end_time + ' s\n';
        });
        out += '\n';
      }
      if (result.analysis) {
        out += 'Analysis:\n';
        Object.keys(result.analysis).forEach(function (k) {
          const v = result.analysis[k];
          if (v != null) out += '  ' + k + ': ' + v + '\n';
        });
      }
      resultsEl.textContent = out || JSON.stringify(result, null, 2);
    } catch (err) {
      resultsEl.textContent = 'Error: ' + err.message;
      log('CMJ error: ' + err.message);
    }
  }

  function clearAll() {
    buffer = [];
    updateChartFromBuffer();
    resultsEl.textContent = '';
    setStatus('Buffer cleared.');
  }

  async function refreshRecordingsList() {
    const apiBase = window.location.origin;
    try {
      const res = await fetch(apiBase + '/api/recordings');
      const data = await parseJsonResponse(res);
      if (data._raw !== undefined || !res.ok) return;
      const list = data.recordings || [];
      loadSelect.innerHTML = '<option value="">-- select --</option>';
      list.forEach(function (r) {
        const opt = document.createElement('option');
        opt.value = r.name;
        opt.textContent = r.name + ' (' + (r.sample_count || 0) + ' samples)';
        loadSelect.appendChild(opt);
      });
    } catch (_) {}
  }

  async function saveRecording() {
    if (buffer.length === 0) {
      resultsEl.textContent = 'No data to save. Record first or load a recording.';
      return;
    }
    const apiBase = window.location.origin;
    try {
      const res = await fetch(apiBase + '/api/save', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ samples: buffer })
      });
      const data = await parseJsonResponse(res);
      if (data._raw !== undefined) {
        resultsEl.textContent = 'Save error: ' + res.status + ' – server returned non-JSON. Restart the server (python serve_web.py --https).';
        log('Save error: ' + res.status + ' ' + (data._raw || '').substring(0, 200));
        return;
      }
      if (!res.ok) {
        resultsEl.textContent = 'Save error: ' + (data.error || res.statusText);
        return;
      }
      setStatus('Saved: ' + (data.name || '') + ' (' + (data.sample_count || 0) + ' samples)');
      log('Saved ' + (data.name || '') + ' (' + (data.sample_count || 0) + ' samples)');
      refreshRecordingsList();
    } catch (err) {
      resultsEl.textContent = 'Save error: ' + err.message;
      log('Save error: ' + err.message);
    }
  }

  async function loadRecording() {
    const name = loadSelect.value;
    if (!name) {
      resultsEl.textContent = 'Select a recording to load.';
      return;
    }
    const apiBase = window.location.origin;
    try {
      const res = await fetch(apiBase + '/api/load?name=' + encodeURIComponent(name));
      const data = await parseJsonResponse(res);
      if (data._raw !== undefined) {
        resultsEl.textContent = 'Load error: ' + res.status + ' – server returned non-JSON. Restart the server.';
        return;
      }
      if (!res.ok) {
        resultsEl.textContent = 'Load error: ' + (data.error || res.statusText);
        return;
      }
      const samples = data.samples || [];
      buffer = samples.map(function (row) { return row.slice ? row.slice() : row; });
      updateChartFromBuffer();
      setStatus('Loaded: ' + name + ' (' + buffer.length + ' samples). You can Run CMJ.');
      log('Loaded ' + name + ' (' + buffer.length + ' samples)');
    } catch (err) {
      resultsEl.textContent = 'Load error: ' + err.message;
      log('Load error: ' + err.message);
    }
  }

  function initChart() {
    const ctx = el('chart').getContext('2d');
    chart = new Chart(ctx, {
      type: 'line',
      data: {
        labels: [],
        datasets: [
          {
            label: 'Left',
            data: [],
            borderColor: '#4dabf7',
            backgroundColor: 'rgba(77, 171, 247, 0.08)',
            fill: true,
            tension: 0,
            pointRadius: 0
          },
          {
            label: 'Right',
            data: [],
            borderColor: '#ffa94d',
            backgroundColor: 'rgba(255, 169, 77, 0.08)',
            fill: true,
            tension: 0,
            pointRadius: 0
          },
          {
            label: 'Total',
            data: [],
            borderColor: '#e94560',
            backgroundColor: 'rgba(233, 69, 96, 0.08)',
            fill: true,
            tension: 0,
            pointRadius: 0
          }
        ]
      },
      options: {
        responsive: true,
        maintainAspectRatio: false,
        animation: false,
        interaction: { intersect: false, mode: 'index' },
        scales: {
          x: { title: { display: true, text: 'Time (s)' }, ticks: { maxTicksLimit: 12 } },
          y: { title: { display: true, text: 'Force (10g units)' } }
        }
      }
    });
    chartIntervalId = setInterval(updateChartFromBuffer, CHART_INTERVAL_MS);
  }

  function sendCalCommand(cmd) {
    if (!commandChar) return;
    sendCommand(cmd);
  }

  el('localTare').addEventListener('click', function () { sendCalCommand('LOCAL_CAL_TARE'); });
  el('localShow').addEventListener('click', function () { sendCalCommand('LOCAL_CAL_SHOW'); });
  el('localRead').addEventListener('click', function () { sendCalCommand('LOCAL_CAL_READ'); });
  el('localClear').addEventListener('click', function () { sendCalCommand('LOCAL_CAL_CLEAR'); });
  el('localPts').addEventListener('click', function () { sendCalCommand('LOCAL_CAL_POINTS'); });
  el('localAdd').addEventListener('click', function () {
    const kg = parseFloat(el('localAddKg').value);
    if (isNaN(kg) || kg <= 0) { localCalOut.textContent = 'Enter a valid kg > 0'; return; }
    sendCalCommand('LOCAL_CAL_ADD_' + kg);
  });
  el('localAddChBtn').addEventListener('click', function () {
    const ch = el('localAddCh').value;
    const kg = parseFloat(el('localAddChKg').value);
    if (isNaN(kg) || kg <= 0) { localCalOut.textContent = 'Enter a valid kg > 0'; return; }
    sendCalCommand('LOCAL_CAL_ADD_CH_' + ch + '_' + kg);
  });

  el('remoteTare').addEventListener('click', function () { sendCalCommand('REMOTE_CAL_TARE'); });
  el('remoteShow').addEventListener('click', function () { sendCalCommand('REMOTE_CAL_SHOW'); });
  el('remoteRead').addEventListener('click', function () { sendCalCommand('REMOTE_CAL_READ'); });
  el('remoteClear').addEventListener('click', function () { sendCalCommand('REMOTE_CAL_CLEAR'); });
  el('remotePts').addEventListener('click', function () { sendCalCommand('REMOTE_CAL_POINTS'); });
  el('remoteAdd').addEventListener('click', function () {
    const kg = parseFloat(el('remoteAddKg').value);
    if (isNaN(kg) || kg <= 0) { remoteCalOut.textContent = 'Enter a valid kg > 0'; return; }
    sendCalCommand('REMOTE_CAL_ADD_' + kg);
  });
  el('remoteAddChBtn').addEventListener('click', function () {
    const ch = el('remoteAddCh').value;
    const kg = parseFloat(el('remoteAddChKg').value);
    if (isNaN(kg) || kg <= 0) { remoteCalOut.textContent = 'Enter a valid kg > 0'; return; }
    sendCalCommand('REMOTE_CAL_ADD_CH_' + ch + '_' + kg);
  });

  btnConnect.addEventListener('click', connect);
  btnDisconnect.addEventListener('click', disconnect);
  btnStartRec.addEventListener('click', startRecording);
  btnStopRec.addEventListener('click', stopRecording);
  btnRunCmj.addEventListener('click', runCmj);
  btnClear.addEventListener('click', clearAll);
  btnSave.addEventListener('click', saveRecording);
  btnLoad.addEventListener('click', loadRecording);

  initChart();
  refreshRecordingsList();
})();
