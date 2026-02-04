(function () {
  'use strict';

  const SERVICE_UUID = '12345678-1234-1234-1234-123456789abc';
  const DATA_UUID = '87654321-4321-4321-4321-cba987654321';
  const CMD_UUID = '11111111-2222-3333-4444-555555555555';

  const DEVICE_NAME = 'LoadCell_BLE_Server';
  const SAMPLE_RATE_HZ = 1000;
  const CHART_UPDATE_INTERVAL_MS = 80;

  let chartPaused = false;
  let lastCMJResult = null;  // { points: [...], areas: [...], analysis: {...} }
  let clipStartTime = null;  // seconds (drag range)
  let clipEndTime = null;
  let clipDragging = null;   // 'left' | 'right' | null
  let clipOverlayEl = null;
  let clipFillEl = null;
  let clipHandleLeftEl = null;
  let clipHandleRightEl = null;
  let clipRangeLabelEl = null;

  let device = null;
  let server = null;
  let dataChar = null;
  let cmdChar = null;
  let sampleCount = 0;

  const buffers = {
    time: [],
    left_force: [],
    right_force: [],
    total_force: []
  };

  let chart = null;
  let chartUpdateTimer = null;

  const logEl = document.getElementById('log');
  const statusEl = document.getElementById('status');
  const btnConnect = document.getElementById('btnConnect');
  const btnDisconnect = document.getElementById('btnDisconnect');
  const analysisResult = document.getElementById('analysisResult');
  const fileLoad = document.getElementById('fileLoad');
  const chartPauseLabel = document.getElementById('chartPauseLabel');
  const btnChartPause = document.getElementById('btnChartPause');

  function log(msg) {
    const line = new Date().toLocaleTimeString() + ' ' + msg;
    logEl.textContent += line + '\n';
    logEl.scrollTop = logEl.scrollHeight;
  }

  function setStatus(text) {
    if (statusEl) statusEl.textContent = text;
  }

  function parseDataPacket(buffer) {
    const view = new DataView(buffer);
    if (view.byteLength < 1) return [];
    const n = view.getUint8(0);
    if (n < 1 || n > 10) return [];
    const samples = [];
    const dt = 1 / SAMPLE_RATE_HZ;
    for (let i = 0; i < n; i++) {
      const offset = 1 + i * 16;
      if (offset + 16 > view.byteLength) break;
      const L1 = view.getInt16(offset + 0, true);
      const L2 = view.getInt16(offset + 2, true);
      const L3 = view.getInt16(offset + 4, true);
      const L4 = view.getInt16(offset + 6, true);
      const R5 = view.getInt16(offset + 8, true);
      const R6 = view.getInt16(offset + 10, true);
      const R7 = view.getInt16(offset + 12, true);
      const R8 = view.getInt16(offset + 14, true);
      const left = L1 + L2 + L3 + L4;
      const right = R5 + R6 + R7 + R8;
      const total = left + right;
      const t = sampleCount * dt;
      sampleCount++;
      samples.push({ t, left, right, total });
    }
    return samples;
  }

  function pushSamples(samples) {
    if (chartPaused) return;
    for (const s of samples) {
      buffers.time.push(s.t);
      buffers.left_force.push(s.left);
      buffers.right_force.push(s.right);
      buffers.total_force.push(s.total);
    }
  }

  function initChart() {
    const ctx = document.getElementById('chart').getContext('2d');
    chart = new Chart(ctx, {
      type: 'line',
      data: {
        datasets: [
          { label: 'Left (local)', data: [], borderColor: '#4ade80', backgroundColor: 'transparent', fill: false, tension: 0, pointRadius: 0 },
          { label: 'Right (remote)', data: [], borderColor: '#f87171', backgroundColor: 'transparent', fill: false, tension: 0, pointRadius: 0 },
          { label: 'Total', data: [], borderColor: '#60a5fa', backgroundColor: 'transparent', fill: false, tension: 0, pointRadius: 0 }
        ]
      },
      options: {
        responsive: true,
        maintainAspectRatio: false,
        animation: false,
        scales: {
          x: { type: 'linear', title: { display: true, text: 'Time (s)' }, min: 0 },
          y: { title: { display: true, text: 'Force (10g units)' } }
        },
        plugins: { legend: { display: true } }
      }
    });
  }

  function updateChartFromBuffers(forceUpdate) {
    if (!chart || (!forceUpdate && chartPaused)) return;
    const n = buffers.time.length;
    if (n === 0) {
      chart.data.datasets[0].data = [];
      chart.data.datasets[1].data = [];
      chart.data.datasets[2].data = [];
      if (chart.options.scales && chart.options.scales.x) {
        chart.options.scales.x.min = undefined;
        chart.options.scales.x.max = undefined;
      }
    } else {
      if (clipStartTime == null) {
        syncClipRangeToFull();
        updateClipOverlay();
      }
      const step = Math.max(1, Math.floor(n / 2000));
      const left = [];
      const right = [];
      const total = [];
      for (let i = 0; i < n; i += step) {
        left.push({ x: buffers.time[i], y: buffers.left_force[i] });
        right.push({ x: buffers.time[i], y: buffers.right_force[i] });
        total.push({ x: buffers.time[i], y: buffers.total_force[i] });
      }
      chart.data.datasets[0].data = left;
      chart.data.datasets[1].data = right;
      chart.data.datasets[2].data = total;
      const tMin = buffers.time[0];
      const tMax = buffers.time[n - 1];
      if (chart.options.scales && chart.options.scales.x) {
        chart.options.scales.x.min = tMin;
        chart.options.scales.x.max = tMax;
      }
    }
    chart.update('none');
  }

  function clearChart() {
    buffers.time = [];
    buffers.left_force = [];
    buffers.right_force = [];
    buffers.total_force = [];
    lastCMJResult = null;
    clipStartTime = null;
    clipEndTime = null;
    if (chart && chart.data.datasets.length > 3) {
      chart.data.datasets = chart.data.datasets.slice(0, 3);
    }
    updateClipOverlay();
    updateChartFromBuffers();
    if (analysisResult) analysisResult.textContent = '(no analysis yet)';
    if (clipRangeLabelEl) clipRangeLabelEl.textContent = '';
    log('Chart cleared.');
  }

  // CMJ point types → chart color and order for legend
  const CMJ_POINT_STYLES = [
    { type: 'Start of Unload', color: '#22d3ee', border: '#0e7490' },
    { type: 'Start of Yield', color: '#a78bfa', border: '#6d28d9' },
    { type: 'Braking Point', color: '#f97316', border: '#c2410c' },
    { type: 'Zero Velocity Point', color: '#eab308', border: '#a16207' },
    { type: 'Lifting Up', color: '#22c55e', border: '#15803d' },
    { type: 'Takeoff Point', color: '#ef4444', border: '#b91c1c' },
    { type: 'Landing Point', color: '#ec4899', border: '#be185d' }
  ];

  function applyCMJPointsToChart(points) {
    if (!chart || !points || !points.length) return;
    const g = 9.81;
    // Convert force from Newtons (from server) to chart units (10g) so points sit on the total curve
    function toChartForce(N) { return (N / g) * 100; }
    const valid = points.filter(function (p) { return p.time != null && p.total_force != null; });
    if (valid.length === 0) return;
    while (chart.data.datasets.length > 3) chart.data.datasets.pop();
    CMJ_POINT_STYLES.forEach(function (style) {
      const p = points.find(function (pt) { return pt.point_type === style.type; });
      if (!p || p.time == null || p.total_force == null) return;
      chart.data.datasets.push({
        label: style.type,
        data: [{ x: p.time, y: toChartForce(p.total_force) }],
        type: 'scatter',
        pointRadius: 10,
        pointHoverRadius: 14,
        pointBackgroundColor: style.color,
        pointBorderColor: style.border,
        pointBorderWidth: 2,
        showLine: false
      });
    });
    chart.update('none');
  }

  function getTimeRange() {
    const n = buffers.time.length;
    if (n === 0) return { min: 0, max: 0 };
    return { min: buffers.time[0], max: buffers.time[n - 1] };
  }

  function syncClipRangeToFull() {
    const r = getTimeRange();
    if (r.min === r.max && buffers.time.length === 0) {
      clipStartTime = null;
      clipEndTime = null;
      return;
    }
    clipStartTime = r.min;
    clipEndTime = r.max;
  }

  function updateClipOverlay() {
    if (!clipOverlayEl || !clipFillEl || !clipHandleLeftEl || !clipHandleRightEl) return;
    const n = buffers.time.length;
    if (n === 0 || clipStartTime == null || clipEndTime == null) {
      clipOverlayEl.style.display = 'none';
      return;
    }
    clipOverlayEl.style.display = 'block';
    const r = getTimeRange();
    const span = r.max - r.min || 1;
    const pctLeft = Math.max(0, Math.min(1, (clipStartTime - r.min) / span));
    const pctRight = Math.max(0, Math.min(1, (clipEndTime - r.min) / span));
    const leftPct = Math.min(pctLeft, pctRight) * 100;
    const rightPct = Math.max(pctLeft, pctRight) * 100;
    clipFillEl.style.left = leftPct + '%';
    clipFillEl.style.width = (rightPct - leftPct) + '%';
    clipHandleLeftEl.style.left = leftPct + '%';
    clipHandleRightEl.style.left = '';
    clipHandleRightEl.style.right = (100 - rightPct) + '%';
    if (clipRangeLabelEl) {
      clipRangeLabelEl.textContent = 'Clip: ' + clipStartTime.toFixed(2) + ' s – ' + clipEndTime.toFixed(2) + ' s';
    }
  }

  function clipPositionToTime(clientX) {
    if (!clipOverlayEl) return 0;
    const rect = clipOverlayEl.getBoundingClientRect();
    const pct = (clientX - rect.left) / rect.width;
    const r = getTimeRange();
    return r.min + pct * (r.max - r.min);
  }

  function initClipOverlay() {
    clipOverlayEl = document.getElementById('clipOverlay');
    clipFillEl = document.getElementById('clipFill');
    clipHandleLeftEl = document.getElementById('clipHandleLeft');
    clipHandleRightEl = document.getElementById('clipHandleRight');
    clipRangeLabelEl = document.getElementById('clipRangeLabel');
    if (!clipHandleLeftEl || !clipHandleRightEl) return;

    function onMouseMove(e) {
      if (!clipDragging) return;
      const t = clipPositionToTime(e.clientX);
      const r = getTimeRange();
      if (clipDragging === 'left') {
        clipStartTime = Math.max(r.min, Math.min(clipEndTime - 0.01, t));
      } else {
        clipEndTime = Math.min(r.max, Math.max(clipStartTime + 0.01, t));
      }
      updateClipOverlay();
    }
    function onMouseUp() {
      clipDragging = null;
      document.removeEventListener('mousemove', onMouseMove);
      document.removeEventListener('mouseup', onMouseUp);
    }
    clipHandleLeftEl.addEventListener('mousedown', function (e) {
      e.preventDefault();
      clipDragging = 'left';
      document.addEventListener('mousemove', onMouseMove);
      document.addEventListener('mouseup', onMouseUp);
    });
    clipHandleRightEl.addEventListener('mousedown', function (e) {
      e.preventDefault();
      clipDragging = 'right';
      document.addEventListener('mousemove', onMouseMove);
      document.addEventListener('mouseup', onMouseUp);
    });
  }

  function applyClip() {
    const n = buffers.time.length;
    if (n === 0) {
      log('No data to clip.');
      return;
    }
    if (clipStartTime == null || clipEndTime == null) {
      syncClipRangeToFull();
    }
    let i0 = 0;
    while (i0 < n && buffers.time[i0] < clipStartTime) i0++;
    let i1 = i0;
    while (i1 < n && buffers.time[i1] <= clipEndTime) i1++;
    if (i0 >= i1) {
      log('Clip range is empty. Drag handles to select a segment.');
      return;
    }
    buffers.time = buffers.time.slice(i0, i1);
    buffers.left_force = buffers.left_force.slice(i0, i1);
    buffers.right_force = buffers.right_force.slice(i0, i1);
    buffers.total_force = buffers.total_force.slice(i0, i1);
    lastCMJResult = null;
    while (chart.data.datasets.length > 3) chart.data.datasets.pop();
    syncClipRangeToFull();
    updateClipOverlay();
    updateChartFromBuffers(true);
    if (analysisResult) analysisResult.textContent = '(no analysis yet)';
    log('Clipped to ' + buffers.time.length + ' samples (' + clipStartTime.toFixed(2) + ' s – ' + clipEndTime.toFixed(2) + ' s).');
  }

  function startChartUpdates() {
    if (chartUpdateTimer) return;
    chartUpdateTimer = setInterval(updateChartFromBuffers, CHART_UPDATE_INTERVAL_MS);
  }

  function stopChartUpdates() {
    if (chartUpdateTimer) {
      clearInterval(chartUpdateTimer);
      chartUpdateTimer = null;
    }
  }

  function setChartPaused(paused) {
    chartPaused = paused;
    if (paused) {
      stopChartUpdates();
      if (chartPauseLabel) chartPauseLabel.textContent = 'Chart: paused';
      if (btnChartPause) { btnChartPause.textContent = 'Resume chart'; btnChartPause.classList.add('primary'); }
    } else {
      startChartUpdates();
      if (chartPauseLabel) chartPauseLabel.textContent = 'Chart: live';
      if (btnChartPause) { btnChartPause.textContent = 'Pause chart'; btnChartPause.classList.remove('primary'); }
    }
  }

  async function connect() {
    if (!navigator.bluetooth) {
      log('Web Bluetooth not supported.');
      return;
    }
    try {
      log('Requesting device...');
      device = await navigator.bluetooth.requestDevice({
        filters: [{ name: DEVICE_NAME }],
        optionalServices: [SERVICE_UUID]
      });
      log('Connecting to GATT...');
      server = await device.gatt.connect();
      const service = await server.getPrimaryService(SERVICE_UUID);
      dataChar = await service.getCharacteristic(DATA_UUID);
      cmdChar = await service.getCharacteristic(CMD_UUID);

      sampleCount = 0;
      buffers.time = [];
      buffers.left_force = [];
      buffers.right_force = [];
      buffers.total_force = [];

      cmdChar.addEventListener('characteristicvaluechanged', function (ev) {
        const val = ev.target.value;
        if (!val || !val.buffer) return;
        const str = new TextDecoder().decode(val);
        try {
          const json = JSON.parse(str);
          log('<< ' + JSON.stringify(json));
        } catch (_) {
          log('<< ' + str);
        }
      });
      await cmdChar.startNotifications();

      dataChar.addEventListener('characteristicvaluechanged', function (ev) {
        const val = ev.target.value;
        if (!val || !val.buffer) return;
        const samples = parseDataPacket(val.buffer);
        if (samples.length) pushSamples(samples);
      });
      await dataChar.startNotifications();

      setStatus('Connected');
      btnConnect.disabled = true;
      btnDisconnect.disabled = false;
      startChartUpdates();
      log('Connected. Notifications enabled.');
    } catch (e) {
      log('Connect error: ' + e.message);
      setStatus('Error');
    }
  }

  async function disconnect() {
    stopChartUpdates();
    try {
      if (device && device.gatt.connected) {
        await device.gatt.disconnect();
      }
    } catch (_) {}
    device = null;
    server = null;
    dataChar = null;
    cmdChar = null;
    setStatus('Disconnected');
    btnConnect.disabled = false;
    btnDisconnect.disabled = true;
    log('Disconnected.');
  }

  async function sendCommand(cmd) {
    if (!cmdChar) {
      log('Not connected.');
      return;
    }
    cmd = String(cmd).trim();
    if (!cmd) return;
    try {
      await cmdChar.writeValue(new TextEncoder().encode(cmd));
      log('>> ' + cmd);
    } catch (e) {
      log('Command error: ' + e.message);
    }
  }

  function saveData() {
    const payload = {
      time: buffers.time.slice(),
      left_force: buffers.left_force.slice(),
      right_force: buffers.right_force.slice(),
      total_force: buffers.total_force.slice(),
      meta: {
        savedAt: new Date().toISOString(),
        duration_s: buffers.time.length ? buffers.time[buffers.time.length - 1] - buffers.time[0] : 0,
        sampleCount: buffers.time.length
      }
    };
    const blob = new Blob([JSON.stringify(payload)], { type: 'application/json' });
    const a = document.createElement('a');
    a.href = URL.createObjectURL(blob);
    a.download = 'jumptest_' + new Date().toISOString().slice(0, 19).replace(/[-:T]/g, '') + '.json';
    a.click();
    URL.revokeObjectURL(a.href);
    log('Saved ' + buffers.time.length + ' samples.');
  }

  function loadData() {
    fileLoad.click();
  }

  function onFileSelected(ev) {
    const file = ev.target.files[0];
    if (!file) return;
    const reader = new FileReader();
    reader.onload = function () {
      try {
        const payload = JSON.parse(reader.result);
        const t = payload.time || [];
        const left = payload.left_force || [];
        const right = payload.right_force || [];
        const total = payload.total_force || [];
        const n = Math.min(t.length, left.length, right.length, total.length);
        if (n === 0) {
          log('No valid data in file.');
          return;
        }
        buffers.time = t.slice(0, n);
        buffers.left_force = left.slice(0, n);
        buffers.right_force = right.slice(0, n);
        buffers.total_force = total.slice(0, n);
        lastCMJResult = null;
        while (chart.data.datasets.length > 3) chart.data.datasets.pop();
        syncClipRangeToFull();
        updateClipOverlay();
        updateChartFromBuffers();
        log('Loaded ' + n + ' samples.');
      } catch (e) {
        log('Load error: ' + e.message);
      }
    };
    reader.readAsText(file);
    ev.target.value = '';
  }

  function exportForCMJ() {
    const clipped = getClippedData();
    const n = clipped.index.length;
    if (n === 0) {
      log('No data to export. Add data and optionally set Start/End to clip.');
      return;
    }
    const g = 9.81;
    const toNewton = function (v) { return (v / 100) * g; };
    const payload = {
      index: clipped.index.slice(),
      left_force: clipped.left_force.map(toNewton),
      right_force: clipped.right_force.map(toNewton),
      total_force: clipped.total_force.map(toNewton)
    };
    const blob = new Blob([JSON.stringify(payload)], { type: 'application/json' });
    const a = document.createElement('a');
    a.href = URL.createObjectURL(blob);
    a.download = 'cmj_export_' + new Date().toISOString().slice(0, 19).replace(/[-:T]/g, '') + '.json';
    a.click();
    URL.revokeObjectURL(a.href);
    log('Exported for CMJ (' + n + ' samples, force in N).');
  }

  const CMJ_SERVER_URL = 'http://localhost:8765';

  function getClipRange() {
    if (clipStartTime == null || clipEndTime == null || buffers.time.length === 0) return null;
    return [clipStartTime, clipEndTime];
  }

  function getClippedData() {
    const n = buffers.time.length;
    if (n === 0) return { index: [], left_force: [], right_force: [], total_force: [] };
    const range = getClipRange();
    let idx0 = 0;
    let idx1 = n;
    if (range) {
      const startT = range[0];
      const endT = range[1];
      while (idx0 < n && buffers.time[idx0] < startT) idx0++;
      idx1 = idx0;
      while (idx1 < n && buffers.time[idx1] <= endT) idx1++;
    }
    return {
      index: buffers.time.slice(idx0, idx1),
      left_force: buffers.left_force.slice(idx0, idx1),
      right_force: buffers.right_force.slice(idx0, idx1),
      total_force: buffers.total_force.slice(idx0, idx1)
    };
  }

  async function runCMJAnalysis() {
    const clipped = getClippedData();
    const n = clipped.index.length;
    if (n === 0) {
      log('No data to analyse. Record or load data, then drag the green handles to select a segment.');
      analysisResult.textContent = '(No data. Load or record data, then drag handles to select segment for CMJ.)';
      return;
    }
    // Send same format as run_cmj_analysis.py input: time + raw 10g forces + meta → server converts to N
    const payload = {
      time: clipped.index.slice(),
      left_force: clipped.left_force.slice(),
      right_force: clipped.right_force.slice(),
      total_force: clipped.total_force.slice(),
      meta: {}
    };
    try {
      const res = await fetch(CMJ_SERVER_URL + '/analyse', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(payload)
      });
      const result = await res.json();
      if (!res.ok) {
        const errMsg = (result && result.error) ? result.error : (res.status + ' ' + res.statusText);
        log('CMJ error: ' + errMsg);
        analysisResult.textContent = 'Error: ' + errMsg;
        return;
      }
      if (result && result.error) {
        log('CMJ error: ' + result.error);
        analysisResult.textContent = 'Error: ' + result.error;
        return;
      }
      lastCMJResult = result;
      analysisResult.textContent = JSON.stringify(result, null, 2);
      if (result.points && result.points.length) {
        applyCMJPointsToChart(result.points);
      } else {
        while (chart.data.datasets.length > 3) chart.data.datasets.pop();
        chart.update('none');
      }
      log('CMJ analysis done. Points drawn on chart.');
    } catch (e) {
      log('CMJ error: ' + e.message + ' (is server.py running on port 8765?)');
      analysisResult.textContent = 'Error: ' + e.message + '\n\nStart the analysis server: python server.py';
    }
  }

  function bindUi() {
    if (btnConnect) btnConnect.addEventListener('click', connect);
    if (btnDisconnect) btnDisconnect.addEventListener('click', disconnect);
    if (fileLoad) fileLoad.addEventListener('change', onFileSelected);
    if (btnChartPause) btnChartPause.addEventListener('click', function () {
      setChartPaused(!chartPaused);
    });

    document.getElementById('cmdAllStart').addEventListener('click', () => sendCommand('ALL_START'));
    document.getElementById('cmdAllStop').addEventListener('click', () => sendCommand('ALL_STOP'));
    document.getElementById('cmdStatus').addEventListener('click', () => sendCommand('STATUS'));
    document.getElementById('cmdBat').addEventListener('click', () => sendCommand('BAT'));
    document.getElementById('cmdLocalTare').addEventListener('click', () => sendCommand('LOCAL_CAL_TARE'));
    document.getElementById('cmdLocalShow').addEventListener('click', () => sendCommand('LOCAL_CAL_SHOW'));
    document.getElementById('cmdLocalPing').addEventListener('click', () => sendCommand('LOCAL_PING'));
    document.getElementById('cmdRemotePing').addEventListener('click', () => sendCommand('REMOTE_PING'));

    document.getElementById('cmdSend').addEventListener('click', function () {
      const input = document.getElementById('customCmd');
      sendCommand(input.value);
    });
    document.getElementById('customCmd').addEventListener('keydown', function (e) {
      if (e.key === 'Enter') sendCommand(this.value);
    });

    document.getElementById('btnSave').addEventListener('click', saveData);
    document.getElementById('btnLoad').addEventListener('click', loadData);
    document.getElementById('btnExportCMJ').addEventListener('click', exportForCMJ);
    document.getElementById('btnRunCMJ').addEventListener('click', runCMJAnalysis);
    if (document.getElementById('btnClearChart')) {
      document.getElementById('btnClearChart').addEventListener('click', clearChart);
    }
    if (document.getElementById('btnApplyClip')) {
      document.getElementById('btnApplyClip').addEventListener('click', applyClip);
    }
  }

  initChart();
  initClipOverlay();
  bindUi();
  setStatus('Disconnected');
  log('Ready. Use Connect (HTTPS or localhost required for Web Bluetooth).');
})();
