#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>
#include <LittleFS.h>

// ============================================================================
// PIN DEFINITIONS (All on the VIN side of ESP32 DOIT DevKit V1)
// ============================================================================
constexpr uint8_t PIN_SENSOR_INLET = 27;  // D27 -> Sensor 1 (Inlet YF-S201 Yellow wire)
constexpr uint8_t PIN_SENSOR_OUTLET = 26; // D26 -> Sensor 2 (Outlet YF-S201 Yellow wire)
constexpr uint8_t PIN_RELAY_VALVE = 25;   // D25 -> Relay IN1 (12V Solenoid Valve control)

// ============================================================================
// LOCAL WI-FI ACCESS POINT CREDENTIALS (100% Local - No Cloud / No Router Needed)
// ============================================================================
const char *AP_SSID = "Smart_Water_Leak_System";
const char *AP_PASS = "12345678"; // Minimum 8 characters required for WPA2 AP

// ============================================================================
// GLOBAL STATE & INTERRUPT PULSE COUNTERS
// ============================================================================
portMUX_TYPE pulseMux = portMUX_INITIALIZER_UNLOCKED;
volatile uint32_t pulseCount1 = 0;
volatile uint32_t pulseCount2 = 0;

uint32_t totalPulses1 = 0;
uint32_t totalPulses2 = 0;

float flowRate1_Lmin = 0.0f; // Inlet flow rate (L/min)
float flowRate2_Lmin = 0.0f; // Outlet flow rate (L/min)
float flowDiff_Lmin = 0.0f;  // Difference: Inlet - Outlet (L/min)
float totalVolume1_L = 0.0f; // Total Inlet volume (Liters)
float totalVolume2_L = 0.0f; // Total Outlet volume (Liters)

bool valveOpen = true;       // Logical state of solenoid valve (true = OPEN, false = CLOSED)
bool leakDetected = false;   // True when a confirmed leak has latched the valve shut
bool manualOverride = false; // True if user manually closed the valve from UI

uint32_t leakConditionStartMs = 0; // Timer for leak confirmation window
uint32_t lastSampleTimeMs = 0;
uint32_t lastPeriodicLogMs = 0;
uint32_t leakCountTotal = 0;

// ============================================================================
// PERSISTENT SETTINGS (Stored in ESP32 NVS Flash via Preferences)
// ============================================================================
Preferences prefs;

struct SystemConfig
{
  float leakThreshold_Lmin; // Flow difference (Q1 - Q2) that triggers a leak (default: 0.50 L/min)
  float minActiveFlow_Lmin; // Minimum inlet flow required to evaluate leak (default: 0.25 L/min)
  uint16_t confirmTimeSec;  // Seconds leak must persist before closing valve (default: 2 sec)
  float pulsesPerLiter1;    // Calibration factor for Sensor 1 (YF-S201 default: 450 pulses/L)
  float pulsesPerLiter2;    // Calibration factor for Sensor 2 (YF-S201 default: 450 pulses/L)
  bool relayActiveLow;      // Most 5-channel relay boards are Active-LOW (LOW = Relay ON)
} config;

// ============================================================================
// LOCAL LOGGING & WEB SERVER
// ============================================================================
WebServer server(80);
bool littleFsReady = false;
const char *LOG_FILE_PATH = "/leak_logs.csv";

// Keep recent events in RAM for instant display on the UI
constexpr size_t MAX_RAM_EVENTS = 20;
String recentEvents[MAX_RAM_EVENTS];
size_t recentEventCount = 0;

// ============================================================================
// INTERRUPT SERVICE ROUTINES (ISRs)
// ============================================================================
void IRAM_ATTR onSensor1Pulse()
{
  portENTER_CRITICAL_ISR(&pulseMux);
  pulseCount1++;
  portEXIT_CRITICAL_ISR(&pulseMux);
}

void IRAM_ATTR onSensor2Pulse()
{
  portENTER_CRITICAL_ISR(&pulseMux);
  pulseCount2++;
  portEXIT_CRITICAL_ISR(&pulseMux);
}

// ============================================================================
// HELPER FUNCTIONS: RELAY, STORAGE & LOGGING
// ============================================================================
void applyRelayOutput(bool openValve)
{
  valveOpen = openValve;
  // Since the Solenoid Valve is wired to Relay NO (Normally Open):
  // Relay ON (energized) -> Solenoid Valve OPEN (water can flow)
  // Relay OFF            -> Solenoid Valve CLOSED (water shut off)
  bool relayEnergized = openValve;
  if (config.relayActiveLow)
  {
    // On a 5V Active-LOW relay connected to a 3.3V ESP32 pin:
    // OUTPUT HIGH (3.3V) still leaves 5.0V - 3.3V = 1.7V across the relay LED,
    // which keeps the green light glowing!
    // Switching pinMode to INPUT (open-drain high-impedance) drops current to 0 mA
    // so the green light and relay switch 100% OFF!
    if (relayEnergized)
    {
      pinMode(PIN_RELAY_VALVE, OUTPUT);
      digitalWrite(PIN_RELAY_VALVE, LOW); // Green light ON, Relay ON (Valve OPEN)
    }
    else
    {
      digitalWrite(PIN_RELAY_VALVE, HIGH);
      pinMode(PIN_RELAY_VALVE, INPUT); // Open circuit (0 mA) -> Green light OFF, Relay OFF (Valve CLOSED)
    }
  }
  else
  {
    pinMode(PIN_RELAY_VALVE, OUTPUT);
    digitalWrite(PIN_RELAY_VALVE, relayEnergized ? HIGH : LOW);
  }
}

String formatUptime(uint32_t ms)
{
  uint32_t totalSec = ms / 1000;
  uint32_t hrs = totalSec / 3600;
  uint32_t mins = (totalSec % 3600) / 60;
  uint32_t secs = totalSec % 60;
  char buf[24];
  snprintf(buf, sizeof(buf), "%02lu:%02lu:%02lu", (unsigned long)hrs, (unsigned long)mins, (unsigned long)secs);
  return String(buf);
}

void logEventLocal(const String &eventType, float q1, float q2, float diff, bool isValveOpen)
{
  String timeStr = formatUptime(millis());
  char line[160];
  snprintf(line, sizeof(line), "%s,%s,%.2f,%.2f,%.2f,%s",
           timeStr.c_str(),
           eventType.c_str(),
           q1, q2, diff,
           isValveOpen ? "OPEN" : "CLOSED");

  // Store in RAM ring buffer (newest first)
  if (recentEventCount < MAX_RAM_EVENTS)
  {
    for (int i = (int)recentEventCount; i > 0; i--)
    {
      recentEvents[i] = recentEvents[i - 1];
    }
    recentEvents[0] = String(line);
    recentEventCount++;
  }
  else
  {
    for (int i = (int)MAX_RAM_EVENTS - 1; i > 0; i--)
    {
      recentEvents[i] = recentEvents[i - 1];
    }
    recentEvents[0] = String(line);
  }

  // Append to local flash CSV file (LittleFS)
  if (littleFsReady)
  {
    File f = LittleFS.open(LOG_FILE_PATH, FILE_APPEND);
    if (f)
    {
      // Keep file size bounded under 64 KB so flash never fills up
      if (f.size() > 65536)
      {
        f.close();
        LittleFS.remove(LOG_FILE_PATH);
        f = LittleFS.open(LOG_FILE_PATH, FILE_WRITE);
        if (f)
        {
          f.println("Uptime,Event,Inlet_Lmin,Outlet_Lmin,Diff_Lmin,Valve_State");
        }
      }
      if (f)
      {
        f.println(line);
        f.close();
      }
    }
  }
}

void loadSettings()
{
  prefs.begin("water-leak", false);
  config.leakThreshold_Lmin = prefs.getFloat("thresh", 0.50f);
  config.minActiveFlow_Lmin = prefs.getFloat("minflow", 0.25f);
  config.confirmTimeSec = prefs.getUShort("confSec", 2);
  config.pulsesPerLiter1 = prefs.getFloat("ppl1", 450.0f);
  config.pulsesPerLiter2 = prefs.getFloat("ppl2", 450.0f);
  config.relayActiveLow = prefs.getBool("actLow", true);
}

void saveSettings()
{
  prefs.putFloat("thresh", config.leakThreshold_Lmin);
  prefs.putFloat("minflow", config.minActiveFlow_Lmin);
  prefs.putUShort("confSec", config.confirmTimeSec);
  prefs.putFloat("ppl1", config.pulsesPerLiter1);
  prefs.putFloat("ppl2", config.pulsesPerLiter2);
  prefs.putBool("actLow", config.relayActiveLow);
}

// ============================================================================
// EMBEDDED LOCAL DASHBOARD UI (100% Offline HTML5 + CSS + Canvas Chart + JS)
// ============================================================================
const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8" />
  <meta name="viewport" content="width=device-width, initial-scale=1.0" />
  <title>Smart Water Leakage Detection System</title>
  <style>
    :root {
      --bg: #0f172a;
      --card: #1e293b;
      --border: #334155;
      --text: #f8fafc;
      --muted: #94a3b8;
      --blue: #38bdf8;
      --green: #22c55e;
      --amber: #f59e0b;
      --red: #ef4444;
    }
    * { box-sizing: border-box; margin: 0; padding: 0; font-family: system-ui, -apple-system, sans-serif; }
    body { background: var(--bg); color: var(--text); padding: 16px; max-width: 1120px; margin: 0 auto; }
    header { display: flex; flex-wrap: wrap; justify-content: space-between; align-items: center; gap: 12px; margin-bottom: 16px; padding-bottom: 12px; border-bottom: 1px solid var(--border); }
    h1 { font-size: 1.35rem; font-weight: 700; display: flex; align-items: center; gap: 8px; }
    .badge { padding: 6px 14px; border-radius: 999px; font-weight: 700; font-size: 0.85rem; letter-spacing: 0.5px; }
    .badge-ok { background: rgba(34, 197, 94, 0.2); color: var(--green); border: 1px solid var(--green); }
    .badge-leak { background: rgba(239, 68, 68, 0.25); color: #fca5a5; border: 1px solid var(--red); animation: pulse 1s infinite; }
    @keyframes pulse { 0%, 100% { transform: scale(1); } 50% { transform: scale(1.04); } }

    .banner { padding: 14px 18px; border-radius: 12px; margin-bottom: 16px; display: flex; justify-content: space-between; align-items: center; flex-wrap: wrap; gap: 10px; font-weight: 600; }
    .banner-normal { background: rgba(34, 197, 94, 0.12); border: 1px solid rgba(34, 197, 94, 0.4); color: #bbf7d0; }
    .banner-alert { background: rgba(239, 68, 68, 0.22); border: 2px solid var(--red); color: #fecaca; }

    .grid { display: grid; grid-template-columns: repeat(auto-fit, minmax(230px, 1fr)); gap: 14px; margin-bottom: 16px; }
    .card { background: var(--card); border: 1px solid var(--border); border-radius: 12px; padding: 16px; }
    .card-title { font-size: 0.82rem; text-transform: uppercase; letter-spacing: 0.6px; color: var(--muted); margin-bottom: 8px; }
    .metric { font-size: 2rem; font-weight: 800; margin-bottom: 4px; }
    .sub { font-size: 0.82rem; color: var(--muted); }

    .layout-2col { display: grid; grid-template-columns: 1.5fr 1fr; gap: 16px; margin-bottom: 16px; }
    @media (max-width: 820px) { .layout-2col { grid-template-columns: 1fr; } }

    canvas { width: 100%; height: 240px; background: #0b1120; border-radius: 8px; border: 1px solid var(--border); display: block; }
    .btn-row { display: flex; flex-wrap: wrap; gap: 8px; margin-top: 12px; }
    button, .btn-link {
      cursor: pointer; border: none; border-radius: 8px; padding: 10px 14px;
      font-weight: 600; font-size: 0.88rem; transition: opacity 0.15s; text-decoration: none; display: inline-block;
    }
    button:hover, .btn-link:hover { opacity: 0.88; }
    .btn-green { background: var(--green); color: #052e16; }
    .btn-red { background: var(--red); color: #fff; }
    .btn-blue { background: var(--blue); color: #082f49; }
    .btn-slate { background: #475569; color: var(--text); }

    .form-row { display: flex; justify-content: space-between; align-items: center; margin-bottom: 10px; gap: 8px; font-size: 0.88rem; }
    .form-row input { width: 95px; padding: 6px 8px; border-radius: 6px; border: 1px solid var(--border); background: #0f172a; color: var(--text); text-align: right; }
    table { width: 100%; border-collapse: collapse; font-size: 0.84rem; margin-top: 8px; }
    th, td { text-align: left; padding: 8px 10px; border-bottom: 1px solid var(--border); }
    th { color: var(--muted); font-weight: 600; }
    .pill-open { color: var(--green); font-weight: 700; }
    .pill-closed { color: var(--red); font-weight: 700; }
  </style>
</head>
<body>

  <header>
    <div>
      <h1>💧 Smart Water Leakage Detection System</h1>
      <div class="sub">ESP32 Local Edge Controller • Pins: D27 (Inlet), D26 (Outlet), D25 (Relay) • Upt: <span id="uptime">00:00:00</span></div>
    </div>
    <div id="statusBadge" class="badge badge-ok">SYSTEM NORMAL</div>
  </header>

  <div id="alertBanner" class="banner banner-normal">
    <span id="bannerText">✅ Pipeline normal. Monitoring Sensor 1 (Inlet) and Sensor 2 (Outlet) in real time.</span>
    <button class="btn-green" onclick="sendCommand('/api/reset_alarm')">Reset Alarm &amp; Open Valve</button>
  </div>

  <div class="grid">
    <div class="card">
      <div class="card-title">Sensor 1 • Inlet Flow (D27)</div>
      <div class="metric" style="color: var(--blue)"><span id="flow1">0.00</span> <small style="font-size:0.9rem">L/min</small></div>
      <div class="sub">Volume: <span id="vol1">0.00</span> L • Pulses: <span id="pulses1">0</span></div>
    </div>

    <div class="card">
      <div class="card-title">Sensor 2 • Outlet Flow (D26)</div>
      <div class="metric" style="color: var(--green)"><span id="flow2">0.00</span> <small style="font-size:0.9rem">L/min</small></div>
      <div class="sub">Volume: <span id="vol2">0.00</span> L • Pulses: <span id="pulses2">0</span></div>
    </div>

    <div class="card">
      <div class="card-title">Flow Difference (Inlet − Outlet)</div>
      <div class="metric" id="diffMetric" style="color: var(--amber)"><span id="flowDiff">0.00</span> <small style="font-size:0.9rem">L/min</small></div>
      <div class="sub">Leak Threshold: <span id="threshLabel">0.50</span> L/min</div>
    </div>

    <div class="card">
      <div class="card-title">12V Solenoid Valve (D25)</div>
      <div class="metric" id="valveStateText" style="color: var(--green)">OPEN</div>
      <div class="sub">Leaks Caught: <span id="leakCount">0</span> • Relay: <span id="relayModeText">Active-LOW</span></div>
    </div>
  </div>

  <div class="layout-2col">
    <div class="card">
      <div class="card-title">Real-Time Sensor Output Graph (Last 60 Seconds)</div>
      <canvas id="flowChart" width="600" height="240"></canvas>
      <div class="btn-row">
        <button class="btn-green" onclick="sendCommand('/api/valve?state=open')">Open Solenoid Valve</button>
        <button class="btn-red" onclick="sendCommand('/api/valve?state=close')">Close Solenoid Valve</button>
        <button class="btn-slate" onclick="sendCommand('/api/reset_totals')">Reset Volume &amp; Pulse Counters</button>
      </div>
    </div>

    <div class="card">
      <div class="card-title">Live Calibration &amp; Settings (Saved in ESP32 Flash)</div>
      <div class="form-row">
        <label>Leak Threshold (L/min):</label>
        <input type="number" step="0.05" id="cfgThresh" value="0.50" />
      </div>
      <div class="form-row">
        <label>Min Active Flow (L/min):</label>
        <input type="number" step="0.05" id="cfgMinFlow" value="0.25" />
      </div>
      <div class="form-row">
        <label>Confirm Delay (seconds):</label>
        <input type="number" step="1" id="cfgConfirm" value="2" />
      </div>
      <div class="form-row">
        <label>Sensor 1 Pulses/Liter:</label>
        <input type="number" step="5" id="cfgPpl1" value="450" />
      </div>
      <div class="form-row">
        <label>Sensor 2 Pulses/Liter:</label>
        <input type="number" step="5" id="cfgPpl2" value="450" />
      </div>
      <div class="btn-row">
        <button class="btn-blue" onclick="saveSettings()">Save Settings</button>
        <button class="btn-slate" onclick="sendCommand('/api/toggle_relay_mode')">Flip Relay Polarity</button>
      </div>
    </div>
  </div>

  <div class="card">
    <div style="display:flex; justify-content:space-between; align-items:center; flex-wrap:wrap; gap:8px;">
      <div class="card-title" style="margin:0;">Local Event &amp; Telemetry Log (Stored Locally on ESP32)</div>
      <div class="btn-row" style="margin:0;">
        <a class="btn-link btn-blue" href="/api/download_csv">⬇ Download CSV Log</a>
        <button class="btn-slate" onclick="sendCommand('/api/clear_logs')">Clear Logs</button>
      </div>
    </div>
    <table>
      <thead>
        <tr>
          <th>Uptime</th>
          <th>Event</th>
          <th>Sensor 1 (L/min)</th>
          <th>Sensor 2 (L/min)</th>
          <th>Diff (L/min)</th>
          <th>Solenoid Valve</th>
        </tr>
      </thead>
      <tbody id="logTableBody">
        <tr><td colspan="6" class="sub">Waiting for events...</td></tr>
      </tbody>
    </table>
  </div>

  <script>
    const maxPoints = 60;
    const hist1 = [];
    const hist2 = [];
    let settingsLoaded = false;

    function drawChart(threshold) {
      const canvas = document.getElementById('flowChart');
      const ctx = canvas.getContext('2d');
      const w = canvas.width;
      const h = canvas.height;
      ctx.clearRect(0, 0, w, h);

      let maxVal = Math.max(5.0, threshold * 2);
      for (let i = 0; i < hist1.length; i++) {
        if (hist1[i] > maxVal) maxVal = hist1[i] * 1.2;
        if (hist2[i] > maxVal) maxVal = hist2[i] * 1.2;
      }

      // Grid lines
      ctx.strokeStyle = '#1e293b';
      ctx.lineWidth = 1;
      for (let g = 1; g <= 4; g++) {
        const y = h - (g / 5) * (h - 24) - 12;
        ctx.beginPath(); ctx.moveTo(36, y); ctx.lineTo(w - 10, y); ctx.stroke();
        ctx.fillStyle = '#64748b';
        ctx.font = '11px sans-serif';
        ctx.fillText(((g / 5) * maxVal).toFixed(1), 4, y + 4);
      }

      const plotLine = (arr, color) => {
        if (arr.length < 2) return;
        ctx.strokeStyle = color;
        ctx.lineWidth = 2.5;
        ctx.beginPath();
        for (let i = 0; i < arr.length; i++) {
          const x = 36 + (i / (maxPoints - 1)) * (w - 46);
          const y = h - 12 - (arr[i] / maxVal) * (h - 28);
          if (i === 0) ctx.moveTo(x, y);
          else ctx.lineTo(x, y);
        }
        ctx.stroke();
      };

      plotLine(hist1, '#38bdf8'); // Sensor 1 Blue
      plotLine(hist2, '#22c55e'); // Sensor 2 Green

      // Legend
      ctx.fillStyle = '#38bdf8'; ctx.fillText('● Sensor 1 (Inlet)', 45, 16);
      ctx.fillStyle = '#22c55e'; ctx.fillText('● Sensor 2 (Outlet)', 165, 16);
    }

    async function fetchTelemetry() {
      try {
        const res = await fetch('/api/status');
        const d = await res.json();

        document.getElementById('uptime').textContent = d.uptime;
        document.getElementById('flow1').textContent = d.flow1.toFixed(2);
        document.getElementById('flow2').textContent = d.flow2.toFixed(2);
        document.getElementById('flowDiff').textContent = d.diff.toFixed(2);
        document.getElementById('vol1').textContent = d.vol1.toFixed(2);
        document.getElementById('vol2').textContent = d.vol2.toFixed(2);
        document.getElementById('pulses1').textContent = d.pulses1;
        document.getElementById('pulses2').textContent = d.pulses2;
        document.getElementById('threshLabel').textContent = d.thresh.toFixed(2);
        document.getElementById('leakCount').textContent = d.leakCount;
        document.getElementById('relayModeText').textContent = d.relayActiveLow ? 'Active-LOW' : 'Active-HIGH';

        const valveEl = document.getElementById('valveStateText');
        if (d.valveOpen) {
          valveEl.textContent = 'OPEN';
          valveEl.style.color = 'var(--green)';
        } else {
          valveEl.textContent = 'CLOSED';
          valveEl.style.color = 'var(--red)';
        }

        const badge = document.getElementById('statusBadge');
        const banner = document.getElementById('alertBanner');
        const bannerText = document.getElementById('bannerText');

        if (d.leakDetected) {
          badge.className = 'badge badge-leak';
          badge.textContent = '🚨 LEAK DETECTED!';
          banner.className = 'banner banner-alert';
          bannerText.textContent = '🚨 WATER LEAK DETECTED! Inlet exceeds Outlet beyond threshold. 12V Solenoid Valve has been CLOSED automatically!';
        } else if (!d.valveOpen) {
          badge.className = 'badge badge-ok';
          badge.textContent = 'VALVE CLOSED (MANUAL)';
          banner.className = 'banner banner-normal';
          bannerText.textContent = '🔒 Solenoid Valve is currently closed manually. Click "Open Solenoid Valve" to resume water flow.';
        } else {
          badge.className = 'badge badge-ok';
          badge.textContent = 'SYSTEM NORMAL';
          banner.className = 'banner banner-normal';
          bannerText.textContent = '✅ Pipeline normal. Monitoring Sensor 1 (Inlet) and Sensor 2 (Outlet) continuously.';
        }

        if (!settingsLoaded) {
          document.getElementById('cfgThresh').value = d.thresh.toFixed(2);
          document.getElementById('cfgMinFlow').value = d.minFlow.toFixed(2);
          document.getElementById('cfgConfirm').value = d.confirmSec;
          document.getElementById('cfgPpl1').value = d.ppl1;
          document.getElementById('cfgPpl2').value = d.ppl2;
          settingsLoaded = true;
        }

        hist1.push(d.flow1);
        hist2.push(d.flow2);
        if (hist1.length > maxPoints) hist1.shift();
        if (hist2.length > maxPoints) hist2.shift();
        drawChart(d.thresh);

        // Render recent logs
        if (d.logs && d.logs.length > 0) {
          const tbody = document.getElementById('logTableBody');
          tbody.innerHTML = d.logs.map(row => {
            const p = row.split(',');
            if (p.length < 6) return '';
            const vClass = p[5] === 'OPEN' ? 'pill-open' : 'pill-closed';
            return `<tr>
              <td>${p[0]}</td>
              <td><strong>${p[1]}</strong></td>
              <td>${p[2]}</td>
              <td>${p[3]}</td>
              <td>${p[4]}</td>
              <td class="${vClass}">${p[5]}</td>
            </tr>`;
          }).join('');
        }
      } catch (e) {
        console.error(e);
      }
    }

    async function sendCommand(url) {
      await fetch(url, { method: 'POST' });
      fetchTelemetry();
    }

    async function saveSettings() {
      const params = new URLSearchParams({
        thresh: document.getElementById('cfgThresh').value,
        minFlow: document.getElementById('cfgMinFlow').value,
        confirmSec: document.getElementById('cfgConfirm').value,
        ppl1: document.getElementById('cfgPpl1').value,
        ppl2: document.getElementById('cfgPpl2').value
      });
      await fetch('/api/settings?' + params.toString(), { method: 'POST' });
      settingsLoaded = false;
      fetchTelemetry();
    }

    setInterval(fetchTelemetry, 1000);
    fetchTelemetry();
  </script>
</body>
</html>
)rawliteral";

// ============================================================================
// HTTP API HANDLERS
// ============================================================================
void handleRoot()
{
  server.send_P(200, "text/html", INDEX_HTML);
}

void handleStatus()
{
  String json = "{";
  json += "\"uptime\":\"" + formatUptime(millis()) + "\",";
  json += "\"flow1\":" + String(flowRate1_Lmin, 2) + ",";
  json += "\"flow2\":" + String(flowRate2_Lmin, 2) + ",";
  json += "\"diff\":" + String(flowDiff_Lmin, 2) + ",";
  json += "\"vol1\":" + String(totalVolume1_L, 2) + ",";
  json += "\"vol2\":" + String(totalVolume2_L, 2) + ",";
  json += "\"pulses1\":" + String(totalPulses1) + ",";
  json += "\"pulses2\":" + String(totalPulses2) + ",";
  json += "\"valveOpen\":" + String(valveOpen ? "true" : "false") + ",";
  json += "\"leakDetected\":" + String(leakDetected ? "true" : "false") + ",";
  json += "\"leakCount\":" + String(leakCountTotal) + ",";
  json += "\"thresh\":" + String(config.leakThreshold_Lmin, 2) + ",";
  json += "\"minFlow\":" + String(config.minActiveFlow_Lmin, 2) + ",";
  json += "\"confirmSec\":" + String(config.confirmTimeSec) + ",";
  json += "\"ppl1\":" + String(config.pulsesPerLiter1, 0) + ",";
  json += "\"ppl2\":" + String(config.pulsesPerLiter2, 0) + ",";
  json += "\"relayActiveLow\":" + String(config.relayActiveLow ? "true" : "false") + ",";
  json += "\"logs\":[";
  for (size_t i = 0; i < recentEventCount; i++)
  {
    if (i > 0)
      json += ",";
    json += "\"" + recentEvents[i] + "\"";
  }
  json += "]}";
  server.send(200, "application/json", json);
}

void handleValveControl()
{
  if (server.hasArg("state"))
  {
    String st = server.arg("state");
    if (st == "open")
    {
      leakDetected = false;
      manualOverride = false;
      leakConditionStartMs = 0;
      applyRelayOutput(true);
      logEventLocal("MANUAL_OPEN", flowRate1_Lmin, flowRate2_Lmin, flowDiff_Lmin, valveOpen);
    }
    else if (st == "close")
    {
      manualOverride = true;
      applyRelayOutput(false);
      logEventLocal("MANUAL_CLOSE", flowRate1_Lmin, flowRate2_Lmin, flowDiff_Lmin, valveOpen);
    }
  }
  server.send(200, "text/plain", "OK");
}

void handleResetAlarm()
{
  leakDetected = false;
  manualOverride = false;
  leakConditionStartMs = 0;
  applyRelayOutput(true);
  logEventLocal("ALARM_RESET", flowRate1_Lmin, flowRate2_Lmin, flowDiff_Lmin, valveOpen);
  server.send(200, "text/plain", "OK");
}

void handleResetTotals()
{
  portENTER_CRITICAL(&pulseMux);
  pulseCount1 = 0;
  pulseCount2 = 0;
  portEXIT_CRITICAL(&pulseMux);
  totalPulses1 = 0;
  totalPulses2 = 0;
  totalVolume1_L = 0.0f;
  totalVolume2_L = 0.0f;
  logEventLocal("COUNTERS_RESET", 0.0f, 0.0f, 0.0f, valveOpen);
  server.send(200, "text/plain", "OK");
}

void handleToggleRelayMode()
{
  config.relayActiveLow = !config.relayActiveLow;
  saveSettings();
  applyRelayOutput(valveOpen);
  logEventLocal(config.relayActiveLow ? "RELAY_ACT_LOW" : "RELAY_ACT_HIGH",
                flowRate1_Lmin, flowRate2_Lmin, flowDiff_Lmin, valveOpen);
  server.send(200, "text/plain", "OK");
}

void handleSaveSettings()
{
  if (server.hasArg("thresh"))
    config.leakThreshold_Lmin = max(0.05f, server.arg("thresh").toFloat());
  if (server.hasArg("minFlow"))
    config.minActiveFlow_Lmin = max(0.05f, server.arg("minFlow").toFloat());
  if (server.hasArg("confirmSec"))
    config.confirmTimeSec = max(1, (int)server.arg("confirmSec").toInt());
  if (server.hasArg("ppl1"))
    config.pulsesPerLiter1 = max(50.0f, server.arg("ppl1").toFloat());
  if (server.hasArg("ppl2"))
    config.pulsesPerLiter2 = max(50.0f, server.arg("ppl2").toFloat());
  saveSettings();
  logEventLocal("SETTINGS_SAVED", flowRate1_Lmin, flowRate2_Lmin, flowDiff_Lmin, valveOpen);
  server.send(200, "text/plain", "OK");
}

void handleDownloadCsv()
{
  if (littleFsReady && LittleFS.exists(LOG_FILE_PATH))
  {
    File f = LittleFS.open(LOG_FILE_PATH, FILE_READ);
    if (f)
    {
      server.sendHeader("Content-Disposition", "attachment; filename=\"water_leak_logs.csv\"");
      server.streamFile(f, "text/csv");
      f.close();
      return;
    }
  }
  String fallback = "Uptime,Event,Inlet_Lmin,Outlet_Lmin,Diff_Lmin,Valve_State\n";
  for (size_t i = 0; i < recentEventCount; i++)
  {
    fallback += recentEvents[i] + "\n";
  }
  server.sendHeader("Content-Disposition", "attachment; filename=\"water_leak_logs.csv\"");
  server.send(200, "text/csv", fallback);
}

void handleClearLogs()
{
  recentEventCount = 0;
  if (littleFsReady)
  {
    LittleFS.remove(LOG_FILE_PATH);
    File f = LittleFS.open(LOG_FILE_PATH, FILE_WRITE);
    if (f)
    {
      f.println("Uptime,Event,Inlet_Lmin,Outlet_Lmin,Diff_Lmin,Valve_State");
      f.close();
    }
  }
  logEventLocal("LOGS_CLEARED", flowRate1_Lmin, flowRate2_Lmin, flowDiff_Lmin, valveOpen);
  server.send(200, "text/plain", "OK");
}

// ============================================================================
// ARDUINO SETUP & LOOP
// ============================================================================
void setup()
{
  Serial.begin(115200);
  delay(200);
  Serial.println("\n====================================================");
  Serial.println("  SMART WATER LEAKAGE DETECTION SYSTEM (LOCAL IOT)  ");
  Serial.println("====================================================");

  // 1. Load saved calibration & threshold settings from NVS flash
  loadSettings();

  // 2. Initialize Local Flash File System (LittleFS) for CSV logs
  littleFsReady = LittleFS.begin(true);
  if (littleFsReady && !LittleFS.exists(LOG_FILE_PATH))
  {
    File f = LittleFS.open(LOG_FILE_PATH, FILE_WRITE);
    if (f)
    {
      f.println("Uptime,Event,Inlet_Lmin,Outlet_Lmin,Diff_Lmin,Valve_State");
      f.close();
    }
  }

  // 3. Configure Relay & Flow Sensor Pins (All on VIN side: D27, D26, D25)
  pinMode(PIN_RELAY_VALVE, OUTPUT);
  applyRelayOutput(true); // Start with Solenoid Valve OPEN for normal flow

  pinMode(PIN_SENSOR_INLET, INPUT_PULLUP);
  pinMode(PIN_SENSOR_OUTLET, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_SENSOR_INLET), onSensor1Pulse, FALLING);
  attachInterrupt(digitalPinToInterrupt(PIN_SENSOR_OUTLET), onSensor2Pulse, FALLING);

  // 4. Start Standalone Local Wi-Fi Access Point (Works anywhere without router/internet)
  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASS);
  IPAddress ip = WiFi.softAPIP();

  Serial.print("Local Wi-Fi Hotspot Name (SSID): ");
  Serial.println(AP_SSID);
  Serial.print("Local Wi-Fi Password           : ");
  Serial.println(AP_PASS);
  Serial.print("Open Live UI in Browser at     : http://");
  Serial.println(ip);
  Serial.println("====================================================");

  // 5. Register Web UI & REST API routes
  server.on("/", HTTP_GET, handleRoot);
  server.on("/api/status", HTTP_GET, handleStatus);
  server.on("/api/valve", HTTP_POST, handleValveControl);
  server.on("/api/reset_alarm", HTTP_POST, handleResetAlarm);
  server.on("/api/reset_totals", HTTP_POST, handleResetTotals);
  server.on("/api/toggle_relay_mode", HTTP_POST, handleToggleRelayMode);
  server.on("/api/settings", HTTP_POST, handleSaveSettings);
  server.on("/api/download_csv", HTTP_GET, handleDownloadCsv);
  server.on("/api/clear_logs", HTTP_POST, handleClearLogs);
  server.begin();

  logEventLocal("SYSTEM_BOOT", 0.0f, 0.0f, 0.0f, valveOpen);
  lastSampleTimeMs = millis();
  lastPeriodicLogMs = millis();
}

void loop()
{
  server.handleClient();

  uint32_t now = millis();
  uint32_t elapsedMs = now - lastSampleTimeMs;

  // Calculate flow rates & check for leaks every 1000 ms (1 second)
  if (elapsedMs >= 1000)
  {
    uint32_t p1 = 0;
    uint32_t p2 = 0;

    portENTER_CRITICAL(&pulseMux);
    p1 = pulseCount1;
    p2 = pulseCount2;
    pulseCount1 = 0;
    pulseCount2 = 0;
    portEXIT_CRITICAL(&pulseMux);

    lastSampleTimeMs = now;

    totalPulses1 += p1;
    totalPulses2 += p2;

    // Volume added during this window (Liters)
    float dVol1 = (float)p1 / config.pulsesPerLiter1;
    float dVol2 = (float)p2 / config.pulsesPerLiter2;
    totalVolume1_L += dVol1;
    totalVolume2_L += dVol2;

    // Instantaneous flow rate in Liters/minute
    float windowSec = (float)elapsedMs / 1000.0f;
    flowRate1_Lmin = (dVol1 / windowSec) * 60.0f;
    flowRate2_Lmin = (dVol2 / windowSec) * 60.0f;
    flowDiff_Lmin = max(0.0f, flowRate1_Lmin - flowRate2_Lmin);

    // Leak Detection Logic:
    // If Inlet Flow is active AND (Inlet - Outlet) > threshold for confirmTimeSec -> Shut Valve!
    if (!leakDetected && !manualOverride)
    {
      bool isLeakingNow = (flowRate1_Lmin >= config.minActiveFlow_Lmin) &&
                          ((flowRate1_Lmin - flowRate2_Lmin) >= config.leakThreshold_Lmin);

      if (isLeakingNow)
      {
        if (leakConditionStartMs == 0)
        {
          leakConditionStartMs = now;
        }
        else if ((now - leakConditionStartMs) >= ((uint32_t)config.confirmTimeSec * 1000UL))
        {
          leakDetected = true;
          leakCountTotal++;
          applyRelayOutput(false); // Immediately CLOSE the 12V Solenoid Valve!
          logEventLocal("LEAK_SHUTOFF", flowRate1_Lmin, flowRate2_Lmin, flowDiff_Lmin, valveOpen);
          Serial.println("🚨 [ALERT] WATER LEAK DETECTED! Closing 12V Solenoid Valve!");
        }
      }
      else
      {
        leakConditionStartMs = 0;
      }
    }

    // Periodic telemetry log to Serial & Local Flash (when water is flowing)
    Serial.printf("[FLOW] S1(Inlet): %.2f L/m | S2(Outlet): %.2f L/m | Diff: %.2f L/m | Valve: %s | Status: %s\n",
                  flowRate1_Lmin, flowRate2_Lmin, flowDiff_Lmin,
                  valveOpen ? "OPEN" : "CLOSED",
                  leakDetected ? "LEAK!" : "OK");

    if ((flowRate1_Lmin > 0.05f || flowRate2_Lmin > 0.05f) && (now - lastPeriodicLogMs >= 10000UL))
    {
      lastPeriodicLogMs = now;
      logEventLocal("FLOW_SAMPLE", flowRate1_Lmin, flowRate2_Lmin, flowDiff_Lmin, valveOpen);
    }
  }
}
