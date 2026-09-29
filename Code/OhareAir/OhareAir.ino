// ===== AIRCUBE (BASE) PORT + STEPPER NEEDLE =====
// ESP32-S3-DevKitC-1 + ENS161 + ENS210 + 28BYJ-48/ULN2003 + onboard RGB LED
//
// Ported from StuckAtPrototype/AirCube firmware (Base model) source:
//   ens210.c, ens16x_driver.c, led.c, led_color_lib.c.
//
// Air quality: uses the ENS161's own onboard AQI-UBA classification (register 0x21,
// 1-5) instead of AirCube's software-derived 0-500 "VOC Level". AQI-UBA is computed
// inside the chip by ScioSense's own algorithm, so it isn't reachable from software
// anyway. eTVOC and eCO2 below are the sensor's raw, uncorrected values.
//
// Only hardware you actually have is used: the two I2C sensors, the ESP32-S3
// board's own RGB LED, the stepper and WiFi. AirCube's button, its 3 external
// LEDs, the Pro sensors, Zigbee and BLE are not part of this sketch.
//
// Your addition: a stepper needle that points at the AQI-UBA rating.
//
// The WiFi page is modeled on AirCube's dashboard features: live readings with a
// color-coded VOC Level, live charts with adjustable depth, 5-minute min/avg/max
// history (same 32-byte slot layout and capacity as AirCube) with CSV export, and
// LED brightness. Open the board's IP address in a browser.
//
//   AQI-UBA   Rating       Needle
//   1         Excellent    0 deg
//   2         Good         90 deg
//   3         Moderate     115 deg
//   4         Poor         140 deg
//   5         Unhealthy    180 deg
//
// Timing: the sensors run from power-up, but nothing is read for the first 3 minutes
// (warm-up). After that one reading is taken per minute. The LED pulses blue until the
// first reading, then follows VOC Level from green to red.
//
// Power-on needle position is assumed to be 0 (home). Positive degrees = counter-clockwise.

#include <Wire.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Adafruit_NeoPixel.h>
#include <math.h>
#include <string.h>

// ===== WiFi =====
const char* ssid = "";
const char* password = "";

// ===== Pins =====
#define I2C_SDA_PIN 9
#define I2C_SCL_PIN 8
#define LED_PIN 48                  // ESP32-S3-DevKitC-1 onboard RGB LED (GPIO48 on v1.0; v1.1 boards use GPIO38)
#define IN1 6
#define IN2 7
#define IN3 5
#define IN4 4

Adafruit_NeoPixel strip(1, LED_PIN, NEO_GRB + NEO_KHZ800);

// ===== Settings =====
#define WARMUP_MS 180000UL          // no readings for the first 3 minutes while the sensors warm up
#define SENSOR_PERIOD_MS 60000UL    // then one reading per minute (AirCube itself reads every second)
#define LED_PERIOD_MS 20UL          // LED animation tick (this is not a sensor reading)

// AirCube corrects ENS210 by -2 C inside its enclosure. Your board runs warmer
// (WiFi, ESP32-S3), so this is the ~10 F you measured. Re-tune with a reference thermometer.
#define TEMP_OFFSET_C 5.56f

// AirCube feeds the RAW ENS210 reading to the ENS161 for compensation (false).
// Set true to compensate with the offset-corrected (closer to room) temperature instead.
const bool COMPENSATE_WITH_CORRECTED_TEMP = false;


// ===== Needle / stepper =====
#define MAX_DEGREES 180.0
#define DIRECTION_SIGN 1            // change to -1 if positive turns the needle clockwise
const float BAND_ANGLE[5] = {0, 90, 115, 140, 180};
const char* BAND_NAME[5] = {"Excellent", "Good", "Moderate", "Poor", "Unhealthy"};
#define STABLE_SAMPLES 2            // a new band must show in this many readings in a row (about 2 minutes) before the needle moves

#define STEPS_PER_ROTATION 2048.0
#define DEGREES_PER_STEP (360.0 / STEPS_PER_ROTATION)
#define MAX_STEPS ((long)(MAX_DEGREES / DEGREES_PER_STEP + 0.5))

int stepSequence[4][4] = {
  {1, 0, 0, 0},
  {0, 1, 0, 0},
  {0, 0, 1, 0},
  {0, 0, 0, 1}
};
long currentSteps = 0;
int stepIndex = 0;
int motorSpeed = 3;                 // ms between steps (lower = faster)

// ===== AirCube VOC Level tables (main.c) =====
#define AQI_UBA_MIN 1
#define AQI_UBA_MAX 5
#define HUE_GREEN 21845

// ===== LED (led.c / led_color_lib.c) =====
#define INTENSITY_RAMP_MS 500
#define TRANSITION_SPEED 0.02f

float currentHue = HUE_GREEN;
uint16_t targetHue = HUE_GREEN;
float ledIntensity = 0.0f;
float ledIntensityTarget = 0.0f;
int ledBrightnessPct = 60;          // AirCube default; changed from the web page, not saved
int lastR = -1, lastG = -1, lastB = -1;

// ===== ENS210 =====
#define ENS210_ADDR 0x43
#define ENS210_REG_PART_ID 0x00
#define ENS210_REG_SYS_CTRL 0x10
#define ENS210_REG_SYS_STAT 0x11
#define ENS210_REG_SENS_RUN 0x21
#define ENS210_REG_SENS_START 0x22
#define ENS210_REG_T_VAL 0x30
#define ENS210_REG_H_VAL 0x33

uint8_t ens210T[2] = {0, 0};
uint8_t ens210H[2] = {0, 0};
float temperatureC = 0;             // raw ENS210 reading
float humidityPct = 0;
bool ens210Present = false;
bool ens210TempValid = false;
bool ens210HumValid = false;

// ===== ENS16X =====
#define ENS16X_ADDR 0x52
#define ENS16X_DEVICE_STATUS 0x20
#define ENS16X_OPMODE 0x10
#define ENS16X_REG_DATA_ETVOC 0x22
#define ENS16X_REG_DATA_ECO2 0x24
#define ENS16X_REG_DATA_AQI_UBA 0x21
#define ENS16X_REG_TH_IN 0x13       // TEMP_IN (2) then RH_IN (2)

#define ENS_OP_OK 0
#define ENS_WARM_UP 1
#define ENS_RESERVED 2
#define ENS_NO_VALID_OUTPUT 3
#define ENS_IDLE 1
#define ENS_STANDARD 2

bool ens16xPresent = false;
int ens16xStatus = ENS_NO_VALID_OUTPUT;
int ens16xTvoc = -1;
int ens16xEco2 = -1;
int ens16xAqiUba = -1;

// ===== Live values =====
int currentAqiUba = 0;              // 1-5, held at last known value if a read fails; 0 = no data yet
float tempCOut = 0;                 // offset-corrected temperature
bool autoMode = true;
int appliedBand = -1;
int candidateBand = -1;
int candidateCount = 0;

unsigned long lastSensorMs = 0;
unsigned long lastLedMs = 0;
unsigned long lastWarmMsgMs = 0;
unsigned long sensorsStartMs = 0;   // when the sensors were switched on
unsigned long lastReadingMs = 0;
unsigned long readingSeq = 0;       // counts valid readings so the page only charts new ones
bool firstStepDone = false;
bool haveReading = false;           // true after the first valid VOC reading

// ===== History (history.c): 5-minute min/avg/max windows, kept in RAM =====
#define HISTORY_CAPACITY 2016            // 7 days of 5-minute windows, same as AirCube
#define HISTORY_WINDOW_MS 300000UL       // 5 minutes
#define HISTORY_NO_DATA_S16 (-32768)     // channel had no valid samples in the window
#define HISTORY_NO_DATA_U16 65535

// Same 32-byte slot as AirCube. Temperature/humidity are x100; VOC Level, eCO2, eTVOC are raw.
struct __attribute__((packed)) HistorySlot {
  uint16_t sequence;
  int16_t t_a, t_n, t_x;                 // temperature avg/min/max (x100 C)
  int16_t h_a, h_n, h_x;                 // humidity avg/min/max (x100 %)
  uint16_t q_a, q_n, q_x;                // VOC Level
  uint16_t c_a, c_n, c_x;                // eCO2 (ppm)
  uint16_t v_a, v_n, v_x;                // eTVOC (ppb)
};
static_assert(sizeof(HistorySlot) == 32, "HistorySlot must be 32 bytes like AirCube");

HistorySlot histSlots[HISTORY_CAPACITY];
uint16_t histWrite = 0;                  // next slot to write
uint16_t histCount = 0;                  // valid entries, up to HISTORY_CAPACITY
uint16_t histNextSeq = 0;
unsigned long histLastFlushMs = 0;       // when the newest slot was saved (boot time until the first one)

struct HistAccum {
  float sumT, sumH;
  uint32_t sumQ, sumC, sumV;
  int16_t minT, maxT, minH, maxH;
  uint16_t minQ, maxQ, minC, maxC, minV, maxV;
  uint32_t samples, tCount, hCount, qCount, cCount, vCount;
  unsigned long windowStartMs;
};
HistAccum accum;

WebServer server(80);

const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Air Quality Monitor</title>
<style>
:root { --bg:#0b1020; --card:#131a2e; --card2:#1a2340; --line:#25304d; --text:#e6ebf5; --muted:#8b97b3; --accent:#7c9cff; --good:#34d399; --warn:#fbbf24; --bad:#f87171; }
* { box-sizing: border-box; margin: 0; padding: 0; }
body { font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, Arial, sans-serif; background: var(--bg); color: var(--text); padding: 16px; }
.wrap { max-width: 980px; margin: 0 auto; }
header { display: flex; align-items: center; justify-content: space-between; gap: 12px; margin-bottom: 14px; flex-wrap: wrap; }
h1 { font-size: 20px; font-weight: 700; }
h1 small { font-size: 12px; color: var(--muted); font-weight: 500; margin-left: 8px; }
h2 { font-size: 14px; font-weight: 700; }
.chip { font-size: 12px; padding: 6px 10px; border-radius: 999px; background: var(--card2); color: var(--muted); border: 1px solid var(--line); }
.chip.ok { color: var(--good); border-color: #1f6f55; }
.chip.warn { color: var(--warn); border-color: #7a5c12; }
.chip.bad { color: var(--bad); border-color: #7a2a2a; }
nav { display: flex; gap: 6px; margin-bottom: 14px; }
nav button { flex: 1; padding: 10px; border: 1px solid var(--line); background: var(--card); color: var(--muted); border-radius: 10px; font-size: 14px; font-weight: 600; cursor: pointer; }
nav button.on { color: var(--text); background: var(--card2); border-color: var(--accent); }
.panel { background: var(--card); border: 1px solid var(--line); border-radius: 14px; padding: 16px; margin-bottom: 14px; }
.muted { color: var(--muted); }
.small { font-size: 12px; }
.row { display: flex; align-items: center; gap: 10px; flex-wrap: wrap; margin-bottom: 10px; }
.spacer { flex: 1; }
.hero { display: flex; align-items: center; gap: 22px; flex-wrap: wrap; }
.ring { width: 150px; height: 150px; border-radius: 50%; border: 10px solid var(--line); display: flex; flex-direction: column; align-items: center; justify-content: center; flex: none; transition: border-color .6s; }
.ring-num { font-size: 46px; font-weight: 800; line-height: 1; transition: color .6s; }
.ring-cap { font-size: 12px; color: var(--muted); margin-top: 6px; }
.hero-side { flex: 1; min-width: 220px; }
.rating { font-size: 30px; font-weight: 800; }
.scale { position: relative; display: flex; height: 12px; border-radius: 6px; overflow: visible; margin: 14px 0 6px; }
.scale .seg { flex: 1; }
.scale .seg:first-child { border-radius: 6px 0 0 6px; }
.scale .seg:nth-child(5) { border-radius: 0 6px 6px 0; }
.marker { position: absolute; top: -5px; width: 4px; height: 22px; background: #fff; border-radius: 2px; box-shadow: 0 0 0 2px var(--bg); left: 0; transition: left .6s; }
.scale-labels { display: flex; font-size: 11px; color: var(--muted); }
.scale-labels span { flex: 1; text-align: center; }
.warm { background: #0f2a4a; color: #bcd9ff; border: 1px solid #1d4a80; padding: 10px; border-radius: 10px; font-size: 13px; text-align: center; margin-bottom: 14px; display: none; }
.cards { display: grid; grid-template-columns: repeat(auto-fit, minmax(150px, 1fr)); gap: 10px; margin-bottom: 14px; }
.card { background: var(--card); border: 1px solid var(--line); border-radius: 14px; padding: 14px; }
.card .k { font-size: 11px; text-transform: uppercase; letter-spacing: .05em; color: var(--muted); font-weight: 700; }
.card .v { font-size: 26px; font-weight: 800; margin-top: 4px; }
.chart-title { font-size: 12px; font-weight: 700; color: var(--muted); text-transform: uppercase; letter-spacing: .05em; margin: 14px 0 6px; }
canvas { width: 100%; height: 170px; display: block; background: #0e1527; border-radius: 10px; border: 1px solid var(--line); }
button.btn, select { padding: 8px 12px; border: 1px solid var(--line); background: var(--card2); color: var(--text); border-radius: 8px; font-size: 13px; font-weight: 600; cursor: pointer; }
button.btn.primary { background: var(--accent); color: #0b1020; border-color: var(--accent); }
button.btn.on { background: var(--good); color: #062; border-color: var(--good); }
button.btn.danger { border-color: #7a2a2a; color: var(--bad); }
.btns { display: flex; gap: 6px; flex-wrap: wrap; }
.btns .btn { flex: 1; min-width: 60px; }
input[type=number] { flex: 1; min-width: 100px; padding: 9px; border-radius: 8px; border: 1px solid var(--line); background: #0e1527; color: var(--text); font-size: 14px; }
.bar { height: 10px; background: #0e1527; border-radius: 5px; overflow: hidden; border: 1px solid var(--line); }
.bar > div { height: 100%; width: 0; background: var(--accent); transition: width .3s; }
.bar-scale { display: flex; justify-content: space-between; font-size: 11px; color: var(--muted); margin-top: 3px; }
table { width: 100%; border-collapse: collapse; font-size: 13px; }
td, th { padding: 7px 6px; text-align: left; border-bottom: 1px solid var(--line); }
th { color: var(--muted); font-size: 11px; text-transform: uppercase; letter-spacing: .05em; }
.toast { min-height: 36px; text-align: center; font-size: 13px; padding: 9px; border-radius: 8px; margin-top: 12px; color: var(--muted); }
.toast.ok { background: #0f2f26; color: var(--good); }
.toast.err { background: #331616; color: var(--bad); }
footer { text-align: center; color: var(--muted); font-size: 12px; padding: 6px 0 20px; }
</style>
</head>
<body>
<div class="wrap">
<header>
  <h1>Air Quality Monitor<small>ENS161 + ENS210</small></h1>
  <div class="chip" id="chip">Connecting...</div>
</header>

<nav>
  <button data-tab="live" class="on">Live</button>
  <button data-tab="history">History</button>
  <button data-tab="needle">Needle</button>
</nav>

<section id="tab-live">
  <div class="warm" id="warm"></div>

  <div class="panel hero">
    <div class="ring" id="ring"><div class="ring-num" id="vocNum">--</div><div class="ring-cap">AQI-UBA</div></div>
    <div class="hero-side">
      <div class="rating" id="ratingName">--</div>
      <div class="muted small" id="ratingSub">Waiting for data</div>
      <div class="scale" id="scale">
        <div class="seg"></div><div class="seg"></div><div class="seg"></div><div class="seg"></div><div class="seg"></div>
        <div class="marker" id="marker"></div>
      </div>
      <div class="scale-labels"><span>Excellent</span><span>Good</span><span>Moderate</span><span>Poor</span><span>Unhealthy</span></div>
    </div>
  </div>

  <div class="cards">
    <div class="card"><div class="k">Temperature</div><div class="v" id="vTemp">--</div></div>
    <div class="card"><div class="k">Humidity</div><div class="v" id="vHum">--</div></div>
    <div class="card"><div class="k">eCO2</div><div class="v" id="vCo2">--</div></div>
    <div class="card"><div class="k">eTVOC</div><div class="v" id="vTvoc">--</div></div>
  </div>

  <div class="panel">
    <div class="row">
      <h2>Live charts</h2><span class="muted small">one point per reading, about once a minute</span><span class="spacer"></span>
      <label class="muted small">History depth
        <select id="depth"><option>50</option><option>100</option><option selected>250</option><option>500</option><option>1000</option></select> points
      </label>
      <button class="btn" id="btnLiveCsv">Export CSV</button>
    </div>
    <div class="chart-title">Temperature &amp; humidity</div><canvas id="cLiveTH"></canvas>
    <div class="chart-title">AQI-UBA</div><canvas id="cLiveVoc"></canvas>
    <div class="chart-title">Gas levels (eCO2 / eTVOC)</div><canvas id="cLiveGas"></canvas>
  </div>

  <div class="panel">
    <div class="row"><h2>LED brightness</h2></div>
    <div class="btns" id="bright">
      <button class="btn" data-bright="0">Off</button>
      <button class="btn" data-bright="10">10%</button>
      <button class="btn" data-bright="30">30%</button>
      <button class="btn" data-bright="60">60%</button>
      <button class="btn" data-bright="100">100%</button>
    </div>
    <div class="muted small" style="margin-top:8px">The board's LED follows the sensor's own AQI-UBA rating from green to red, and pulses blue until the first reading. Brightness resets to 60% when the board restarts.</div>
  </div>
</section>

<section id="tab-history" style="display:none">
  <div class="panel">
    <div class="row">
      <h2>Device history</h2><span class="spacer"></span>
      <select id="histRange">
        <option value="72">Last 6 hours</option>
        <option value="288" selected>Last 24 hours</option>
        <option value="864">Last 3 days</option>
        <option value="2016">Last 7 days</option>
      </select>
      <button class="btn" id="btnHistLoad">Refresh</button>
      <button class="btn primary" id="btnHistCsv">Export CSV</button>
    </div>
    <div class="muted small" id="histInfo">Not loaded yet.</div>
    <div class="chart-title">Temperature &amp; humidity (average)</div><canvas id="cHistTH"></canvas>
    <div class="chart-title">AQI-UBA (average, min and max)</div><canvas id="cHistVoc"></canvas>
    <div class="chart-title">Gas levels (average)</div><canvas id="cHistGas"></canvas>
  </div>
  <div class="panel muted small">The board saves one entry every 5 minutes (minimum, average and maximum), up to 7 days, the same way AirCube does. Entries are kept in memory, so they are cleared when the board restarts or loses power. AQI-UBA is the ENS161 chip's own onboard rating; the enclosure offset only corrects the eTVOC/eCO2 numbers shown alongside it, not this rating.</div>
</section>

<section id="tab-needle" style="display:none">
  <div class="panel">
    <div class="row"><h2>Needle</h2><span class="spacer"></span><span class="muted small">Position: <b id="angle">0.0</b>&deg;</span></div>
    <div class="bar"><div id="barfill"></div></div>
    <div class="bar-scale"><span>0&deg;</span><span>180&deg;</span></div>
    <div class="row" style="margin-top:14px">
      <div class="btns" style="flex:1">
        <button class="btn" id="btnAuto" data-mode="1">Auto (follows AQI-UBA)</button>
        <button class="btn" id="btnManual" data-mode="0">Manual</button>
      </div>
    </div>
  </div>

  <div class="panel">
    <div class="row"><h2>Angle for each rating</h2></div>
    <table>
      <tr><th>Rating</th><th>AQI-UBA</th><th>Needle</th></tr>
      <tr><td>Excellent</td><td>1</td><td>0&deg;</td></tr>
      <tr><td>Good</td><td>2</td><td>90&deg;</td></tr>
      <tr><td>Moderate</td><td>3</td><td>115&deg;</td></tr>
      <tr><td>Poor</td><td>4</td><td>140&deg;</td></tr>
      <tr><td>Unhealthy</td><td>5</td><td>180&deg;</td></tr>
    </table>
  </div>

  <div class="panel">
    <div class="row"><h2>Go to angle</h2><span class="muted small">switches to manual</span></div>
    <div class="btns">
      <button class="btn" data-goto="0">0&deg;</button>
      <button class="btn" data-goto="90">90&deg;</button>
      <button class="btn" data-goto="115">115&deg;</button>
      <button class="btn" data-goto="140">140&deg;</button>
      <button class="btn" data-goto="180">180&deg;</button>
    </div>
    <div class="row" style="margin-top:14px"><h2>Rotate by</h2><span class="muted small">negative = back</span></div>
    <div class="row">
      <input type="number" id="degrees" placeholder="e.g. 10 or -10" step="0.1">
      <button class="btn primary" id="btnRotate">Go</button>
    </div>
    <div class="row"><h2>Speed</h2></div>
    <div class="btns">
      <button class="btn" data-speed="slower">Slower</button>
      <button class="btn" data-speed="faster">Faster</button>
    </div>
  </div>
</section>

<div class="toast" id="toast"></div>
<footer id="foot">&nbsp;</footer>
</div>

<script>
(function () {
  'use strict';

  var NAMES = ['Excellent', 'Good', 'Moderate', 'Poor', 'Unhealthy'];
  var S16 = -32768, U16 = 65535;
  var C_TEMP = '#fbbf24', C_HUM = '#38bdf8', C_VOC = '#e2e8f0', C_CO2 = '#a78bfa', C_TVOC = '#34d399';

  function $(id) { return document.getElementById(id); }
  function pad(n) { return n < 10 ? '0' + n : '' + n; }
  function fix0(v) { return v.toFixed(0); }
  function fix1(v) { return v.toFixed(1); }
  function cToF(c) { return c * 9 / 5 + 32; }
  function fmtClock(t) { var d = new Date(t); return pad(d.getHours()) + ':' + pad(d.getMinutes()) + ':' + pad(d.getSeconds()); }
  function fmtClockShort(t) { var d = new Date(t); return pad(d.getHours()) + ':' + pad(d.getMinutes()); }
  function fmtDay(t) { var d = new Date(t); return (d.getMonth() + 1) + '/' + d.getDate() + ' ' + pad(d.getHours()) + ':' + pad(d.getMinutes()); }

  // ---- AQI-UBA colour: 1 (Excellent) green -> 5 (Unhealthy) red, same mapping as the LED ----
  function aqiHue(a) { a = Math.max(1, Math.min(5, a)); return 120 * (5 - a) / 4; }
  function aqiColor(a, alpha) { return 'hsla(' + Math.round(aqiHue(a)) + ',85%,50%,' + (alpha === undefined ? 1 : alpha) + ')'; }
  function bandOf(a) { return Math.max(0, Math.min(4, Math.round(a) - 1)); }
  function scalePos(a) { return (Math.max(1, Math.min(5, a)) - 1) / 4; }
  var BANDS = [];
  (function () {
    for (var i = 1; i <= 5; i++) BANDS.push({ lo: i - 0.5, hi: i + 0.5, color: aqiColor(i, 0.16) });
  })();

  // ---- tiny canvas line chart ----
  function niceStep(range, ticks) {
    var raw = range / ticks;
    if (!(raw > 0)) return 1;
    var mag = Math.pow(10, Math.floor(Math.log(raw) / Math.LN10));
    var n = raw / mag;
    var step = n < 1.5 ? 1 : n < 3 ? 2 : n < 7 ? 5 : 10;
    return step * mag;
  }
  function axisRange(series, axis, floor0, minSpan) {
    var lo = Infinity, hi = -Infinity;
    series.forEach(function (s) {
      if ((s.axis || 'l') !== axis) return;
      s.pts.forEach(function (p) {
        if (p[1] == null) return;
        if (p[1] < lo) lo = p[1];
        if (p[1] > hi) hi = p[1];
      });
    });
    if (lo === Infinity) return null;
    if (floor0 && lo > 0) lo = 0;
    if (hi - lo < (minSpan || 1e-6)) hi = lo + (minSpan || 1);
    var step = niceStep(hi - lo, 4);
    lo = Math.floor(lo / step) * step;
    hi = Math.ceil(hi / step) * step;
    if (hi <= lo) hi = lo + step;
    return { lo: lo, hi: hi, step: step };
  }
  function drawChart(cv, o) {
    var dpr = window.devicePixelRatio || 1;
    var w = cv.clientWidth || (cv.parentNode && cv.parentNode.clientWidth) || 320;
    var h = cv.clientHeight || 170;
    cv.width = Math.round(w * dpr);
    cv.height = Math.round(h * dpr);
    var g = cv.getContext('2d');
    if (!g) return;
    g.setTransform(dpr, 0, 0, dpr, 0, 0);
    g.clearRect(0, 0, w, h);
    g.font = '11px sans-serif';

    var L = axisRange(o.series, 'l', o.floorL, o.minSpanL);
    var R = axisRange(o.series, 'r', false, 0);
    if (!L && !R) {
      g.fillStyle = '#8b97b3';
      g.textAlign = 'center';
      g.fillText('No data yet', w / 2, h / 2);
      return;
    }
    var m = { l: 48, r: R ? 48 : 12, t: 24, b: 22 };
    var pw = w - m.l - m.r, ph = h - m.t - m.b;
    var x0 = o.xmin, x1 = o.xmax;
    if (!(x1 > x0)) x1 = x0 + 1;
    function X(t) { return m.l + (t - x0) / (x1 - x0) * pw; }
    function YL(v) { return m.t + ph - (v - L.lo) / (L.hi - L.lo) * ph; }
    function YR(v) { return m.t + ph - (v - R.lo) / (R.hi - R.lo) * ph; }

    // rating bands behind the VOC chart
    if (L && o.bands) {
      o.bands.forEach(function (b) {
        var lo = Math.max(b.lo, L.lo), hi = Math.min(b.hi, L.hi);
        if (hi <= lo) return;
        g.fillStyle = b.color;
        g.fillRect(m.l, YL(hi), pw, YL(lo) - YL(hi));
      });
    }

    // grid + left labels
    g.lineWidth = 1;
    g.textBaseline = 'middle';
    if (L) {
      var n = 0;
      for (var v = L.lo; v <= L.hi + L.step * 0.001 && n < 30; v += L.step, n++) {
        var y = YL(v);
        g.strokeStyle = '#25304d';
        g.beginPath(); g.moveTo(m.l, y); g.lineTo(m.l + pw, y); g.stroke();
        g.fillStyle = '#8b97b3';
        g.textAlign = 'right';
        g.fillText((o.fmtL || fix0)(v), m.l - 6, y);
      }
    }
    if (R) {
      var k = 0;
      for (var u = R.lo; u <= R.hi + R.step * 0.001 && k < 30; u += R.step, k++) {
        g.fillStyle = '#8b97b3';
        g.textAlign = 'left';
        g.fillText((o.fmtR || fix0)(u), m.l + pw + 6, YR(u));
      }
    }

    // x labels
    g.textBaseline = 'alphabetic';
    for (var i = 0; i <= 3; i++) {
      var tt = x0 + (x1 - x0) * i / 3;
      g.fillStyle = '#8b97b3';
      g.textAlign = i === 0 ? 'left' : i === 3 ? 'right' : 'center';
      g.fillText((o.xfmt || fmtClock)(tt), X(tt), h - 6);
    }

    // series
    o.series.forEach(function (s) {
      var Y = (s.axis === 'r') ? YR : YL;
      if (s.axis === 'r' ? !R : !L) return;
      g.strokeStyle = s.color;
      g.globalAlpha = s.alpha || 1;
      g.lineWidth = s.w || 1.8;
      g.beginPath();
      var pen = false;
      s.pts.forEach(function (p) {
        if (p[1] == null) { pen = false; return; }
        var px = X(p[0]), py = Y(p[1]);
        if (!pen) { g.moveTo(px, py); pen = true; } else { g.lineTo(px, py); }
      });
      g.stroke();
      g.globalAlpha = 1;
    });

    // legend
    var lx = m.l;
    g.textBaseline = 'middle';
    o.series.forEach(function (s) {
      if (!s.label) return;
      g.strokeStyle = s.color; g.lineWidth = 3;
      g.beginPath(); g.moveTo(lx, 11); g.lineTo(lx + 14, 11); g.stroke();
      g.fillStyle = '#cbd5e1';
      g.textAlign = 'left';
      g.fillText(s.label, lx + 19, 11);
      lx += 19 + g.measureText(s.label).width + 16;
    });
  }

  // ---- helpers ----
  function getJSON(url) {
    return fetch(url, { cache: 'no-store' }).then(function (r) {
      if (!r.ok) throw new Error('HTTP ' + r.status);
      return r.json();
    });
  }
  function getText(url) {
    return fetch(url, { cache: 'no-store' }).then(function (r) { return r.text(); });
  }
  var toastTimer = null;
  function toast(msg, cls) {
    var t = $('toast');
    t.textContent = msg;
    t.className = 'toast ' + (cls || '');
    if (toastTimer) clearTimeout(toastTimer);
    toastTimer = setTimeout(function () { t.textContent = ''; t.className = 'toast'; }, 4000);
  }
  function download(name, text) {
    var b = new Blob([text], { type: 'text/csv' });
    var a = document.createElement('a');
    a.href = URL.createObjectURL(b);
    a.download = name;
    document.body.appendChild(a);
    a.click();
    document.body.removeChild(a);
    setTimeout(function () { URL.revokeObjectURL(a.href); }, 1000);
  }
  function csvNum(v, d) { return (v === null || v === undefined) ? '' : v.toFixed(d); }
  function stamp() { var d = new Date(); return d.getFullYear() + pad(d.getMonth() + 1) + pad(d.getDate()) + '-' + pad(d.getHours()) + pad(d.getMinutes()); }

  // ---- live ----
  var live = [];
  var depth = 250;
  var busy = false, busySince = 0;

  function renderLive() {
    var d = live.slice(-depth);
    var now = Date.now();
    var x0 = d.length ? d[0].t : now - 1000;
    var x1 = d.length ? d[d.length - 1].t : now;
    function pts(k) { return d.map(function (p) { return [p.t, p[k]]; }); }
    drawChart($('cLiveTH'), { xmin: x0, xmax: x1, xfmt: fmtClockShort, fmtL: fix1, fmtR: fix0, series: [
      { label: 'Temperature (\u00b0F)', color: C_TEMP, pts: pts('temp'), axis: 'l' },
      { label: 'Humidity (%)', color: C_HUM, pts: pts('hum'), axis: 'r' } ] });
    drawChart($('cLiveVoc'), { xmin: x0, xmax: x1, xfmt: fmtClockShort, floorL: true, minSpanL: 1, bands: BANDS, series: [
      { label: 'AQI-UBA', color: C_VOC, pts: pts('aqi'), axis: 'l' } ] });
    drawChart($('cLiveGas'), { xmin: x0, xmax: x1, xfmt: fmtClockShort, series: [
      { label: 'eCO2 (ppm)', color: C_CO2, pts: pts('eco2'), axis: 'l' },
      { label: 'eTVOC (ppb)', color: C_TVOC, pts: pts('etvoc'), axis: 'r' } ] });
  }

  function setChip(text, cls) {
    var c = $('chip');
    c.textContent = text;
    c.className = 'chip ' + (cls || '');
  }

  var lastSeq = -1;

  function fmtMS(sec) { var m = Math.floor(sec / 60); return m + ':' + pad(sec % 60); }
  function ago(sec) { return sec < 90 ? sec + ' s ago' : Math.round(sec / 60) + ' min ago'; }

  function onStatus(s) {
    // things that are always available
    $('angle').textContent = s.angle.toFixed(1);
    $('barfill').style.width = (s.angle / 180 * 100) + '%';
    $('btnAuto').className = 'btn' + (s.auto ? ' on' : '');
    $('btnManual').className = 'btn' + (s.auto ? '' : ' on');
    var bs = document.querySelectorAll('[data-bright]');
    for (var i = 0; i < bs.length; i++) {
      bs[i].className = 'btn' + (parseInt(bs[i].getAttribute('data-bright'), 10) === s.brightness ? ' on' : '');
    }
    var up = s.uptime_s, hh = Math.floor(up / 3600), mm = Math.floor((up % 3600) / 60);
    var foot = 'Uptime ' + hh + 'h ' + mm + 'm \u00b7 ' + s.hist + ' history entries stored';

    var w = $('warm');

    // no reading yet: the sensors are still warming up
    if (!s.have) {
      $('vocNum').textContent = '--';
      $('vocNum').style.color = '';
      $('ring').style.borderColor = '';
      $('ratingName').textContent = '--';
      $('ratingName').style.color = '';
      $('ratingSub').textContent = 'No readings yet';
      $('marker').style.left = '0';
      $('vTemp').textContent = '--';
      $('vHum').textContent = '--';
      $('vCo2').textContent = '--';
      $('vTvoc').textContent = '--';
      w.style.display = 'block';
      if (s.warmup_s > 0) {
        w.textContent = 'Sensors are warming up. The first reading is taken in about ' + fmtMS(s.warmup_s) + ', then once a minute.';
        setChip('Warming up ' + fmtMS(s.warmup_s), 'warn');
      } else {
        w.textContent = 'Taking the first reading...';
        setChip('Reading...', 'warn');
      }
      $('foot').textContent = foot;
      return;
    }

    var b = (s.band >= 0 && s.band <= 4) ? s.band : 0;
    var col = aqiColor(s.aqi_uba);
    $('vocNum').textContent = s.aqi_uba;
    $('vocNum').style.color = col;
    $('ring').style.borderColor = col;
    $('ratingName').textContent = NAMES[b];
    $('ratingName').style.color = col;
    $('ratingSub').textContent = 'AQI-UBA ' + s.aqi_uba + ' of 5 (sensor-reported) \u00b7 eTVOC ' + (s.etvoc >= 0 ? s.etvoc + ' ppb' : '--') + ' \u00b7 updated ' + ago(s.age_s);
    $('marker').style.left = 'calc(' + (scalePos(s.aqi_uba) * 100).toFixed(1) + '% - 2px)';

    $('vTemp').textContent = s.temp.toFixed(1) + ' \u00b0F';
    $('vHum').textContent = s.hum.toFixed(1) + ' %';
    $('vCo2').textContent = s.eco2 >= 0 ? s.eco2 + ' ppm' : '--';
    $('vTvoc').textContent = s.etvoc >= 0 ? s.etvoc + ' ppb' : '--';

    if (s.ready) {
      w.style.display = 'none';
      setChip('ENS161: ' + s.status, 'ok');
    } else {
      w.style.display = 'block';
      w.textContent = 'ENS161 status: ' + s.status + '. Readings can be off until it reports OK; the needle holds until then.';
      setChip('ENS161: ' + s.status, s.status === 'No Valid Output' ? 'bad' : 'warn');
    }

    $('foot').textContent = foot + ' \u00b7 next reading in about ' + s.next_s + ' s';

    // one chart point per new reading, not per page refresh
    if (s.seq !== lastSeq) {
      lastSeq = s.seq;
      live.push({ t: Date.now(), temp: s.temp, hum: s.hum, aqi: s.aqi_uba,
        eco2: s.eco2 >= 0 ? s.eco2 : null, etvoc: s.etvoc >= 0 ? s.etvoc : null, status: s.status });
      if (live.length > 1000) live.splice(0, live.length - 1000);
      renderLive();
    }
  }

  function poll() {
    if (busy && Date.now() - busySince < 6000) return;
    busy = true; busySince = Date.now();
    getJSON('/status').then(function (s) { busy = false; onStatus(s); })
      .catch(function () { busy = false; setChip('Disconnected', 'bad'); });
  }

  // ---- history ----
  var hist = null;
  function s16(v) { return v === S16 ? null : v / 100; }
  function u16(v) { return v === U16 ? null : v; }
  function triple(a, b, c, f) { return [f(a), f(b), f(c)]; }
  function tf(v) { return v === S16 ? null : cToF(v / 100); }

  function loadHistory() {
    var n = parseInt($('histRange').value, 10);
    $('histInfo').textContent = 'Loading...';
    getJSON('/history?n=' + n).then(function (j) {
      var now = Date.now();
      var win = j.window_s * 1000;
      var cnt = j.history.length;
      var rows = j.history.map(function (a, i) {
        return {
          t: now - j.newest_age_s * 1000 - (cnt - 1 - i) * win,
          seq: a[0],
          temp: triple(a[1], a[2], a[3], tf),
          hum: triple(a[4], a[5], a[6], s16),
          aqi: triple(a[7], a[8], a[9], u16),
          eco2: triple(a[10], a[11], a[12], u16),
          etvoc: triple(a[13], a[14], a[15], u16)
        };
      });
      hist = { rows: rows, span: cnt * win, entries: j.entries, capacity: j.capacity, win: win };
      if (!cnt) {
        $('histInfo').textContent = 'No history yet. The first entry is saved 5 minutes after the board starts.';
      } else {
        $('histInfo').textContent = j.entries + ' of ' + j.capacity + ' entries stored \u00b7 showing ' + cnt +
          ' (' + (cnt * j.window_s / 3600).toFixed(1) + ' hours, 5-minute windows)';
      }
      renderHistory();
    }).catch(function (e) { $('histInfo').textContent = 'Could not load history (' + e.message + ').'; });
  }

  function renderHistory() {
    var rows = hist ? hist.rows : [];
    var now = Date.now();
    var x0 = rows.length ? rows[0].t : now - 1000;
    var x1 = rows.length ? rows[rows.length - 1].t : now;
    var xf = (x1 - x0) > 86400000 ? fmtDay : fmtClockShort;
    function pts(k, i) { return rows.map(function (r) { return [r.t, r[k][i]]; }); }
    drawChart($('cHistTH'), { xmin: x0, xmax: x1, xfmt: xf, fmtL: fix1, fmtR: fix0, series: [
      { label: 'Temperature (\u00b0F)', color: C_TEMP, pts: pts('temp', 0), axis: 'l' },
      { label: 'Humidity (%)', color: C_HUM, pts: pts('hum', 0), axis: 'r' } ] });
    drawChart($('cHistVoc'), { xmin: x0, xmax: x1, xfmt: xf, floorL: true, minSpanL: 1, bands: BANDS, series: [
      { color: C_VOC, pts: pts('aqi', 1), axis: 'l', alpha: 0.45, w: 1 },
      { color: C_VOC, pts: pts('aqi', 2), axis: 'l', alpha: 0.45, w: 1 },
      { label: 'AQI-UBA (avg)', color: C_VOC, pts: pts('aqi', 0), axis: 'l' } ] });
    drawChart($('cHistGas'), { xmin: x0, xmax: x1, xfmt: xf, series: [
      { label: 'eCO2 (ppm)', color: C_CO2, pts: pts('eco2', 0), axis: 'l' },
      { label: 'eTVOC (ppb)', color: C_TVOC, pts: pts('etvoc', 0), axis: 'r' } ] });
  }

  function exportHistory() {
    if (!hist || !hist.rows.length) { toast('Nothing to export yet', 'err'); return; }
    var lines = ['timestamp_utc,temp_f_avg,temp_f_min,temp_f_max,humidity_avg,humidity_min,humidity_max,' +
      'aqi_uba_avg,aqi_uba_min,aqi_uba_max,eco2_ppm_avg,eco2_ppm_min,eco2_ppm_max,etvoc_ppb_avg,etvoc_ppb_min,etvoc_ppb_max'];
    hist.rows.forEach(function (r) {
      lines.push([new Date(r.t).toISOString(),
        csvNum(r.temp[0], 2), csvNum(r.temp[1], 2), csvNum(r.temp[2], 2),
        csvNum(r.hum[0], 2), csvNum(r.hum[1], 2), csvNum(r.hum[2], 2),
        csvNum(r.aqi[0], 0), csvNum(r.aqi[1], 0), csvNum(r.aqi[2], 0),
        csvNum(r.eco2[0], 0), csvNum(r.eco2[1], 0), csvNum(r.eco2[2], 0),
        csvNum(r.etvoc[0], 0), csvNum(r.etvoc[1], 0), csvNum(r.etvoc[2], 0)].join(','));
    });
    download('air-quality-history-' + stamp() + '.csv', lines.join('\n') + '\n');
  }

  function exportLive() {
    if (!live.length) { toast('Nothing to export yet', 'err'); return; }
    var lines = ['timestamp_utc,temp_f,humidity,aqi_uba,eco2_ppm,etvoc_ppb,ens161_status'];
    live.forEach(function (p) {
      lines.push([new Date(p.t).toISOString(), csvNum(p.temp, 2), csvNum(p.hum, 2), csvNum(p.aqi, 0),
        csvNum(p.eco2, 0), csvNum(p.etvoc, 0), p.status].join(','));
    });
    download('air-quality-live-' + stamp() + '.csv', lines.join('\n') + '\n');
  }

  // ---- controls ----
  function cmd(url) {
    getText(url).then(function (m) { toast(m, 'ok'); poll(); })
      .catch(function (e) { toast('Error: ' + e, 'err'); });
  }
  function wire(sel, attr, fn) {
    var els = document.querySelectorAll(sel);
    for (var i = 0; i < els.length; i++) {
      (function (el) { el.addEventListener('click', function () { fn(el.getAttribute(attr)); }); })(els[i]);
    }
  }
  wire('[data-goto]', 'data-goto', function (v) { cmd('/goto?degrees=' + v); });
  wire('[data-bright]', 'data-bright', function (v) { cmd('/brightness?pct=' + v); });
  wire('[data-speed]', 'data-speed', function (v) { cmd('/speed?direction=' + v); });
  wire('[data-mode]', 'data-mode', function (v) { cmd('/mode?auto=' + v); });
  $('btnRotate').addEventListener('click', function () {
    var v = $('degrees').value;
    if (v === '') { toast('Enter a degree value', 'err'); return; }
    cmd('/rotate?degrees=' + encodeURIComponent(v));
  });

  // ---- tabs ----
  var TABS = ['live', 'history', 'needle'];
  function showTab(name) {
    TABS.forEach(function (t) {
      $('tab-' + t).style.display = (t === name) ? 'block' : 'none';
      document.querySelector('nav button[data-tab="' + t + '"]').className = (t === name) ? 'on' : '';
    });
    if (name === 'history' && !hist) loadHistory();
    renderLive();
    renderHistory();
  }
  var tb = document.querySelectorAll('nav button');
  for (var ti = 0; ti < tb.length; ti++) {
    (function (b) { b.addEventListener('click', function () { showTab(b.getAttribute('data-tab')); }); })(tb[ti]);
  }

  $('depth').addEventListener('change', function () { depth = parseInt($('depth').value, 10); renderLive(); });
  $('histRange').addEventListener('change', loadHistory);
  $('btnHistLoad').addEventListener('click', loadHistory);
  $('btnHistCsv').addEventListener('click', exportHistory);
  $('btnLiveCsv').addEventListener('click', exportLive);
  window.addEventListener('resize', function () { renderLive(); renderHistory(); });

  // rating scale colours
  var segs = document.querySelectorAll('#scale .seg');
  for (var si = 0; si < segs.length; si++) segs[si].style.background = aqiColor(si + 1);

  poll();
  setInterval(poll, 5000);
})();
</script>
</body>
</html>
)rawliteral";

// ===================== SETUP / LOOP =====================

void setup() {
  Serial.begin(115200);
  delay(2000);
  Serial.println("\n===== AIRCUBE (BASE) PORT + STEPPER NEEDLE =====");

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  stopMotor();

  // LED fades up to ledBrightnessPct; it pulses blue until the first reading.
  strip.begin();
  strip.show();
  ledIntensityTarget = ledBrightnessPct / 100.0f;

  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setClock(100000);

  ens16xInit();
  sensorsStartMs = millis();          // warm-up clock starts when the ENS161 goes into STANDARD mode
  ens210Init();

  connectToWiFi();

  server.on("/", handleRoot);
  server.on("/status", handleStatus);
  server.on("/rotate", handleRotate);
  server.on("/goto", handleGoto);
  server.on("/mode", handleMode);
  server.on("/speed", handleSpeed);
  server.on("/history", handleHistory);
  server.on("/brightness", handleBrightness);
  server.begin();

  Serial.print("Open: http://");
  Serial.println(WiFi.localIP());

  lastLedMs = millis();
  lastWarmMsgMs = millis();
  histLastFlushMs = millis();
  historyResetAccum();
  Serial.println("Sensors are warming up: no readings for 3 minutes, then one reading per minute.");
}

void loop() {
  server.handleClient();

  unsigned long now = millis();

  if (now - lastLedMs >= LED_PERIOD_MS) {
    lastLedMs = now;
    ledTick();
  }

  // Warm-up: the sensors are running, but nothing is read for the first 3 minutes.
  if (!warmupDone()) {
    if (now - lastWarmMsgMs >= 30000UL) {
      lastWarmMsgMs = now;
      Serial.printf("[Warm-up] %lu s until the first reading\n", (WARMUP_MS - (now - sensorsStartMs)) / 1000UL);
    }
    return;
  }

  // First reading as soon as warm-up ends, then one per minute.
  if (!firstStepDone || (now - lastSensorMs) >= SENSOR_PERIOD_MS) {
    if (!firstStepDone) {
      firstStepDone = true;
      historyResetAccum();              // the first 5-minute history window starts with the first reading
    }
    lastSensorMs = now;
    sensorStep();
  }
}

// ===================== I2C HELPERS =====================

bool i2cProbe(uint8_t addr) {
  Wire.beginTransmission(addr);
  return Wire.endTransmission() == 0;
}

bool i2cWrite(uint8_t addr, const uint8_t* data, uint8_t len) {
  Wire.beginTransmission(addr);
  for (uint8_t i = 0; i < len; i++) {
    Wire.write(data[i]);
  }
  return Wire.endTransmission() == 0;
}

bool i2cRead(uint8_t addr, uint8_t reg, uint8_t* buf, uint8_t len) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if (Wire.endTransmission() != 0) return false;
  if (Wire.requestFrom(addr, len) != len) return false;
  for (uint8_t i = 0; i < len; i++) {
    buf[i] = Wire.read();
  }
  return true;
}

// ===================== ENS210 (ens210.c) =====================

void ens210Init() {
  ens210Present = false;

  Serial.print("ENS210 at 0x43... ");
  if (!i2cProbe(ENS210_ADDR)) {
    Serial.println("not present");
    return;
  }

  // Active mode, done before the PART_ID read so the device is awake.
  uint8_t sc[2] = {ENS210_REG_SYS_CTRL, 0x00};
  i2cWrite(ENS210_ADDR, sc, 2);
  delay(10);

  // PART_ID low 12 bits must be 0x210 (top nibble reads 0xA on real parts).
  uint8_t part[2] = {0, 0};
  if (i2cRead(ENS210_ADDR, ENS210_REG_PART_ID, part, 2)) {
    uint16_t partId = (uint16_t)part[0] | ((uint16_t)part[1] << 8);
    ens210Present = ((partId & 0x0FFF) == 0x0210);
    Serial.print("PART_ID 0x");
    Serial.print(partId, HEX);
    Serial.print(ens210Present ? " ok" : " UNEXPECTED");
  }
  if (!ens210Present) {
    Serial.println(" - not present");
    return;
  }

  // Continuous mode for temperature and humidity, then start.
  uint8_t run[2] = {ENS210_REG_SENS_RUN, 0x03};
  i2cWrite(ENS210_ADDR, run, 2);
  uint8_t start[2] = {ENS210_REG_SENS_START, 0x03};
  i2cWrite(ENS210_ADDR, start, 2);
  delay(250);

  uint8_t stat = 0;
  i2cRead(ENS210_ADDR, ENS210_REG_SYS_STAT, &stat, 1);
  Serial.print(", continuous mode, SYS_STAT 0x");
  Serial.println(stat, HEX);
}

// T_VAL / H_VAL are 3 bytes: [DATA_LSB, DATA_MSB, VALID(bit0)+CRC].
void ens210ReadEnvir() {
  ens210TempValid = false;
  ens210HumValid = false;
  if (!ens210Present) return;

  uint8_t d[3];

  memset(d, 0, sizeof(d));
  if (!i2cRead(ENS210_ADDR, ENS210_REG_T_VAL, d, 3)) {
    Serial.println("[ens210] Temperature read failed");
    return;
  }
  uint32_t tVal = (uint32_t)d[0] | ((uint32_t)d[1] << 8) | ((uint32_t)d[2] << 16);
  uint32_t tData = tVal & 0xFFFF;
  uint32_t tValid = (tVal >> 16) & 0x1;
  if (tValid) {
    ens210T[0] = d[0];
    ens210T[1] = d[1];
    float tinK = (float)tData / 64.0f;
    temperatureC = tinK - 273.15f;
    ens210TempValid = true;
  }

  memset(d, 0, sizeof(d));
  if (!i2cRead(ENS210_ADDR, ENS210_REG_H_VAL, d, 3)) {
    Serial.println("[ens210] Humidity read failed");
    return;
  }
  uint32_t hVal = (uint32_t)d[0] | ((uint32_t)d[1] << 8) | ((uint32_t)d[2] << 16);
  uint32_t hData = hVal & 0xFFFF;
  uint32_t hValid = (hVal >> 16) & 0x1;
  if (hValid) {
    ens210H[0] = d[0];
    ens210H[1] = d[1];
    humidityPct = (float)hData / 512.0f;
    ens210HumValid = true;
  }
}

// ===================== ENS16X (ens16x_driver.c) =====================

int ens16xGetDeviceStatus() {
  uint8_t b = 0;
  if (!i2cRead(ENS16X_ADDR, ENS16X_DEVICE_STATUS, &b, 1)) {
    ens16xStatus = ENS_NO_VALID_OUTPUT;
    return -1;
  }
  // bit 1 NEWDAT, bits 2-3 VALIDITY (0 OK, 1 warm-up, 2 initial start-up, 3 no valid output)
  ens16xStatus = (b >> 2) & 0x03;
  return b;
}

int ens16xGetOpmode() {
  uint8_t b = 0xFF;
  i2cRead(ENS16X_ADDR, ENS16X_OPMODE, &b, 1);
  return b;
}

void ens16xSetOpmode(int mode) {
  // Go through IDLE first, then to the requested mode, then confirm.
  uint8_t idle[2] = {ENS16X_OPMODE, ENS_IDLE};
  i2cWrite(ENS16X_ADDR, idle, 2);
  delay(10);
  uint8_t want[2] = {ENS16X_OPMODE, (uint8_t)mode};
  i2cWrite(ENS16X_ADDR, want, 2);
  delay(10);

  if (ens16xGetOpmode() == mode) {
    Serial.println("[ens16x] Mode change success");
  } else {
    Serial.println("[ens16x] Error changing modes");
  }
}

int ens16xReadEtvoc() {
  uint8_t b[2] = {0, 0};
  if (!i2cRead(ENS16X_ADDR, ENS16X_REG_DATA_ETVOC, b, 2)) {
    ens16xTvoc = -1;
    return -1;
  }
  ens16xTvoc = (int)((uint16_t)b[0] | ((uint16_t)b[1] << 8));   // raw, no offset
  return ens16xTvoc;
}

int ens16xReadEco2() {
  uint8_t b[2] = {0, 0};
  if (!i2cRead(ENS16X_ADDR, ENS16X_REG_DATA_ECO2, b, 2)) {
    ens16xEco2 = -1;
    return -1;
  }
  ens16xEco2 = (int)((uint16_t)b[0] | ((uint16_t)b[1] << 8));   // raw, no offset (chip already floors at 400)
  return ens16xEco2;
}

// AQI-UBA is bits 0-2 of register 0x21: the chip's own 1-5 hygienic rating, computed
// onboard from its internal (uncorrected) eTVOC signal. Not affected by our offsets.
int ens16xReadAqiUba() {
  uint8_t b = 0;
  if (!i2cRead(ENS16X_ADDR, ENS16X_REG_DATA_AQI_UBA, &b, 1)) {
    ens16xAqiUba = -1;
    return -1;
  }
  ens16xAqiUba = b & 0x07;
  if (ens16xAqiUba < AQI_UBA_MIN || ens16xAqiUba > AQI_UBA_MAX) {
    ens16xAqiUba = -1;   // outside the defined 1-5 range (e.g. still warming up)
  }
  return ens16xAqiUba;
}

// Writes TEMP_IN (1/64 K) and RH_IN (1/512 %) in one transaction, like the firmware.
void ens16xWriteEns210Data(const uint8_t* t, const uint8_t* h) {
  uint8_t d[5] = {ENS16X_REG_TH_IN, t[0], t[1], h[0], h[1]};
  if (!i2cWrite(ENS16X_ADDR, d, 5)) {
    Serial.println("[ens16x] Compensation write failed; VOC output uses stale temp/RH");
  }
}

const char* ens16xStatusStr(int s) {
  switch (s) {
    case ENS_OP_OK: return "OK";
    case ENS_WARM_UP: return "Warming Up";
    case ENS_NO_VALID_OUTPUT: return "No Valid Output";
    case ENS_RESERVED: return "Reserved";
    default: return "Unknown";
  }
}

void ens16xInit() {
  Serial.print("ENS16X at 0x52... ");
  if (!i2cProbe(ENS16X_ADDR)) {
    Serial.println("NOT FOUND");
    ens16xPresent = false;
    return;
  }
  ens16xPresent = true;

  uint8_t id[2] = {0, 0};
  i2cRead(ENS16X_ADDR, 0x00, id, 2);
  uint16_t partId = (uint16_t)id[0] | ((uint16_t)id[1] << 8);
  Serial.print("part ID 0x");
  Serial.print(partId, HEX);

  ens16xGetDeviceStatus();
  Serial.print(", status: ");
  Serial.println(ens16xStatusStr(ens16xStatus));

  if (ens16xGetOpmode() != ENS_STANDARD) {
    Serial.println("[ens16x] setting mode to STANDARD");
    ens16xSetOpmode(ENS_STANDARD);
  } else {
    Serial.println("[ens16x] operating mode already STANDARD");
  }
  // AirCube reads eTVOC once here; we skip it so nothing is read during warm-up.
}

// ===================== AQI-UBA =====================

// AQI-UBA (1-5) -> band index 0..4 (Excellent .. Unhealthy).
int aqiUbaToBand(int aqiUba) {
  if (aqiUba < AQI_UBA_MIN) aqiUba = AQI_UBA_MIN;
  if (aqiUba > AQI_UBA_MAX) aqiUba = AQI_UBA_MAX;
  return aqiUba - 1;
}

// ===================== SENSOR STEP (sensor_task, Base branch) =====================

void sensorStep() {
  // Base: ENS210 temperature and humidity.
  ens210ReadEnvir();
  bool tempValid = ens210TempValid;
  bool humValid = ens210HumValid;
  tempCOut = temperatureC - TEMP_OFFSET_C;

  // Compensation for the ENS161, only while both channels are current.
  if (tempValid && humValid) {
    uint8_t compT[2] = {ens210T[0], ens210T[1]};
    uint8_t compH[2] = {ens210H[0], ens210H[1]};
    if (COMPENSATE_WITH_CORRECTED_TEMP) {
      uint16_t tRaw = (uint16_t)((tempCOut + 273.15f) * 64.0f);
      compT[0] = (uint8_t)(tRaw & 0xFF);
      compT[1] = (uint8_t)(tRaw >> 8);
    }
    ens16xWriteEns210Data(compT, compH);

    // The ENS161 applies new temperature/humidity on its next 1 s measurement and we only
    // read once a minute, so give it a moment first (the page keeps being served meanwhile).
    unsigned long waitStart = millis();
    while (millis() - waitStart < 1200UL) {
      server.handleClient();
      delay(5);
    }
  }

  ens16xGetDeviceStatus();

  int etvoc = ens16xReadEtvoc();
  ens16xReadEco2();
  int aqiUba = ens16xReadAqiUba();   // chip's own 1-5 rating

  // Keep the last known rating rather than jumping to "best" on a failed/out-of-range read.
  if (aqiUba >= AQI_UBA_MIN) {
    currentAqiUba = aqiUba;
    haveReading = true;
    readingSeq++;
    lastReadingMs = millis();
  }

  // Feed the 5-minute history (aqiUba, eCO2 and eTVOC are -1 when a read failed).
  historyRecordSample(tempCOut, tempValid, humidityPct, humValid, aqiUba, ens16xEco2, etvoc);
  historyCheckFlush();

  printStatus();

  if (autoMode) {
    updateGauge(aqiUba);
  }
}

void printStatus() {
  int band = aqiUbaToBand(currentAqiUba);
  Serial.printf("[AQI-UBA %d %s] ENS161 %s | eTVOC=%dppb eCO2=%dppm | %.1fF %.1f%%RH | needle %.1f deg | %s\n",
                currentAqiUba, BAND_NAME[band], ens16xStatusStr(ens16xStatus),
                ens16xTvoc, ens16xEco2, tempCOut * 1.8f + 32.0f, humidityPct,
                currentPositionDegrees(), autoMode ? "AUTO" : "MANUAL");
}

// ===================== HISTORY (history.c) =====================

void historyResetAccum() {
  memset(&accum, 0, sizeof(accum));
  accum.minT = INT16_MAX;
  accum.maxT = INT16_MIN;
  accum.minH = INT16_MAX;
  accum.maxH = INT16_MIN;
  accum.minQ = UINT16_MAX;
  accum.minC = UINT16_MAX;
  accum.minV = UINT16_MAX;
  accum.windowStartMs = millis();
}

int16_t floatToX100(float val) {
  float scaled = val * 100.0f;
  if (scaled > (float)INT16_MAX) return INT16_MAX;
  if (scaled < (float)INT16_MIN) return INT16_MIN;
  return (int16_t)(scaled + (scaled >= 0 ? 0.5f : -0.5f));
}

uint16_t clampU16(int val) {
  if (val < 0) return 0;
  if (val > UINT16_MAX) return UINT16_MAX;
  return (uint16_t)val;
}

// Only values the caller vouches for are recorded; negative aqi/eco2/etvoc means "unavailable".
void historyRecordSample(float tempC, bool tempValid, float hum, bool humValid,
                         int aqi, int eco2, int etvoc) {
  if (tempValid) {
    int16_t t = floatToX100(tempC);
    accum.sumT += tempC;
    if (t < accum.minT) accum.minT = t;
    if (t > accum.maxT) accum.maxT = t;
    accum.tCount++;
  }
  if (humValid) {
    int16_t h = floatToX100(hum);
    accum.sumH += hum;
    if (h < accum.minH) accum.minH = h;
    if (h > accum.maxH) accum.maxH = h;
    accum.hCount++;
  }
  if (aqi >= 0) {
    uint16_t q = clampU16(aqi);
    accum.sumQ += q;
    if (q < accum.minQ) accum.minQ = q;
    if (q > accum.maxQ) accum.maxQ = q;
    accum.qCount++;
  }
  if (eco2 >= 0) {
    uint16_t c = clampU16(eco2);
    accum.sumC += c;
    if (c < accum.minC) accum.minC = c;
    if (c > accum.maxC) accum.maxC = c;
    accum.cCount++;
  }
  if (etvoc >= 0) {
    uint16_t v = clampU16(etvoc);
    accum.sumV += v;
    if (v < accum.minV) accum.minV = v;
    if (v > accum.maxV) accum.maxV = v;
    accum.vCount++;
  }
  accum.samples++;
}

// Once the 5-minute window has elapsed, save min/avg/max as one slot (channels with no
// valid samples are stored as "no data" instead of an invented number).
bool historyCheckFlush() {
  if (accum.samples == 0) return false;
  if ((millis() - accum.windowStartMs) < HISTORY_WINDOW_MS) return false;

  HistorySlot& slot = histSlots[histWrite];
  slot.sequence = histNextSeq;

  if (accum.tCount > 0) {
    slot.t_a = floatToX100(accum.sumT / (float)accum.tCount);
    slot.t_n = accum.minT;
    slot.t_x = accum.maxT;
  } else {
    slot.t_a = slot.t_n = slot.t_x = HISTORY_NO_DATA_S16;
  }
  if (accum.hCount > 0) {
    slot.h_a = floatToX100(accum.sumH / (float)accum.hCount);
    slot.h_n = accum.minH;
    slot.h_x = accum.maxH;
  } else {
    slot.h_a = slot.h_n = slot.h_x = HISTORY_NO_DATA_S16;
  }
  if (accum.qCount > 0) {
    slot.q_a = (uint16_t)(accum.sumQ / accum.qCount);
    slot.q_n = accum.minQ;
    slot.q_x = accum.maxQ;
  } else {
    slot.q_a = slot.q_n = slot.q_x = HISTORY_NO_DATA_U16;
  }
  if (accum.cCount > 0) {
    slot.c_a = (uint16_t)(accum.sumC / accum.cCount);
    slot.c_n = accum.minC;
    slot.c_x = accum.maxC;
  } else {
    slot.c_a = slot.c_n = slot.c_x = HISTORY_NO_DATA_U16;
  }
  if (accum.vCount > 0) {
    slot.v_a = (uint16_t)(accum.sumV / accum.vCount);
    slot.v_n = accum.minV;
    slot.v_x = accum.maxV;
  } else {
    slot.v_a = slot.v_n = slot.v_x = HISTORY_NO_DATA_U16;
  }

  Serial.printf("[History] Saved entry %u (seq %u, %lu samples, VOC avg %u)\n",
                histWrite, slot.sequence, (unsigned long)accum.samples, slot.q_a);

  histWrite = (histWrite + 1) % HISTORY_CAPACITY;
  if (histCount < HISTORY_CAPACITY) histCount++;
  histNextSeq++;
  if (histNextSeq == 0xFFFF) histNextSeq = 0;      // 0xFFFF marks an empty slot in AirCube
  histLastFlushMs = millis();

  historyResetAccum();
  return true;
}

// ===================== NEEDLE LOGIC =====================

bool warmupDone() {
  return (millis() - sensorsStartMs) >= WARMUP_MS;
}

bool sensorReady() {
  return ens16xPresent && haveReading && ens16xStatus == ENS_OP_OK && ens16xTvoc >= 0;
}

// Move the needle only after a new band has held for STABLE_SAMPLES readings.
// Like the AirCube status flag, nothing moves until the ENS161 reports OK.
void updateGauge(int aqiUba) {
  if (!sensorReady() || aqiUba < AQI_UBA_MIN) return;

  int band = aqiUbaToBand(currentAqiUba);

  if (band == appliedBand) {
    candidateCount = 0;
    return;
  }

  if (band == candidateBand) {
    candidateCount++;
  } else {
    candidateBand = band;
    candidateCount = 1;
  }

  if (candidateCount >= STABLE_SAMPLES) {
    Serial.printf("[Gauge] %s -> %.0f deg\n", BAND_NAME[band], BAND_ANGLE[band]);
    moveToAngle(BAND_ANGLE[band]);
    appliedBand = band;
    candidateCount = 0;
  }
}

// ===================== LED (main.c loop + led.c + led_color_lib.c) =====================

// AQI-UBA 1 (Excellent) -> green, 5 (Unhealthy) -> red, linear in between.
uint16_t aqiToHue(int aqiUba) {
  if (aqiUba < AQI_UBA_MIN) aqiUba = AQI_UBA_MIN;
  if (aqiUba > AQI_UBA_MAX) aqiUba = AQI_UBA_MAX;

  float ratio = (float)(aqiUba - AQI_UBA_MIN) / (float)(AQI_UBA_MAX - AQI_UBA_MIN);
  uint16_t hue = HUE_GREEN - (uint16_t)(ratio * HUE_GREEN);
  return hue;
}

void hueToRgb(float h, float* r, float* g, float* b) {
  float x = 1 - fabsf(fmodf(h * 6, 2) - 1);

  if (h < 1.0f / 6.0f)      { *r = 1; *g = x; *b = 0; }
  else if (h < 2.0f / 6.0f) { *r = x; *g = 1; *b = 0; }
  else if (h < 3.0f / 6.0f) { *r = 0; *g = 1; *b = x; }
  else if (h < 4.0f / 6.0f) { *r = 0; *g = x; *b = 1; }
  else if (h < 5.0f / 6.0f) { *r = x; *g = 0; *b = 1; }
  else                      { *r = 1; *g = 0; *b = x; }
}

void ledTick() {
  uint8_t r8, g8, b8;

  if (!haveReading) {
    // No reading yet (sensors warming up): slow blue pulse instead of a misleading green.
    float pulse = 0.35f + 0.65f * (0.5f + 0.5f * sinf(millis() * 0.00157f));
    r8 = 0;
    g8 = 0;
    b8 = (uint8_t)(255.0f * pulse);
  } else {
    // Color: smooth transition toward the hue for the current AQI-UBA rating.
    targetHue = (currentAqiUba > 0) ? aqiToHue(currentAqiUba) : HUE_GREEN;
    float hueDiff = (float)targetHue - currentHue;
    currentHue += hueDiff * TRANSITION_SPEED;

    float r, g, b;
    hueToRgb((uint16_t)currentHue / 65536.0f, &r, &g, &b);
    r8 = (uint8_t)(r * 255);
    g8 = (uint8_t)(g * 255);
    b8 = (uint8_t)(b * 255);
  }

  // Brightness: ramp the displayed intensity toward the target over INTENSITY_RAMP_MS.
  float diff = ledIntensityTarget - ledIntensity;
  if (fabsf(diff) < 0.002f) {
    ledIntensity = ledIntensityTarget;
  } else {
    const float step = 1.0f / ((float)INTENSITY_RAMP_MS / (float)LED_PERIOD_MS);
    if (diff > 0.0f) {
      ledIntensity += (diff < step) ? diff : step;
    } else {
      ledIntensity += (diff > -step) ? diff : -step;
    }
  }

  int fr = (int)(r8 * ledIntensity + 0.5f);
  int fg = (int)(g8 * ledIntensity + 0.5f);
  int fb = (int)(b8 * ledIntensity + 0.5f);

  // Only push to the LED when the output actually changed.
  if (fr != lastR || fg != lastG || fb != lastB) {
    lastR = fr;
    lastG = fg;
    lastB = fb;
    strip.setPixelColor(0, strip.Color(fr, fg, fb));
    strip.show();
  }
}

// ===================== WIFI / WEB =====================

void connectToWiFi() {
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);

  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 40) {
    delay(500);
    Serial.print(".");
    attempts++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi connected");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("\nWiFi connection FAILED - check SSID/password");
  }
}

void handleRoot() {
  server.send(200, "text/html", INDEX_HTML);
}

void handleStatus() {
  int band = aqiUbaToBand(currentAqiUba);

  unsigned long nowMs = millis();
  unsigned long warmLeftS = 0;
  if (!warmupDone()) warmLeftS = (WARMUP_MS - (nowMs - sensorsStartMs)) / 1000UL;
  unsigned long nextInS = 0;
  if (firstStepDone && (nowMs - lastSensorMs) < SENSOR_PERIOD_MS) nextInS = (SENSOR_PERIOD_MS - (nowMs - lastSensorMs)) / 1000UL;
  unsigned long ageS = haveReading ? (nowMs - lastReadingMs) / 1000UL : 0UL;

  String j = "{";
  j += "\"aqi_uba\":" + String(currentAqiUba);
  j += ",\"band\":" + String(band);
  j += ",\"etvoc\":" + String(ens16xTvoc);
  j += ",\"eco2\":" + String(ens16xEco2);
  j += ",\"temp\":" + String(tempCOut * 1.8f + 32.0f, 1);
  j += ",\"hum\":" + String(humidityPct, 1);
  j += ",\"angle\":" + String(currentPositionDegrees(), 1);
  j += ",\"auto\":" + String(autoMode ? "true" : "false");
  j += ",\"status\":\"" + String(ens16xStatusStr(ens16xStatus)) + "\"";
  j += ",\"ready\":" + String(sensorReady() ? "true" : "false");
  j += ",\"brightness\":" + String(ledBrightnessPct);
  j += ",\"uptime_s\":" + String(millis() / 1000UL);
  j += ",\"hist\":" + String((int)histCount);
  j += ",\"have\":" + String(haveReading ? "true" : "false");
  j += ",\"seq\":" + String(readingSeq);
  j += ",\"age_s\":" + String(ageS);
  j += ",\"warmup_s\":" + String(warmLeftS);
  j += ",\"next_s\":" + String(nextInS);
  j += "}";
  server.send(200, "application/json", j);
}

void handleRotate() {
  if (!server.hasArg("degrees")) {
    server.send(400, "text/plain", "Missing degrees parameter");
    return;
  }
  autoMode = false;
  float req = server.arg("degrees").toFloat();
  server.send(200, "text/plain", moveRelative(req));
}

void handleGoto() {
  if (!server.hasArg("degrees")) {
    server.send(400, "text/plain", "Missing degrees parameter");
    return;
  }
  autoMode = false;
  float target = server.arg("degrees").toFloat();
  server.send(200, "text/plain", moveToAngle(target));
}

void handleMode() {
  bool wantAuto = server.arg("auto") == "1";
  autoMode = wantAuto;

  String msg;
  if (wantAuto) {
    if (sensorReady()) {
      int band = aqiUbaToBand(currentAqiUba);
      moveToAngle(BAND_ANGLE[band]);
      appliedBand = band;
      candidateCount = 0;
      msg = "Auto mode: needle set to " + String(BAND_NAME[band]);
    } else {
      appliedBand = -1;
      msg = "Auto mode on (waiting for ENS161 status OK)";
    }
  } else {
    msg = "Manual mode";
  }
  server.send(200, "text/plain", msg);
}

// Streams the newest n entries (oldest first) as compact JSON, using AirCube's column names.
// The page works out timestamps: the newest entry is newest_age_s old, entries are 5 minutes apart.
void handleHistory() {
  int n = HISTORY_CAPACITY;
  if (server.hasArg("n")) n = server.arg("n").toInt();
  if (n > (int)histCount) n = histCount;
  if (n < 0) n = 0;

  unsigned long newestAgeS = (millis() - histLastFlushMs) / 1000UL;

  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  server.send(200, "application/json", "");

  char buf[1400];
  int pos = snprintf(buf, sizeof(buf),
    "{\"window_s\":%lu,\"capacity\":%d,\"entries\":%u,\"returned\":%d,\"newest_age_s\":%lu,"
    "\"cols\":[\"seq\",\"t_a\",\"t_n\",\"t_x\",\"h_a\",\"h_n\",\"h_x\",\"q_a\",\"q_n\",\"q_x\","
    "\"c_a\",\"c_n\",\"c_x\",\"v_a\",\"v_n\",\"v_x\"],\"history\":[",
    HISTORY_WINDOW_MS / 1000UL, HISTORY_CAPACITY, (unsigned)histCount, n, newestAgeS);

  for (int i = 0; i < n; i++) {
    // logical 0 = oldest valid entry; the newest sits just before histWrite
    int logical = (int)histCount - n + i;
    int idx = ((int)histWrite + HISTORY_CAPACITY - (int)histCount + logical) % HISTORY_CAPACITY;
    const HistorySlot& s = histSlots[idx];

    if (pos > (int)sizeof(buf) - 130) {
      buf[pos] = 0;
      server.sendContent(buf);
      pos = 0;
    }
    pos += snprintf(buf + pos, sizeof(buf) - pos,
      "%s[%u,%d,%d,%d,%d,%d,%d,%u,%u,%u,%u,%u,%u,%u,%u,%u]",
      i ? "," : "", s.sequence,
      s.t_a, s.t_n, s.t_x, s.h_a, s.h_n, s.h_x,
      s.q_a, s.q_n, s.q_x, s.c_a, s.c_n, s.c_x, s.v_a, s.v_n, s.v_x);
  }

  pos += snprintf(buf + pos, sizeof(buf) - pos, "]}");
  buf[pos] = 0;
  server.sendContent(buf);
  server.sendContent("");                 // zero-length chunk ends the response
}

void handleBrightness() {
  if (!server.hasArg("pct")) {
    server.send(400, "text/plain", "Missing pct parameter");
    return;
  }
  int pct = server.arg("pct").toInt();
  if (pct < 0) pct = 0;
  if (pct > 100) pct = 100;
  ledBrightnessPct = pct;
  ledIntensityTarget = pct / 100.0f;
  server.send(200, "text/plain", "LED brightness " + String(pct) + "%");
}

void handleSpeed() {
  String dir = server.arg("direction");
  if (dir == "faster") {
    motorSpeed = max(1, motorSpeed - 1);
  } else if (dir == "slower") {
    motorSpeed = min(20, motorSpeed + 1);
  } else {
    server.send(400, "text/plain", "Bad direction");
    return;
  }
  server.send(200, "text/plain", "Speed: " + String(motorSpeed) + " ms/step");
}

// ===================== STEPPER =====================

float currentPositionDegrees() {
  return currentSteps * DEGREES_PER_STEP;
}

String moveRelative(float requestedDegrees) {
  return moveToSteps(currentSteps + (long)round(requestedDegrees / DEGREES_PER_STEP));
}

String moveToAngle(float degrees) {
  return moveToSteps((long)round(degrees / DEGREES_PER_STEP));
}

// Move to an absolute step count, clamped to 0..MAX_STEPS.
String moveToSteps(long target) {
  bool limited = false;
  if (target > MAX_STEPS) {
    target = MAX_STEPS;
    limited = true;
  }
  if (target < 0) {
    target = 0;
    limited = true;
  }

  long delta = target - currentSteps;
  if (delta != 0) {
    moveSteps(delta);
  }

  String msg;
  if (delta == 0 && limited) {
    msg = "At limit - cannot move further";
  } else if (limited) {
    msg = "Limited to range. Now at " + String(currentPositionDegrees(), 1) + " deg";
  } else if (delta == 0) {
    msg = "Already at " + String(currentPositionDegrees(), 1) + " deg";
  } else {
    msg = "Now at " + String(currentPositionDegrees(), 1) + " deg";
  }
  return msg;
}

// Positive delta = counter-clockwise (flip with DIRECTION_SIGN).
void moveSteps(long delta) {
  int dir = (delta > 0) ? 1 : -1;
  dir *= DIRECTION_SIGN;
  long count = labs(delta);

  for (long i = 0; i < count; i++) {
    digitalWrite(IN1, stepSequence[stepIndex][0]);
    digitalWrite(IN2, stepSequence[stepIndex][1]);
    digitalWrite(IN3, stepSequence[stepIndex][2]);
    digitalWrite(IN4, stepSequence[stepIndex][3]);

    stepIndex = (stepIndex + dir + 4) % 4;
    delay(motorSpeed);
  }

  currentSteps += delta;
  stopMotor();
}

void stopMotor() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}
