#include "web.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"

#include "rtcm.h"

static const char *TAG = "gps_web";
static httpd_handle_t s_server;


/* ------------------------------------------------------------------- index */

static const char INDEX_HTML[] =
"<!DOCTYPE html><html lang='zh'><head><meta charset='utf-8'>"
"<meta name='viewport' content='width=device-width,initial-scale=1'>"
"<title>USV GPS/RTK 模拟器</title><style>"
":root{--bg:#0f1417;--card:#171f24;--ink:#e6edf3;--dim:#8b9aa6;--ok:#3fb950;--warn:#d29922;--bad:#f85149;--acc:#58a6ff}"
"*{box-sizing:border-box}body{margin:0;font:14px/1.5 system-ui,Segoe UI,sans-serif;background:var(--bg);color:var(--ink)}"
"header{padding:14px 18px;border-bottom:1px solid #253037;display:flex;gap:16px;align-items:baseline;flex-wrap:wrap}"
"h1{font-size:16px;margin:0;font-weight:600}#ip{color:var(--dim);font-size:12px}"
"main{padding:16px;display:grid;gap:14px;grid-template-columns:repeat(auto-fit,minmax(320px,1fr));max-width:1200px;margin:0 auto}"
".card{background:var(--card);border:1px solid #253037;border-radius:10px;padding:14px}"
".card h2{font-size:13px;margin:0 0 10px;color:var(--dim);font-weight:600;letter-spacing:.04em;text-transform:uppercase}"
".row{display:flex;justify-content:space-between;padding:3px 0;font-variant-numeric:tabular-nums}"
".row span:first-child{color:var(--dim)}"
".pill{padding:1px 8px;border-radius:99px;font-size:12px;font-weight:600}"
".p-ok{background:#13361d;color:var(--ok)}.p-warn{background:#3a2d0b;color:var(--warn)}.p-bad{background:#3d1613;color:var(--bad)}"
"label{display:block;margin:8px 0 3px;color:var(--dim);font-size:12px}"
"input,select{width:100%;padding:7px 9px;border-radius:6px;border:1px solid #2d3a42;background:#0f1417;color:var(--ink);font:inherit}"
"input[type=range]{padding:0}"
"button{margin-top:10px;padding:8px 14px;border-radius:6px;border:1px solid #2d3a42;background:#1f2a31;color:var(--ink);font:inherit;cursor:pointer}"
"button:hover{background:#27343c}button.primary{background:#1f6feb;border-color:#1f6feb}"
"table{width:100%;border-collapse:collapse;font-size:13px}td{padding:3px 0;font-variant-numeric:tabular-nums}"
"td:last-child{text-align:right;color:var(--dim)}#msg{color:var(--acc);font-size:12px;min-height:16px}"
"</style></head><body>"
"<header><h1>USV GPS/RTK 模拟器</h1><span id='ip'></span><span id='msg'></span></header>"
"<main>"
"<section class='card'><h2>接收机状态</h2><div id='stat'></div></section>"
"<section class='card'><h2>差分 (RTCM) 输入</h2><div id='rtcm'></div>"
"<table id='rtcmtypes'></table></section>"
"<section class='card'><h2>位置与运动</h2>"
"<label>原点纬度</label><input id='olat' type='number' step='0.0000001'>"
"<label>原点经度</label><input id='olon' type='number' step='0.0000001'>"
"<label>运动模式</label><select id='mode'>"
"<option value='0'>静止</option><option value='1'>圆周</option>"
"<option value='2'>直线</option><option value='3'>航点</option></select>"
"<label>航向 (°)</label><input id='hdg' type='number' step='0.1'>"
"<label>速度 (m/s)</label><input id='spd' type='number' step='0.1'>"
"<button class='primary' onclick='applyMotion()'>应用</button></section>"
"<section class='card'><h2>接收机质量</h2>"
"<label>Fix 质量</label><select id='fix'>"
"<option value='0'>无定位</option><option value='1'>单点 3D (1.5m)</option>"
"<option value='2'>DGPS (0.5m)</option><option value='5'>RTK Float (0.2m)</option>"
"<option value='4'>RTK Fixed (1cm)</option></select>"
"<label>卫星数 <b id='satv'></b></label><input id='sat' type='range' min='0' max='40' step='1'>"
"<label>HDOP <b id='hdopv'></b></label><input id='hdop' type='range' min='0.5' max='9.9' step='0.1'>"
"<label>基线长度 (m)</label><input id='base' type='number' step='0.1'>"
"<label>水平位置噪声 σ (m, RTK≈0.02, 0=关)</label><input id='pnoise' type='number' min='0' step='0.01'>"
"<label>航向噪声 σ (°, RTK≈0.3, 0=关)</label><input id='hnoise' type='number' min='0' step='0.1'>"
"<label>HDT 航向语句</label><select id='hen'><option value='1'>输出</option><option value='0'>关闭</option></select>"
"<label>输出周期 (ms)</label><input id='rate' type='number' step='50'>"
"<label>UART 波特率</label><select id='baud'>"
"<option>57600</option><option>115200</option><option>230400</option><option>460800</option><option>921600</option></select>"
"<button class='primary' onclick='applyRcvr()'>应用</button></section>"
"</main><script>"
"const $=id=>document.getElementById(id);"
"async function j(u){const r=await fetch(u);return r.json()}"
"async function post(u,o){const r=await fetch(u,{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(o)});$('msg').textContent=(await r.text());setTimeout(()=>$('msg').textContent='',2500)}"
"function pill(t,c){return `<span class='pill ${c}'>${t}</span>`}"
"const FIXN={0:'无定位',1:'单点 3D',2:'DGPS',4:'RTK Fixed',5:'RTK Float'};"
"async function tick(){try{const s=await j('/api/status');"
"$('ip').textContent=`${s.ip||s.ap_ssid} · UART ${s.baud}`;"
"const q=s.fix_quality,c=q==2?'p-ok':q==1?'p-warn':'p-bad';"
"$('stat').innerHTML=["
"['Fix',pill(FIXN[q]||q,c)],['纬度',s.lat.toFixed(8)],['经度',s.lon.toFixed(8)],"
"['航向',s.heading_deg.toFixed(2)+' °'],['航向精度',s.yaw_acc_deg.toFixed(3)+' ° (1σ)'],"
"['速度',s.speed_mps.toFixed(2)+' m/s'],['卫星',s.satellites],['HDOP',s.hdop.toFixed(1)],"
"['基线',s.baseline_m.toFixed(2)+' m'],['输出周期',s.output_rate_ms+' ms']]"
".map(([k,v])=>`<div class='row'><span>${k}</span><span>${v}</span></div>`).join('');"
"const age=s.rtcm_age_ms,rk=s.rtk_ready;"
"$('rtcm').innerHTML=[['累计差分字节',s.rtcm_bytes],['RTCM 帧数',s.rtcm_frames],"
"['CRC 错误',s.rtcm_crc_errors],['最近帧类型',s.rtcm_last_type||'-'],"
"['最近帧年龄',age<0?'从未收到':age+' ms'],"
"['链路',pill(rk?'正常':'中断',rk?'p-ok':'p-bad')]]"
".map(([k,v])=>`<div class='row'><span>${k}</span><span>${v}</span></div>`).join('');"
"$('rtcmtypes').innerHTML=(s.rtcm_types||[]).map(t=>`<tr><td>${t.type}</td><td>${t.name}</td><td>${t.count}</td></tr>`).join('');"
"if(document.activeElement.tagName!=='INPUT'){"
"$('olat').value=s.origin_lat;$('olon').value=s.origin_lon;$('mode').value=s.mode;"
"$('hdg').value=s.heading_deg.toFixed(1);$('spd').value=s.speed_mps.toFixed(2);"
"$('fix').value=s.fix_quality;$('sat').value=s.satellites;$('hdop').value=s.hdop;"
"$('base').value=s.baseline_m;$('hen').value=s.heading_enabled?'1':'0';"
"$('pnoise').value=s.pos_noise_m.toFixed(2);$('hnoise').value=s.hdg_noise_deg.toFixed(1);"
"$('rate').value=s.output_rate_ms;}"
"$('satv').textContent=s.satellites;$('hdopv').textContent=s.hdop.toFixed(1);"
"}catch(e){$('ip').textContent='连接断开'}}"
"function applyMotion(){post('/api/set',{origin_lat:+$('olat').value,origin_lon:+$('olon').value,"
"mode:+$('mode').value,heading_deg:+$('hdg').value,speed_mps:+$('spd').value})}"
"function applyRcvr(){post('/api/set',{fix_quality:+$('fix').value,satellites:+$('sat').value,"
"hdop:+$('hdop').value,baseline_len_m:+$('base').value,"
"position_noise_m:+$('pnoise').value,heading_noise_deg:+$('hnoise').value,"
"heading_enabled:+$('hen').value,"
"output_rate_ms:+$('rate').value,baud:+$('baud').value})}"
"setInterval(tick,1000);tick();"
"</script></body></html>";

/* ------------------------------------------------------------------ helpers */

static esp_err_t send_json_str(httpd_req_t *req, const char *json)
{
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    return httpd_resp_sendstr(req, json);
}

/* Minimal JSON number extraction: the API only carries flat numeric/boolean
   fields, so a full parser would be dead weight in flash. */
static bool json_num(const char *body, const char *key, double *out)
{
    char pat[40];
    snprintf(pat, sizeof(pat), "\"%s\"", key);
    const char *p = strstr(body, pat);
    if (!p) {
        return false;
    }
    p += strlen(pat);
    while (*p == ' ' || *p == ':' || *p == '\t') {
        p++;
    }
    if (*p == '\0') {
        return false;
    }
    char *end = NULL;
    const double v = strtod(p, &end);
    if (end == p) {
        return false;
    }
    *out = v;
    return true;
}

/* ----------------------------------------------------------------- handlers */

static esp_err_t h_index(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html; charset=utf-8");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    return httpd_resp_send(req, INDEX_HTML, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t h_status(httpd_req_t *req)
{
    char buf[2048];
    double lat, lon;

    gps_sim_state_lock();
    sim_state_t s = *gps_sim_state();
    gps_sim_state_unlock();

    link_state_t lk;
    lk = *gps_sim_link();

    rtcm_parser_t *rp = gps_sim_rtcm();
    sim_position(&s, &lat, &lon);

    const uint32_t last = gps_sim_last_rtcm_ms();
    const int64_t now = esp_timer_get_time() / 1000;
    const long age = (last == 0) ? -1 : (long)(now - (int64_t)last);

    int n = snprintf(buf, sizeof(buf),
        "{\"origin_lat\":%.7f,\"origin_lon\":%.7f,\"lat\":%.7f,\"lon\":%.7f,"
        "\"north_m\":%.2f,\"east_m\":%.2f,\"heading_deg\":%.2f,\"speed_mps\":%.2f,"
        "\"mode\":%d,\"fix_quality\":%d,\"satellites\":%d,\"hdop\":%.1f,"
        "\"baseline_m\":%.2f,\"heading_enabled\":%d,\"output_rate_ms\":%d,"
        "\"yaw_acc_deg\":%.3f,\"h_acc_m\":%.3f,"
        "\"pos_noise_m\":%.3f,\"hdg_noise_deg\":%.3f,"
        "\"rtcm_bytes\":%u,\"rtcm_frames\":%u,\"rtcm_crc_errors\":%u,"
        "\"rtcm_last_type\":%u,\"rtcm_age_ms\":%ld,\"rtk_ready\":%d,"
        "\"baud\":%u,\"ip\":\"%s\",\"ap_ssid\":\"%s\",\"sta_ssid\":\"%s\","
        "\"free_heap\":%u,\"rtcm_types\":[",
        s.origin_lat, s.origin_lon, lat, lon,
        s.north_m, s.east_m, s.heading_deg, s.speed_mps,
        (int)s.mode, (int)s.fix_quality, (int)s.satellites, s.hdop,
        s.baseline_len_m, s.heading_enabled ? 1 : 0, (int)s.output_rate_ms,
        sim_yaw_accuracy_deg(&s), sim_h_accuracy_m(&s),
        s.position_noise_m, s.heading_noise_deg,
        (unsigned)gps_sim_rtcm_bytes(), (unsigned)rp->frame_count,
        (unsigned)rp->crc_errors, (unsigned)rp->last_msg_type, age,
        s.rtk_engine_ready ? 1 : 0,
        (unsigned)lk.baud, lk.ip, lk.ap_ssid, lk.sta_ssid,
        (unsigned)esp_get_free_heap_size());
    if (n < 0 || n >= (int)sizeof(buf)) {
        return send_json_str(req, "{\"error\":\"status overflow\"}");
    }

    for (uint8_t i = 0; i < rp->type_used && n < (int)sizeof(buf) - 80; ++i) {
        n += snprintf(buf + n, sizeof(buf) - n,
                      "%s{\"type\":%u,\"name\":\"%s\",\"count\":%u}",
                      i ? "," : "", (unsigned)rp->types[i],
                      rtcm_type_name(rp->types[i]), (unsigned)rp->type_counts[i]);
    }
    snprintf(buf + n, sizeof(buf) - n, "]}");

    return send_json_str(req, buf);
}

static int clamp_int(int v, int lo, int hi)
{
    return (v < lo) ? lo : (v > hi) ? hi : v;
}

static esp_err_t h_set(httpd_req_t *req)
{
    char body[512];
    int total = req->content_len;
    if (total <= 0 || total >= (int)sizeof(body)) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "body too large");
        return ESP_FAIL;
    }
    int got = 0;
    while (got < total) {
        const int r = httpd_req_recv(req, body + got, total - got);
        if (r <= 0) {
            httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "read error");
            return ESP_FAIL;
        }
        got += r;
    }
    body[got] = '\0';

    double v;
    uint32_t new_baud = 0;

    gps_sim_state_lock();
    sim_state_t *s = gps_sim_state();

    if (json_num(body, "origin_lat", &v) && v >= -90.0 && v <= 90.0) {
        s->origin_lat = v;
    }
    if (json_num(body, "origin_lon", &v) && v >= -180.0 && v <= 180.0) {
        s->origin_lon = v;
    }
    if (json_num(body, "mode", &v)) {
        s->mode = (sim_mode_t)clamp_int((int)v, 0, 3);
    }
    if (json_num(body, "heading_deg", &v)) {
        v = fmod(v, 360.0);
        if (v < 0.0) v += 360.0;
        s->heading_deg = v;
        s->course_deg = v;
    }
    if (json_num(body, "speed_mps", &v)) {
        s->speed_mps = (v < 0.0) ? 0.0 : (v > 10.0) ? 10.0 : v;
        s->waypoint_speed_mps = s->speed_mps;
    }
    if (json_num(body, "fix_quality", &v)) {
        const int q = (int)v;
        if (q == 0 || q == 1 || q == 2 || q == 4 || q == 5) {
            s->fix_quality = (sim_fix_quality_t)q;
        }
    }
    if (json_num(body, "satellites", &v)) {
        s->satellites = (uint8_t)clamp_int((int)v, 0, 40);
    }
    if (json_num(body, "hdop", &v)) {
        s->hdop = (v < 0.5) ? 0.5 : (v > 9.9) ? 9.9 : v;
    }
    if (json_num(body, "baseline_len_m", &v)) {
        s->baseline_len_m = (v < 0.2) ? 0.2 : (v > 10.0) ? 10.0 : v;
    }
    if (json_num(body, "position_noise_m", &v)) {
        s->position_noise_m = (v < 0.0) ? 0.0 : (v > 50.0) ? 50.0 : v;
    }
    if (json_num(body, "heading_noise_deg", &v)) {
        s->heading_noise_deg = (v < 0.0) ? 0.0 : (v > 10.0) ? 10.0 : v;
    }
    if (json_num(body, "heading_enabled", &v)) {
        s->heading_enabled = (v != 0.0);
    }
    if (json_num(body, "output_rate_ms", &v)) {
        s->output_rate_ms = (uint16_t)clamp_int((int)v, 50, 1000);
    }
    if (json_num(body, "rtk_engine_ready", &v)) {
        s->rtk_engine_ready = (v != 0.0);
    }
    if (json_num(body, "baud", &v)) {
        new_baud = (uint32_t)v;
    }
    gps_sim_state_unlock();

    if (new_baud) {
        gps_sim_set_baud(new_baud);
    }

    return send_json_str(req, "{\"ok\":true}");
}

static esp_err_t h_restart(httpd_req_t *req)
{
    send_json_str(req, "{\"ok\":true}");
    ESP_LOGW(TAG, "restart requested via HTTP");
    vTaskDelay(pdMS_TO_TICKS(200));
    esp_restart();
    return ESP_OK;
}

static const httpd_uri_t URIS[] = {
    { .uri = "/",             .method = HTTP_GET,  .handler = h_index },
    { .uri = "/api/status",   .method = HTTP_GET,  .handler = h_status },
    { .uri = "/api/set",      .method = HTTP_POST, .handler = h_set },
    { .uri = "/api/set",      .method = HTTP_GET,  .handler = h_set },
    { .uri = "/api/restart",  .method = HTTP_POST, .handler = h_restart },
};

void web_start(uint32_t stack_size)
{
    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
    cfg.stack_size = stack_size;
    cfg.max_uri_handlers = sizeof(URIS) / sizeof(URIS[0]) + 2;
    cfg.lru_purge_enable = true;

    /* The default is 7 open sockets, but LWIP_MAX_SOCKETS in the stock esp32c3
       config reserves enough for only 5 - the server refuses to start with
       "max_open_sockets is too large".  A bench tool serving one browser needs
       three at most, so cap it rather than raise the LWIP limit and pay for the
       extra pcb memory. */
    if (cfg.max_open_sockets > 3) {
        cfg.max_open_sockets = 3;
    }

    if (httpd_start(&s_server, &cfg) != ESP_OK) {
        ESP_LOGE(TAG, "httpd_start failed");
        return;
    }
    for (size_t i = 0; i < sizeof(URIS) / sizeof(URIS[0]); ++i) {
        httpd_register_uri_handler(s_server, &URIS[i]);
    }
    ESP_LOGI(TAG, "web UI on http://<ip>/");
}

