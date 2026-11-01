// =============================================================================
// main.cpp — AGENTE P | Sistema de Filtrado Digital Adaptativo
// UTN-FRP | Técnicas Digitales III | 2026
// =============================================================================

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/event_groups.h"
#include "driver/i2s_std.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_http_server.h"
#include "esp_netif.h"
#include "nvs_flash.h"
#include "lwip/sockets.h"
#include "lwip/netdb.h"

static const char *TAG = "agente_p";

// =============================================================================
// PARÁMETROS FIJOS
// =============================================================================
#define SAMPLE_RATE_HZ      16000
#define I2S_GPIO_MCLK       GPIO_NUM_0
#define I2S_GPIO_BCK        GPIO_NUM_26
#define I2S_GPIO_LRC        GPIO_NUM_25
#define I2S_GPIO_DOUT       GPIO_NUM_35
#define DMA_FRAME_NUM       64
#define DMA_BUFFER_COUNT    4
#define QUEUE_SIZE          4096
#define BATCH_SIZE          32
#define TCP_PORT            3333
#define HTTP_PROXY_PORT     8080
#define MAX_FIR_TAPS        201
#define MAX_IIR_SECTIONS    6

// Red WiFi e IP del servidor proxy RAG: copiar config.example.h como config.h y completarlo.
#if __has_include("config.h")
#include "config.h"
#else
#include "config.example.h"
#endif

#define WIFI_CONNECTED_BIT  BIT0
#define WIFI_FAIL_BIT       BIT1
#define WIFI_MAX_RETRY      10

// =============================================================================
// ESTRUCTURAS DE FILTRO
// =============================================================================
typedef enum { FILTER_NONE, FILTER_FIR, FILTER_IIR } filter_type_t;

typedef struct {
    filter_type_t tipo;
    float b_fir[MAX_FIR_TAPS];
    int   n_taps;
    float delay_fir[MAX_FIR_TAPS];
    float sos[MAX_IIR_SECTIONS][6];
    int   n_secciones;
    float w[MAX_IIR_SECTIONS][2];
    char  descripcion[128];
} filtro_t;

// =============================================================================
// VARIABLES GLOBALES
// =============================================================================
static QueueHandle_t      sample_queue;
static i2s_chan_handle_t  rx_chan;
static SemaphoreHandle_t  filtro_mutex;
static filtro_t           filtro_activo;
static bool               filtro_listo    = false;
static EventGroupHandle_t wifi_event_group;
static int                wifi_retry_num  = 0;

#define JSON_BUF_SIZE (32 * 1024)
static char json_response_buf[JSON_BUF_SIZE];

typedef enum { STATE_IDLE, STATE_CONSULTANDO, STATE_FILTRANDO, STATE_ERROR } system_state_t;
static system_state_t system_state = STATE_IDLE;

// =============================================================================
// HTML
// =============================================================================
static const char *HTML_PAGE = R"rawhtml(
<!DOCTYPE html>
<html lang="es">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>Agente P — UTN FRP</title>
<style>
  @import url('https://fonts.googleapis.com/css2?family=Share+Tech+Mono&family=Inter:wght@300;400;600&display=swap');
  :root {
    --bg:#0d1117; --surface:#161b22; --border:#30363d;
    --accent:#00d4aa; --accent2:#7c3aed;
    --text:#e6edf3; --muted:#8b949e; --error:#f85149; --ok:#3fb950;
  }
  * { box-sizing:border-box; margin:0; padding:0; }
  body { background:var(--bg); color:var(--text); font-family:'Inter',sans-serif;
    min-height:100vh; display:flex; flex-direction:column; align-items:center; padding:2rem 1rem; }
  header { text-align:center; margin-bottom:2.5rem; }
  header .eyebrow { font-family:'Share Tech Mono',monospace; font-size:0.7rem;
    letter-spacing:0.18em; color:var(--accent); text-transform:uppercase; margin-bottom:0.5rem; }
  header h1 { font-size:2rem; font-weight:600; letter-spacing:-0.02em; }
  header h1 span { color:var(--accent); }
  header .sub { font-size:0.82rem; color:var(--muted); margin-top:0.4rem; font-family:'Share Tech Mono',monospace; }
  .card { background:var(--surface); border:1px solid var(--border); border-radius:10px;
    padding:1.8rem; width:100%; max-width:480px; margin-bottom:1.2rem; }
  .card h2 { font-size:0.72rem; font-family:'Share Tech Mono',monospace; letter-spacing:0.12em;
    color:var(--muted); text-transform:uppercase; margin-bottom:1.4rem;
    border-bottom:1px solid var(--border); padding-bottom:0.6rem; }
  .fixed-params { display:grid; grid-template-columns:1fr 1fr; gap:0.6rem; }
  .param-chip { background:var(--bg); border:1px solid var(--border); border-radius:6px;
    padding:0.5rem 0.8rem; font-family:'Share Tech Mono',monospace; font-size:0.8rem; }
  .param-chip .label { color:var(--muted); font-size:0.65rem; display:block; margin-bottom:2px; }
  .param-chip .val { color:var(--accent); }
  .field { margin-bottom:1.1rem; }
  .field label { display:block; font-size:0.75rem; color:var(--muted);
    font-family:'Share Tech Mono',monospace; letter-spacing:0.08em; margin-bottom:0.4rem; }
  .field input[type="number"] { width:100%; background:var(--bg); border:1px solid var(--border);
    border-radius:6px; color:var(--text); font-family:'Share Tech Mono',monospace;
    font-size:1rem; padding:0.6rem 0.8rem; outline:none; transition:border-color 0.2s; }
  .field input[type="number"]:focus { border-color:var(--accent); }
  .toggle-group { display:flex; gap:0.5rem; }
  .toggle-btn { flex:1; background:var(--bg); border:1px solid var(--border); border-radius:6px;
    color:var(--muted); font-family:'Share Tech Mono',monospace; font-size:0.85rem;
    padding:0.55rem; cursor:pointer; transition:all 0.15s; }
  .toggle-btn.active { background:var(--accent2); border-color:var(--accent2); color:#fff; }
  .btn-submit { width:100%; background:var(--accent); border:none; border-radius:8px;
    color:#0d1117; font-family:'Share Tech Mono',monospace; font-size:0.95rem; font-weight:700;
    letter-spacing:0.06em; padding:0.85rem; cursor:pointer; transition:opacity 0.2s; margin-top:0.5rem; }
  .btn-submit:hover { opacity:0.85; }
  .btn-submit:disabled { opacity:0.4; cursor:not-allowed; }
  #status-panel { width:100%; max-width:480px; display:none; }
  .status-line { font-family:'Share Tech Mono',monospace; font-size:0.78rem; padding:0.5rem 0.8rem;
    border-radius:6px; margin-bottom:0.5rem; display:flex; align-items:center; gap:0.6rem; }
  .status-line.info { background:#161b22; border:1px solid var(--border); color:var(--muted); }
  .status-line.ok { background:#0d2614; border:1px solid var(--ok); color:var(--ok); }
  .status-line.error { background:#2d1117; border:1px solid var(--error); color:var(--error); }
  .status-line.loading { background:#0d1a2e; border:1px solid var(--accent); color:var(--accent); }
  .dot { width:7px; height:7px; border-radius:50%; background:currentColor; flex-shrink:0; }
  .dot.pulse { animation:pulse 1s infinite; }
  @keyframes pulse { 0%,100%{opacity:1} 50%{opacity:0.2} }
  #result-card { display:none; }
  .result-row { display:flex; justify-content:space-between; align-items:baseline;
    padding:0.45rem 0; border-bottom:1px solid var(--border); font-size:0.85rem; }
  .result-row:last-child { border-bottom:none; }
  .result-row .rkey { color:var(--muted); font-family:'Share Tech Mono',monospace; font-size:0.72rem; }
  .result-row .rval { color:var(--accent); font-family:'Share Tech Mono',monospace; }
  .justif { font-size:0.78rem; color:var(--muted); font-style:italic; margin-top:1rem; line-height:1.5; }
  footer { margin-top:2rem; font-size:0.68rem; color:var(--muted);
    font-family:'Share Tech Mono',monospace; text-align:center; }
</style>
</head>
<body>
<header>
  <div class="eyebrow">UTN FR Paraná · Técnicas Digitales III · 2026</div>
  <h1>Agente <span>P</span></h1>
  <div class="sub">fs=16000Hz · As=40dB · Rp=1dB · ESP32 NodeMCU</div>
</header>
<div class="card">
  <h2>Parámetros del sistema</h2>
  <div class="fixed-params">
    <div class="param-chip"><span class="label">fs</span><span class="val">16000 Hz</span></div>
    <div class="param-chip"><span class="label">As</span><span class="val">40 dB</span></div>
    <div class="param-chip"><span class="label">Rp</span><span class="val">1 dB</span></div>
    <div class="param-chip"><span class="label">Hardware</span><span class="val">ESP32 + PCM1808</span></div>
  </div>
</div>
<div class="card">
  <h2>Configurar filtro</h2>
  <div class="field">
    <label>fp — Frecuencia de paso (Hz)</label>
    <input type="number" id="fp" value="3000" min="100" max="7900" step="100">
  </div>
  <div class="field">
    <label>fr — Frecuencia de rechazo (Hz)</label>
    <input type="number" id="fr" value="3500" min="100" max="7900" step="100">
  </div>
  <div class="field">
    <label>Fase lineal</label>
    <div class="toggle-group">
      <button class="toggle-btn" id="btn-no" onclick="setFase(false)">NO</button>
      <button class="toggle-btn" id="btn-si" onclick="setFase(true)">SÍ → FIR</button>
    </div>
  </div>
  <button class="btn-submit" id="btn-consultar" onclick="consultar()">Consultar Agente P</button>
</div>
<div id="status-panel">
  <div id="status-msg" class="status-line info">
    <span class="dot"></span><span id="status-text">Listo</span>
  </div>
</div>
<div class="card" id="result-card">
  <h2>Filtro activo</h2>
  <div id="result-rows"></div>
</div>
<footer>AGENTE P — Sistema Experto DSP · RAG + Qwen3:14b + llama3.2:3b · scipy.signal</footer>
<script>
  let fase_lineal = false;
  function setFase(val) {
    fase_lineal = val;
    document.getElementById('btn-no').classList.toggle('active', !val);
    document.getElementById('btn-si').classList.toggle('active', val);
  }
  setFase(false);
  function setStatus(msg, tipo) {
    const panel = document.getElementById('status-panel');
    const box = document.getElementById('status-msg');
    const txt = document.getElementById('status-text');
    panel.style.display = 'block';
    box.className = 'status-line ' + tipo;
    box.querySelector('.dot').className = 'dot' + (tipo === 'loading' ? ' pulse' : '');
    txt.textContent = msg;
  }
  function showResult(data) {
    document.getElementById('result-card').style.display = 'block';
    const rows = document.getElementById('result-rows');
    const tipo_resp = data.tipo_respuesta === 'lowpass' ? 'Pasa Bajos' : 'Pasa Altos';
    const items = [
      ['Tipo', data.tipo], ['Familia', data.familia],
      ['Estructura', data.estructura], ['Respuesta', tipo_resp],
      ['Orden', data.orden], ['fp / fr', data.fp + ' Hz  /  ' + data.fr + ' Hz'],
      ['Secciones', data.n_secciones != null ? data.n_secciones : (data.n_taps + ' taps')],
    ];
    rows.innerHTML = items.map(([k,v]) =>
      '<div class="result-row"><span class="rkey">'+k+'</span><span class="rval">'+v+'</span></div>'
    ).join('') + (data.justificacion ? '<div class="justif">"'+data.justificacion+'"</div>' : '');
  }
  async function consultar() {
    const fp = parseFloat(document.getElementById('fp').value);
    const fr = parseFloat(document.getElementById('fr').value);
    if (isNaN(fp)||isNaN(fr)||fp<=0||fr<=0) { setStatus('fp y fr deben ser mayores a 0.','error'); return; }
    if (fp===fr) { setStatus('fp y fr no pueden ser iguales.','error'); return; }
    if (fp>=8000||fr>=8000) { setStatus('fp y fr deben ser menores a 8000 Hz.','error'); return; }
    document.getElementById('btn-consultar').disabled = true;
    document.getElementById('result-card').style.display = 'none';
    setStatus('Consultando al Agente P...', 'loading');
    try {
      const resp = await fetch('/configurar', {
        method:'POST', headers:{'Content-Type':'application/json'},
        body: JSON.stringify({fp, fr, fase_lineal})
      });
      const data = await resp.json();
      if (!resp.ok || data.error) { setStatus(data.error || 'Error del servidor.', 'error'); }
      else { setStatus('Filtro activo — streameando a MATLAB en puerto 3333.', 'ok'); showResult(data); }
    } catch(e) { setStatus('Sin respuesta del ESP32: ' + e.message, 'error'); }
    finally { document.getElementById('btn-consultar').disabled = false; }
  }
</script>
</body>
</html>
)rawhtml";

// =============================================================================
// WIFI — event handler
// =============================================================================
static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                                int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();

    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (wifi_retry_num < WIFI_MAX_RETRY) {
            esp_wifi_connect();
            wifi_retry_num++;
            ESP_LOGW(TAG, "Reintentando WiFi (%d/%d)...", wifi_retry_num, WIFI_MAX_RETRY);
        } else {
            xEventGroupSetBits(wifi_event_group, WIFI_FAIL_BIT);
            ESP_LOGE(TAG, "No se pudo conectar al WiFi.");
        }

    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "WiFi conectado. IP: " IPSTR, IP2STR(&event->ip_info.ip));
        ESP_LOGI(TAG, ">>> Abrí http://" IPSTR " en tu navegador <<<",
                 IP2STR(&event->ip_info.ip));
        wifi_retry_num = 0;
        xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

static void wifi_init_sta(void)
{
    wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, &instance_got_ip));

    wifi_config_t wifi_cfg = {};
    strncpy((char *)wifi_cfg.sta.ssid,     WIFI_SSID, sizeof(wifi_cfg.sta.ssid));
    strncpy((char *)wifi_cfg.sta.password,  WIFI_PASS,  sizeof(wifi_cfg.sta.password));
    wifi_cfg.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_cfg));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));

    ESP_LOGI(TAG, "Conectando a '%s'...", WIFI_SSID);

    EventBits_t bits = xEventGroupWaitBits(wifi_event_group,
        WIFI_CONNECTED_BIT | WIFI_FAIL_BIT, pdFALSE, pdFALSE,
        pdMS_TO_TICKS(30000));

    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "Conexión WiFi establecida.");
    } else {
        ESP_LOGE(TAG, "Falló la conexión WiFi — el sistema continúa sin red.");
    }
}

// =============================================================================
// PARSERS JSON
// =============================================================================
static float parse_float_field(const char *json, const char *key)
{
    char search[64];
    snprintf(search, sizeof(search), "\"%s\":", key);
    const char *p = strstr(json, search);
    if (!p) return 0.0f;
    p += strlen(search);
    while (*p == ' ') p++;
    return strtof(p, NULL);
}

static int parse_int_field(const char *json, const char *key)
{
    char search[64];
    snprintf(search, sizeof(search), "\"%s\":", key);
    const char *p = strstr(json, search);
    if (!p) return 0;
    p += strlen(search);
    while (*p == ' ') p++;
    return (int)strtol(p, NULL, 10);
}

static bool parse_string_field(const char *json, const char *key, char *out, size_t out_len)
{
    char search[64];
    snprintf(search, sizeof(search), "\"%s\":", key);
    const char *p = strstr(json, search);
    if (!p) return false;
    p += strlen(search);
    while (*p == ' ') p++;
    if (*p != '"') return false;
    p++;
    size_t i = 0;
    while (*p && *p != '"' && i < out_len - 1) out[i++] = *p++;
    out[i] = '\0';
    return true;
}

static int parse_float_array(const char *json, const char *key, float *arr, int max_len)
{
    char search[64];
    snprintf(search, sizeof(search), "\"%s\":", key);
    const char *p = strstr(json, search);
    if (!p) return 0;
    p += strlen(search);
    while (*p == ' ') p++;
    if (*p != '[') return 0;
    p++;
    int count = 0;
    while (*p && *p != ']' && count < max_len) {
        while (*p == ' ' || *p == '\n' || *p == '\r') p++;
        if (*p == ']') break;
        char *end;
        arr[count++] = strtof(p, &end);
        p = end;
        while (*p == ' ' || *p == ',') p++;
    }
    return count;
}

static int parse_sos(const char *json, float sos[][6], int max_sections)
{
    const char *p = strstr(json, "\"sos\":");
    if (!p) return 0;
    p += 6;
    while (*p == ' ') p++;
    if (*p != '[') return 0;
    p++;
    int sec = 0;
    while (*p && *p != ']' && sec < max_sections) {
        while (*p == ' ' || *p == '\n' || *p == ',') p++;
        if (*p == ']') break;
        if (*p != '[') { p++; continue; }
        p++;
        for (int i = 0; i < 6; i++) {
            while (*p == ' ' || *p == ',') p++;
            char *end;
            sos[sec][i] = strtof(p, &end);
            p = end;
        }
        while (*p && *p != ']') p++;
        if (*p == ']') p++;
        sec++;
    }
    return sec;
}

// =============================================================================
// FILTRO
// =============================================================================
static inline float aplicar_fir(filtro_t *f, float x)
{
    memmove(&f->delay_fir[1], &f->delay_fir[0], (f->n_taps - 1) * sizeof(float));
    f->delay_fir[0] = x;
    float y = 0.0f;
    for (int i = 0; i < f->n_taps; i++) y += f->b_fir[i] * f->delay_fir[i];
    return y;
}

static inline float aplicar_iir(filtro_t *f, float x)
{
    float y = x;
    for (int k = 0; k < f->n_secciones; k++) {
        float b0=f->sos[k][0], b1=f->sos[k][1], b2=f->sos[k][2];
        float a1=f->sos[k][4], a2=f->sos[k][5];
        float w0 = y - a1*f->w[k][0] - a2*f->w[k][1];
        float out = b0*w0 + b1*f->w[k][0] + b2*f->w[k][1];
        f->w[k][1] = f->w[k][0];
        f->w[k][0] = w0;
        y = out;
    }
    return y;
}

static float aplicar_filtro(filtro_t *f, float x)
{
    switch (f->tipo) {
        case FILTER_FIR: return aplicar_fir(f, x);
        case FILTER_IIR: return aplicar_iir(f, x);
        default:         return x;
    }
}

static bool cargar_coeficientes(const char *json)
{
    filtro_t nuevo;
    memset(&nuevo, 0, sizeof(nuevo));

    char tipo_str[16] = "";
    parse_string_field(json, "tipo", tipo_str, sizeof(tipo_str));

    if (strcmp(tipo_str, "FIR") == 0) {
        nuevo.tipo   = FILTER_FIR;
        nuevo.n_taps = parse_int_field(json, "n_taps");
        if (nuevo.n_taps <= 0 || nuevo.n_taps > MAX_FIR_TAPS) {
            ESP_LOGE(TAG, "n_taps inválido: %d", nuevo.n_taps); return false;
        }
        int parsed = parse_float_array(json, "b", nuevo.b_fir, MAX_FIR_TAPS);
        if (parsed != nuevo.n_taps) {
            ESP_LOGE(TAG, "FIR incompleto: %d/%d", parsed, nuevo.n_taps); return false;
        }
        snprintf(nuevo.descripcion, sizeof(nuevo.descripcion),
                 "FIR orden %d (%d taps)", nuevo.n_taps - 1, nuevo.n_taps);

    } else if (strcmp(tipo_str, "IIR") == 0) {
        nuevo.tipo        = FILTER_IIR;
        nuevo.n_secciones = parse_int_field(json, "n_secciones");
        if (nuevo.n_secciones <= 0 || nuevo.n_secciones > MAX_IIR_SECTIONS) {
            ESP_LOGE(TAG, "n_secciones inválido: %d", nuevo.n_secciones); return false;
        }
        int parsed = parse_sos(json, nuevo.sos, MAX_IIR_SECTIONS);
        if (parsed != nuevo.n_secciones) {
            ESP_LOGE(TAG, "SOS incompleto: %d/%d", parsed, nuevo.n_secciones); return false;
        }
        char familia[32] = "";
        parse_string_field(json, "familia", familia, sizeof(familia));
        snprintf(nuevo.descripcion, sizeof(nuevo.descripcion),
                 "IIR %s orden %d (%d SOS)", familia,
                 parse_int_field(json, "orden"), nuevo.n_secciones);
    } else {
        ESP_LOGE(TAG, "Tipo desconocido: %s", tipo_str); return false;
    }

    xSemaphoreTake(filtro_mutex, portMAX_DELAY);
    memcpy(&filtro_activo, &nuevo, sizeof(filtro_t));
    xSemaphoreGive(filtro_mutex);

    ESP_LOGI(TAG, "Filtro cargado: %s", filtro_activo.descripcion);
    return true;
}

// =============================================================================
// HTTP CLIENT — proxy RAG
// =============================================================================
static bool consultar_proxy(float fp, float fr, bool fase_lineal,
                             char *resp_buf, size_t resp_buf_len)
{
    int sock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (sock < 0) { ESP_LOGE(TAG, "socket() falló"); return false; }

    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_port   = htons(HTTP_PROXY_PORT);
    inet_pton(AF_INET, PROXY_IP, &addr.sin_addr);

    struct timeval tv = { 90, 0 };
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    if (connect(sock, (struct sockaddr *)&addr, sizeof(addr)) != 0) {
        ESP_LOGE(TAG, "connect() falló: errno %d", errno);
        close(sock); return false;
    }

    char body[128];
    snprintf(body, sizeof(body),
             "{\"fp\":%.1f,\"fr\":%.1f,\"fase_lineal\":%s}",
             fp, fr, fase_lineal ? "true" : "false");

    char req[512];
    int req_len = snprintf(req, sizeof(req),
        "POST /filtro HTTP/1.0\r\n"
        "Host: %s:%d\r\n"
        "Content-Type: application/json\r\n"
        "Content-Length: %d\r\n"
        "Connection: close\r\n"
        "\r\n%s",
        PROXY_IP, HTTP_PROXY_PORT, (int)strlen(body), body);

    if (send(sock, req, req_len, 0) < 0) {
        ESP_LOGE(TAG, "send() falló"); close(sock); return false;
    }

    size_t total = 0;
    int n;
    while ((n = recv(sock, resp_buf + total, resp_buf_len - total - 1, 0)) > 0) total += n;
    resp_buf[total] = '\0';
    close(sock);

    char *body_start = strstr(resp_buf, "\r\n\r\n");
    if (!body_start) { ESP_LOGE(TAG, "Sin body HTTP"); return false; }
    body_start += 4;
    size_t body_len = strlen(body_start);
    memmove(resp_buf, body_start, body_len + 1);

    ESP_LOGI(TAG, "Proxy: %d bytes JSON", (int)body_len);
    return true;
}

// =============================================================================
// WEB SERVER
// =============================================================================
static esp_err_t handler_configurar(httpd_req_t *req)
{
    char body[256] = "";
    int received = httpd_req_recv(req, body, sizeof(body) - 1);
    if (received <= 0) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Body vacío");
        return ESP_FAIL;
    }
    body[received] = '\0';

    float fp         = parse_float_field(body, "fp");
    float fr         = parse_float_field(body, "fr");
    bool fase_lineal = strstr(body, "\"fase_lineal\":true") != NULL;

    ESP_LOGI(TAG, "Web: fp=%.1f fr=%.1f fase_lineal=%d", fp, fr, fase_lineal);

    float nyq = SAMPLE_RATE_HZ / 2.0f;
    if (fp <= 0 || fr <= 0 || fp >= nyq || fr >= nyq || fp == fr) {
        httpd_resp_set_type(req, "application/json");
        httpd_resp_sendstr(req, "{\"error\":\"Valores fp/fr inválidos.\"}");
        return ESP_OK;
    }

    system_state = STATE_CONSULTANDO;

    bool ok = consultar_proxy(fp, fr, fase_lineal, json_response_buf, JSON_BUF_SIZE);
    if (!ok) {
        system_state = STATE_ERROR;
        httpd_resp_set_type(req, "application/json");
        httpd_resp_sendstr(req, "{\"error\":\"No se pudo conectar al proxy RAG.\"}");
        return ESP_OK;
    }

    if (strstr(json_response_buf, "\"error\"")) {
        system_state = STATE_ERROR;
        httpd_resp_set_type(req, "application/json");
        httpd_resp_sendstr(req, json_response_buf);
        return ESP_OK;
    }

    if (!cargar_coeficientes(json_response_buf)) {
        system_state = STATE_ERROR;
        httpd_resp_set_type(req, "application/json");
        httpd_resp_sendstr(req, "{\"error\":\"Error al parsear coeficientes.\"}");
        return ESP_OK;
    }

    filtro_listo = true;
    system_state = STATE_FILTRANDO;
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, json_response_buf);
    return ESP_OK;
}

static esp_err_t handler_root(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html");
    httpd_resp_sendstr(req, HTML_PAGE);
    return ESP_OK;
}

static void web_server_init(void)
{
    httpd_handle_t server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.stack_size        = 8192;
    config.recv_wait_timeout = 90;
    config.send_wait_timeout = 90;

    if (httpd_start(&server, &config) != ESP_OK) {
        ESP_LOGE(TAG, "No se pudo iniciar el web server"); return;
    }

    httpd_uri_t uri_root = {
        .uri = "/", .method = HTTP_GET, .handler = handler_root, .user_ctx = NULL
    };
    httpd_uri_t uri_config = {
        .uri = "/configurar", .method = HTTP_POST, .handler = handler_configurar, .user_ctx = NULL
    };

    httpd_register_uri_handler(server, &uri_root);
    httpd_register_uri_handler(server, &uri_config);
    ESP_LOGI(TAG, "Web server iniciado en puerto 80");
}

// =============================================================================
// TASK I2S RX (Core 0)
// =============================================================================
static void i2s_rx_task(void *pvParameters)
{
    const size_t buf_bytes = DMA_FRAME_NUM * 2 * sizeof(int32_t);
    int32_t *dma_buf = (int32_t *)malloc(buf_bytes);
    if (!dma_buf) { ESP_LOGE(TAG, "malloc DMA falló"); vTaskDelete(NULL); return; }

    size_t   bytes_read = 0;
    uint32_t seq        = 0;
    uint32_t overflows  = 0;
    int64_t  last_log   = esp_timer_get_time();

    while (1) {
        if (i2s_channel_read(rx_chan, dma_buf, buf_bytes, &bytes_read, portMAX_DELAY) != ESP_OK)
            continue;

        size_t frames = bytes_read / (2 * sizeof(int32_t));
        for (size_t i = 0; i < frames; i++) {
            int32_t raw     = dma_buf[i * 2];
            int16_t s16     = (int16_t)((raw >> 8) >> 8);
            float   xf      = (float)s16;
            float   yf      = xf;

            if (filtro_listo) {
                if (xSemaphoreTake(filtro_mutex, 0) == pdTRUE) {
                    yf = aplicar_filtro(&filtro_activo, xf);
                    xSemaphoreGive(filtro_mutex);
                }
            }

            int16_t y16 = (int16_t)fmaxf(-32768.0f, fminf(32767.0f, yf));
            uint64_t packed = ((uint64_t)seq << 16) | (uint16_t)y16;
            seq++;

            if (xQueueSend(sample_queue, &packed, 0) != pdTRUE) overflows++;
        }

        int64_t now = esp_timer_get_time();
        if (now - last_log >= 2000000) {
            ESP_LOGI(TAG, "I2S: seq=%lu overflows=%lu filtro=%s",
                     (unsigned long)seq, (unsigned long)overflows,
                     filtro_listo ? filtro_activo.descripcion : "NINGUNO");
            last_log = now;
        }
    }
}

// =============================================================================
// TASK WIFI TX (Core 1)
// =============================================================================
static void wifi_tx_task(void *pvParameters)
{
    int listen_sock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    int opt = 1;
    setsockopt(listen_sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    struct sockaddr_in srv;
    memset(&srv, 0, sizeof(srv));
    srv.sin_family      = AF_INET;
    srv.sin_addr.s_addr = htonl(INADDR_ANY);
    srv.sin_port        = htons(TCP_PORT);

    bind(listen_sock, (struct sockaddr *)&srv, sizeof(srv));
    listen(listen_sock, 1);
    ESP_LOGI(TAG, "TCP server MATLAB en puerto %d", TCP_PORT);

    while (1) {
        struct sockaddr_in cli;
        socklen_t cli_len = sizeof(cli);
        int sock = accept(listen_sock, (struct sockaddr *)&cli, &cli_len);
        if (sock < 0) { vTaskDelay(pdMS_TO_TICKS(100)); continue; }

        int nd = 1;
        setsockopt(sock, IPPROTO_TCP, TCP_NODELAY, &nd, sizeof(nd));
        ESP_LOGI(TAG, "MATLAB conectado: %s", inet_ntoa(cli.sin_addr));

        uint8_t batch[BATCH_SIZE * 6];
        int     bc    = 0;
        bool    alive = true;

        while (alive) {
            uint64_t packed;
            if (xQueueReceive(sample_queue, &packed, portMAX_DELAY) != pdTRUE) continue;
            uint32_t s  = (uint32_t)(packed >> 16);
            uint16_t u  = (uint16_t)(packed & 0xFFFF);
            uint8_t *p  = &batch[bc * 6];
            p[0]=s&0xFF; p[1]=(s>>8)&0xFF; p[2]=(s>>16)&0xFF; p[3]=(s>>24)&0xFF;
            p[4]=u&0xFF; p[5]=(u>>8)&0xFF;
            bc++;
            if (bc == BATCH_SIZE) {
                if (send(sock, batch, sizeof(batch), 0) < 0) alive = false;
                bc = 0;
            }
        }
        close(sock);
        ESP_LOGI(TAG, "MATLAB desconectado.");
    }
}

// =============================================================================
// I2S — PCM1808
// =============================================================================
static void i2s_init_pcm1808(void)
{
    i2s_chan_config_t chan_cfg = {
        .id            = I2S_NUM_0,
        .role          = I2S_ROLE_MASTER,
        .dma_desc_num  = DMA_BUFFER_COUNT,
        .dma_frame_num = DMA_FRAME_NUM,
        .auto_clear    = false,
    };
    ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, NULL, &rx_chan));

    i2s_std_config_t std_cfg = {
        .clk_cfg = {
            .sample_rate_hz = SAMPLE_RATE_HZ,
            .clk_src        = I2S_CLK_SRC_APLL,
            .mclk_multiple  = I2S_MCLK_MULTIPLE_256,
        },
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(
            I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {
            .mclk = I2S_GPIO_MCLK,
            .bclk = I2S_GPIO_BCK,
            .ws   = I2S_GPIO_LRC,
            .dout = I2S_GPIO_UNUSED,
            .din  = I2S_GPIO_DOUT,
            .invert_flags = { false, false, false },
        },
    };
    ESP_ERROR_CHECK(i2s_channel_init_std_mode(rx_chan, &std_cfg));
    ESP_ERROR_CHECK(i2s_channel_enable(rx_chan));
    ESP_LOGI(TAG, "PCM1808 iniciado. Fs=%d Hz", SAMPLE_RATE_HZ);
}

// =============================================================================
// APP MAIN
// =============================================================================
#ifdef __cplusplus
extern "C" {
#endif

void app_main(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    filtro_mutex = xSemaphoreCreateMutex();
    memset(&filtro_activo, 0, sizeof(filtro_activo));
    filtro_activo.tipo = FILTER_NONE;

    sample_queue = xQueueCreate(QUEUE_SIZE, sizeof(uint64_t));
    if (!sample_queue) { ESP_LOGE(TAG, "Queue falló"); return; }

    wifi_init_sta();
    i2s_init_pcm1808();
    web_server_init();

    xTaskCreatePinnedToCore(i2s_rx_task,  "i2s_rx",  8192, NULL, 10, NULL, 0);
    xTaskCreatePinnedToCore(wifi_tx_task, "wifi_tx",  4096, NULL,  5, NULL, 1);

    ESP_LOGI(TAG, "Sistema listo.");
}

#ifdef __cplusplus
}
#endif