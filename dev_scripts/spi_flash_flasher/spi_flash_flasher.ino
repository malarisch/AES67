#include <WiFi.h>
#include <WebServer.h>
#include <SPI.h>

#include "wifi_credentials.h"  // WIFI_SSID, WIFI_PASS — not tracked in git

// ── SPI pins (directly directly directly directly to W25Q64) ────────
// Directly connected — no level-shift needed if ESP32 runs at 3.3 V.
#define FLASH_CS   5
#define SPI_CLK   18
#define SPI_MISO  19
#define SPI_MOSI  23
#define SPI_SPEED 8000000  // 8 MHz — W25Q64 supports up to 104 MHz

// ── W25Q64 commands ─────────────────────────────────────────────────
#define CMD_WRITE_ENABLE      0x06
#define CMD_WRITE_DISABLE     0x04
#define CMD_READ_STATUS1      0x05
#define CMD_READ_DATA         0x03
#define CMD_PAGE_PROGRAM      0x02
#define CMD_SECTOR_ERASE      0x20   // 4 KB
#define CMD_BLOCK_ERASE_32K   0x52
#define CMD_BLOCK_ERASE_64K   0xD8
#define CMD_CHIP_ERASE        0xC7
#define CMD_READ_JEDEC_ID     0x9F

#define FLASH_PAGE_SIZE       256
#define FLASH_SECTOR_SIZE     4096

// ── Web server ──────────────────────────────────────────────────────
WebServer server(80);

// ── Flash low-level helpers ─────────────────────────────────────────

static void flash_begin() {
  digitalWrite(FLASH_CS, LOW);
}

static void flash_end() {
  digitalWrite(FLASH_CS, HIGH);
}

static void flash_write_enable() {
  flash_begin();
  SPI.transfer(CMD_WRITE_ENABLE);
  flash_end();
}

static uint8_t flash_read_status() {
  flash_begin();
  SPI.transfer(CMD_READ_STATUS1);
  uint8_t s = SPI.transfer(0x00);
  flash_end();
  return s;
}

static void flash_wait_busy() {
  while (flash_read_status() & 0x01) {
    delay(1);
  }
}

static uint32_t flash_read_jedec_id() {
  flash_begin();
  SPI.transfer(CMD_READ_JEDEC_ID);
  uint8_t mfr  = SPI.transfer(0x00);
  uint8_t type = SPI.transfer(0x00);
  uint8_t cap  = SPI.transfer(0x00);
  flash_end();
  return ((uint32_t)mfr << 16) | ((uint32_t)type << 8) | cap;
}

static void flash_read(uint32_t addr, uint8_t *buf, size_t len) {
  flash_begin();
  SPI.transfer(CMD_READ_DATA);
  SPI.transfer((addr >> 16) & 0xFF);
  SPI.transfer((addr >>  8) & 0xFF);
  SPI.transfer( addr        & 0xFF);
  for (size_t i = 0; i < len; i++) {
    buf[i] = SPI.transfer(0x00);
  }
  flash_end();
}

static void flash_sector_erase(uint32_t addr) {
  flash_write_enable();
  flash_begin();
  SPI.transfer(CMD_SECTOR_ERASE);
  SPI.transfer((addr >> 16) & 0xFF);
  SPI.transfer((addr >>  8) & 0xFF);
  SPI.transfer( addr        & 0xFF);
  flash_end();
  flash_wait_busy();
}

static void flash_page_program(uint32_t addr, const uint8_t *data, size_t len) {
  if (len > FLASH_PAGE_SIZE) len = FLASH_PAGE_SIZE;
  flash_write_enable();
  flash_begin();
  SPI.transfer(CMD_PAGE_PROGRAM);
  SPI.transfer((addr >> 16) & 0xFF);
  SPI.transfer((addr >>  8) & 0xFF);
  SPI.transfer( addr        & 0xFF);
  for (size_t i = 0; i < len; i++) {
    SPI.transfer(data[i]);
  }
  flash_end();
  flash_wait_busy();
}

static void flash_chip_erase() {
  flash_write_enable();
  flash_begin();
  SPI.transfer(CMD_CHIP_ERASE);
  flash_end();
  flash_wait_busy();  // can take 20-100 s
}

// ── HTML page ───────────────────────────────────────────────────────

static const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>W25Q64 SPI Flash Programmer</title>
  <style>
    * { box-sizing: border-box; }
    body { font-family: monospace; max-width: 600px; margin: 2em auto; padding: 0 1em; background: #1a1a2e; color: #eee; }
    h1 { color: #0ff; font-size: 1.3em; }
    .card { background: #16213e; border: 1px solid #0f3460; border-radius: 8px; padding: 1.2em; margin-bottom: 1em; }
    label { display: block; margin-bottom: .5em; color: #aaa; }
    input[type=file] { margin-bottom: 1em; }
    button { background: #0f3460; color: #0ff; border: 1px solid #0ff; padding: .6em 1.4em; border-radius: 4px; cursor: pointer; font-family: monospace; }
    button:hover { background: #0ff; color: #1a1a2e; }
    button:disabled { opacity: .4; cursor: default; }
    #progress { margin-top: 1em; }
    .bar-outer { background: #0f3460; border-radius: 4px; height: 22px; margin: .5em 0; }
    .bar-inner { background: #0ff; height: 100%; border-radius: 4px; width: 0%; transition: width .3s; }
    #log { background: #0d1117; padding: .8em; border-radius: 4px; max-height: 200px; overflow-y: auto; font-size: .85em; white-space: pre-wrap; margin-top: 1em; }
    #jedec { color: #0ff; }
  </style>
</head>
<body>
  <h1>W25Q64 SPI Flash Programmer</h1>

  <div class="card">
    <p>JEDEC ID: <span id="jedec">loading...</span></p>
    <button onclick="chipErase()" id="btnErase">Chip Erase</button>
  </div>

  <div class="card">
    <h2 style="color:#0ff;font-size:1.1em;margin-top:0;">BIOS <span style="color:#888;font-size:.85em;">@ 0x000000</span></h2>
    <input type="file" id="fileBios" accept=".bin,.rpd,.jic,.rbf,.bit">
    <br>
    <button onclick="startFlash('bios')" id="btnBios">Flash BIOS</button>
    <div id="progress_bios" style="display:none;margin-top:.5em;">
      <div class="bar-outer"><div class="bar-inner" id="bar_bios"></div></div>
      <span id="pct_bios">0%</span>
    </div>
  </div>

  <div class="card">
    <h2 style="color:#0ff;font-size:1.1em;margin-top:0;">Firmware <span style="color:#888;font-size:.85em;">@ 0x010000</span></h2>
    <input type="file" id="fileFw" accept=".bin,.rpd,.jic,.rbf,.bit">
    <br>
    <button onclick="startFlash('fw')" id="btnFw">Flash FW</button>
    <div id="progress_fw" style="display:none;margin-top:.5em;">
      <div class="bar-outer"><div class="bar-inner" id="bar_fw"></div></div>
      <span id="pct_fw">0%</span>
    </div>
  </div>

  <div class="card" style="text-align:center;">
    <button onclick="flashBoth()" id="btnBoth">Flash Both</button>
  </div>

  <div class="card">
    <h2 style="color:#0ff;font-size:1.1em;margin-top:0;">Hex Reader</h2>
    <label>Address (hex): <input type="text" id="hexAddr" value="0" size="8" style="background:#0d1117;color:#0ff;border:1px solid #0f3460;padding:.3em;font-family:monospace;"></label>
    <label>Length: <select id="hexLen" style="background:#0d1117;color:#eee;border:1px solid #0f3460;padding:.3em;font-family:monospace;">
      <option value="256">256 B</option>
      <option value="512">512 B</option>
      <option value="1024">1 KB</option>
      <option value="4096">4 KB</option>
    </select></label>
    <button onclick="hexRead()" id="btnRead">Read</button>
    <button onclick="hexPrev()" id="btnPrev" style="margin-left:.3em;">&lt; Prev</button>
    <button onclick="hexNext()" id="btnNext">&gt; Next</button>
    <div id="hexdump" style="background:#0d1117;padding:.8em;border-radius:4px;margin-top:.8em;font-size:.78em;overflow-x:auto;white-space:pre;display:none;line-height:1.5;"></div>
  </div>

  <div id="log"></div>

<script>
  const log = document.getElementById('log');
  function addLog(msg) { log.textContent += msg + '\n'; log.scrollTop = log.scrollHeight; }

  // Fetch JEDEC ID on load
  fetch('/jedec').then(r => r.text()).then(t => {
    document.getElementById('jedec').textContent = t;
    addLog('JEDEC ID: ' + t);
  });

  async function chipErase() {
    if (!confirm('Erase entire chip?')) return;
    document.getElementById('btnErase').disabled = true;
    addLog('Chip erase started...');
    const r = await fetch('/erase', { method: 'POST' });
    addLog(await r.text());
    document.getElementById('btnErase').disabled = false;
  }

  const SLOTS = {
    bios: { fileId: 'fileBios', btnId: 'btnBios', barId: 'bar_bios', pctId: 'pct_bios', progId: 'progress_bios', baseAddr: 0x000000 },
    fw:   { fileId: 'fileFw',   btnId: 'btnFw',   barId: 'bar_fw',   pctId: 'pct_fw',   progId: 'progress_fw',   baseAddr: 0x010000 }
  };

  async function startFlash(slot) {
    const s = SLOTS[slot];
    const fileInput = document.getElementById(s.fileId);
    if (!fileInput.files.length) { alert('Select a ' + slot + ' file first'); return; }
    const file = fileInput.files[0];
    const CHUNK = 4096;
    const total = file.size;
    let fileOffset = 0;

    document.getElementById(s.btnId).disabled = true;
    document.getElementById(s.progId).style.display = 'block';
    document.getElementById(s.barId).style.width = '0%';
    document.getElementById(s.pctId).textContent = '0% (writing)';
    addLog('Flashing ' + slot.toUpperCase() + ' (' + file.name + ', ' + total + ' bytes) @ 0x' + s.baseAddr.toString(16).toUpperCase().padStart(6,'0') + '...');

    // ── Write phase ──
    while (fileOffset < total) {
      const end = Math.min(fileOffset + CHUNK, total);
      const blob = file.slice(fileOffset, end);
      const formData = new FormData();
      formData.append('data', blob);

      const url = '/write?offset=' + (s.baseAddr + fileOffset);
      const resp = await fetch(url, { method: 'POST', body: formData });
      if (!resp.ok) {
        const errText = await resp.text();
        addLog('ERROR ' + slot + ' write at 0x' + (s.baseAddr + fileOffset).toString(16) + ': ' + errText);
        document.getElementById(s.btnId).disabled = false;
        return false;
      }
      fileOffset = end;
      const pct = Math.round(fileOffset / total * 100);
      document.getElementById(s.barId).style.width = pct + '%';
      document.getElementById(s.pctId).textContent = pct + '% (writing)';
    }

    addLog(slot.toUpperCase() + ' write complete, verifying...');

    // ── Full verify phase — re-upload file for comparison ──
    document.getElementById(s.barId).style.width = '0%';
    document.getElementById(s.pctId).textContent = '0% (verifying)';

    fileOffset = 0;
    while (fileOffset < total) {
      const end = Math.min(fileOffset + CHUNK, total);
      const blob = file.slice(fileOffset, end);
      const formData = new FormData();
      formData.append('data', blob);

      const url = '/verify?addr=' + (s.baseAddr + fileOffset);
      const resp = await fetch(url, { method: 'POST', body: formData });
      const respText = await resp.text();
      if (!resp.ok) {
        addLog('VERIFY FAILED ' + slot.toUpperCase() + ': ' + respText);
        document.getElementById(s.pctId).textContent = 'VERIFY FAILED';
        document.getElementById(s.barId).style.background = '#f44';
        document.getElementById(s.btnId).disabled = false;
        return false;
      }
      fileOffset = end;
      const pct = Math.round(fileOffset / total * 100);
      document.getElementById(s.barId).style.width = pct + '%';
      document.getElementById(s.pctId).textContent = pct + '% (verifying)';
    }

    addLog(slot.toUpperCase() + ' done! ' + total + ' bytes written and verified.');
    document.getElementById(s.pctId).textContent = '100% ✓';
    document.getElementById(s.btnId).disabled = false;
    return true;
  }

  async function flashBoth() {
    document.getElementById('btnBoth').disabled = true;
    const hasBios = document.getElementById('fileBios').files.length > 0;
    const hasFw   = document.getElementById('fileFw').files.length > 0;
    if (!hasBios && !hasFw) { alert('Select at least one file'); document.getElementById('btnBoth').disabled = false; return; }
    if (hasBios) { if (!await startFlash('bios')) { document.getElementById('btnBoth').disabled = false; return; } }
    if (hasFw)   { if (!await startFlash('fw'))   { document.getElementById('btnBoth').disabled = false; return; } }
    addLog('All done!');
    document.getElementById('btnBoth').disabled = false;
  }

  // ── Hex reader ──────────────────────────────────────────────────
  function getHexAddr() {
    return parseInt(document.getElementById('hexAddr').value, 16) || 0;
  }
  function getHexLen() {
    return parseInt(document.getElementById('hexLen').value, 10);
  }
  function setHexAddr(v) {
    document.getElementById('hexAddr').value = v.toString(16).toUpperCase();
  }

  function hexPrev() {
    let a = getHexAddr() - getHexLen();
    if (a < 0) a = 0;
    setHexAddr(a);
    hexRead();
  }
  function hexNext() {
    setHexAddr(getHexAddr() + getHexLen());
    hexRead();
  }

  async function hexRead() {
    const addr = getHexAddr();
    const len = getHexLen();
    document.getElementById('btnRead').disabled = true;
    const resp = await fetch('/read?addr=' + addr + '&len=' + len);
    if (!resp.ok) { addLog('Read error'); document.getElementById('btnRead').disabled = false; return; }
    const buf = new Uint8Array(await resp.arrayBuffer());
    const dump = document.getElementById('hexdump');
    dump.style.display = 'block';

    let html = '<span style="color:#666">          00 01 02 03 04 05 06 07  08 09 0A 0B 0C 0D 0E 0F  |ASCII           |</span>\n';
    for (let i = 0; i < buf.length; i += 16) {
      // Address
      let line = '<span style="color:#0ff">' + (addr + i).toString(16).toUpperCase().padStart(8, '0') + '</span>  ';
      // Hex bytes
      let ascii = '';
      for (let j = 0; j < 16; j++) {
        if (j === 8) line += ' ';
        if (i + j < buf.length) {
          const b = buf[i + j];
          const hex = b.toString(16).toUpperCase().padStart(2, '0');
          if (b === 0x00) line += '<span style="color:#444">' + hex + '</span> ';
          else if (b === 0xFF) line += '<span style="color:#555">' + hex + '</span> ';
          else line += '<span style="color:#eee">' + hex + '</span> ';
          ascii += (b >= 0x20 && b <= 0x7E) ? String.fromCharCode(b) : '.';
        } else {
          line += '   ';
          ascii += ' ';
        }
      }
      line += ' <span style="color:#888">|' + ascii.replace(/&/g,'&amp;').replace(/</g,'&lt;') + '|</span>';
      html += line + '\n';
    }
    dump.innerHTML = html;
    document.getElementById('btnRead').disabled = false;
  }
</script>
</body>
</html>
)rawliteral";

// ── Upload state (must precede handlers) ────────────────────────────
static uint32_t write_offset = 0;
static uint8_t  sector_buf[FLASH_SECTOR_SIZE];
static size_t   sector_fill = 0;

// ── Web server handlers ─────────────────────────────────────────────

static void handle_root() {
  server.send(200, "text/html", INDEX_HTML);
}

static void handle_jedec() {
  uint32_t id = flash_read_jedec_id();
  char buf[32];
  snprintf(buf, sizeof(buf), "0x%06X", id);

  // Decode known IDs
  const char *name = "Unknown";
  if (id == 0xEF4017) name = "Winbond W25Q64";
  else if (id == 0xEF4018) name = "Winbond W25Q128";
  else if (id == 0xEF4016) name = "Winbond W25Q32";
  else if (id == 0xC84017) name = "GigaDevice GD25Q64";
  else if (id == 0x1F8701) name = "Atmel/Microchip AT25SF641";

  char resp[64];
  snprintf(resp, sizeof(resp), "%s (%s)", buf, name);
  server.send(200, "text/plain", resp);
}

static void handle_erase() {
  Serial.println("Chip erase requested...");
  flash_chip_erase();
  Serial.println("Chip erase complete.");
  server.send(200, "text/plain", "Chip erase complete.");
}

static void handle_read() {
  uint32_t addr = 0;
  size_t len = 256;
  if (server.hasArg("addr")) addr = strtoul(server.arg("addr").c_str(), NULL, 10);
  if (server.hasArg("len"))  len  = strtoul(server.arg("len").c_str(), NULL, 10);
  if (len > FLASH_SECTOR_SIZE) len = FLASH_SECTOR_SIZE;

  uint8_t *buf = (uint8_t *)malloc(len);
  if (!buf) {
    server.send(500, "text/plain", "Out of memory");
    return;
  }
  flash_read(addr, buf, len);

  server.sendHeader("Content-Type", "application/octet-stream");
  server.send_P(200, "application/octet-stream", (const char *)buf, len);
  free(buf);
}

static void handle_write() {
  server.send(200, "text/plain", "OK");
}

// ── Full verify endpoint ─────────────────────────────────────────────
// POST /verify with file upload, ?addr=N — reads back entire flash region
// and compares against uploaded data. Returns mismatch offset or "OK".

static uint32_t verify_addr = 0;
static bool     verify_ok = true;
static uint32_t verify_fail_offset = 0;
static uint8_t  verify_upload_buf[FLASH_SECTOR_SIZE];
static size_t   verify_upload_fill = 0;
static uint32_t verify_checked = 0;

static void verify_flush() {
  if (verify_upload_fill == 0 || !verify_ok) return;

  uint8_t readback[256];
  size_t off = 0;
  while (off < verify_upload_fill) {
    size_t chunk = min((size_t)256, verify_upload_fill - off);
    flash_read(verify_addr + verify_checked + off, readback, chunk);
    if (memcmp(readback, verify_upload_buf + off, chunk) != 0) {
      // Find exact byte offset of first mismatch
      for (size_t i = 0; i < chunk; i++) {
        if (readback[i] != verify_upload_buf[off + i]) {
          verify_fail_offset = verify_addr + verify_checked + off + i;
          verify_ok = false;
          Serial.printf("Verify mismatch at 0x%06X: expected 0x%02X, got 0x%02X\n",
                        verify_fail_offset, verify_upload_buf[off + i], readback[i]);
          return;
        }
      }
    }
    off += chunk;
  }
  verify_checked += verify_upload_fill;
  verify_upload_fill = 0;
}

static void handle_verify() {
  if (verify_ok) {
    char buf[64];
    snprintf(buf, sizeof(buf), "OK (%u bytes verified)", verify_checked);
    server.send(200, "text/plain", buf);
  } else {
    char buf[64];
    snprintf(buf, sizeof(buf), "MISMATCH at 0x%06X", verify_fail_offset);
    server.send(500, "text/plain", buf);
  }
}

static void handle_verify_upload() {
  HTTPUpload &upload = server.upload();

  if (upload.status == UPLOAD_FILE_START) {
    if (server.hasArg("addr")) {
      verify_addr = strtoul(server.arg("addr").c_str(), NULL, 10);
    }
    verify_ok = true;
    verify_fail_offset = 0;
    verify_upload_fill = 0;
    verify_checked = 0;
    Serial.printf("Full verify start at 0x%06X\n", verify_addr);

  } else if (upload.status == UPLOAD_FILE_WRITE) {
    size_t remaining = upload.currentSize;
    const uint8_t *src = upload.buf;

    while (remaining > 0 && verify_ok) {
      size_t space = FLASH_SECTOR_SIZE - verify_upload_fill;
      size_t copy = min(remaining, space);
      memcpy(verify_upload_buf + verify_upload_fill, src, copy);
      verify_upload_fill += copy;
      src += copy;
      remaining -= copy;

      if (verify_upload_fill == FLASH_SECTOR_SIZE) {
        verify_flush();
      }
    }

  } else if (upload.status == UPLOAD_FILE_END) {
    verify_flush();
    Serial.printf("Full verify complete: %s (%u bytes)\n",
                  verify_ok ? "OK" : "FAILED", verify_checked);
  }
}

static void flush_sector_buf() {
  if (sector_fill == 0) return;

  // Erase the sector covering write_offset
  uint32_t sector_base = write_offset & ~(FLASH_SECTOR_SIZE - 1);
  flash_sector_erase(sector_base);

  // Program page by page
  size_t written = 0;
  while (written < sector_fill) {
    size_t chunk = min((size_t)FLASH_PAGE_SIZE, sector_fill - written);
    flash_page_program(write_offset + written, sector_buf + written, chunk);
    written += chunk;
  }

  write_offset += sector_fill;
  sector_fill = 0;
}

static void handle_write_upload() {
  HTTPUpload &upload = server.upload();

  if (upload.status == UPLOAD_FILE_START) {
    // Parse offset and verify flag from args (available during upload)
    if (server.hasArg("offset")) {
      write_offset = strtoul(server.arg("offset").c_str(), NULL, 10);
    }
    sector_fill = 0;
    Serial.printf("Write start at 0x%06X\n", write_offset);

  } else if (upload.status == UPLOAD_FILE_WRITE) {
    // Accumulate into sector buffer and flush when full
    size_t remaining = upload.currentSize;
    const uint8_t *src = upload.buf;

    while (remaining > 0) {
      size_t space = FLASH_SECTOR_SIZE - sector_fill;
      size_t copy = min(remaining, space);
      memcpy(sector_buf + sector_fill, src, copy);
      sector_fill += copy;
      src += copy;
      remaining -= copy;

      if (sector_fill == FLASH_SECTOR_SIZE) {
        flush_sector_buf();
      }
    }

  } else if (upload.status == UPLOAD_FILE_END) {
    // Flush remaining data — response is sent by handle_write()
    flush_sector_buf();
    Serial.printf("Write complete up to 0x%06X\n", write_offset);
  }
}

// ── Setup & Loop ────────────────────────────────────────────────────

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n=== W25Q64 SPI Flash Programmer ===");

  // SPI flash CS
  pinMode(FLASH_CS, OUTPUT);
  digitalWrite(FLASH_CS, HIGH);

  // Init SPI
  SPI.begin(SPI_CLK, SPI_MISO, SPI_MOSI, FLASH_CS);
  SPI.setFrequency(SPI_SPEED);
  SPI.setDataMode(SPI_MODE0);

  // Read JEDEC ID
  uint32_t jedec = flash_read_jedec_id();
  Serial.printf("JEDEC ID: 0x%06X\n", jedec);

  // Connect WiFi
  Serial.printf("Connecting to %s", WIFI_SSID);
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.printf("\nConnected! IP: %s\n", WiFi.localIP().toString().c_str());

  // Register web handlers
  server.on("/",      HTTP_GET,  handle_root);
  server.on("/jedec", HTTP_GET,  handle_jedec);
  server.on("/erase", HTTP_POST, handle_erase);
  server.on("/read",  HTTP_GET,  handle_read);
  server.on("/write",  HTTP_POST, handle_write,  handle_write_upload);
  server.on("/verify", HTTP_POST, handle_verify, handle_verify_upload);

  server.begin();
  Serial.println("HTTP server started on port 80");
}

void loop() {
  server.handleClient();
}
