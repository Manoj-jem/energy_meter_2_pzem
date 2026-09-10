// ================================================================
// pzem.cpp  
// ================================================================

#include <HardwareSerial.h>
#include "config.h"
#include "calibration.h"
#include "pzem.h"

static HardwareSerial pzemSerial1(2);   // UART2

// ── Modbus constants ──────────────────────────────────────────────
static const uint8_t  FUNC_READ_INPUT   = 0x04;
static const uint8_t  FUNC_WRITE_REG    = 0x06;
static const uint16_t REG_VOLTAGE       = 0x0000;
static const uint16_t REG_ADDR_PARAM    = 0x0002; 
static const uint8_t  NUM_REGS          = 10;      

static uint16_t crc16(const uint8_t* data, uint16_t len) {
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x0001) {
                crc = (crc >> 1) ^ 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}

static Stream* get_serial(uint8_t channel) {
    (void)channel;
    return &pzemSerial1;
}

static float calibration_multiplier(uint8_t channel) {
    return CT_NOMINAL_MULTIPLIER * calibration_phase_gain(channel);
}

static void flush_rx(Stream* s) {
    delay(5);
    while (s->available()) s->read();
}

static void send_frame(Stream* s, const uint8_t* frame, uint8_t len) {
    uint16_t crc = crc16(frame, len);
    s->write(frame, len);
    s->write((uint8_t)(crc & 0xFF));        
    s->write((uint8_t)((crc >> 8) & 0xFF)); 
}

static uint8_t recv_frame(Stream* s, uint8_t* buf, uint8_t maxLen) {
    uint8_t  idx      = 0;
    uint32_t deadline = millis() + PZEM_TIMEOUT_MS;

   while (millis() < deadline) {

    while (s->available()) {

        buf[idx++] = s->read();

        if (idx >= maxLen)
            break;
    }

    // PZEM response is normally 25 bytes
    if (idx >= 25)
        break;

    delay(1);
}

    if (idx < 4) {
        DBGLN("[PZEM] Timeout or too few bytes");
        return 0;
    }

    uint16_t received_crc = (uint16_t)buf[idx - 2] | ((uint16_t)buf[idx - 1] << 8);
    uint16_t computed_crc = crc16(buf, idx - 2);
    if (received_crc != computed_crc) {
        DBGF("[PZEM] CRC mismatch: got 0x%04X expected 0x%04X\n",
             received_crc, computed_crc);
        return 0;
    }

    if (buf[1] & 0x80) {
        DBGF("[PZEM] Modbus exception code: 0x%02X\n", buf[2]);
        return 0;
    }

    return idx;
}

void pzem_init() {
    pzemSerial1.begin(PZEM_BAUD, SERIAL_8N1, PZEM1_RX_PIN, PZEM1_TX_PIN);

    DBGLN("[PZEM] UART2 initialised; phase pins are switched per transaction");
    delay(30);
}

PzemReading pzem_read(uint8_t slaveAddr, uint8_t channel) {
    PzemReading result = {0};
    result.valid = false;

    // Remap the single HardwareSerial UART2 to the correct pins
    // before each read. .begin() with new pins reconfigures the
    // GPIO matrix — takes microseconds, costs nothing.
    pzemSerial1.end();
    switch(channel){

case 1:
    pzemSerial1.begin(
        PZEM_BAUD,
        SERIAL_8N1,
        PZEM1_RX_PIN,
        PZEM1_TX_PIN
    );
    break;

case 2:
    pzemSerial1.begin(
        PZEM_BAUD,
        SERIAL_8N1,
        PZEM2_RX_PIN,
        PZEM2_TX_PIN
    );
    break;

case 3:
    pzemSerial1.begin(
        PZEM_BAUD,
        SERIAL_8N1,
        PZEM3_RX_PIN,
        PZEM3_TX_PIN
    );
    break;
}

    delay(5);  // GPIO matrix settle time.

    Stream* s = get_serial(channel);
    if (!s) return result;
    uint8_t request[6] = {
        slaveAddr,
        FUNC_READ_INPUT,
        (uint8_t)(REG_VOLTAGE >> 8),    
        (uint8_t)(REG_VOLTAGE & 0xFF), 
        0x00,                          
        NUM_REGS                        
    };

    flush_rx(s);
    send_frame(s, request, sizeof(request));

    uint8_t response[32] = {0};
    uint8_t rxLen = recv_frame(s, response, sizeof(response));

    if (rxLen < 25) {
        DBGF("[PZEM] ch%d: short response (%d bytes)\n", channel, rxLen);
        return result;
    }
    pzemSerial1.end();
    delay(10);

    if (response[0] != slaveAddr) {
        DBGF("[PZEM] ch%d: wrong slave address in response\n", channel);
        return result;
    }
    #define REG(n) ((uint16_t)(response[3 + (n)*2] << 8) | response[4 + (n)*2])

    uint16_t volt_raw   = REG(0);
    uint32_t curr_raw   = ((uint32_t)REG(2) << 16) | REG(1); 
    uint32_t power_raw  = ((uint32_t)REG(4) << 16) | REG(3);
    uint32_t energy_raw = ((uint32_t)REG(6) << 16) | REG(5);
    uint16_t freq_raw   = REG(7);
    uint16_t pf_raw     = REG(8);
    uint16_t alarm_raw  = REG(9);

    #undef REG

    result.voltage      = volt_raw   * 0.1f;    
    const float multiplier = calibration_multiplier(channel);
    result.current      = curr_raw   * 0.001f * multiplier;
    result.power        = power_raw  * 0.1f * multiplier;
    result.energy       = (float)energy_raw * multiplier;
    result.frequency    = freq_raw   * 0.1f;  
    result.power_factor = pf_raw     * 0.01f;  
    result.alarm        = (alarm_raw == 0xFFFF);
    result.valid        = true;

    DBGF("[PZEM] ch%d: V=%.1fV I=%.3fA P=%.1fW E=%.0fWh F=%.1fHz PF=%.2f\n",
         channel,
         result.voltage, result.current, result.power,
         result.energy, result.frequency, result.power_factor);
   
    return result;
}

void pzem_read_all(PzemReading readings[3]) {
    readings[0] = pzem_read(PZEM1_ADDR, 1);
    delay(5);
    readings[1] = pzem_read(PZEM2_ADDR,2);
    delay(5);
    readings[2] = pzem_read(PZEM3_ADDR, 3);
    delay(5);
    // Park UART2 back on PZEM #1 pins (known safe state)
    // so GPIO18/19 are released before GSM UART operations
    pzemSerial1.begin(PZEM_BAUD, SERIAL_8N1, PZEM1_RX_PIN, PZEM1_TX_PIN);
    delay(5);
}
 
bool pzem_set_address(uint8_t channel, uint8_t oldAddr, uint8_t newAddr) {
    Stream* s = get_serial(channel);
    if (!s) return false;

    if (newAddr < 0x01 || newAddr > 0xF7) {
        DBGLN("[PZEM] Address out of range (0x01–0xF7)");
        return false;
    }

    uint8_t request[6] = {
        oldAddr,
        FUNC_WRITE_REG,
        (uint8_t)(REG_ADDR_PARAM >> 8),
        (uint8_t)(REG_ADDR_PARAM & 0xFF),
        0x00,
        newAddr
    };

    flush_rx(s);
    send_frame(s, request, sizeof(request));
    uint8_t response[16] = {0};
    uint8_t rxLen = recv_frame(s, response, sizeof(response));

    if (rxLen < 8) {
        DBGLN("[PZEM] set_address: no/short response");
        return false;
    }

    bool ok = (response[0] == oldAddr && response[3] == 0x00 && response[4] == newAddr);
    DBGF("[PZEM] set_address ch%d: 0x%02X → 0x%02X %s\n",
         channel, oldAddr, newAddr, ok ? "OK" : "FAILED");
    return ok;
}
