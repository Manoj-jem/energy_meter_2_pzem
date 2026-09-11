// ================================================================
// pzem.cpp - 3x PZEM-004T on separate pin pairs using ESP32 UART2
// ================================================================

#include <Arduino.h>
#include <HardwareSerial.h>
#include "config.h"
#include "calibration.h"
#include "pzem.h"

// UART2 is remapped to one PZEM pin pair at a time. UART1 remains
// dedicated to the SIM7670C modem on GPIO26/27.
static HardwareSerial pzemSerial(2);

static const uint8_t  FUNC_READ_INPUT = 0x04;
static const uint8_t  FUNC_WRITE_REG  = 0x06;
static const uint16_t REG_VOLTAGE     = 0x0000;
static const uint16_t REG_ADDR_PARAM  = 0x0002;
static const uint8_t  NUM_REGS        = 10;

static uint16_t crc16(const uint8_t* data, uint16_t len) {
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < len; ++i) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; ++j) {
            crc = (crc & 1) ? (crc >> 1) ^ 0xA001 : (crc >> 1);
        }
    }
    return crc;
}

// Crossed-wire self-check (see pzem_read): per channel, whether its RX/TX
// pins are used swapped, and whether the swap has already been tried.
static bool _swapped[4]     = {false, false, false, false};
static bool _swapProbed[4]  = {false, false, false, false};

static bool channel_pins(uint8_t channel, int8_t* rx, int8_t* tx) {
    switch (channel) {
        case 1: *rx = PZEM1_RX_PIN; *tx = PZEM1_TX_PIN; return true;
        case 2: *rx = PZEM2_RX_PIN; *tx = PZEM2_TX_PIN; return true;
        case 3: *rx = PZEM3_RX_PIN; *tx = PZEM3_TX_PIN; return true;
        default: return false;
    }
}

static bool bind_channel(uint8_t channel) {
    // Stop the previous pin mapping before assigning a new one.
    pzemSerial.end();

    int8_t rx = -1;
    int8_t tx = -1;
    if (!channel_pins(channel, &rx, &tx)) {
        DBGF("[PZEM] Invalid channel %u\n", channel);
        return false;
    }
    if (_swapped[channel]) {
        const int8_t t = rx; rx = tx; tx = t;
    }

    pzemSerial.begin(PZEM_BAUD, SERIAL_8N1, rx, tx);
    delay(5);
    return true;
}

static void unbind_pzem() {
    pzemSerial.flush();
    pzemSerial.end();
    delay(2);
}

static void flush_rx() {
    delay(3);
    while (pzemSerial.available()) (void)pzemSerial.read();
}

static void send_frame(const uint8_t* frame, uint8_t len) {
    const uint16_t crc = crc16(frame, len);
    pzemSerial.write(frame, len);
    pzemSerial.write((uint8_t)(crc & 0xFF));
    pzemSerial.write((uint8_t)(crc >> 8));
    pzemSerial.flush();
}

static uint8_t recv_frame(uint8_t* buf, uint8_t maxLen, uint8_t expectedLen) {
    uint8_t idx = 0;
    const uint32_t deadline = millis() + PZEM_TIMEOUT_MS;

    while (millis() < deadline && idx < maxLen) {
        while (pzemSerial.available() && idx < maxLen) {
            buf[idx++] = (uint8_t)pzemSerial.read();
            // A PZEM read of 10 registers is exactly 25 bytes.
            if (idx >= expectedLen) break;
        }
        if (idx >= expectedLen) break;
        delay(1);
    }

    if (idx < expectedLen) {
        DBGF("[PZEM] Timeout: received %u/%u bytes\n", idx, expectedLen);
        return 0;
    }

    const uint16_t received = (uint16_t)buf[expectedLen - 2] |
                              ((uint16_t)buf[expectedLen - 1] << 8);
    const uint16_t computed = crc16(buf, expectedLen - 2);
    if (received != computed) {
        DBGF("[PZEM] CRC mismatch: got 0x%04X expected 0x%04X\n",
             received, computed);
        return 0;
    }

    if (buf[1] & 0x80) {
        DBGF("[PZEM] Modbus exception: 0x%02X\n", buf[2]);
        return 0;
    }

    return expectedLen;
}

void pzem_init() {
    pzemSerial.begin(PZEM_BAUD, SERIAL_8N1, PZEM1_RX_PIN, PZEM1_TX_PIN);
    delay(20);
    DBGLN("[PZEM] UART2 ready; channel pin mapping is switched sequentially");
    DBGF("[PZEM] CH1 addr=0x%02X RX=%d TX=%d\n", PZEM1_ADDR, PZEM1_RX_PIN, PZEM1_TX_PIN);
    DBGF("[PZEM] CH2 addr=0x%02X RX=%d TX=%d\n", PZEM2_ADDR, PZEM2_RX_PIN, PZEM2_TX_PIN);
    DBGF("[PZEM] CH3 addr=0x%02X RX=%d TX=%d\n", PZEM3_ADDR, PZEM3_RX_PIN, PZEM3_TX_PIN);
}

PzemReading pzem_read(uint8_t slaveAddr, uint8_t channel) {
    PzemReading result = {};
    result.valid = false;

    if (!bind_channel(channel)) return result;

    flush_rx();

    const uint8_t request[6] = {
        slaveAddr,
        FUNC_READ_INPUT,
        (uint8_t)(REG_VOLTAGE >> 8),
        (uint8_t)(REG_VOLTAGE & 0xFF),
        0x00,
        NUM_REGS
    };

    DBGF("[PZEM] CH%u -> addr 0x%02X\n", channel, slaveAddr);
    send_frame(request, sizeof(request));

    uint8_t response[25] = {};
    uint8_t rxLen = recv_frame(response, sizeof(response), 25);

    // Each PZEM is alone on its pin pair and is queried on the general
    // address, so silence means wiring, not addressing. Once per boot, try the
    // pair the other way round: a PZEM whose TX wire sits on the ESP32's TX
    // pin answers then, and the log says exactly which wires are crossed.
    if (rxLen != 25 && !_swapProbed[channel]) {
        _swapProbed[channel] = true;
        _swapped[channel] = true;
        int8_t rx = -1, tx = -1;
        channel_pins(channel, &rx, &tx);
        DBGF("[PZEM] CH%u: no reply - retrying once with RX/TX swapped\n", channel);
        bind_channel(channel);
        flush_rx();
        send_frame(request, sizeof(request));
        rxLen = recv_frame(response, sizeof(response), 25);
        if (rxLen == 25) {
            DBGF("[PZEM] CH%u WARNING: answers only with RX/TX SWAPPED - the PZEM's TX wire is on "
                 "GPIO%d (defined as its TX pin) and its RX wire on GPIO%d. Using the swapped pins "
                 "until reboot; fix the wiring.\n", channel, tx, rx);
        } else {
            _swapped[channel] = false;
            DBGF("[PZEM] CH%u: no reply either way - nothing answers on GPIO%d/GPIO%d. Check the PZEM's "
                 "5V/GND and mains, and that its wires are on these GPIO numbers (not board labels).\n",
                 channel, rx, tx);
        }
    }

    if (rxLen != 25) {
        DBGF("[PZEM] CH%u addr 0x%02X: no valid response\n", channel, slaveAddr);
        unbind_pzem();
        return result;
    }

    // A query to the general address 0xF8 is answered with the unit's own
    // configured address (0x01-0xF7), so only a specific query can demand an
    // exact echo of the address byte.
    const bool addrOk = (slaveAddr == PZEM_GENERAL_ADDR)
                        ? (response[0] >= 0x01 && response[0] <= PZEM_GENERAL_ADDR)
                        : (response[0] == slaveAddr);
    if (!addrOk || response[1] != FUNC_READ_INPUT || response[2] != 20) {
        DBGF("[PZEM] CH%u: invalid header addr=0x%02X func=0x%02X bytes=%u\n",
             channel, response[0], response[1], response[2]);
        unbind_pzem();
        return result;
    }

    #define REG(n) ((uint16_t)(((uint16_t)response[3 + (n) * 2] << 8) | response[4 + (n) * 2]))
    const uint16_t volt_raw = REG(0);
    const uint32_t curr_raw = ((uint32_t)REG(2) << 16) | REG(1);
    const uint32_t power_raw = ((uint32_t)REG(4) << 16) | REG(3);
    const uint32_t energy_raw = ((uint32_t)REG(6) << 16) | REG(5);
    const uint16_t freq_raw = REG(7);
    const uint16_t pf_raw = REG(8);
    const uint16_t alarm_raw = REG(9);
    #undef REG

    const float multiplier = CT_NOMINAL_MULTIPLIER * calibration_phase_gain(channel);
    result.voltage = volt_raw * 0.1f;
    result.current = curr_raw * 0.001f * multiplier;
    result.power = power_raw * 0.1f * multiplier;
    result.energy = (float)energy_raw * multiplier;
    result.frequency = freq_raw * 0.1f;
    result.power_factor = pf_raw * 0.01f;
    result.alarm = (alarm_raw == 0xFFFF);
    result.valid = true;

    DBGF("[PZEM] CH%u OK (unit addr 0x%02X): V=%.1fV I=%.3fA P=%.1fW E=%.0fWh F=%.1fHz PF=%.2f\n",
         channel, response[0], result.voltage, result.current, result.power,
         result.energy, result.frequency, result.power_factor);

    unbind_pzem();
    return result;
}

void pzem_read_all(PzemReading readings[3]) {
    // IMPORTANT: these are physical PZEM addresses, not UART addresses.
    // The three transactions are always sequential because UART2 is remapped.
    readings[0] = pzem_read(PZEM1_ADDR, 1);
    delay(10);
    readings[1] = pzem_read(PZEM2_ADDR, 2);
    delay(10);
    readings[2] = pzem_read(PZEM3_ADDR, 3);
    delay(10);

    // Leave UART2 disconnected after metering so it cannot drive a PZEM
    // while the rest of the application is running.
    unbind_pzem();
}

bool pzem_set_address(uint8_t channel, uint8_t oldAddr, uint8_t newAddr) {
    if (newAddr < 0x01 || newAddr > 0xF7) {
        DBGLN("[PZEM] Address out of range (0x01-0xF7)");
        return false;
    }
    if (!bind_channel(channel)) return false;

    flush_rx();
    const uint8_t request[6] = {
        oldAddr,
        FUNC_WRITE_REG,
        (uint8_t)(REG_ADDR_PARAM >> 8),
        (uint8_t)(REG_ADDR_PARAM & 0xFF),
        0x00,
        newAddr
    };

    DBGF("[PZEM] CH%u set address 0x%02X -> 0x%02X\n", channel, oldAddr, newAddr);
    send_frame(request, sizeof(request));

    uint8_t response[8] = {};
    const uint8_t rxLen = recv_frame(response, sizeof(response), 8);
    const bool ok = rxLen == 8 &&
                    (oldAddr == PZEM_GENERAL_ADDR || response[0] == oldAddr) &&
                    response[1] == FUNC_WRITE_REG &&
                    response[2] == (uint8_t)(REG_ADDR_PARAM >> 8) &&
                    response[3] == (uint8_t)(REG_ADDR_PARAM & 0xFF) &&
                    response[4] == 0x00 &&
                    response[5] == newAddr;

    DBGF("[PZEM] CH%u address change %s\n", channel, ok ? "OK" : "FAILED");
    unbind_pzem();
    return ok;
}
