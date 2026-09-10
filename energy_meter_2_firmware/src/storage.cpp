#include <Arduino.h>
#include <SPI.h>
#include <stddef.h>
#include <string.h>
#include <freertos/semphr.h>

#include "config.h"
#include "storage.h"

namespace {

constexpr uint8_t CMD_WRITE_ENABLE = 0x06;
constexpr uint8_t CMD_READ_STATUS1 = 0x05;
constexpr uint8_t CMD_READ_ID = 0x9F;
constexpr uint8_t CMD_READ_DATA = 0x03;
constexpr uint8_t CMD_PAGE_PROGRAM = 0x02;
constexpr uint8_t CMD_SECTOR_ERASE = 0x20;
constexpr uint32_t SECTOR_SIZE = 4096;
constexpr uint32_t PAGE_SIZE = 256;
constexpr uint32_t HEADER_A_ADDR = 0;
constexpr uint32_t HEADER_B_ADDR = SECTOR_SIZE;
constexpr uint32_t DATA_ADDR = SECTOR_SIZE * 2;
constexpr uint32_t SLOT_SIZE = PAGE_SIZE;
constexpr uint32_t SLOT_COUNT = (FLASH_SIZE_BYTES - DATA_ADDR) / SLOT_SIZE;
constexpr uint32_t RECORD_MAGIC = 0x45575132UL;
constexpr uint32_t HEADER_MAGIC = 0x45574832UL;
constexpr uint16_t FORMAT_VERSION = 2;
constexpr uint8_t COMMIT_PENDING = 0xA5;
constexpr uint8_t COMMIT_DELIVERED = 0xA4;
constexpr uint32_t CHECKPOINT_INTERVAL = 64;

struct __attribute__((packed)) FlashRecord {
    uint32_t magic;
    uint16_t version;
    uint16_t reserved;
    uint64_t order;
    TelemetryRecord record;
    uint32_t crc32;
    uint8_t commit;
};

struct __attribute__((packed)) HeaderEntry {
    uint32_t magic;
    uint16_t version;
    uint16_t reserved;
    uint32_t generation;
    uint32_t head;
    uint32_t tail;
    uint32_t count;
    uint32_t dropped;
    uint64_t next_order;
    uint64_t last_seq;
    uint32_t crc32;
    uint8_t commit;
};

static_assert(sizeof(FlashRecord) <= SLOT_SIZE, "Flash record must fit one NOR page");
static_assert(sizeof(HeaderEntry) <= 64, "Header journal entry must remain compact");

SPIClass flashSPI(VSPI);
SemaphoreHandle_t flashMutex = nullptr;

struct QueueState {
    bool ready = false;
    uint32_t head = 0;
    uint32_t tail = 0;
    uint32_t count = 0;
    uint32_t dropped = 0;
    uint64_t nextOrder = 1;
    uint32_t generation = 0;
    uint32_t appendSinceCheckpoint = 0;
    uint8_t activeHeader = 0;
};

QueueState state;

uint32_t crc32(const uint8_t* data, size_t length) {
    uint32_t crc = 0xFFFFFFFFUL;
    for (size_t i = 0; i < length; ++i) {
        crc ^= data[i];
        for (uint8_t bit = 0; bit < 8; ++bit) {
            crc = (crc >> 1) ^ (0xEDB88320UL & (-(int32_t)(crc & 1UL)));
        }
    }
    return ~crc;
}

uint32_t slotAddress(uint32_t slot) { return DATA_ADDR + (slot % SLOT_COUNT) * SLOT_SIZE; }
void selectFlash() { digitalWrite(FLASH_CS_PIN, LOW); }
void deselectFlash() { digitalWrite(FLASH_CS_PIN, HIGH); }

void flashRead(uint32_t address, uint8_t* output, size_t length) {
    flashSPI.beginTransaction(SPISettings(20000000, MSBFIRST, SPI_MODE0));
    selectFlash();
    flashSPI.transfer(CMD_READ_DATA);
    flashSPI.transfer((address >> 16) & 0xFF);
    flashSPI.transfer((address >> 8) & 0xFF);
    flashSPI.transfer(address & 0xFF);
    while (length--) *output++ = flashSPI.transfer(0x00);
    deselectFlash();
    flashSPI.endTransaction();
}

uint8_t flashStatus() {
    flashSPI.beginTransaction(SPISettings(20000000, MSBFIRST, SPI_MODE0));
    selectFlash();
    flashSPI.transfer(CMD_READ_STATUS1);
    const uint8_t status = flashSPI.transfer(0x00);
    deselectFlash();
    flashSPI.endTransaction();
    return status;
}

bool waitReady(uint32_t timeoutMs = 3000) {
    const uint32_t deadline = millis() + timeoutMs;
    while (millis() < deadline) {
        if ((flashStatus() & 0x01) == 0) return true;
        delay(1);
    }
    return false;
}

void writeEnable() {
    flashSPI.beginTransaction(SPISettings(20000000, MSBFIRST, SPI_MODE0));
    selectFlash();
    flashSPI.transfer(CMD_WRITE_ENABLE);
    deselectFlash();
    flashSPI.endTransaction();
}

bool flashProgram(uint32_t address, const uint8_t* data, size_t length) {
    if (length == 0 || length > PAGE_SIZE || (address / PAGE_SIZE) != ((address + length - 1) / PAGE_SIZE)) return false;
    writeEnable();
    flashSPI.beginTransaction(SPISettings(20000000, MSBFIRST, SPI_MODE0));
    selectFlash();
    flashSPI.transfer(CMD_PAGE_PROGRAM);
    flashSPI.transfer((address >> 16) & 0xFF);
    flashSPI.transfer((address >> 8) & 0xFF);
    flashSPI.transfer(address & 0xFF);
    while (length--) flashSPI.transfer(*data++);
    deselectFlash();
    flashSPI.endTransaction();
    return waitReady();
}

bool flashEraseSector(uint32_t address) {
    writeEnable();
    flashSPI.beginTransaction(SPISettings(20000000, MSBFIRST, SPI_MODE0));
    selectFlash();
    flashSPI.transfer(CMD_SECTOR_ERASE);
    flashSPI.transfer((address >> 16) & 0xFF);
    flashSPI.transfer((address >> 8) & 0xFF);
    flashSPI.transfer(address & 0xFF);
    deselectFlash();
    flashSPI.endTransaction();
    return waitReady(5000);
}

bool recordValid(const FlashRecord& candidate) {
    if (candidate.magic != RECORD_MAGIC || candidate.version != FORMAT_VERSION) return false;
    if (candidate.commit != COMMIT_PENDING && candidate.commit != COMMIT_DELIVERED) return false;
    return candidate.crc32 == crc32(reinterpret_cast<const uint8_t*>(&candidate), offsetof(FlashRecord, crc32));
}

bool readRecord(uint32_t slot, FlashRecord& output) {
    flashRead(slotAddress(slot), reinterpret_cast<uint8_t*>(&output), sizeof(output));
    return recordValid(output);
}

bool headerValid(const HeaderEntry& candidate) {
    return candidate.magic == HEADER_MAGIC && candidate.version == FORMAT_VERSION &&
           candidate.commit == COMMIT_PENDING &&
           candidate.crc32 == crc32(reinterpret_cast<const uint8_t*>(&candidate), offsetof(HeaderEntry, crc32));
}

bool findHeader(HeaderEntry& output, uint8_t& headerIndex) {
    bool found = false;
    for (uint8_t sector = 0; sector < 2; ++sector) {
        const uint32_t base = sector == 0 ? HEADER_A_ADDR : HEADER_B_ADDR;
        for (uint32_t offset = 0; offset + PAGE_SIZE <= SECTOR_SIZE; offset += 64) {
            HeaderEntry candidate{};
            flashRead(base + offset, reinterpret_cast<uint8_t*>(&candidate), sizeof(candidate));
            if (!headerValid(candidate)) continue;
            if (!found || candidate.generation > output.generation) {
                output = candidate;
                headerIndex = sector;
                found = true;
            }
        }
    }
    return found;
}

void rebuildFromFlash() {
    bool found = false;
    uint64_t minOrder = UINT64_MAX;
    uint64_t maxOrder = 0;
    uint32_t minSlot = 0;
    uint32_t maxSlot = 0;
    uint32_t pending = 0;
    for (uint32_t slot = 0; slot < SLOT_COUNT; ++slot) {
        FlashRecord candidate{};
        if (!readRecord(slot, candidate)) continue;
        if (candidate.order >= maxOrder) { maxOrder = candidate.order; maxSlot = slot; }
        if (candidate.commit == COMMIT_PENDING) {
            ++pending;
            if (candidate.order < minOrder) { minOrder = candidate.order; minSlot = slot; }
        }
        found = true;
    }
    state.count = pending;
    state.tail = pending ? minSlot : ((maxSlot + 1) % SLOT_COUNT);
    state.head = found ? ((maxSlot + 1) % SLOT_COUNT) : 0;
    state.nextOrder = found ? maxOrder + 1 : 1;
}

bool checkpointLocked() {
    HeaderEntry entry{};
    entry.magic = HEADER_MAGIC;
    entry.version = FORMAT_VERSION;
    entry.generation = ++state.generation;
    entry.head = state.head;
    entry.tail = state.tail;
    entry.count = state.count;
    entry.dropped = state.dropped;
    entry.next_order = state.nextOrder;
    entry.last_seq = state.nextOrder ? state.nextOrder - 1 : 0;
    entry.crc32 = crc32(reinterpret_cast<const uint8_t*>(&entry), offsetof(HeaderEntry, crc32));
    entry.commit = 0xFF;

    uint8_t target = state.activeHeader;
    uint32_t base = target == 0 ? HEADER_A_ADDR : HEADER_B_ADDR;
    uint32_t offset = 0;
    for (; offset + PAGE_SIZE <= SECTOR_SIZE; offset += 64) {
        uint32_t marker = 0;
        flashRead(base + offset, reinterpret_cast<uint8_t*>(&marker), sizeof(marker));
        if (marker == 0xFFFFFFFFUL) break;
    }
    if (offset + PAGE_SIZE > SECTOR_SIZE) {
        target = state.activeHeader == 0 ? 1 : 0;
        base = target == 0 ? HEADER_A_ADDR : HEADER_B_ADDR;
        if (!flashEraseSector(base)) return false;
        state.activeHeader = target;
        offset = 0;
    }
    if (!flashProgram(base + offset, reinterpret_cast<const uint8_t*>(&entry), offsetof(HeaderEntry, commit))) return false;
    const uint8_t commit = COMMIT_PENDING;
    if (!flashProgram(base + offset + offsetof(HeaderEntry, commit), &commit, 1)) return false;
    state.appendSinceCheckpoint = 0;
    return true;
}

bool ensureWritableSlotLocked() {
    uint32_t marker = 0;
    const uint32_t address = slotAddress(state.head);
    flashRead(address, reinterpret_cast<uint8_t*>(&marker), sizeof(marker));
    if (marker == 0xFFFFFFFFUL) return true;

    const uint32_t sectorAddress = address & ~(SECTOR_SIZE - 1);
    for (uint32_t offset = 0; offset < SECTOR_SIZE; offset += SLOT_SIZE) {
        const uint32_t slot = (sectorAddress + offset - DATA_ADDR) / SLOT_SIZE;
        if (slot >= SLOT_COUNT) continue;
        FlashRecord candidate{};
        if (readRecord(slot, candidate) && candidate.commit == COMMIT_PENDING) {
            if (state.count) --state.count;
            ++state.dropped;
        }
    }
    if (!flashEraseSector(sectorAddress)) return false;
    rebuildFromFlash();
    return true;
}

bool advanceTailLocked() {
    for (uint32_t checked = 0; checked < SLOT_COUNT; ++checked) {
        state.tail = (state.tail + 1) % SLOT_COUNT;
        FlashRecord candidate{};
        if (readRecord(state.tail, candidate) && candidate.commit == COMMIT_PENDING) return true;
    }
    state.count = 0;
    state.tail = state.head;
    return false;
}

}  // namespace

bool storage_init() {
    pinMode(FLASH_CS_PIN, OUTPUT);
    deselectFlash();
    flashSPI.begin(FLASH_SCK_PIN, FLASH_MISO_PIN, FLASH_MOSI_PIN, FLASH_CS_PIN);
    if (flashMutex == nullptr) flashMutex = xSemaphoreCreateMutex();
    if (flashMutex == nullptr) return false;

    flashSPI.beginTransaction(SPISettings(20000000, MSBFIRST, SPI_MODE0));
    selectFlash();
    flashSPI.transfer(CMD_READ_ID);
    const uint8_t manufacturer = flashSPI.transfer(0x00);
    const uint8_t memoryType = flashSPI.transfer(0x00);
    const uint8_t capacity = flashSPI.transfer(0x00);
    deselectFlash();
    flashSPI.endTransaction();
    if (manufacturer == 0x00 || manufacturer == 0xFF || capacity < 0x17) {
        DBGF("[STORAGE] Unsupported SPI NOR JEDEC ID %02X %02X %02X\n", manufacturer, memoryType, capacity);
        return false;
    }

    xSemaphoreTake(flashMutex, portMAX_DELAY);
    HeaderEntry header{};
    uint8_t headerIndex = 0;
    if (findHeader(header, headerIndex)) {
        state.generation = header.generation;
        state.dropped = header.dropped;
        state.activeHeader = headerIndex;
    }
    rebuildFromFlash();
    state.ready = true;
    xSemaphoreGive(flashMutex);
    DBGF("[STORAGE] SPI NOR ready, pending=%lu capacity=%lu\n", state.count, SLOT_COUNT);
    return true;
}

bool storage_append(const TelemetryRecord& record, bool forceCheckpoint) {
    if (!state.ready || flashMutex == nullptr) return false;
    xSemaphoreTake(flashMutex, portMAX_DELAY);
    bool ok = ensureWritableSlotLocked();
    if (ok) {
        FlashRecord flashRecord{};
        flashRecord.magic = RECORD_MAGIC;
        flashRecord.version = FORMAT_VERSION;
        flashRecord.order = state.nextOrder++;
        flashRecord.record = record;
        flashRecord.crc32 = crc32(reinterpret_cast<const uint8_t*>(&flashRecord), offsetof(FlashRecord, crc32));
        flashRecord.commit = 0xFF;
        const uint32_t address = slotAddress(state.head);
        ok = flashProgram(address, reinterpret_cast<const uint8_t*>(&flashRecord), offsetof(FlashRecord, commit));
        if (ok) {
            const uint8_t commit = COMMIT_PENDING;
            ok = flashProgram(address + offsetof(FlashRecord, commit), &commit, 1);
        }
        if (ok) {
            state.head = (state.head + 1) % SLOT_COUNT;
            ++state.count;
            ++state.appendSinceCheckpoint;
            if (forceCheckpoint || state.appendSinceCheckpoint >= CHECKPOINT_INTERVAL) ok = checkpointLocked();
        }
    }
    xSemaphoreGive(flashMutex);
    return ok;
}

bool storage_peek(TelemetryRecord& record) {
    if (!state.ready || !state.count || flashMutex == nullptr) return false;
    xSemaphoreTake(flashMutex, portMAX_DELAY);
    FlashRecord candidate{};
    bool ok = readRecord(state.tail, candidate) && candidate.commit == COMMIT_PENDING;
    if (!ok) {
        rebuildFromFlash();
        ok = state.count && readRecord(state.tail, candidate) && candidate.commit == COMMIT_PENDING;
    }
    if (ok) record = candidate.record;
    xSemaphoreGive(flashMutex);
    return ok;
}

bool storage_pop() {
    if (!state.ready || !state.count || flashMutex == nullptr) return false;
    xSemaphoreTake(flashMutex, portMAX_DELAY);
    FlashRecord candidate{};
    bool ok = readRecord(state.tail, candidate) && candidate.commit == COMMIT_PENDING;
    if (ok) {
        const uint8_t delivered = COMMIT_DELIVERED;
        ok = flashProgram(slotAddress(state.tail) + offsetof(FlashRecord, commit), &delivered, 1);
        if (ok) {
            --state.count;
            if (state.count) advanceTailLocked(); else state.tail = state.head;
            ++state.appendSinceCheckpoint;
            if (state.appendSinceCheckpoint >= CHECKPOINT_INTERVAL) ok = checkpointLocked();
        }
    }
    xSemaphoreGive(flashMutex);
    return ok;
}

bool storage_has_pending() { return state.ready && state.count > 0; }

void storage_flush() {
    if (!state.ready || flashMutex == nullptr) return;
    xSemaphoreTake(flashMutex, portMAX_DELAY);
    checkpointLocked();
    xSemaphoreGive(flashMutex);
}

StorageStats storage_stats() {
    StorageStats result{};
    result.ready = state.ready;
    result.capacity = SLOT_COUNT > UINT16_MAX ? UINT16_MAX : SLOT_COUNT;
    if (flashMutex == nullptr) return result;
    xSemaphoreTake(flashMutex, portMAX_DELAY);
    result.depth = state.count > UINT16_MAX ? UINT16_MAX : state.count;
    result.dropped_records = state.dropped;
    xSemaphoreGive(flashMutex);
    return result;
}
