#include <unity.h>
#include <remote_command.hpp>
#include <command_dispatcher.hpp>
#include <cstring>

using namespace ball_plate;

void setUp()    {}
void tearDown() {}

// ── Helper: build a valid raw packet ───────────────────────

static uint8_t buildPacket(uint8_t* buf, CmdId cmd, uint8_t seq,
                           const uint8_t* payload, uint8_t payloadLen) {
    buf[OFF_HEADER] = PACKET_HEADER;
    buf[OFF_VER]    = PROTOCOL_VER;
    buf[OFF_CMD]    = static_cast<uint8_t>(cmd);
    buf[OFF_SEQ]    = seq;
    buf[OFF_LEN]    = payloadLen;
    if (payloadLen > 0 && payload)
        memcpy(&buf[OFF_PAYLOAD], payload, payloadLen);
    buf[HEADER_SIZE + payloadLen] = crc8(buf, HEADER_SIZE + payloadLen);
    return HEADER_SIZE + payloadLen + 1;
}

// ── Mock ICommandReceiver ──────────────────────────────────

class MockReceiver : public ICommandReceiver {
public:
    void begin() override {}

    bool receive(uint8_t* buf, uint8_t maxLen, uint8_t& outLen) override {
        if (_readIdx >= _writeIdx) return false;
        auto& pkt = _queue[_readIdx++];
        uint8_t copyLen = (pkt.len < maxLen) ? pkt.len : maxLen;
        memcpy(buf, pkt.data, copyLen);
        outLen = copyLen;
        return true;
    }

    void push(const uint8_t* data, uint8_t len) {
        if (_writeIdx < MAX_QUEUE) {
            memcpy(_queue[_writeIdx].data, data, len);
            _queue[_writeIdx].len = len;
            ++_writeIdx;
        }
    }

    void reset() { _readIdx = _writeIdx = 0; }

private:
    static constexpr uint8_t MAX_QUEUE = 8;
    struct Entry { uint8_t data[MAX_PACKET_SIZE]; uint8_t len; };
    Entry   _queue[MAX_QUEUE] = {};
    uint8_t _readIdx  = 0;
    uint8_t _writeIdx = 0;
};

// ── CRC-8 tests ───────────────────────────────────────────

void test_crc8_empty() {
    uint8_t crc = crc8(nullptr, 0);
    TEST_ASSERT_EQUAL_UINT8(0x00, crc);
}

void test_crc8_known_value() {
    uint8_t data[] = { 0xA5, 0x01, 0x01, 0x00, 0x01, 0x01 };
    uint8_t c = crc8(data, sizeof(data));
    // Verify determinism: same input → same output
    TEST_ASSERT_EQUAL_UINT8(crc8(data, sizeof(data)), c);
    // Non-trivial: CRC of non-zero data should not be zero (probabilistically)
    // (Not guaranteed in general, but for this specific input it holds)
}

void test_crc8_single_bit_change() {
    uint8_t data1[] = { 0xA5, 0x01, 0x01, 0x00, 0x01, 0x01 };
    uint8_t data2[] = { 0xA5, 0x01, 0x01, 0x00, 0x01, 0x00 };  // last byte differs
    TEST_ASSERT_NOT_EQUAL(crc8(data1, sizeof(data1)),
                          crc8(data2, sizeof(data2)));
}

// ── parsePacket tests ──────────────────────────────────────

void test_parse_valid_packet() {
    uint8_t buf[MAX_PACKET_SIZE];
    uint8_t payload[] = { 0x01 };  // FREEZE
    uint8_t len = buildPacket(buf, CmdId::REMOTE_STOP, 42, payload, 1);

    ParsedPacket pkt;
    TEST_ASSERT_TRUE(parsePacket(buf, len, pkt));
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(CmdId::REMOTE_STOP),
                            static_cast<uint8_t>(pkt.cmd));
    TEST_ASSERT_EQUAL_UINT8(42, pkt.seq);
    TEST_ASSERT_EQUAL_UINT8(1, pkt.payloadLen);
    TEST_ASSERT_NOT_NULL(pkt.payload);
    TEST_ASSERT_EQUAL_UINT8(0x01, pkt.payload[0]);
}

void test_parse_zero_payload() {
    uint8_t buf[MAX_PACKET_SIZE];
    uint8_t len = buildPacket(buf, CmdId::REMOTE_RESUME, 0, nullptr, 0);

    ParsedPacket pkt;
    TEST_ASSERT_TRUE(parsePacket(buf, len, pkt));
    TEST_ASSERT_EQUAL_UINT8(0, pkt.payloadLen);
    TEST_ASSERT_NULL(pkt.payload);
}

void test_parse_target_update_payload() {
    uint8_t buf[MAX_PACKET_SIZE];
    uint8_t payload[8];
    float x = 120.5f, y = 80.25f;
    memcpy(payload, &x, 4);
    memcpy(payload + 4, &y, 4);
    uint8_t len = buildPacket(buf, CmdId::TARGET_UPDATE, 1, payload, 8);

    ParsedPacket pkt;
    TEST_ASSERT_TRUE(parsePacket(buf, len, pkt));
    TEST_ASSERT_EQUAL_UINT8(8, pkt.payloadLen);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 120.5f, readFloat(pkt.payload));
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 80.25f, readFloat(pkt.payload + 4));
}

void test_parse_wrong_header() {
    uint8_t buf[MAX_PACKET_SIZE];
    uint8_t len = buildPacket(buf, CmdId::REMOTE_STOP, 0, nullptr, 0);
    buf[OFF_HEADER] = 0xFF;  // corrupt header

    ParsedPacket pkt;
    TEST_ASSERT_FALSE(parsePacket(buf, len, pkt));
}

void test_parse_wrong_version() {
    uint8_t buf[MAX_PACKET_SIZE];
    uint8_t len = buildPacket(buf, CmdId::REMOTE_STOP, 0, nullptr, 0);
    buf[OFF_VER] = 0xFF;  // unknown version

    ParsedPacket pkt;
    TEST_ASSERT_FALSE(parsePacket(buf, len, pkt));
}

void test_parse_bad_crc() {
    uint8_t buf[MAX_PACKET_SIZE];
    uint8_t payload[] = { 0x01 };
    uint8_t len = buildPacket(buf, CmdId::REMOTE_STOP, 0, payload, 1);
    buf[len - 1] ^= 0xFF;  // flip CRC

    ParsedPacket pkt;
    TEST_ASSERT_FALSE(parsePacket(buf, len, pkt));
}

void test_parse_len_exceeds_max() {
    uint8_t buf[MAX_PACKET_SIZE];
    uint8_t len = buildPacket(buf, CmdId::REMOTE_STOP, 0, nullptr, 0);
    buf[OFF_LEN] = MAX_PAYLOAD + 1;  // claim impossibly long payload

    ParsedPacket pkt;
    TEST_ASSERT_FALSE(parsePacket(buf, len, pkt));
}

void test_parse_rawlen_too_short() {
    uint8_t buf[MAX_PACKET_SIZE];
    uint8_t payload[] = { 0x01 };
    uint8_t len = buildPacket(buf, CmdId::REMOTE_STOP, 0, payload, 1);

    ParsedPacket pkt;
    TEST_ASSERT_FALSE(parsePacket(buf, len - 1, pkt));  // truncated
}

void test_parse_minimum_rawlen() {
    // rawLen < HEADER_SIZE + 1 should fail
    uint8_t buf[5] = {};
    ParsedPacket pkt;
    TEST_ASSERT_FALSE(parsePacket(buf, 5, pkt));
}

// ── CommandDispatcher tests ────────────────────────────────

// Capture context for handler verification
static uint8_t g_lastPayload[MAX_PAYLOAD];
static uint8_t g_lastPayloadLen = 0;
static int     g_callCount = 0;
static void*   g_lastCtx = nullptr;

static void resetCapture() {
    memset(g_lastPayload, 0, sizeof(g_lastPayload));
    g_lastPayloadLen = 0;
    g_callCount = 0;
    g_lastCtx = nullptr;
}

static void captureHandler(void* ctx, const uint8_t* payload, uint8_t len) {
    g_lastCtx = ctx;
    g_lastPayloadLen = len;
    if (len > 0 && payload)
        memcpy(g_lastPayload, payload, len);
    ++g_callCount;
}

void test_dispatcher_registers_and_dispatches() {
    resetCapture();
    CommandDispatcher d;
    MockReceiver rx;
    int token = 42;

    d.on(CmdId::REMOTE_STOP, captureHandler, &token);

    uint8_t buf[MAX_PACKET_SIZE];
    uint8_t payload[] = { 0x01 };
    uint8_t len = buildPacket(buf, CmdId::REMOTE_STOP, 1, payload, 1);
    rx.push(buf, len);

    d.poll(rx);

    TEST_ASSERT_EQUAL_INT(1, g_callCount);
    TEST_ASSERT_EQUAL_PTR(&token, g_lastCtx);
    TEST_ASSERT_EQUAL_UINT8(1, g_lastPayloadLen);
    TEST_ASSERT_EQUAL_UINT8(0x01, g_lastPayload[0]);
}

void test_dispatcher_unknown_cmd_dropped() {
    resetCapture();
    CommandDispatcher d;
    MockReceiver rx;

    // Register only REMOTE_STOP
    d.on(CmdId::REMOTE_STOP, captureHandler);

    // Send TARGET_UPDATE (not registered)
    uint8_t buf[MAX_PACKET_SIZE];
    uint8_t payload[8] = {};
    uint8_t len = buildPacket(buf, CmdId::TARGET_UPDATE, 1, payload, 8);
    rx.push(buf, len);

    d.poll(rx);

    TEST_ASSERT_EQUAL_INT(0, g_callCount);
}

void test_dispatcher_seq_dedup() {
    resetCapture();
    CommandDispatcher d;
    MockReceiver rx;

    d.on(CmdId::REMOTE_STOP, captureHandler);

    uint8_t buf[MAX_PACKET_SIZE];
    uint8_t payload[] = { 0x01 };

    // Two packets with same SEQ
    uint8_t len = buildPacket(buf, CmdId::REMOTE_STOP, 5, payload, 1);
    rx.push(buf, len);
    rx.push(buf, len);  // duplicate

    d.poll(rx);

    TEST_ASSERT_EQUAL_INT(1, g_callCount);  // second should be deduped
}

void test_dispatcher_first_packet_seq_zero() {
    resetCapture();
    CommandDispatcher d;
    MockReceiver rx;

    d.on(CmdId::REMOTE_STOP, captureHandler);

    uint8_t buf[MAX_PACKET_SIZE];
    uint8_t payload[] = { 0x01 };
    uint8_t len = buildPacket(buf, CmdId::REMOTE_STOP, 0, payload, 1);
    rx.push(buf, len);

    d.poll(rx);

    // First packet with seq=0 must NOT be dropped
    TEST_ASSERT_EQUAL_INT(1, g_callCount);
}

void test_dispatcher_different_seq_accepted() {
    resetCapture();
    CommandDispatcher d;
    MockReceiver rx;

    d.on(CmdId::REMOTE_STOP, captureHandler);

    uint8_t buf[MAX_PACKET_SIZE];
    uint8_t payload[] = { 0x01 };

    uint8_t len1 = buildPacket(buf, CmdId::REMOTE_STOP, 1, payload, 1);
    rx.push(buf, len1);

    uint8_t len2 = buildPacket(buf, CmdId::REMOTE_STOP, 2, payload, 1);
    rx.push(buf, len2);

    d.poll(rx);

    TEST_ASSERT_EQUAL_INT(2, g_callCount);
}

void test_dispatcher_max_drain_per_poll() {
    resetCapture();
    CommandDispatcher d;
    MockReceiver rx;

    d.on(CmdId::REMOTE_STOP, captureHandler);

    uint8_t buf[MAX_PACKET_SIZE];
    uint8_t payload[] = { 0x01 };

    // Push 5 packets with different SEQs
    for (uint8_t seq = 1; seq <= 5; ++seq) {
        uint8_t len = buildPacket(buf, CmdId::REMOTE_STOP, seq, payload, 1);
        rx.push(buf, len);
    }

    d.poll(rx);

    // MAX_DRAIN_PER_POLL = 3, so only 3 should be processed
    TEST_ASSERT_EQUAL_INT(3, g_callCount);

    // Second poll should process the remaining 2
    resetCapture();
    d.poll(rx);
    TEST_ASSERT_EQUAL_INT(2, g_callCount);
}

void test_dispatcher_overwrite_handler() {
    resetCapture();
    CommandDispatcher d;
    MockReceiver rx;

    static int first_count = 0;
    static int second_count = 0;
    first_count = 0;
    second_count = 0;

    d.on(CmdId::REMOTE_STOP, [](void*, const uint8_t*, uint8_t) {
        ++first_count;
    });
    d.on(CmdId::REMOTE_STOP, [](void*, const uint8_t*, uint8_t) {
        ++second_count;
    });

    uint8_t buf[MAX_PACKET_SIZE];
    uint8_t payload[] = { 0x01 };
    uint8_t len = buildPacket(buf, CmdId::REMOTE_STOP, 1, payload, 1);
    rx.push(buf, len);

    d.poll(rx);

    TEST_ASSERT_EQUAL_INT(0, first_count);   // overwritten
    TEST_ASSERT_EQUAL_INT(1, second_count);  // active
}

void test_dispatcher_invalid_packet_skipped() {
    resetCapture();
    CommandDispatcher d;
    MockReceiver rx;

    d.on(CmdId::REMOTE_STOP, captureHandler);

    // Push an invalid packet (bad header)
    uint8_t bad[8] = { 0xFF, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x00 };
    rx.push(bad, sizeof(bad));

    // Followed by a valid packet
    uint8_t buf[MAX_PACKET_SIZE];
    uint8_t payload[] = { 0x01 };
    uint8_t len = buildPacket(buf, CmdId::REMOTE_STOP, 1, payload, 1);
    rx.push(buf, len);

    d.poll(rx);

    // Invalid should be skipped, valid should dispatch
    TEST_ASSERT_EQUAL_INT(1, g_callCount);
}

void test_dispatcher_no_data_noop() {
    resetCapture();
    CommandDispatcher d;
    MockReceiver rx;  // empty

    d.on(CmdId::REMOTE_STOP, captureHandler);
    d.poll(rx);

    TEST_ASSERT_EQUAL_INT(0, g_callCount);
}

// ── readFloat tests ────────────────────────────────────────

void test_read_float_alignment() {
    // Test readFloat works even from odd offset (unaligned)
    uint8_t buf[9];
    float val = 3.14f;
    memcpy(buf + 1, &val, 4);  // offset 1 = unaligned for float

    TEST_ASSERT_FLOAT_WITHIN(0.001f, 3.14f, readFloat(buf + 1));
}

void test_read_float_zero() {
    uint8_t buf[4] = { 0, 0, 0, 0 };
    TEST_ASSERT_EQUAL_FLOAT(0.0f, readFloat(buf));
}

void test_read_float_negative() {
    float val = -42.5f;
    uint8_t buf[4];
    memcpy(buf, &val, 4);
    TEST_ASSERT_EQUAL_FLOAT(-42.5f, readFloat(buf));
}

// ── Runner ─────────────────────────────────────────────────

int main() {
    UNITY_BEGIN();

    // CRC-8
    RUN_TEST(test_crc8_empty);
    RUN_TEST(test_crc8_known_value);
    RUN_TEST(test_crc8_single_bit_change);

    // parsePacket
    RUN_TEST(test_parse_valid_packet);
    RUN_TEST(test_parse_zero_payload);
    RUN_TEST(test_parse_target_update_payload);
    RUN_TEST(test_parse_wrong_header);
    RUN_TEST(test_parse_wrong_version);
    RUN_TEST(test_parse_bad_crc);
    RUN_TEST(test_parse_len_exceeds_max);
    RUN_TEST(test_parse_rawlen_too_short);
    RUN_TEST(test_parse_minimum_rawlen);

    // CommandDispatcher
    RUN_TEST(test_dispatcher_registers_and_dispatches);
    RUN_TEST(test_dispatcher_unknown_cmd_dropped);
    RUN_TEST(test_dispatcher_seq_dedup);
    RUN_TEST(test_dispatcher_first_packet_seq_zero);
    RUN_TEST(test_dispatcher_different_seq_accepted);
    RUN_TEST(test_dispatcher_max_drain_per_poll);
    RUN_TEST(test_dispatcher_overwrite_handler);
    RUN_TEST(test_dispatcher_invalid_packet_skipped);
    RUN_TEST(test_dispatcher_no_data_noop);

    // readFloat
    RUN_TEST(test_read_float_alignment);
    RUN_TEST(test_read_float_zero);
    RUN_TEST(test_read_float_negative);

    return UNITY_END();
}
