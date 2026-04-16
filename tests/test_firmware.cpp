#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <cassert>

#include "../shared/protocol.h"
#include "../shared/dongle_logic.h"
#include "../shared/wristband_logic.h"

static int tests_run = 0;
static int tests_passed = 0;

#define TEST(name) \
    static void test_##name(); \
    static void run_##name() { \
        tests_run++; \
        printf("  %-50s ", #name); \
        test_##name(); \
        tests_passed++; \
        printf("OK\n"); \
    } \
    static void test_##name()

#define ASSERT_EQ(a, b) do { \
    auto _a = (a); auto _b = (b); \
    if (_a != _b) { \
        printf("FAIL\n    %s:%d: %s == %lld, expected %lld\n", \
            __FILE__, __LINE__, #a, (long long)_a, (long long)_b); \
        exit(1); \
    } \
} while(0)

#define ASSERT_TRUE(x) do { \
    if (!(x)) { \
        printf("FAIL\n    %s:%d: %s is false\n", __FILE__, __LINE__, #x); \
        exit(1); \
    } \
} while(0)

#define ASSERT_FALSE(x) ASSERT_TRUE(!(x))

// Helper: build a valid v2 packet
static void make_packet(uint8_t *buf,
                        int64_t send_time, int64_t next_db,
                        uint16_t tempo_x100, uint8_t beat_in_bar,
                        uint8_t flags, int64_t next_beat,
                        uint8_t on_air, uint8_t master) {
    memset(buf, 0, FIREFLY_PACKET_SIZE);
    buf[0] = FIREFLY_SYNC_0;
    buf[1] = FIREFLY_SYNC_1;
    buf[2] = FIREFLY_VERSION;
    buf[3] = FIREFLY_TOTAL_LEN;
    memcpy(&buf[4],  &send_time, 8);
    memcpy(&buf[12], &next_db,   8);
    memcpy(&buf[20], &tempo_x100, 2);
    buf[22] = beat_in_bar;
    buf[23] = flags;
    memcpy(&buf[24], &next_beat, 8);
    buf[32] = on_air;
    buf[33] = master;
    buf[34] = 0;
    buf[35] = firefly_crc8(&buf[2], 33);
}

// ── CRC tests ──────────────────────────────────────────────────────

TEST(crc8_check_string) {
    uint8_t data[] = "123456789";
    ASSERT_EQ(firefly_crc8(data, 9), 0xA2);
}

TEST(crc8_all_zeros) {
    uint8_t data[10] = {0};
    ASSERT_EQ(firefly_crc8(data, 10), 0x00);
}

TEST(crc8_all_ones) {
    uint8_t data[10];
    memset(data, 0xFF, 10);
    uint8_t crc = firefly_crc8(data, 10);
    ASSERT_TRUE(crc != 0x00); // non-trivial
}

TEST(crc8_single_byte) {
    uint8_t data[] = {0x02};
    uint8_t crc = firefly_crc8(data, 1);
    // Deterministic — just verify it's stable
    ASSERT_EQ(crc, firefly_crc8(data, 1));
}

// ── Protocol struct tests ──────────────────────────────────────────

TEST(packet_struct_size) {
    ASSERT_EQ((int)sizeof(firefly_packet_t), FIREFLY_PACKET_SIZE);
}

TEST(packet_struct_layout) {
    // Verify the packed struct has correct field offsets
    firefly_packet_t pkt;
    memset(&pkt, 0, sizeof(pkt));
    uint8_t *base = (uint8_t *)&pkt;

    // Write known values and verify offsets
    pkt.sync[0] = 0xBE;
    pkt.sync[1] = 0xA7;
    pkt.version = 0x02;
    pkt.total_len = 36;
    pkt.tempo_bpm_x100 = 12630;
    pkt.beat_in_bar = 2;
    pkt.flags = FIREFLY_FLAG_PLAYING | FIREFLY_FLAG_CDJ_ACTIVE;
    pkt.on_air_mask = 0x05;
    pkt.master_device = 3;

    ASSERT_EQ(base[0], 0xBE);
    ASSERT_EQ(base[1], 0xA7);
    ASSERT_EQ(base[2], 0x02);
    ASSERT_EQ(base[3], 36);

    uint16_t tempo;
    memcpy(&tempo, &base[20], 2);
    ASSERT_EQ(tempo, 12630);

    ASSERT_EQ(base[22], 2);
    ASSERT_EQ(base[23], FIREFLY_FLAG_PLAYING | FIREFLY_FLAG_CDJ_ACTIVE);
    ASSERT_EQ(base[32], 0x05);
    ASSERT_EQ(base[33], 3);
}

// ── Dongle framing tests ───────────────────────────────────────────

TEST(dongle_valid_packet) {
    dongle_framer_t f;
    dongle_framer_init(&f);

    uint8_t pkt[FIREFLY_PACKET_SIZE];
    make_packet(pkt, 1000000, 2000000, 12000, 0,
                FIREFLY_FLAG_PLAYING, 1500000, 0, 0);

    bool got_packet = false;
    for (int i = 0; i < FIREFLY_PACKET_SIZE; i++) {
        if (dongle_framer_feed(&f, pkt[i])) got_packet = true;
    }
    ASSERT_TRUE(got_packet);
    ASSERT_EQ(f.packets_forwarded, 1u);
    ASSERT_EQ(f.crc_errors, 0u);
    ASSERT_EQ(f.version_errors, 0u);
}

TEST(dongle_bad_crc) {
    dongle_framer_t f;
    dongle_framer_init(&f);

    uint8_t pkt[FIREFLY_PACKET_SIZE];
    make_packet(pkt, 1000000, 2000000, 12000, 0,
                FIREFLY_FLAG_PLAYING, 1500000, 0, 0);
    pkt[35] ^= 0xFF; // corrupt CRC

    for (int i = 0; i < FIREFLY_PACKET_SIZE; i++) {
        dongle_framer_feed(&f, pkt[i]);
    }
    ASSERT_EQ(f.packets_forwarded, 0u);
    ASSERT_EQ(f.crc_errors, 1u);
}

TEST(dongle_bad_version) {
    dongle_framer_t f;
    dongle_framer_init(&f);

    uint8_t pkt[FIREFLY_PACKET_SIZE];
    make_packet(pkt, 1000000, 2000000, 12000, 0, 0, 0, 0, 0);
    pkt[2] = 0x01; // wrong version
    pkt[35] = firefly_crc8(&pkt[2], 33); // recompute CRC

    for (int i = 0; i < FIREFLY_PACKET_SIZE; i++) {
        dongle_framer_feed(&f, pkt[i]);
    }
    ASSERT_EQ(f.packets_forwarded, 0u);
    ASSERT_EQ(f.version_errors, 1u);
    ASSERT_EQ(f.crc_errors, 0u);
}

TEST(dongle_garbage_before_sync) {
    dongle_framer_t f;
    dongle_framer_init(&f);

    uint8_t pkt[FIREFLY_PACKET_SIZE];
    make_packet(pkt, 1000000, 2000000, 12000, 0,
                FIREFLY_FLAG_PLAYING, 1500000, 0, 0);

    // Feed garbage first
    uint8_t garbage[] = {0x00, 0x11, 0x22, 0x33, 0x44};
    for (int i = 0; i < 5; i++) {
        ASSERT_FALSE(dongle_framer_feed(&f, garbage[i]));
    }

    // Then feed valid packet
    bool got_packet = false;
    for (int i = 0; i < FIREFLY_PACKET_SIZE; i++) {
        if (dongle_framer_feed(&f, pkt[i])) got_packet = true;
    }
    ASSERT_TRUE(got_packet);
    ASSERT_EQ(f.packets_forwarded, 1u);
}

TEST(dongle_false_sync) {
    dongle_framer_t f;
    dongle_framer_init(&f);

    // 0xBE followed by non-0xA7
    dongle_framer_feed(&f, 0xBE);
    dongle_framer_feed(&f, 0x00);

    // Then valid packet
    uint8_t pkt[FIREFLY_PACKET_SIZE];
    make_packet(pkt, 1000000, 2000000, 12000, 0, 0, 0, 0, 0);

    bool got_packet = false;
    for (int i = 0; i < FIREFLY_PACKET_SIZE; i++) {
        if (dongle_framer_feed(&f, pkt[i])) got_packet = true;
    }
    ASSERT_TRUE(got_packet);
}

TEST(dongle_back_to_back) {
    dongle_framer_t f;
    dongle_framer_init(&f);

    uint8_t pkt1[FIREFLY_PACKET_SIZE], pkt2[FIREFLY_PACKET_SIZE];
    make_packet(pkt1, 100, 200, 12000, 0, FIREFLY_FLAG_PLAYING, 150, 0, 0);
    make_packet(pkt2, 300, 400, 12800, 1,
                FIREFLY_FLAG_PLAYING | FIREFLY_FLAG_CDJ_ACTIVE, 350, 0x03, 2);

    int count = 0;
    for (int i = 0; i < FIREFLY_PACKET_SIZE; i++) {
        if (dongle_framer_feed(&f, pkt1[i])) count++;
    }
    for (int i = 0; i < FIREFLY_PACKET_SIZE; i++) {
        if (dongle_framer_feed(&f, pkt2[i])) count++;
    }
    ASSERT_EQ(count, 2);
    ASSERT_EQ(f.packets_forwarded, 2u);
}

TEST(dongle_split_packet) {
    dongle_framer_t f;
    dongle_framer_init(&f);

    uint8_t pkt[FIREFLY_PACKET_SIZE];
    make_packet(pkt, 1000000, 2000000, 12000, 0, 0, 0, 0, 0);

    // Feed first half
    for (int i = 0; i < 18; i++) {
        ASSERT_FALSE(dongle_framer_feed(&f, pkt[i]));
    }
    // Feed second half
    bool got_packet = false;
    for (int i = 18; i < FIREFLY_PACKET_SIZE; i++) {
        if (dongle_framer_feed(&f, pkt[i])) got_packet = true;
    }
    ASSERT_TRUE(got_packet);
}

TEST(dongle_double_sync0_recovery) {
    // Two 0xBE bytes in a row — second one should restart sync detection
    dongle_framer_t f;
    dongle_framer_init(&f);

    dongle_framer_feed(&f, 0xBE);
    dongle_framer_feed(&f, 0xBE); // false first, restart with this as SYNC_0
    dongle_framer_feed(&f, 0xA7); // this should be SYNC_1

    // Now feed remaining 34 bytes of a valid packet
    uint8_t pkt[FIREFLY_PACKET_SIZE];
    make_packet(pkt, 1000, 2000, 12000, 0, 0, 0, 0, 0);

    bool got_packet = false;
    for (int i = 2; i < FIREFLY_PACKET_SIZE; i++) {
        if (dongle_framer_feed(&f, pkt[i])) got_packet = true;
    }
    ASSERT_TRUE(got_packet);
}

// ── Wristband parsing tests ────────────────────────────────────────

TEST(wristband_parse_all_fields) {
    wristband_state_t s;
    wristband_state_init(&s);

    uint8_t pkt[FIREFLY_PACKET_SIZE];
    make_packet(pkt, 1000000, 2000000, 12800, 2,
                FIREFLY_FLAG_PLAYING | FIREFLY_FLAG_CDJ_ACTIVE,
                1500000, 0x05, 3);

    ASSERT_TRUE(wristband_process_packet(&s, pkt, FIREFLY_PACKET_SIZE, 999000));

    ASSERT_EQ(s.next_downbeat_us, 2000000);
    ASSERT_EQ(s.next_beat_us, 1500000);
    ASSERT_EQ(s.tempo_x100, 12800);
    ASSERT_EQ(s.beat_in_bar, 2);
    ASSERT_TRUE(wristband_is_playing(&s));
    ASSERT_TRUE(wristband_is_cdj_active(&s));
    ASSERT_EQ(s.on_air_mask, 0x05);
    ASSERT_EQ(s.master_device, 3);
    ASSERT_EQ(s.packets_received, 1u);
}

TEST(wristband_rejects_bad_sync) {
    wristband_state_t s;
    wristband_state_init(&s);

    uint8_t pkt[FIREFLY_PACKET_SIZE];
    make_packet(pkt, 1000000, 2000000, 12000, 0, 0, 0, 0, 0);
    pkt[0] = 0x00; // break sync

    ASSERT_FALSE(wristband_process_packet(&s, pkt, FIREFLY_PACKET_SIZE, 999000));
    ASSERT_EQ(s.packets_received, 0u);
}

TEST(wristband_rejects_bad_version) {
    wristband_state_t s;
    wristband_state_init(&s);

    uint8_t pkt[FIREFLY_PACKET_SIZE];
    make_packet(pkt, 1000000, 2000000, 12000, 0, 0, 0, 0, 0);
    pkt[2] = 0x01;
    pkt[35] = firefly_crc8(&pkt[2], 33);

    ASSERT_FALSE(wristband_process_packet(&s, pkt, FIREFLY_PACKET_SIZE, 999000));
}

TEST(wristband_rejects_bad_crc) {
    wristband_state_t s;
    wristband_state_init(&s);

    uint8_t pkt[FIREFLY_PACKET_SIZE];
    make_packet(pkt, 1000000, 2000000, 12000, 0, 0, 0, 0, 0);
    pkt[35] ^= 0xFF;

    ASSERT_FALSE(wristband_process_packet(&s, pkt, FIREFLY_PACKET_SIZE, 999000));
}

TEST(wristband_rejects_wrong_length) {
    wristband_state_t s;
    wristband_state_init(&s);

    uint8_t pkt[FIREFLY_PACKET_SIZE];
    make_packet(pkt, 1000000, 2000000, 12000, 0, 0, 0, 0, 0);

    ASSERT_FALSE(wristband_process_packet(&s, pkt, 35, 999000)); // too short
    ASSERT_FALSE(wristband_process_packet(&s, pkt, 37, 999000)); // too long
}

TEST(wristband_clock_offset_first_packet) {
    wristband_state_t s;
    wristband_state_init(&s);

    int64_t send_time = 10000000;
    int64_t local_now = 9990000;

    uint8_t pkt[FIREFLY_PACKET_SIZE];
    make_packet(pkt, send_time, 0, 12000, 0, FIREFLY_FLAG_PLAYING, 0, 0, 0);

    wristband_process_packet(&s, pkt, FIREFLY_PACKET_SIZE, local_now);

    ASSERT_TRUE(s.offset_initialized);
    ASSERT_EQ(s.clock_offset_us, send_time - local_now); // 10000
}

TEST(wristband_clock_offset_ema_converges) {
    wristband_state_t s;
    wristband_state_init(&s);

    int64_t true_offset = 50000;

    for (int i = 0; i < 30; i++) {
        int64_t local_now = 1000000 + (int64_t)i * 50000;
        int64_t send_time = local_now + true_offset;

        uint8_t pkt[FIREFLY_PACKET_SIZE];
        make_packet(pkt, send_time, 0, 12000, 0, FIREFLY_FLAG_PLAYING, 0, 0, 0);
        wristband_process_packet(&s, pkt, FIREFLY_PACKET_SIZE, local_now);
    }

    int64_t error = llabs(s.clock_offset_us - true_offset);
    ASSERT_TRUE(error < 500); // should converge within 500us
}

TEST(wristband_clock_offset_ema_with_jitter) {
    wristband_state_t s;
    wristband_state_init(&s);

    int64_t true_offset = 100000;
    int jitter[] = {1500, -800, 2000, -1200, 500, -1800, 1000, -600,
                    1900, -1500, 700, -900, 1600, -400, 1100, -1700,
                    800, -1000, 1400, -500, 900, -1300, 1800, -200,
                    600, -1600, 1200, -700, 1500, -1100};

    for (int i = 0; i < 30; i++) {
        int64_t local_now = 1000000 + (int64_t)i * 50000;
        int64_t send_time = local_now + true_offset + jitter[i];

        uint8_t pkt[FIREFLY_PACKET_SIZE];
        make_packet(pkt, send_time, 0, 12000, 0, FIREFLY_FLAG_PLAYING, 0, 0, 0);
        wristband_process_packet(&s, pkt, FIREFLY_PACKET_SIZE, local_now);
    }

    int64_t error = llabs(s.clock_offset_us - true_offset);
    ASSERT_TRUE(error < 3000); // bounded under jitter
}

TEST(wristband_to_local_time) {
    wristband_state_t s;
    wristband_state_init(&s);

    uint8_t pkt[FIREFLY_PACKET_SIZE];
    make_packet(pkt, 10000000, 0, 12000, 0, FIREFLY_FLAG_PLAYING, 0, 0, 0);
    wristband_process_packet(&s, pkt, FIREFLY_PACKET_SIZE, 9995000);

    // offset = 10000000 - 9995000 = 5000
    int64_t local = wristband_to_local(&s, 10500000);
    ASSERT_EQ(local, 10500000 - 5000); // 10495000
}

TEST(wristband_flag_helpers) {
    wristband_state_t s;
    wristband_state_init(&s);

    // Not playing, no CDJ
    uint8_t pkt[FIREFLY_PACKET_SIZE];
    make_packet(pkt, 1000000, 0, 12000, 0, 0, 0, 0, 0);
    wristband_process_packet(&s, pkt, FIREFLY_PACKET_SIZE, 999000);
    ASSERT_FALSE(wristband_is_playing(&s));
    ASSERT_FALSE(wristband_is_cdj_active(&s));

    // Playing + CDJ active
    make_packet(pkt, 1001000, 0, 12000, 0,
                FIREFLY_FLAG_PLAYING | FIREFLY_FLAG_CDJ_ACTIVE, 0, 0, 0);
    wristband_process_packet(&s, pkt, FIREFLY_PACKET_SIZE, 1000000);
    ASSERT_TRUE(wristband_is_playing(&s));
    ASSERT_TRUE(wristband_is_cdj_active(&s));
}

// ── End-to-end: coordinator packet → dongle → wristband ───────────

TEST(e2e_packet_through_dongle_to_wristband) {
    // Build packet (simulating coordinator)
    uint8_t pkt[FIREFLY_PACKET_SIZE];
    make_packet(pkt, 10000000, 12000000, 12630, 2,
                FIREFLY_FLAG_PLAYING | FIREFLY_FLAG_CDJ_ACTIVE,
                10500000, 0x05, 1);

    // Feed through dongle framer
    dongle_framer_t dongle;
    dongle_framer_init(&dongle);
    bool forwarded = false;
    for (int i = 0; i < FIREFLY_PACKET_SIZE; i++) {
        if (dongle_framer_feed(&dongle, pkt[i])) forwarded = true;
    }
    ASSERT_TRUE(forwarded);
    ASSERT_EQ(dongle.crc_errors, 0u);

    // Wristband receives the forwarded packet
    wristband_state_t wb;
    wristband_state_init(&wb);
    ASSERT_TRUE(wristband_process_packet(&wb, dongle.rx_buf, FIREFLY_PACKET_SIZE, 9995000));

    // Verify all fields survived the journey
    ASSERT_EQ(wb.next_downbeat_us, 12000000);
    ASSERT_EQ(wb.next_beat_us, 10500000);
    ASSERT_EQ(wb.tempo_x100, 12630);
    ASSERT_EQ(wb.beat_in_bar, 2);
    ASSERT_TRUE(wristband_is_playing(&wb));
    ASSERT_TRUE(wristband_is_cdj_active(&wb));
    ASSERT_EQ(wb.on_air_mask, 0x05);
    ASSERT_EQ(wb.master_device, 1);

    // Verify flash scheduling
    int64_t flash_local = wristband_to_local(&wb, wb.next_beat_us);
    ASSERT_TRUE(flash_local > 9995000); // in the future
}

TEST(e2e_multi_beat_bar) {
    dongle_framer_t dongle;
    dongle_framer_init(&dongle);
    wristband_state_t wb;
    wristband_state_init(&wb);

    int64_t base_time = 10000000;
    int64_t beat_interval_us = 468750; // ~128 BPM

    for (int beat = 0; beat < 4; beat++) {
        int64_t now = base_time + beat * beat_interval_us;
        uint8_t pkt[FIREFLY_PACKET_SIZE];
        make_packet(pkt, now, now + (4 - beat) * beat_interval_us,
                    12800, (uint8_t)beat,
                    FIREFLY_FLAG_PLAYING | FIREFLY_FLAG_CDJ_ACTIVE,
                    now + beat_interval_us, 0x03, 1);

        for (int i = 0; i < FIREFLY_PACKET_SIZE; i++) {
            dongle_framer_feed(&dongle, pkt[i]);
        }
        wristband_process_packet(&wb, dongle.rx_buf, FIREFLY_PACKET_SIZE,
                                 now - 5000); // 5ms transport delay
    }

    ASSERT_EQ(dongle.packets_forwarded, 4u);
    ASSERT_EQ(wb.packets_received, 4u);
    ASSERT_TRUE(wb.offset_initialized);
    ASSERT_EQ(wb.beat_in_bar, 3); // last beat was beat 3 (0-based)
}

// ── Main ───────────────────────────────────────────────────────────

int main() {
    printf("Firefly firmware tests\n\n");

    printf("CRC:\n");
    run_crc8_check_string();
    run_crc8_all_zeros();
    run_crc8_all_ones();
    run_crc8_single_byte();

    printf("\nProtocol:\n");
    run_packet_struct_size();
    run_packet_struct_layout();

    printf("\nDongle framing:\n");
    run_dongle_valid_packet();
    run_dongle_bad_crc();
    run_dongle_bad_version();
    run_dongle_garbage_before_sync();
    run_dongle_false_sync();
    run_dongle_back_to_back();
    run_dongle_split_packet();
    run_dongle_double_sync0_recovery();

    printf("\nWristband parsing:\n");
    run_wristband_parse_all_fields();
    run_wristband_rejects_bad_sync();
    run_wristband_rejects_bad_version();
    run_wristband_rejects_bad_crc();
    run_wristband_rejects_wrong_length();
    run_wristband_clock_offset_first_packet();
    run_wristband_clock_offset_ema_converges();
    run_wristband_clock_offset_ema_with_jitter();
    run_wristband_to_local_time();
    run_wristband_flag_helpers();

    printf("\nEnd-to-end:\n");
    run_e2e_packet_through_dongle_to_wristband();
    run_e2e_multi_beat_bar();

    printf("\n%d/%d tests passed\n", tests_passed, tests_run);
    return (tests_passed == tests_run) ? 0 : 1;
}
