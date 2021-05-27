#ifndef MODALITY_HELPERS_H
#define MODALITY_HELPERS_H

/* Keep asserts around in release mode */
#ifdef NDEBUG
#undef NDEBUG
#endif

#include <assert.h>
#include <inttypes.h>

#include <modality/probe.h>
#include <modality/mutator.h>
#include <modality/mutation_interface.h>
#include <modality/control_message_transport.h>

#ifdef __cplusplus
extern "C" {
#endif

#define LOG_PROBE_INIT_W_RECVR(p, r) printf("Probe " #p " (ID %lu) initialized, control receiver %s\n", p, r)

/* Probe log storage size in bytes */
#define PROBE_SIZE (1024 * 2)

/* 1,400 byte report buffers, for sending over UDP socket */
#define REPORT_SIZE (1400)

/* Reserve a portion of the report buffer for mutator announcements */
#define ANNOUNCEMENT_RESERVATION_SIZE (64)

#define WALL_CLOCK_ID MODALITY_PROBE_WALL_CLOCK_ID_LOCAL_ONLY
#define WALL_CLOCK_RESOLUTION_NS MODALITY_PROBE_TIME_RESOLUTION_UNSPECIFIED

/* Modalityd's UDP collector address that receives probe reports, see Modality.toml */
#define COLLECTOR_ADDRESS "127.0.0.1"
#define COLLECTOR_PORT (2718)

/* Probes will send reports every ~25 ms by default, some probes will use their own reporting cadence */
#define REPORT_INTERVAL_MS (25)

/* Control message receivers, see Modality.toml */
#define UDP_CONTROL_RECVR_GAZEBO_PLUGIN "127.0.0.1:33000"

/* Helper and utility functions */
void probe_report_socket_init(
        int * const report_socket);
void probe_report_socket_deinit(
        int * const report_socket);
size_t next_persistent_sequence_id(
        const uint32_t probe_id,
        void * const user_state,
        uint16_t * const out_sequence_id);
void control_msg_callback(
        const uint8_t * const bytes,
        const size_t length,
        void * const opaque_probe);
void send_probe_report(
        modality_probe * const probe,
        const int socket_fd,
        uint8_t * const buffer,
        const size_t buffer_size);
uint32_t u32_from_bytes(
        const uint8_t * const four_bytes);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* MODALITY_HELPERS_H */
