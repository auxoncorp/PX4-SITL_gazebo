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

/* Microseconds to nanoseconds */
#define US_TO_NS(us) (us * 1000UL)

#define LOG_PROBE_INIT_W_RECVR(p, r) printf("Probe " #p " (ID %lu) initialized, control receiver %s\n", p, r)

/* Probe log storage size in bytes */
#define PROBE_SIZE (1024 * 2)

/* 1,400 byte report buffers, for sending over UDP socket */
#define REPORT_SIZE (1400)

/* Simulator and px4 are on the same time domain */
#define WALL_CLOCK_ID (1)
#define WALL_CLOCK_RESOLUTION_NS MODALITY_PROBE_TIME_RESOLUTION_UNSPECIFIED

/* Modalityd's UDP collector address that receives probe reports, see Modality.toml */
#define COLLECTOR_ADDRESS "127.0.0.1"
#define COLLECTOR_PORT (2700)

/* Send reports every ~50 ms */
#define REPORT_INTERVAL_MS (50)

/* Send mutator announcements every 2s */
#define MUTATOR_ANNOUNCEMENT_INTERVAL_MS (2000)

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
void send_mutator_announcement(
        modality_probe * const probe,
        const int socket_fd,
        uint8_t * const buffer,
        const size_t buffer_size);
uint32_t u32_from_bytes(
        const uint8_t * const four_bytes);
uint64_t sim_time_to_ns(
        const uint64_t us);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* MODALITY_HELPERS_H */
