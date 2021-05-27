#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <inttypes.h>
#include <time.h>
#include <errno.h>
#include <assert.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <modality/probe.h>

#include "modality_helpers.h"

#ifndef MODALITY_PERSISTENT_EPOCH_CACHE_DIR
#error "Missing definition MODALITY_PERSISTENT_EPOCH_CACHE_DIR"
#endif

void probe_report_socket_init(
        int * const report_socket)
{
    assert(report_socket != NULL);
    *report_socket = socket(AF_INET, SOCK_DGRAM, 0);
    assert(*report_socket != -1);
}

void probe_report_socket_deinit(
        int * const report_socket)
{
    assert(report_socket != NULL);
    const int err = close(*report_socket);
    *report_socket = -1;
    assert(err == 0);
}

size_t next_persistent_sequence_id(
        const uint32_t probe_id,
        void * const user_state,
        uint16_t * const out_sequence_id)
{
    int err;
    char path_buffer[256];
    unsigned int epoch;

    assert(probe_id != 0);
    assert(user_state == NULL);
    assert(out_sequence_id != NULL);

    const int count = snprintf(
            path_buffer,
            sizeof(path_buffer),
            "%s/probe_%" PRIu32 "",
            MODALITY_PERSISTENT_EPOCH_CACHE_DIR,
            probe_id);
    assert(count < (int) sizeof(path_buffer));

    struct stat st = {0};
    if(stat(MODALITY_PERSISTENT_EPOCH_CACHE_DIR, &st) == -1)
    {
        err = mkdir(MODALITY_PERSISTENT_EPOCH_CACHE_DIR, 0777);
        assert((err == 0) || (errno == EEXIST));
    }

    const int fd = open(path_buffer, O_RDWR | O_CREAT, 0666);
    assert(fd != -1);
    FILE * const file = fdopen(fd, "r+");
    assert(file != NULL);
    rewind(file);

    const int status = fscanf(file, "%u", &epoch);
    if(status == EOF)
    {
        /* First run, default epoch */
        epoch = 0;
    }
    else
    {
        assert(status == 1);
        if(epoch >= 0xFFFF)
        {
            epoch = 0;
        }
        else
        {
            epoch += 1;
        }
    }

    rewind(file);
    err = fprintf(file, "%u", epoch);
    assert(err > 0);
    *out_sequence_id = (uint16_t) epoch;

    err = fflush(file);
    assert(err == 0);
    err = fclose(file);
    assert(err == 0);

    return MODALITY_PROBE_ERROR_OK;
}

void control_msg_callback(
        const uint8_t * const bytes,
        const size_t length,
        void * const opaque_probe)
{
    assert(bytes != NULL);
    assert(opaque_probe != NULL);
    modality_probe * const probe = (modality_probe*) opaque_probe;

    if(length != 0)
    {
        size_t should_forward;
        size_t err = modality_probe_process_control_message(
                probe,
                &bytes[0],
                length,
                &should_forward);
        assert(err == MODALITY_PROBE_ERROR_OK);
    }
}

void send_probe_report(
        modality_probe * const probe,
        const int socket_fd,
        uint8_t * const buffer,
        const size_t buffer_size)
{
    size_t report_size;

    struct sockaddr_in collector_addr = {0};
    collector_addr.sin_family = AF_INET;
    collector_addr.sin_port = htons(COLLECTOR_PORT);
    collector_addr.sin_addr.s_addr = inet_addr(COLLECTOR_ADDRESS);

    assert(buffer_size >= REPORT_SIZE);
    size_t err = modality_probe_report(
            probe,
            &buffer[0],
            buffer_size - ANNOUNCEMENT_RESERVATION_SIZE,
            &report_size);
    assert(err == MODALITY_PROBE_ERROR_OK);

    if(report_size != 0)
    {
        /* Tack on a mutator announcement if there are report bytes to send */
        size_t announcement_size;
        err = modality_probe_announce_mutators(
                probe,
                &buffer[report_size],
                buffer_size - report_size,
                &announcement_size);
        assert(err == MODALITY_PROBE_ERROR_OK);

        const size_t total_size = report_size + announcement_size;
        assert(total_size != 0);
        assert(total_size <= buffer_size);

        const ssize_t status = sendto(
                socket_fd,
                (const char*) &buffer[0],
                total_size,
                0,
                (const struct sockaddr*) &collector_addr,
                sizeof(collector_addr));
        assert(status != -1);
    }
}

uint32_t u32_from_bytes(
        const uint8_t * const four_bytes)
{
    assert(four_bytes != NULL);
    return (four_bytes[0] | (four_bytes[1] << 8) | (four_bytes[2] << 16) | (four_bytes[3] << 24));
}
