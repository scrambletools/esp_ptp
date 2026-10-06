#pragma once
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <time.h>
#include "ptp.h"

/* Caller serializes all access. Network I/O must happen outside that lock. */
typedef struct {
    uint32_t generation, lifecycle;
    uint16_t sequence;
    uint8_t requester[10];
    int64_t deadline_us;
    bool published, response_seen, consumed, multiple, loss_counted;
    bool sending, follow_up_pending;
    struct ptp_delay_resp_follow_up_s pending_follow_up;
    uint16_t lost_responses;
    uint32_t missing_response, missing_follow_up;
    uint16_t response_count, response_rejections;
    uint16_t follow_up_count, follow_up_rejections;
    int64_t first_response_us, published_us;
    struct timespec transmit, receive;
    struct ptp_delay_resp_s response;
} ptp_peer_exchange_t;

typedef struct {
    uint32_t generation, lifecycle;
    struct timespec transmit, receive;
    struct ptp_delay_resp_s response;
} ptp_peer_measurement_t;

enum {
    PTP_PEER_RX_UNPUBLISHED = 1, PTP_PEER_RX_FAULTED = 2,
    PTP_PEER_RX_EXPIRED = 4, PTP_PEER_RX_REQUESTER = 8,
    PTP_PEER_RX_TIMESTAMP = 16
};

enum { PTP_PEER_IGNORED, PTP_PEER_ACCEPTED, PTP_PEER_MULTIPLE, PTP_PEER_DUPLICATE, PTP_PEER_SELF };

static inline void ptp_peer_invalidate(ptp_peer_exchange_t *exchange)
{
    uint32_t generation = exchange->generation + 1;
    uint32_t lifecycle = exchange->lifecycle + 1;
    memset(exchange, 0, sizeof(*exchange));
    exchange->generation = generation ? generation : 1;
    exchange->lifecycle = lifecycle ? lifecycle : 1;
}

/* Count each expired published request once, including missing Follow_Up. */
static inline bool ptp_peer_expire(ptp_peer_exchange_t *exchange, int64_t now_us)
{
    if (!exchange->published || exchange->consumed || exchange->loss_counted ||
        now_us < exchange->deadline_us) return false;
    exchange->loss_counted = true;
    if (exchange->lost_responses < UINT16_MAX) exchange->lost_responses++;
    if (exchange->response_seen) exchange->missing_follow_up++;
    else exchange->missing_response++;
    return true;
}

static inline uint32_t ptp_peer_begin(ptp_peer_exchange_t *exchange,
    uint16_t sequence, const struct ptp_header_s *request, int64_t deadline_us)
{
    uint32_t lifecycle = exchange->lifecycle;
    uint16_t lost_responses = exchange->lost_responses;
    uint32_t missing_response = exchange->missing_response;
    uint32_t missing_follow_up = exchange->missing_follow_up;
    ptp_peer_invalidate(exchange);
    exchange->lifecycle = lifecycle;
    exchange->lost_responses = lost_responses;
    exchange->missing_response = missing_response;
    exchange->missing_follow_up = missing_follow_up;
    exchange->sequence = sequence;
    memcpy(exchange->requester, request->sourceidentity, 8);
    memcpy(exchange->requester + 8, request->sourceportindex, 2);
    exchange->deadline_us = deadline_us;
    exchange->sending = true;
    return exchange->generation;
}

static inline bool ptp_peer_publish(ptp_peer_exchange_t *exchange,
    uint32_t generation, const struct timespec *transmit, int64_t now_us)
{
    if (exchange->generation != generation || !exchange->sending || exchange->published ||
        now_us >= exchange->deadline_us || transmit->tv_nsec < 0 ||
        transmit->tv_nsec >= 1000000000L) return false;
    exchange->transmit = *transmit;
    exchange->published = true;
    exchange->sending = false;
    exchange->published_us = now_us;
    return true;
}

static inline void ptp_peer_cancel(ptp_peer_exchange_t *exchange, uint32_t generation)
{
    if (exchange->generation != generation) return;
    exchange->sending = exchange->published = exchange->response_seen = false;
    exchange->follow_up_pending = false;
}

static inline bool ptp_peer_timestamp_valid(const uint8_t stamp[10])
{
    uint32_t nanoseconds = ((uint32_t)stamp[6] << 24) |
        ((uint32_t)stamp[7] << 16) | ((uint32_t)stamp[8] << 8) | stamp[9];
    return nanoseconds < 1000000000U;
}

static inline bool ptp_peer_matches(const ptp_peer_exchange_t *exchange,
    const struct ptp_header_s *header, const uint8_t requester[8],
    const uint8_t port[2], int64_t now_us)
{
    return (exchange->published || exchange->sending) && !exchange->multiple &&
        now_us < exchange->deadline_us &&
        exchange->sequence == (((uint16_t)header->sequenceid[0] << 8) |
                               header->sequenceid[1]) &&
        !memcmp(exchange->requester, requester, 8) &&
        !memcmp(exchange->requester + 8, port, 2);
}

static inline int ptp_peer_response(ptp_peer_exchange_t *exchange,
    const struct ptp_delay_resp_s *response, const struct timespec *receive,
    int64_t now_us)
{
    uint16_t sequence = ((uint16_t)response->header.sequenceid[0] << 8) |
                         response->header.sequenceid[1];
    if (sequence != exchange->sequence) return PTP_PEER_IGNORED;
    if (!exchange->response_count) exchange->first_response_us = now_us;
    if (exchange->response_count < UINT16_MAX) exchange->response_count++;
    uint16_t rejection = 0;
    if (!exchange->published && !exchange->sending) rejection |= PTP_PEER_RX_UNPUBLISHED;
    if (exchange->multiple) rejection |= PTP_PEER_RX_FAULTED;
    if (now_us >= exchange->deadline_us) rejection |= PTP_PEER_RX_EXPIRED;
    if (memcmp(exchange->requester, response->reqidentity, 8) ||
        memcmp(exchange->requester + 8, response->reqportindex, 2))
        rejection |= PTP_PEER_RX_REQUESTER;
    if (!ptp_peer_timestamp_valid(response->receivetimestamp) ||
        receive->tv_nsec < 0 || receive->tv_nsec >= 1000000000L)
        rejection |= PTP_PEER_RX_TIMESTAMP;
    exchange->response_rejections |= rejection;
    if (rejection) return PTP_PEER_IGNORED;
    if (!memcmp(response->header.sourceidentity, exchange->requester, 8)) {
        exchange->multiple = true;
        return PTP_PEER_SELF;
    }
    if (exchange->response_seen) {
        if (memcmp(exchange->response.header.sourceidentity,
                   response->header.sourceidentity, 8) ||
            memcmp(exchange->response.header.sourceportindex,
                   response->header.sourceportindex, 2)) {
            exchange->multiple = true;
            return PTP_PEER_MULTIPLE;
        }
        exchange->multiple = true;
        return PTP_PEER_DUPLICATE;
    }
    exchange->response = *response;
    exchange->receive = *receive;
    exchange->response_seen = true;
    return PTP_PEER_ACCEPTED;
}

static inline bool ptp_peer_finish(ptp_peer_exchange_t *exchange,
    const struct ptp_delay_resp_follow_up_s *follow_up, int64_t now_us,
    ptp_peer_measurement_t *measurement)
{
    uint16_t sequence = ((uint16_t)follow_up->header.sequenceid[0] << 8) |
                         follow_up->header.sequenceid[1];
    if (sequence == exchange->sequence) {
        if (exchange->follow_up_count < UINT16_MAX) exchange->follow_up_count++;
        uint16_t rejection = 0;
        if (!exchange->published && !exchange->sending) rejection |= 1;
        if (exchange->multiple) rejection |= 2;
        if (now_us >= exchange->deadline_us) rejection |= 4;
        if (memcmp(exchange->requester, follow_up->reqidentity, 8) ||
            memcmp(exchange->requester + 8, follow_up->reqportindex, 2)) rejection |= 8;
        if (!exchange->response_seen) rejection |= 16;
        if (!ptp_peer_timestamp_valid(follow_up->origintimestamp)) rejection |= 32;
        if (memcmp(exchange->response.header.sourceidentity,
                   follow_up->header.sourceidentity, 8) ||
            memcmp(exchange->response.header.sourceportindex,
                   follow_up->header.sourceportindex, 2)) rejection |= 64;
        if (exchange->consumed || exchange->follow_up_pending) rejection |= 128;
        exchange->follow_up_rejections |= rejection;
    }
    if (!ptp_peer_matches(exchange, &follow_up->header, follow_up->reqidentity,
                         follow_up->reqportindex, now_us) ||
        !exchange->response_seen ||
        !ptp_peer_timestamp_valid(follow_up->origintimestamp) ||
        memcmp(exchange->response.header.sourceidentity,
               follow_up->header.sourceidentity, 8) ||
        memcmp(exchange->response.header.sourceportindex,
               follow_up->header.sourceportindex, 2)) return false;
    if (exchange->consumed || exchange->follow_up_pending) {
        exchange->multiple = true;
        return false;
    }
    if (!exchange->published) {
        exchange->pending_follow_up = *follow_up;
        exchange->follow_up_pending = true;
        return false;
    }
    measurement->generation = exchange->generation;
    measurement->lifecycle = exchange->lifecycle;
    measurement->transmit = exchange->transmit;
    measurement->receive = exchange->receive;
    measurement->response = exchange->response;
    exchange->consumed = true;
    exchange->lost_responses = 0;
    return true;
}

/* The daemon replays a retained Follow_Up only after T1 publication. */
static inline bool ptp_peer_take_follow_up(ptp_peer_exchange_t *exchange,
    int64_t now_us, struct ptp_delay_resp_follow_up_s *follow_up,
    uint32_t *generation)
{
    if (!exchange->published || !exchange->follow_up_pending || exchange->multiple ||
        exchange->consumed || now_us >= exchange->deadline_us) return false;
    *follow_up = exchange->pending_follow_up;
    *generation = exchange->generation;
    exchange->follow_up_pending = false;
    return true;
}
