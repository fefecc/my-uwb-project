#ifndef APP_DEVICE_GNSS_PARSER_H_
#define APP_DEVICE_GNSS_PARSER_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "../service/time_service.h"

#ifdef __cplusplus
extern "C" {
#endif

#define GNSS_SYNC_1              (0xAAU)
#define GNSS_SYNC_2              (0x44U)
#define GNSS_SYNC_3              (0xB5U)
#define GNSS_HEADER_LEN          (24U)
#define GNSS_CRC_LEN             (4U)
#define GNSS_MAX_PAYLOAD_LEN     (1024U)
#define GNSS_MAX_FRAME_LEN       (GNSS_HEADER_LEN + GNSS_MAX_PAYLOAD_LEN + GNSS_CRC_LEN)

#define GNSS_MSG_ID_OFFSET       (4U)
#define GNSS_LENGTH_OFFSET       (6U)
#define GNSS_TIME_REF_OFFSET     (8U)
#define GNSS_TIME_STATUS_OFFSET  (9U)
#define GNSS_WEEK_OFFSET         (10U)
#define GNSS_WEEK_MS_OFFSET      (12U)

typedef void (*GnssParserHandler)(uint16_t msg_id,
                                  const uint8_t *frame,
                                  uint16_t frame_len,
                                  void *user);

typedef enum {
    GNSS_PARSER_WAIT_SYNC_1 = 0,
    GNSS_PARSER_WAIT_SYNC_2,
    GNSS_PARSER_WAIT_SYNC_3,
    GNSS_PARSER_READ_HEADER,
    GNSS_PARSER_READ_BODY,
} GnssParserState;

typedef struct {
    GnssParserState state;
    GnssParserHandler handler;
    void *user;
    uint8_t frame[GNSS_MAX_FRAME_LEN];
    uint16_t bytes_read;
    uint16_t payload_len;
    uint16_t frame_len;
    uint32_t crc_error_count;
    uint32_t frame_count;
} GnssParser;

void GnssParser_Init(GnssParser *parser, GnssParserHandler handler, void *user);
void GnssParser_ProcessBlock(GnssParser *parser, const uint8_t *data, size_t len);
bool GnssParser_ExtractUtc(const uint8_t *frame, uint16_t frame_len, TimeUtcClock *utc);

#ifdef __cplusplus
}
#endif

#endif /* APP_DEVICE_GNSS_PARSER_H_ */
