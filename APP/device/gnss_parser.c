#include "gnss_parser.h"

#include <string.h>

static uint16_t read_le16(const uint8_t *p)
{
    return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

static uint32_t read_le32(const uint8_t *p)
{
    return (uint32_t)p[0] |
           ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16) |
           ((uint32_t)p[3] << 24);
}

static uint32_t crc32_update(uint32_t crc, uint8_t data)
{
    crc ^= data;
    for (uint32_t i = 0; i < 8U; ++i) {
        crc = (crc & 1U) ? ((crc >> 1) ^ 0xEDB88320UL) : (crc >> 1);
    }
    return crc;
}

static uint32_t gnss_crc32(const uint8_t *data, uint16_t len)
{
    uint32_t crc = 0U;
    for (uint16_t i = 0; i < len; ++i) {
        crc = crc32_update(crc, data[i]);
    }
    return crc;
}

static void reset_parser(GnssParser *parser)
{
    parser->state = GNSS_PARSER_WAIT_SYNC_1;
    parser->bytes_read = 0;
    parser->payload_len = 0;
    parser->frame_len = 0;
}

void GnssParser_Init(GnssParser *parser, GnssParserHandler handler, void *user)
{
    if (parser == NULL) {
        return;
    }

    memset(parser, 0, sizeof(*parser));
    parser->handler = handler;
    parser->user = user;
    parser->state = GNSS_PARSER_WAIT_SYNC_1;
}

static void finish_frame(GnssParser *parser)
{
    uint16_t crc_offset = (uint16_t)(parser->frame_len - GNSS_CRC_LEN);
    uint32_t received = read_le32(&parser->frame[crc_offset]);
    uint32_t calculated = gnss_crc32(parser->frame, crc_offset);

    if (received == calculated) {
        parser->frame_count++;
        if (parser->handler != NULL) {
            uint16_t msg_id = read_le16(&parser->frame[GNSS_MSG_ID_OFFSET]);
            parser->handler(msg_id, parser->frame, parser->frame_len, parser->user);
        }
    } else {
        parser->crc_error_count++;
    }

    reset_parser(parser);
}

static void process_byte(GnssParser *parser, uint8_t byte)
{
    switch (parser->state) {
        case GNSS_PARSER_WAIT_SYNC_1:
            if (byte == GNSS_SYNC_1) {
                parser->frame[0] = byte;
                parser->bytes_read = 1;
                parser->state = GNSS_PARSER_WAIT_SYNC_2;
            }
            break;

        case GNSS_PARSER_WAIT_SYNC_2:
            if (byte == GNSS_SYNC_2) {
                parser->frame[parser->bytes_read++] = byte;
                parser->state = GNSS_PARSER_WAIT_SYNC_3;
            } else {
                reset_parser(parser);
            }
            break;

        case GNSS_PARSER_WAIT_SYNC_3:
            if (byte == GNSS_SYNC_3) {
                parser->frame[parser->bytes_read++] = byte;
                parser->state = GNSS_PARSER_READ_HEADER;
            } else {
                reset_parser(parser);
            }
            break;

        case GNSS_PARSER_READ_HEADER:
            parser->frame[parser->bytes_read++] = byte;
            if (parser->bytes_read == GNSS_HEADER_LEN) {
                parser->payload_len = read_le16(&parser->frame[GNSS_LENGTH_OFFSET]);
                if (parser->payload_len > GNSS_MAX_PAYLOAD_LEN) {
                    reset_parser(parser);
                    break;
                }
                parser->frame_len = (uint16_t)(GNSS_HEADER_LEN +
                                               parser->payload_len +
                                               GNSS_CRC_LEN);
                parser->state = GNSS_PARSER_READ_BODY;
            }
            break;

        case GNSS_PARSER_READ_BODY:
            parser->frame[parser->bytes_read++] = byte;
            if (parser->bytes_read >= parser->frame_len) {
                finish_frame(parser);
            }
            break;

        default:
            reset_parser(parser);
            break;
    }
}

void GnssParser_ProcessBlock(GnssParser *parser, const uint8_t *data, size_t len)
{
    if (parser == NULL || data == NULL) {
        return;
    }

    for (size_t i = 0; i < len; ++i) {
        process_byte(parser, data[i]);
    }
}

bool GnssParser_ExtractUtc(const uint8_t *frame, uint16_t frame_len, TimeUtcClock *utc)
{
    if (frame == NULL || utc == NULL || frame_len < GNSS_HEADER_LEN) {
        return false;
    }

    utc->week = read_le16(&frame[GNSS_WEEK_OFFSET]);
    utc->week_ms = read_le32(&frame[GNSS_WEEK_MS_OFFSET]);

    return utc->week_ms < TIME_SERVICE_UTC_MS_PER_WEEK;
}
