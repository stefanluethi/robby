#include "uplink_writer.h"

#include <pb_encode.h>

using namespace robby;

namespace {

/// Bytes of a varint
std::size_t varint_size(uint64_t value)
{
    std::size_t size {1U};
    while (value >= 0x80U) {
        value >>= 7U;
        ++size;
    }
    return size;
}

}  // namespace

UplinkWriter::UplinkWriter(ITransport& transport, Crc16Function crc, uint32_t first_seq) :
    _transport {transport},
    _crc {crc},
    _seq {first_seq}
{
}

bool UplinkWriter::add(const robby_link_Record& record)
{
    std::size_t size {0U};
    if (!pb_get_encoded_size(&size, robby_link_Record_fields, &record)) {
        ++_counters.records_dropped;
        return false;
    }
    // field 2 (records) with its length, then the record
    const std::size_t needed {1U + varint_size(size) + size};

    if (_open && (_length + needed > frame::MAX_PAYLOAD || _records == MAX_RECORDS)) {
        flush();
    }
    if (!_open) {
        open_frame();
    }
    if (_length + needed > frame::MAX_PAYLOAD) {
        // can't happen with the limits of robby_link.options, any record fits an empty frame
        ++_counters.records_dropped;
        return false;
    }

    pb_ostream_t stream = pb_ostream_from_buffer(_payload.data() + _length, frame::MAX_PAYLOAD - _length);
    if (!pb_encode_tag(&stream, PB_WT_STRING, robby_link_Uplink_records_tag) || !pb_encode_varint(&stream, size) ||
        !pb_encode(&stream, robby_link_Record_fields, &record) || stream.bytes_written != needed) {
        ++_counters.records_dropped;
        return false;
    }

    _length += stream.bytes_written;
    ++_records;
    return true;
}

void UplinkWriter::flush()
{
    if (!_open || _records == 0U) {
        return;
    }

    auto& frame = _frames[_next_frame];
    const std::size_t frame_length {frame::encode(_payload.data(), _length, _crc, frame.data(), frame.size())};

    if (frame_length > 0U && _transport.write(frame.data(), frame_length)) {
        _counters.tx_bytes += frame_length;
        ++_counters.tx_frames;
    } else {
        ++_counters.tx_frames_dropped;
    }

    // a dropped frame still takes its seq, the receiver sees it as a gap
    ++_seq;
    _open = false;
    _next_frame = (_next_frame + 1U) % _frames.size();
}

void UplinkWriter::open_frame()
{
    _length = 0U;
    _records = 0U;
    _open = true;

    // proto3: a seq of 0 is left out
    if (_seq != 0U) {
        pb_ostream_t stream = pb_ostream_from_buffer(_payload.data(), frame::MAX_PAYLOAD);
        pb_encode_tag(&stream, PB_WT_VARINT, robby_link_Uplink_seq_tag);
        pb_encode_varint(&stream, _seq);
        _length = stream.bytes_written;
    }
}
