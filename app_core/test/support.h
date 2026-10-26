#pragma once

#include "trace/uplink_writer.h"

#include <cstdint>
#include <nlohmann/json.hpp>
#include <optional>

#include <robby_link.pb.h>

#include <string>
#include <vector>

namespace test {

using Bytes = std::vector<uint8_t>;

/// A file of proto/vectors, e.g. "uplink_log.frame"
Bytes read_vector(const std::string& name);
/// The .json of a vector
nlohmann::json read_json(const std::string& name);

/// An Uplink decoded record by record, like the outpost does
struct Uplink
{
    uint32_t seq {0U};
    std::vector<robby_link_Record> records {};
};
std::optional<Uplink> decode_uplink(const Bytes& payload);

/// COBS-decodes a frame (with its 0x00) and checks the CRC; the payload without the CRC
std::optional<Bytes> unframe(const Bytes& frame);
/// The frame of a payload, CRC and COBS
Bytes frame_of(Bytes payload);

Bytes encode_downlink(const robby_link_Downlink& downlink);

/// In the form of the vectors' "value": all fields, of a oneof only the member that is set
nlohmann::json to_json(const robby_link_Record& record);
nlohmann::json to_json(const Uplink& uplink);
nlohmann::json to_json(const robby_link_Downlink& downlink);

/// Collects the frames of an UplinkWriter
class Transport final : public robby::ITransport
{
public:
    bool write(const uint8_t* data, std::size_t length) override
    {
        if (drop) {
            return false;
        }
        frames.emplace_back(data, data + length);
        return true;
    }

    std::vector<Bytes> frames {};
    bool drop {false};
};

}  // namespace test
