#include "serializer.h"

#include <cbor.h>

std::size_t robby::serialize(uint8_t* buffer, std::size_t buffer_size, const AccelerationFrame& frame)
{
    CborEncoder encoder;
    CborEncoder root_map;
    CborEncoder payload_map;
    CborEncoder stream_frame_map;
    CborEncoder values;

    cbor_encoder_init(&encoder, buffer, buffer_size, 0);
    cbor_encoder_create_map(&encoder, &root_map, 1U);

    cbor_encode_text_stringz(&root_map, "payload");
    cbor_encoder_create_map(&root_map, &payload_map, 1U);

    cbor_encode_text_stringz(&payload_map, "StreamFrame");
    cbor_encoder_create_map(&payload_map, &stream_frame_map, 3U);

    cbor_encode_text_stringz(&stream_frame_map, "id");
    cbor_encode_uint(&stream_frame_map, 0U);  // todo!

    cbor_encode_text_stringz(&stream_frame_map, "sequence");
    cbor_encode_uint(&stream_frame_map, 0U);  // todo!

    cbor_encode_text_stringz(&stream_frame_map, "values");
    cbor_encoder_create_array(&stream_frame_map, &values, frame.size());
    for (size_t i = 0; i < frame.size(); ++i) {
        cbor_encode_int(&values, frame[i].z);
    }

    cbor_encoder_close_container(&stream_frame_map, &values);
    cbor_encoder_close_container(&payload_map, &stream_frame_map);
    cbor_encoder_close_container(&root_map, &payload_map);
    CborError result = cbor_encoder_close_container(&encoder, &root_map);

    if (result != CborNoError) {
        return 0U;
    }

    return cbor_encoder_get_buffer_size(&encoder, buffer);
}

std::size_t robby::serialize(uint8_t* buffer, std::size_t buffer_size, const DistanceMap& distances)
{
    const std::size_t n_sensors {distances.sensors.size()};
    const std::size_t resolution_x {distances.sensors[0].size()};
    const std::size_t resolution_y {distances.sensors[0][0].size()};
    CborEncoder encoder;
    CborEncoder root_map;
    CborEncoder payload_map;
    CborEncoder distance_map;
    CborEncoder array_x;
    CborEncoder array_y;

    cbor_encoder_init(&encoder, buffer, buffer_size, 0);
    cbor_encoder_create_map(&encoder, &root_map, 1U);

    cbor_encode_text_stringz(&root_map, "payload");
    cbor_encoder_create_map(&root_map, &payload_map, 1U);

    cbor_encode_text_stringz(&payload_map, "DistanceMap");
    cbor_encoder_create_map(&payload_map, &distance_map, 1U);

    cbor_encode_text_stringz(&distance_map, "distances_mm");
    cbor_encoder_create_array(&distance_map, &array_x, n_sensors * resolution_x);

    for (size_t i = 0; i < distances.sensors.size(); ++i) {
        for (size_t x = 0; x < resolution_x; ++x) {
            cbor_encoder_create_array(&array_x, &array_y, resolution_y);

            for (size_t y = 0; y < resolution_y; ++y) {
                int16_t distance = distances.sensors[i][x][y];
                cbor_encode_uint(&array_y, distance >= 0 ? distance : UINT16_MAX);
            }

            cbor_encoder_close_container(&array_x, &array_y);
        }
    }

    cbor_encoder_close_container(&distance_map, &array_x);
    cbor_encoder_close_container(&payload_map, &distance_map);
    cbor_encoder_close_container(&root_map, &payload_map);
    CborError result = cbor_encoder_close_container(&encoder, &root_map);

    if (result != CborNoError) {
        return 0U;
    }

    return cbor_encoder_get_buffer_size(&encoder, buffer);
}
