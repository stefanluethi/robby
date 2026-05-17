#include "serializer.h"

#include <cbor.h>

std::size_t robby::serialize(uint8_t* buffer, std::size_t buffer_size, const int16_t* array, std::size_t length, uint8_t stream_id, uint32_t sequence_number)
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
    cbor_encode_uint(&stream_frame_map, stream_id);

    cbor_encode_text_stringz(&stream_frame_map, "sequence");
    cbor_encode_uint(&stream_frame_map, sequence_number);

    cbor_encode_text_stringz(&stream_frame_map, "values");
    cbor_encoder_create_array(&stream_frame_map, &values, length);
    for (size_t i = 0; i < length; ++i) {
        cbor_encode_int(&values, *(array + i));
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

std::size_t robby::serialize(uint8_t* buffer, std::size_t buffer_size, const std::array<DistanceSensorImage, 3>& sensors)
{
    const std::size_t n_sensors {sensors.size()};
    const std::size_t resolution_x {sensors[0].size()};
    const std::size_t resolution_y {sensors[0][0].size()};
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

    for (const auto& sensor : sensors) {
        for (const auto& row : sensor) {
            cbor_encoder_create_array(&array_x, &array_y, resolution_y);

            for (int16_t distance : row) {
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
std::size_t robby::serialize(uint8_t* buffer, std::size_t buffer_size, const StreamDescriptor& descriptor)
{
    CborEncoder encoder;
    CborEncoder root_map;
    CborEncoder payload_map;
    CborEncoder stream_descriptor_map;

    cbor_encoder_init(&encoder, buffer, buffer_size, 0);
    cbor_encoder_create_map(&encoder, &root_map, 1U);

    cbor_encode_text_stringz(&root_map, "payload");
    cbor_encoder_create_map(&root_map, &payload_map, 1U);

    cbor_encode_text_stringz(&payload_map, "StreamDescriptor");
    cbor_encoder_create_map(&payload_map, &stream_descriptor_map, 5U);

    cbor_encode_text_stringz(&stream_descriptor_map, "id");
    cbor_encode_uint(&stream_descriptor_map, descriptor.id);

    cbor_encode_text_stringz(&stream_descriptor_map, "name");
    cbor_encode_text_stringz(&stream_descriptor_map, descriptor.name);

    cbor_encode_text_stringz(&stream_descriptor_map, "scale_factor");
    cbor_encode_float(&stream_descriptor_map, descriptor.scale_factor);

    cbor_encode_text_stringz(&stream_descriptor_map, "unit");
    cbor_encode_text_stringz(&stream_descriptor_map, descriptor.unit);

    cbor_encode_text_stringz(&stream_descriptor_map, "sampling_time");
    cbor_encode_float(&stream_descriptor_map, descriptor.sampling_time);

    cbor_encoder_close_container(&payload_map, &stream_descriptor_map);
    cbor_encoder_close_container(&root_map, &payload_map);
    CborError result = cbor_encoder_close_container(&encoder, &root_map);

    if (result != CborNoError) {
        return 0U;
    }

    return cbor_encoder_get_buffer_size(&encoder, buffer);
}
