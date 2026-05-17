#include "data_publisher.h"

#include "proto/serializer.h"

using namespace robby;

DataPublisher::DataPublisher(
    SerialDriver& serial,
    rtos::MessageQueue<AccelerationFrame>& accelerationFrames,
    DistanceMap& distance_map
    ) :
    _serial(serial),
    _acceleration_frames(accelerationFrames),
    _distance_map(distance_map)
{ }

void DataPublisher::publish()
{
    // IMU measurement streams
    while (_acceleration_frames.tryReceive(_acceleration_frame)) {
        auto length = serialize(
            _cbor_buffer.data(),
            _cbor_buffer.size(),
            _acceleration_frame.x.data(),
            _acceleration_frame.x.size(),
            to_uint(StreamId::ACCELERATION_X),
            _acceleration_sequence_counter
        );
        encode_and_write(length);

        length = serialize(
            _cbor_buffer.data(),
            _cbor_buffer.size(),
            _acceleration_frame.y.data(),
            _acceleration_frame.y.size(),
            to_uint(StreamId::ACCELERATION_Y),
            _acceleration_sequence_counter
        );
        encode_and_write(length);

        length = serialize(
            _cbor_buffer.data(),
            _cbor_buffer.size(),
            _acceleration_frame.z.data(),
            _acceleration_frame.z.size(),
            to_uint(StreamId::ACCELERATION_Z),
            _acceleration_sequence_counter
        );
        encode_and_write(length);

        _acceleration_sequence_counter++;
    }

    // Distance map
    if (_distance_map.updated.tryAcquire(0U) && _distance_map.lock.tryLock(100U)) {
        std::ranges::copy(_distance_map.sensors, _sensors.begin());
        _distance_map.lock.unlock();

        auto length = serialize(_cbor_buffer.data(), _cbor_buffer.size(), _sensors);
        encode_and_write(length);
    }
}
void DataPublisher::encode_and_write(std::size_t length)
{
    std::size_t cobs_length {0U};
    auto encoder_result = cobs_encode(_cbor_buffer.data(), length, _cobs_buffer.data(), _cobs_buffer.size(), &cobs_length);
    if (encoder_result == COBS_RET_SUCCESS) {
        _serial.write(_cobs_buffer.data(), cobs_length);
    }
}

