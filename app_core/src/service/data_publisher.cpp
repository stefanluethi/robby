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
    while (_acceleration_frames.tryReceive(_acceleration_frame)) {
        auto length = serialize(_cbor_buffer.data(), _cbor_buffer.size(), _acceleration_frame);
        encode_and_write(length);
    }

    if (_distance_map.updated.tryAcquire(0U) && _distance_map.lock.tryLock(100U)) {
        auto length = serialize(_cbor_buffer.data(), _cbor_buffer.size(), _distance_map);
        encode_and_write(length);
        _distance_map.lock.unlock();
        // todo: make copy to reduce lock time
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

