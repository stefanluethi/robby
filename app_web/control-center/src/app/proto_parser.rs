use crate::app::model::Model;

pub fn parse_message(message: Vec<u8>, model: &mut Model) {
    log::log!(target: "robby", log::Level::Info, "received {}B", message.len());
    if let Ok(message) = serde_cbor_2::from_slice::<proto::Message>(&message) {
        match message.payload {
            proto::Payload::StreamDescriptor(descriptor) => {
                model.streams.update_descriptor(descriptor)
            }
            proto::Payload::StreamFrame(frame) => {
                log::log!(target: "robby", log::Level::Trace, "ws received {} samples", frame.values.len());
                model.streams.append(&frame);
            }
            proto::Payload::Log(_) => todo!(),
            proto::Payload::ThreadTable(thread_table) => {
                for thread in thread_table.threads.iter() {
                    log::log!(target: "robby", log::Level::Info, "thread info \"{}\" stack usage: {}, cpu usage {}",
                                        thread.name, thread.stack_usage, thread.runtime);
                }
                model.thread_table = thread_table;
            }
            proto::Payload::DistanceMap(distances) => {
                // log::log!(target: "robby", log::Level::Info, "distances {:?}", &distances);
                model.distance_map = distances;
            }
        };
    }
}