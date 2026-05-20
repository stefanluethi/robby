use crate::app::model::Model;

pub fn parse_message(message: Vec<u8>, model: &mut Model) {
    log::info!("received {}B", message.len());
    match serde_cbor_2::from_slice::<proto::Message>(&message) {
        Ok(message) => {
            match message.payload {
                proto::Payload::StreamDescriptor(descriptor) => {
                    log::info!("received stream descriptor {}", descriptor.id);
                    model.streams.update_descriptor(descriptor)
                }
                proto::Payload::StreamFrame(frame) => {
                    log::info!("received {} samples", frame.values.len());
                    model.streams.append(&frame);
                }
                proto::Payload::Log(message) => {
                    log::info!("received log message");
                    let level = match message.level {
                        0 => log::Level::Trace,
                        1 => log::Level::Debug,
                        2 => log::Level::Info,
                        3 => log::Level::Warn,
                        4 => log::Level::Error,
                        _ => log::Level::Trace,
                    };
                    log::log!(target: "robby", level, "{}", message.message);
                },
                proto::Payload::ThreadTable(thread_table) => {
                    log::info!("received thread info");
                    model.thread_table = thread_table;
                }
                proto::Payload::DistanceMap(distances) => {
                    log::info!("received distance map");
                    // log::info!("distances {:?}", &distances);
                    model.distance_map = distances;
                }
            };
        }
        Err(e) => {
            log::warn!("error parsing message: {}", e);
        }
    }
}