use std::collections::HashMap;
use proto::StreamDescriptor;

#[derive(serde::Deserialize, serde::Serialize)]
#[serde(default)]
pub struct Stream {
    pub values_raw: Vec<i32>,
    pub descriptor: StreamDescriptor,
    last_sequence_count: u16,
}

impl Default for Stream {
    fn default() -> Self {
        Self::new(0xFFFF)
    }
}

impl Stream {
    pub fn new(id: u16) -> Self {
        Self {
            values_raw: vec![],
            descriptor: StreamDescriptor {
                id,
                name: format!("{id}").to_string(),
                scale_factor: 1.0,
                unit: "u".to_string(),
                sampling_time: 1.0,
            },
            last_sequence_count: 0,
        }
    }
}

impl From<&StreamDescriptor> for Stream {
    fn from(descriptor: &StreamDescriptor) -> Self {
        Stream {
            values_raw: vec![],
            descriptor: (*descriptor).clone(),
            last_sequence_count: 0,
        }
    }
}

#[derive(serde::Deserialize, serde::Serialize)]
#[serde(default)]
pub struct Streams {
    map: HashMap<u16, Stream>,
}

impl Default for Streams {
    fn default() -> Self {
        Self::new()
    }
}

impl Streams {
    pub fn new() -> Self {
        Self { map: HashMap::new() }
    }

    pub fn iter(&self) -> impl Iterator<Item = &Stream> {
        self.map.values()
    }

    pub fn append(&mut self, frame: &proto::StreamFrameRaw) {
        if !self.map.contains_key(&frame.id) {
            self.map.insert(frame.id, Stream::new(frame.id));
        }
        let stream = self.map.get_mut(&frame.id).unwrap();
        if stream.last_sequence_count + 1 != frame.sequence {
            log::warn!("stream {}: expected sequence {}, got {}", frame.id, stream.last_sequence_count + 1, frame.sequence);
            // todo: handle jump in data?
        }
        stream.last_sequence_count = frame.sequence;
        stream.values_raw.append(&mut frame.values.clone());
    }

    pub fn update_descriptor(&mut self, descriptor: proto::StreamDescriptor) {
        if self.map.contains_key(&descriptor.id) {
            self.map.get_mut(&descriptor.id).unwrap().descriptor = descriptor.clone();
        } else {
            self.map.insert(descriptor.id, (&descriptor).into());
        }
    }

    pub fn clear(&mut self) {
        for stream in self.map.values_mut() {
            stream.values_raw.clear();
        }
    }
}
