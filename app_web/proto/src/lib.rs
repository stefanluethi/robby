use serde::{Serialize, Deserialize};

#[derive(Serialize, Deserialize, Debug, PartialEq)]
pub struct AdcMessage {
    pub samples: Vec<u16>,
}

#[derive(Serialize, Deserialize, Debug, PartialEq)]
pub struct StreamAnnounce {
    pub id: u32,
    pub name: String,
    pub scale_factor: f32,
    pub sampling_time: f32,
    // todo: sync
}

#[derive(Serialize, Deserialize, Debug, PartialEq)]
pub struct StreamFrameRaw {
    pub id: u32,
    pub values: Vec<u32>,
}

#[derive(Serialize, Deserialize, Debug, PartialEq)]
pub struct ThreadInfo {
    pub stack_usage: u8,
    pub name: String,
    pub runtime: f32,
}

#[derive(Serialize, Deserialize, Debug, PartialEq)]
pub struct ThreadTable {
    pub threads: Vec<ThreadInfo>,
}

#[derive(Serialize, Deserialize, Debug, PartialEq)]
pub struct LogRecord {
    pub message: String,
}

#[derive(Serialize, Deserialize, Debug, PartialEq)]
pub struct DistanceMap {
    pub distances_mm: [[i16; 8]; 24],
}

#[derive(Serialize, Deserialize, Debug, PartialEq)]
pub enum Payload {
    StreamAnnounce(StreamAnnounce),
    StreamFrame(StreamFrameRaw),
    Log(LogRecord),
    ThreadTable(ThreadTable),
    DistanceMap(DistanceMap)
}

#[derive(Serialize, Deserialize, Debug, PartialEq)]
pub struct Message {
    pub payload: Payload,
}