use serde::{Serialize, Deserialize};

#[derive(Serialize, Deserialize, Debug, PartialEq, Clone)]
pub struct StreamDescriptor {
    pub id: u16,
    pub name: String,
    pub scale_factor: f32,
    pub unit: String,
    pub sampling_time: f32,
}


#[derive(Serialize, Deserialize, Debug, PartialEq)]
pub struct StreamSyncId {
    pub id: u16,
    /// Frame sequence count the sync packet applies to
    pub sequence: u16,
}

#[derive(Serialize, Deserialize, Debug, PartialEq)]
pub struct StreamSync {
    pub time: u64,
    pub streams: Vec<StreamSyncId>,
}

#[derive(Serialize, Deserialize, Debug, PartialEq)]
pub struct StreamFrameRaw {
    pub id: u16,
    /// Frame packet sequence counter
    pub sequence: u16,
    pub values: Vec<i32>,
}

#[derive(Serialize, Deserialize, Debug, PartialEq)]
pub struct ThreadInfo {
    pub name: String,
    pub priority: u8,
    pub stack_usage: f32,
    pub stack_size: u32,
    pub runtime: f32,
}

#[derive(Serialize, Deserialize, Debug, PartialEq, Default)]
pub struct ThreadTable {
    pub threads: Vec<ThreadInfo>,
}

#[derive(Serialize, Deserialize, Debug, PartialEq)]
pub struct LogRecord {
    pub message: String,
}

#[derive(Serialize, Deserialize, Debug, PartialEq, Default)]
pub struct DistanceMap {
    pub distances_mm: [[u16; 8]; 24],
}

#[derive(Serialize, Deserialize, Debug, PartialEq)]
pub enum Payload {
    StreamDescriptor(StreamDescriptor),
    StreamFrame(StreamFrameRaw),
    Log(LogRecord),
    ThreadTable(ThreadTable),
    DistanceMap(DistanceMap)
}

#[derive(Serialize, Deserialize, Debug, PartialEq)]
pub struct Message {
    pub payload: Payload,
}


#[derive(Serialize, Deserialize, Debug, PartialEq)]
pub enum Command {
    SetMotorSpeed(u16),
}
