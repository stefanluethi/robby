
#[derive(serde::Serialize, serde::Deserialize, Debug, Eq, PartialEq)]
pub struct AdcMessage {
    pub samples: Vec<u16>,
}

#[derive(serde::Serialize, serde::Deserialize, Debug, PartialEq)]
pub struct ThreadInfo {
    pub stack_usage: u8,
    pub name: String,
    pub runtime: f32,
}

#[derive(serde::Serialize, serde::Deserialize, Debug, PartialEq)]
pub struct ThreadTable {
    pub threads: Vec<ThreadInfo>,
}
