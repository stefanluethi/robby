
#[derive(serde::Serialize, serde::Deserialize, Debug, Eq, PartialEq)]
pub struct AdcMessage {
    pub samples: Vec<u16>,
}