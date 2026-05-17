pub mod streams;

use proto::{DistanceMap, ThreadTable};
use crate::app::model::streams::Streams;

#[derive(serde::Deserialize, serde::Serialize)]
#[serde(default)]
pub struct Model {
    pub streams: Streams,
    pub distance_map: DistanceMap,
    pub thread_table: ThreadTable
}

impl Default for Model {
    fn default() -> Self {
        Self {
            streams: Streams::new(),
            distance_map: DistanceMap::default(),
            thread_table: ThreadTable::default(),
        }
    }
}