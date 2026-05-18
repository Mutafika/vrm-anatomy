//! glTFノード情報（seimei 非依存の自前定義）

use serde::{Deserialize, Serialize};

/// glTFノード階層情報
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct GltfNodeInfo {
    pub name: String,
    pub index: usize,
    pub parent: Option<usize>,
    pub children: Vec<usize>,
    pub translation: [f32; 3],
    pub rotation: [f32; 4],
    pub scale: [f32; 3],
}
