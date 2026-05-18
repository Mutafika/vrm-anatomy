//! vrm-core — VRMアバター基盤ライブラリ
//!
//! スケルトン、アニメーション、VRMパース、CPUスキニングを提供。
//! 数学ライブラリ非依存（[f32; 3]/[f32; 4]ベース）。

pub mod animation;
pub mod expression;
pub mod gltf_types;
pub mod skeleton;
pub mod skinning;
pub mod vrm;

pub use animation::{
    AnimationClip, AnimationPlayer, BoneTrack, Easing, Keyframe, LoopMode,
};
pub use expression::{
    Expression, ExpressionKind, ExpressionPreset, Expressions, MorphAddressing,
    MorphTargetBind, OverrideMode, parse_vrm0_blend_shapes, parse_vrmc_expressions,
    resolve_weights,
};
pub use gltf_types::GltfNodeInfo;
pub use skeleton::{BoneId, BoneDef, JointType, Skeleton};
pub use skinning::{SkinMesh, SkinVertex, SkinnedVertex, cpu_skin_lbs, compute_joint_matrices};
pub use vrm::{BoneBodyType, parse_vrmc_vrm, parse_vrm0};
