//! VRM拡張パース・スケルトン生成
//!
//! VRM 1.0 (VRMC_vrm) と VRM 0.x (VRM) のヒューマノイドボーンマッピングを読み取り、
//! glTFノード階層からSkeletonを自動生成する。

use crate::gltf_types::GltfNodeInfo;
use crate::skeleton::{BoneDef, BoneId, JointType, Skeleton};
use std::collections::HashMap;

/// VRM 1.0 (VRMC_vrm) ヒューマノイドボーン
#[derive(Debug, Clone)]
pub struct VrmHumanBone {
    pub node: usize,
}

/// VRM 1.0 ヒューマノイド定義
#[derive(Debug, Clone)]
pub struct VrmcHumanoid {
    pub human_bones: HashMap<String, VrmHumanBone>,
}

/// VRM 0.x ヒューマノイドボーン
#[derive(Debug, Clone)]
pub struct Vrm0HumanBone {
    pub bone: String,
    pub node: usize,
}

/// VRM 0.x ヒューマノイド定義
#[derive(Debug, Clone)]
pub struct Vrm0Humanoid {
    pub human_bones: Vec<Vrm0HumanBone>,
}

/// VRMc SpringBone（Phase 3用、定義のみ先行）
#[derive(Debug, Clone)]
pub struct VrmcSpringBone {
    pub springs: Vec<VrmcSpring>,
    pub colliders: Vec<VrmcCollider>,
}

#[derive(Debug, Clone)]
pub struct VrmcSpring {
    pub name: Option<String>,
    pub joints: Vec<VrmcSpringJoint>,
}

#[derive(Debug, Clone)]
pub struct VrmcSpringJoint {
    pub node: usize,
    pub hit_radius: f32,
    pub stiffness: f32,
    pub gravity_power: f32,
    pub drag_force: f32,
}

#[derive(Debug, Clone)]
pub struct VrmcCollider {
    pub node: usize,
    pub shape_type: String, // "sphere" or "capsule"
    pub radius: f32,
}

/// VRM拡張JSONからVRM 1.0 (VRMC_vrm) ヒューマノイドをパース
pub fn parse_vrmc_vrm(extensions: &serde_json::Value) -> Option<VrmcHumanoid> {
    let vrmc = extensions.get("VRMC_vrm")?;
    let humanoid = vrmc.get("humanoid")?;
    let human_bones = humanoid.get("humanBones")?.as_object()?;

    let mut bones = HashMap::new();
    for (name, info) in human_bones {
        if let Some(node) = info.get("node").and_then(|n| n.as_u64()) {
            bones.insert(name.clone(), VrmHumanBone { node: node as usize });
        }
    }

    if bones.is_empty() {
        return None;
    }

    Some(VrmcHumanoid { human_bones: bones })
}

/// VRM拡張JSONからVRM 0.x ヒューマノイドをパース
pub fn parse_vrm0(extensions: &serde_json::Value) -> Option<Vrm0Humanoid> {
    let vrm = extensions.get("VRM")?;
    let humanoid = vrm.get("humanoid")?;
    let bones_arr = humanoid.get("humanBones")?.as_array()?;

    let mut bones = Vec::new();
    for bone_val in bones_arr {
        let bone_name = bone_val.get("bone").and_then(|b| b.as_str())?;
        let node = bone_val.get("node").and_then(|n| n.as_u64())?;
        bones.push(Vrm0HumanBone {
            bone: bone_name.to_string(),
            node: node as usize,
        });
    }

    if bones.is_empty() {
        return None;
    }

    Some(Vrm0Humanoid { human_bones: bones })
}

/// VRMc SpringBoneをパース（Phase 3用）
pub fn parse_vrmc_spring_bone(extensions: &serde_json::Value) -> Option<VrmcSpringBone> {
    let sb = extensions.get("VRMC_springBone")?;

    let springs: Vec<VrmcSpring> = sb.get("springs")
        .and_then(|s| s.as_array())
        .map(|arr| {
            arr.iter().filter_map(|spring| {
                let joints = spring.get("joints")?.as_array()?
                    .iter()
                    .filter_map(|j| {
                        Some(VrmcSpringJoint {
                            node: j.get("node")?.as_u64()? as usize,
                            hit_radius: j.get("hitRadius").and_then(|v| v.as_f64()).unwrap_or(0.0) as f32,
                            stiffness: j.get("stiffness").and_then(|v| v.as_f64()).unwrap_or(1.0) as f32,
                            gravity_power: j.get("gravityPower").and_then(|v| v.as_f64()).unwrap_or(0.0) as f32,
                            drag_force: j.get("dragForce").and_then(|v| v.as_f64()).unwrap_or(0.5) as f32,
                        })
                    })
                    .collect();
                Some(VrmcSpring {
                    name: spring.get("name").and_then(|n| n.as_str()).map(String::from),
                    joints,
                })
            }).collect()
        })
        .unwrap_or_default();

    let colliders: Vec<VrmcCollider> = sb.get("colliders")
        .and_then(|c| c.as_array())
        .map(|arr| {
            arr.iter().filter_map(|c| {
                let node = c.get("node")?.as_u64()? as usize;
                let shape = c.get("shape")?;
                // sphere or capsule
                let (shape_type, radius) = if let Some(sphere) = shape.get("sphere") {
                    ("sphere".to_string(), sphere.get("radius").and_then(|r| r.as_f64()).unwrap_or(0.05) as f32)
                } else if let Some(capsule) = shape.get("capsule") {
                    ("capsule".to_string(), capsule.get("radius").and_then(|r| r.as_f64()).unwrap_or(0.05) as f32)
                } else {
                    return None;
                };
                Some(VrmcCollider { node, shape_type, radius })
            }).collect()
        })
        .unwrap_or_default();

    Some(VrmcSpringBone { springs, colliders })
}

/// VRMボーン名がキネマティック制御対象かどうかを判定
///
/// キネマティック: 体幹 + 手・足・つま先・Endボーン（位置制御、安定した土台）
/// ダイナミック: 腕・脚・指のみ（モーター制御、アニメーション対応）
pub fn is_kinematic_bone(vrm_bone_name: &str) -> bool {
    // 体幹: キネマティック（安定した土台、位置制御）
    if matches!(vrm_bone_name,
        // VRM名
        "hips" | "spine" | "chest" | "upperChest" |
        "neck" | "head" | "jaw" |
        "leftShoulder" | "rightShoulder" |
        // Mixamo名
        "Hips" | "Spine" | "Spine1" | "Spine2" |
        "Neck" | "Head" | "Jaw" |
        "LeftShoulder" | "RightShoulder" |
        // 手・足・つま先
        "leftHand" | "rightHand" |
        "leftFoot" | "rightFoot" |
        "leftToes" | "rightToes" |
        "LeftHand" | "RightHand" |
        "LeftFoot" | "RightFoot" |
        "LeftToes" | "RightToes" |
        "LeftToeBase" | "RightToeBase"
    ) {
        return true;
    }
    let n = vrm_bone_name;
    // End bone: キネマティック（末端リーフノード）
    if n.ends_with("_End") || n.ends_with("Top_End") || n.ends_with("Toe_End") {
        return true;
    }
    // 指ボーン: キネマティック（物理で暴れるのを防止）
    if n.contains("Thumb") || n.contains("Index") || n.contains("Middle")
        || n.contains("Ring") || n.contains("Little")
        || n.contains("thumb") || n.contains("index") || n.contains("middle")
        || n.contains("ring") || n.contains("little") {
        return true;
    }
    // ダイナミック: ヒューマノイド標準の腕・脚のみ
    if matches!(n,
        // VRM名
        "leftUpperArm" | "rightUpperArm" |
        "leftLowerArm" | "rightLowerArm" |
        "leftUpperLeg" | "rightUpperLeg" |
        "leftLowerLeg" | "rightLowerLeg" |
        // Mixamo名
        "LeftArm" | "RightArm" |
        "LeftForeArm" | "RightForeArm" |
        "LeftUpLeg" | "RightUpLeg" |
        "LeftLeg" | "RightLeg"
    ) {
        return false; // Dynamic
    }
    // それ以外（補助ボーン、髪、SpringBone対象等）は全てKinematic
    true
}

/// VRMボーン名からJointTypeを自動決定
pub fn vrm_bone_to_joint_type(vrm_bone_name: &str) -> JointType {
    match vrm_bone_name {
        // ヒンジジョイント（1軸）
        "leftLowerArm" | "rightLowerArm" | "LeftLowerArm" | "RightLowerArm" => {
            JointType::Hinge {
                axis: [0.0, 1.0, 0.0],
                min_angle: 0.0,
                max_angle: 145.0_f32.to_radians(),
            }
        }
        "leftLowerLeg" | "rightLowerLeg" | "LeftLowerLeg" | "RightLowerLeg" => {
            JointType::Hinge {
                axis: [0.0, 1.0, 0.0],
                min_angle: 0.0,
                max_angle: 140.0_f32.to_radians(),
            }
        }
        // 指のヒンジ（VRM名: Distal/Intermediate/Proximal, Mixamo名: RightHandIndex1等）
        n if n.contains("Distal") || n.contains("Intermediate") || n.contains("Proximal")
            || n.contains("Metacarpal")
            || (n.starts_with("RightHand") || n.starts_with("LeftHand"))
               && (n.contains("Thumb") || n.contains("Index") || n.contains("Middle")
                   || n.contains("Ring") || n.contains("Pinky")) => {
            JointType::Hinge {
                axis: [0.0, 1.0, 0.0],
                min_angle: -10.0_f32.to_radians(),
                max_angle: 90.0_f32.to_radians(),
            }
        }
        // 肩: ボールジョイント（広い可動域）
        "leftUpperArm" | "rightUpperArm" | "LeftUpperArm" | "RightUpperArm" => {
            JointType::Ball {
                swing_limit: 150.0_f32.to_radians(),
                twist_limit: 45.0_f32.to_radians(),
            }
        }
        // 股関節
        "leftUpperLeg" | "rightUpperLeg" | "LeftUpperLeg" | "RightUpperLeg" => {
            JointType::Ball {
                swing_limit: 100.0_f32.to_radians(),
                twist_limit: 30.0_f32.to_radians(),
            }
        }
        // 足首
        "leftFoot" | "rightFoot" | "LeftFoot" | "RightFoot" => {
            JointType::Ball {
                swing_limit: 30.0_f32.to_radians(),
                twist_limit: 10.0_f32.to_radians(),
            }
        }
        // つま先
        "leftToes" | "rightToes" | "LeftToes" | "RightToes" => {
            JointType::Hinge {
                axis: [1.0, 0.0, 0.0],
                min_angle: -30.0_f32.to_radians(),
                max_angle: 60.0_f32.to_radians(),
            }
        }
        // 手首
        "leftHand" | "rightHand" | "LeftHand" | "RightHand" => {
            JointType::Ball {
                swing_limit: 40.0_f32.to_radians(),
                twist_limit: 20.0_f32.to_radians(),
            }
        }
        // 肩帯
        "leftShoulder" | "rightShoulder" | "LeftShoulder" | "RightShoulder" => {
            JointType::Ball {
                swing_limit: 15.0_f32.to_radians(),
                twist_limit: 10.0_f32.to_radians(),
            }
        }
        // 体幹: 各椎骨で分散するため1椎骨あたりは控えめだが、
        // 3椎骨合計で前屈45°+ひねり30°程度を許容
        "spine" | "chest" | "upperChest" | "Spine" | "Chest" | "UpperChest" => {
            JointType::Ball {
                swing_limit: 25.0_f32.to_radians(),
                twist_limit: 15.0_f32.to_radians(),
            }
        }
        "neck" | "Neck" => {
            JointType::Ball {
                swing_limit: 40.0_f32.to_radians(),
                twist_limit: 30.0_f32.to_radians(),
            }
        }
        "head" | "Head" => {
            JointType::Ball {
                swing_limit: 35.0_f32.to_radians(),
                twist_limit: 25.0_f32.to_radians(),
            }
        }
        "jaw" | "Jaw" => {
            JointType::Hinge {
                axis: [0.0, 1.0, 0.0],
                min_angle: 0.0,
                max_angle: 40.0_f32.to_radians(),
            }
        }
        // ルート
        "hips" | "Hips" => JointType::Fixed,
        // デフォルト: ボールジョイント
        _ => JointType::Ball {
            swing_limit: 30.0_f32.to_radians(),
            twist_limit: 10.0_f32.to_radians(),
        },
    }
}

/// VRMボーン名+ボーン長から質量を自動推定 (kg)
pub fn vrm_bone_mass(vrm_bone_name: &str, bone_length_mm: f32) -> f32 {
    match vrm_bone_name {
        "hips" | "Hips" => 8.0,
        "spine" | "Spine" => 5.0,
        "chest" | "Chest" => 5.0,
        "upperChest" | "UpperChest" => 4.0,
        "neck" | "Neck" => 1.0,
        "head" | "Head" => 4.5,
        "jaw" | "Jaw" => 0.3,
        "leftShoulder" | "rightShoulder" | "LeftShoulder" | "RightShoulder" => 0.5,
        "leftUpperArm" | "rightUpperArm" | "LeftUpperArm" | "RightUpperArm" => 2.5,
        "leftLowerArm" | "rightLowerArm" | "LeftLowerArm" | "RightLowerArm" => 1.0,
        "leftHand" | "rightHand" | "LeftHand" | "RightHand" => 0.4,
        "leftUpperLeg" | "rightUpperLeg" | "LeftUpperLeg" | "RightUpperLeg" => 8.0,
        "leftLowerLeg" | "rightLowerLeg" | "LeftLowerLeg" | "RightLowerLeg" => 3.0,
        "leftFoot" | "rightFoot" | "LeftFoot" | "RightFoot" => 1.0,
        "leftToes" | "rightToes" | "LeftToes" | "RightToes" => 0.2,
        // 指: 長さに比例して小さい
        n if n.contains("Distal") => 0.005,
        n if n.contains("Intermediate") => 0.008,
        n if n.contains("Proximal") => 0.01,
        // 不明: 長さから推定
        _ => (bone_length_mm / 1000.0 * 2.0).max(0.01),
    }
}

/// VRMボーン名のポーズ剛性
fn vrm_bone_stiffness(vrm_bone_name: &str) -> (f32, f32) {
    // (stiffness, damping)
    match vrm_bone_name {
        "hips" | "Hips" => (0.0, 0.0),
        "spine" | "chest" | "upperChest" | "Spine" | "Chest" | "UpperChest" => (500.0, 50.0),
        "neck" | "Neck" => (1000.0, 80.0),
        "head" | "Head" => (500.0, 50.0),
        "leftShoulder" | "rightShoulder" | "LeftShoulder" | "RightShoulder" => (300.0, 40.0),
        "leftUpperArm" | "rightUpperArm" | "LeftUpperArm" | "RightUpperArm" => (300.0, 50.0),
        "leftLowerArm" | "rightLowerArm" | "LeftLowerArm" | "RightLowerArm" => (100.0, 20.0),
        "leftUpperLeg" | "rightUpperLeg" | "LeftUpperLeg" | "RightUpperLeg" => (600.0, 80.0),
        "leftLowerLeg" | "rightLowerLeg" | "LeftLowerLeg" | "RightLowerLeg" => (200.0, 30.0),
        "leftHand" | "rightHand" | "LeftHand" | "RightHand" => (400.0, 40.0),
        "leftFoot" | "rightFoot" | "LeftFoot" | "RightFoot" => (300.0, 30.0),
        "leftToes" | "rightToes" | "LeftToes" | "RightToes" => (200.0, 20.0),
        // 指: 高剛性でデロデロ防止
        n if n.contains("Thumb") || n.contains("Index") || n.contains("Middle")
            || n.contains("Ring") || n.contains("Little")
            || n.contains("thumb") || n.contains("index") || n.contains("middle")
            || n.contains("ring") || n.contains("little") => (500.0, 50.0),
        _ => (100.0, 20.0),
    }
}

/// VRMボーンの半径推定
fn vrm_bone_radius(vrm_bone_name: &str, bone_length_mm: f32) -> f32 {
    match vrm_bone_name {
        "hips" | "Hips" => 100.0,
        "head" | "Head" => 95.0,
        "leftUpperLeg" | "rightUpperLeg" | "LeftUpperLeg" | "RightUpperLeg" => 50.0,
        "leftUpperArm" | "rightUpperArm" | "LeftUpperArm" | "RightUpperArm" => 35.0,
        "leftLowerLeg" | "rightLowerLeg" | "LeftLowerLeg" | "RightLowerLeg" => 30.0,
        n if n.contains("Distal") || n.contains("Intermediate") || n.contains("Proximal") => 4.0,
        _ => (bone_length_mm * 0.15).max(5.0),
    }
}

/// VRMボーン名→anatomical name (UpperArm_L 系)にマッピング
///
/// VRM標準ボーン名はキャメルケース (leftUpperArm) だが、
/// anatomical naming (UpperArm_L 系)はPascalCase+アンダースコア (UpperArm_L)
pub fn vrm_to_anatomical_bone_name(vrm_name: &str) -> String {
    // そのままVRM名をanatomical bone nameとして使用（VRMモデル用のSkeletonでは独自命名）
    vrm_name.to_string()
}

/// ノード階層を走査してワールド座標系でのトランスレーションを取得
/// 親ノードのスケールを子の翻訳に累積適用する（Armatureのスケール対応）
/// クォータニオン [x, y, z, w] でベクトルを回転
fn quat_rotate(q: [f32; 4], v: [f32; 3]) -> [f32; 3] {
    let [qx, qy, qz, qw] = q;
    // t = 2 * cross(q.xyz, v)
    let tx = 2.0 * (qy * v[2] - qz * v[1]);
    let ty = 2.0 * (qz * v[0] - qx * v[2]);
    let tz = 2.0 * (qx * v[1] - qy * v[0]);
    // result = v + qw * t + cross(q.xyz, t)
    [
        v[0] + qw * tx + (qy * tz - qz * ty),
        v[1] + qw * ty + (qz * tx - qx * tz),
        v[2] + qw * tz + (qx * ty - qy * tx),
    ]
}

/// クォータニオン乗算 a * b
fn quat_mul(a: [f32; 4], b: [f32; 4]) -> [f32; 4] {
    [
        a[3] * b[0] + a[0] * b[3] + a[1] * b[2] - a[2] * b[1],
        a[3] * b[1] - a[0] * b[2] + a[1] * b[3] + a[2] * b[0],
        a[3] * b[2] + a[0] * b[1] - a[1] * b[0] + a[2] * b[3],
        a[3] * b[3] - a[0] * b[0] - a[1] * b[1] - a[2] * b[2],
    ]
}

fn compute_node_world_translations(
    nodes: &[GltfNodeInfo],
) -> Vec<[f32; 3]> {
    let mut world_translations = vec![[0.0f32; 3]; nodes.len()];
    // 累積回転（クォータニオン [x, y, z, w]）
    let mut world_rotations = vec![[0.0f32, 0.0, 0.0, 1.0]; nodes.len()];
    // 累積スケール
    let mut world_scales = vec![[1.0f32; 3]; nodes.len()];

    let mut processed = vec![false; nodes.len()];
    let mut queue: Vec<usize> = Vec::new();

    for (i, node) in nodes.iter().enumerate() {
        if node.parent.is_none() {
            // ルートノード: 自身のTRS適用
            let scaled_t = [
                node.translation[0] * node.scale[0],
                node.translation[1] * node.scale[1],
                node.translation[2] * node.scale[2],
            ];
            world_translations[i] = scaled_t;
            world_rotations[i] = node.rotation;
            world_scales[i] = node.scale;
            processed[i] = true;
            queue.push(i);
        }
    }

    // BFS
    while let Some(idx) = queue.pop() {
        let parent_wt = world_translations[idx];
        let parent_rot = world_rotations[idx];
        let parent_scale = world_scales[idx];
        for &child_idx in &nodes[idx].children {
            if child_idx < nodes.len() && !processed[child_idx] {
                let child = &nodes[child_idx];
                // child_world = parent_world + parent_rot * (parent_scale * child_translation)
                let scaled = [
                    parent_scale[0] * child.translation[0],
                    parent_scale[1] * child.translation[1],
                    parent_scale[2] * child.translation[2],
                ];
                let rotated = quat_rotate(parent_rot, scaled);
                world_translations[child_idx] = [
                    parent_wt[0] + rotated[0],
                    parent_wt[1] + rotated[1],
                    parent_wt[2] + rotated[2],
                ];
                // 回転累積: parent_rot * child_rot
                world_rotations[child_idx] = quat_mul(parent_rot, child.rotation);
                // スケール累積（回転を考慮すると厳密には異方性スケール+回転は非可換だが、
                // glTFでは均一スケールが一般的なので簡易積で十分）
                world_scales[child_idx] = [
                    parent_scale[0] * child.scale[0],
                    parent_scale[1] * child.scale[1],
                    parent_scale[2] * child.scale[2],
                ];
                processed[child_idx] = true;
                queue.push(child_idx);
            }
        }
    }

    world_translations
}

/// Y-up メートル → Z-up ミリメートル変換
fn yup_to_zup_mm(v: [f32; 3]) -> [f32; 3] {
    // (x, y, z) → (x, -z, y) * 1000
    [v[0] * 1000.0, -v[2] * 1000.0, v[1] * 1000.0]
}


/// ノードのtranslationからボーン長を算出 (mm)
fn compute_bone_length(parent_wt: [f32; 3], child_wt: [f32; 3]) -> f32 {
    // ボーン長は軸変換に依存しない（距離はどの座標系でも同じ）
    // スケールだけ考慮すればよい
    let parent = yup_to_zup_mm(parent_wt);
    let child = yup_to_zup_mm(child_wt);
    let dx = child[0] - parent[0];
    let dy = child[1] - parent[1];
    let dz = child[2] - parent[2];
    (dx * dx + dy * dy + dz * dz).sqrt()
}


impl Skeleton {
    /// VRMヒューマノイド+glTFノード階層からSkeletonを自動生成
    ///
    /// VRM 1.0ボーン名をキーとして使い、ノード階層の親子関係からBoneDefを構築する。
    /// 座標系はY-up→Z-up、メートル→ミリメートルに変換。
    pub fn from_vrm(
        humanoid: &VrmcHumanoid,
        nodes: &[GltfNodeInfo],
    ) -> Self {
        let world_translations = compute_node_world_translations(nodes);

        // VRMボーン名→ノードインデックスのマップ
        // + ノードインデックス→VRMボーン名の逆マップ
        let mut node_to_vrm: HashMap<usize, String> = HashMap::new();
        for (name, bone) in &humanoid.human_bones {
            node_to_vrm.insert(bone.node, name.clone());
        }

        // VRMの定義順でボーンを生成（hips→spine→chest→...→末端）
        // 先にhips（ルート）、次に体幹、腕、脚の順
        let bone_order: Vec<&str> = vec![
            "hips",
            "spine", "chest", "upperChest",
            "neck", "head", "jaw",
            "leftShoulder", "leftUpperArm", "leftLowerArm", "leftHand",
            "rightShoulder", "rightUpperArm", "rightLowerArm", "rightHand",
            "leftUpperLeg", "leftLowerLeg", "leftFoot", "leftToes",
            "rightUpperLeg", "rightLowerLeg", "rightFoot", "rightToes",
            // 指 (左)
            "leftThumbMetacarpal", "leftThumbProximal", "leftThumbDistal",
            "leftIndexProximal", "leftIndexIntermediate", "leftIndexDistal",
            "leftMiddleProximal", "leftMiddleIntermediate", "leftMiddleDistal",
            "leftRingProximal", "leftRingIntermediate", "leftRingDistal",
            "leftLittleProximal", "leftLittleIntermediate", "leftLittleDistal",
            // 指 (右)
            "rightThumbMetacarpal", "rightThumbProximal", "rightThumbDistal",
            "rightIndexProximal", "rightIndexIntermediate", "rightIndexDistal",
            "rightMiddleProximal", "rightMiddleIntermediate", "rightMiddleDistal",
            "rightRingProximal", "rightRingIntermediate", "rightRingDistal",
            "rightLittleProximal", "rightLittleIntermediate", "rightLittleDistal",
        ];

        let mut bones: Vec<BoneDef> = Vec::new();
        let mut vrm_name_to_bone_idx: HashMap<String, usize> = HashMap::new();

        for vrm_name in &bone_order {
            let Some(vrm_bone) = humanoid.human_bones.get(*vrm_name) else {
                continue;
            };

            let node_idx = vrm_bone.node;
            if node_idx >= nodes.len() {
                continue;
            }

            let _node = &nodes[node_idx];
            let bone_idx = bones.len();

            // 親ボーンを探す: glTFノードの親をたどってVRMボーンにヒットするまで上る
            let parent_bone_idx = find_vrm_parent(node_idx, nodes, &node_to_vrm, &vrm_name_to_bone_idx);

            // ボーン長の計算: 最も近いVRM子ボーンまでの距離
            let bone_length = compute_vrm_bone_length(
                *vrm_name, node_idx, &world_translations, &humanoid.human_bones, nodes,
            );

            // オフセット: 親からの相対位置
            let offset = if let Some(parent_idx) = parent_bone_idx {
                let parent_vrm_name = bones[parent_idx].name.clone();
                let parent_node_idx = humanoid.human_bones.get(&parent_vrm_name)
                    .map(|b| b.node)
                    .unwrap_or(0);
                let parent_wt = world_translations[parent_node_idx];
                let child_wt = world_translations[node_idx];
                let parent_mm = yup_to_zup_mm(parent_wt);
                let child_mm = yup_to_zup_mm(child_wt);
                [
                    child_mm[0] - parent_mm[0],
                    child_mm[1] - parent_mm[1],
                    child_mm[2] - parent_mm[2],
                ]
            } else {
                // ルートボーン: ワールド位置
                let wt = world_translations[node_idx];
                let mm = yup_to_zup_mm(wt);
                [mm[0], mm[1], mm[2]]
            };

            let joint_type = vrm_bone_to_joint_type(vrm_name);
            let mass = vrm_bone_mass(vrm_name, bone_length);
            let radius = vrm_bone_radius(vrm_name, bone_length);
            let (pose_stiffness, pose_damping) = vrm_bone_stiffness(vrm_name);

            bones.push(BoneDef {
                name: vrm_name.to_string(),
                parent: parent_bone_idx.map(BoneId),
                offset,
                length: bone_length.max(1.0), // 最小1mm
                radius,
                mass,
                joint_type,
                pose_stiffness,
                pose_damping,
                use_direct_offset: false,
            });

            vrm_name_to_bone_idx.insert(vrm_name.to_string(), bone_idx);
        }

        tracing::info!("VRM Skeleton generated: {} bones", bones.len());
        for (i, bone) in bones.iter().enumerate() {
            tracing::debug!("  [{}] {} (parent={:?}, length={:.1}mm, mass={:.2}kg)",
                i, bone.name, bone.parent, bone.length, bone.mass);
        }

        Self { bones }
    }

    /// VRM 0.x形式のヒューマノイドからSkeletonを生成
    pub fn from_vrm0(
        humanoid: &Vrm0Humanoid,
        nodes: &[GltfNodeInfo],
    ) -> Self {
        // VRM 0.xのボーン名をVRM 1.0形式に変換してfrom_vrmに委譲
        let mut human_bones = HashMap::new();
        for bone in &humanoid.human_bones {
            let vrm1_name = vrm0_to_vrm1_bone_name(&bone.bone);
            human_bones.insert(vrm1_name, VrmHumanBone { node: bone.node });
        }
        let vrm1_humanoid = VrmcHumanoid { human_bones };
        Self::from_vrm(&vrm1_humanoid, nodes)
    }

    /// glTFスキンジョイント順序でVRMスケルトンを構築
    ///
    /// メッシュの頂点joint indexがそのままボーンindexになるように、
    /// glTFスキンのジョイントリスト順にボーンを配列する。
    /// VRMヒューマノイドに含まれないジョイント（髪ボーン等）もダミーとして追加。
    pub fn from_vrm_skin_order(
        humanoid: &VrmcHumanoid,
        nodes: &[GltfNodeInfo],
        skin_joint_names: &[String],
    ) -> Self {
        let world_translations = compute_node_world_translations(nodes);

        // VRMボーン名→ノードインデックス
        let mut node_to_vrm: HashMap<usize, String> = HashMap::new();
        for (name, bone) in &humanoid.human_bones {
            node_to_vrm.insert(bone.node, name.clone());
        }

        // ノード名→ノードインデックスのマップ
        let name_to_node: HashMap<&str, usize> = nodes.iter()
            .map(|n| (n.name.as_str(), n.index))
            .collect();

        let mut bones: Vec<BoneDef> = Vec::new();
        let mut node_idx_to_bone_idx: HashMap<usize, usize> = HashMap::new();

        // glTFスキンジョイント順にボーンを作成
        for (skin_idx, joint_name) in skin_joint_names.iter().enumerate() {
            let node_idx = name_to_node.get(joint_name.as_str()).copied()
                .unwrap_or(skin_idx); // フォールバック

            let bone_idx = bones.len();

            // VRMヒューマノイドボーンかどうか
            let vrm_name = node_to_vrm.get(&node_idx).cloned()
                .unwrap_or_else(|| joint_name.clone());

            // 親ボーン: glTFノードの親をたどってスキンジョイントにヒットするまで上る
            let parent_bone_idx = {
                let mut current = node_idx;
                let mut found = None;
                while let Some(p) = nodes.get(current).and_then(|n| n.parent) {
                    if let Some(&bidx) = node_idx_to_bone_idx.get(&p) {
                        found = Some(bidx);
                        break;
                    }
                    current = p;
                }
                found
            };

            // ボーン長
            let bone_length = if humanoid.human_bones.contains_key(&vrm_name) {
                compute_vrm_bone_length(
                    &vrm_name, node_idx, &world_translations, &humanoid.human_bones, nodes,
                )
            } else {
                // 非VRMボーン（髪等）: 子ノードまでの距離 or デフォルト
                15.0
            };

            // オフセット
            let offset = if let Some(parent_idx) = parent_bone_idx {
                // 親のノードインデックスを探す
                let parent_node = nodes.iter()
                    .find(|n| node_idx_to_bone_idx.get(&n.index) == Some(&parent_idx))
                    .map(|n| n.index)
                    .unwrap_or(0);
                let parent_wt = world_translations[parent_node];
                let child_wt = world_translations[node_idx];
                let parent_mm = yup_to_zup_mm(parent_wt);
                let child_mm = yup_to_zup_mm(child_wt);
                [
                    child_mm[0] - parent_mm[0],
                    child_mm[1] - parent_mm[1],
                    child_mm[2] - parent_mm[2],
                ]
            } else {
                let wt = world_translations.get(node_idx)
                    .copied()
                    .unwrap_or([0.0; 3]);
                let mm = yup_to_zup_mm(wt);
                [mm[0], mm[1], mm[2]]
            };

            let joint_type = vrm_bone_to_joint_type(&vrm_name);
            let mass = vrm_bone_mass(&vrm_name, bone_length);
            let radius = vrm_bone_radius(&vrm_name, bone_length);
            let (pose_stiffness, pose_damping) = vrm_bone_stiffness(&vrm_name);

            bones.push(BoneDef {
                name: vrm_name,
                parent: parent_bone_idx.map(BoneId),
                offset,
                length: bone_length.max(1.0),
                radius,
                mass,
                joint_type,
                pose_stiffness,
                pose_damping,
                use_direct_offset: false,
            });

            node_idx_to_bone_idx.insert(node_idx, bone_idx);
        }

        tracing::info!("VRM Skeleton (skin order): {} bones", bones.len());
        for (i, bone) in bones.iter().enumerate() {
            tracing::debug!("  [{}] {} (parent={:?}, length={:.1}mm, mass={:.2}kg)",
                i, bone.name, bone.parent, bone.length, bone.mass);
        }

        Self { bones }
    }

    /// 汎用glTFスキンジョイントからSkeleton生成（VRM拡張なし）
    /// Mixamo/ReadyPlayerMe等のglTFモデル用。ジョイント名からボーンパラメータを推定。
    pub fn from_gltf_skin_joints(
        nodes: &[GltfNodeInfo],
        skin_joint_names: &[String],
    ) -> Self {
        let world_translations = compute_node_world_translations(nodes);

        // Hips位置をログ出力（デバッグ用）
        if let Some(wt) = world_translations.first() {
            let mm = yup_to_zup_mm(*wt);
            tracing::info!("glTFスケルトン: Hips world=({:.4},{:.4},{:.4}) → Z-up mm=({:.1},{:.1},{:.1})",
                wt[0], wt[1], wt[2], mm[0], mm[1], mm[2]);
        }

        // gltf_loaderは常にY-up→Z-up変換 (x,-z,y)*1000 を適用するので
        // スケルトンも同じ変換を使う（軸検出は不要、頂点と座標系を一致させる）

        // ノード名→ノードインデックスのマップ
        let name_to_node: HashMap<&str, usize> = nodes.iter()
            .map(|n| (n.name.as_str(), n.index))
            .collect();

        let mut bones: Vec<BoneDef> = Vec::new();
        let mut node_idx_to_bone_idx: HashMap<usize, usize> = HashMap::new();

        for (skin_idx, joint_name) in skin_joint_names.iter().enumerate() {
            let node_idx = name_to_node.get(joint_name.as_str()).copied()
                .unwrap_or(skin_idx);

            let bone_idx = bones.len();

            // 親ボーン: glTFノードの親をたどってスキンジョイントにヒットするまで上る
            let parent_bone_idx = {
                let mut current = node_idx;
                let mut found = None;
                while let Some(p) = nodes.get(current).and_then(|n| n.parent) {
                    if let Some(&bidx) = node_idx_to_bone_idx.get(&p) {
                        found = Some(bidx);
                        break;
                    }
                    current = p;
                }
                found
            };

            // ボーン長: 子ジョイントまでの距離 or デフォルト
            let bone_length = {
                let children_in_skin: Vec<usize> = nodes.get(node_idx)
                    .map(|n| n.children.iter()
                        .filter(|&&c| node_idx_to_bone_idx.contains_key(&c) || skin_joint_names.iter().any(|s| {
                            nodes.get(c).map(|cn| cn.name == *s).unwrap_or(false)
                        }))
                        .copied()
                        .collect())
                    .unwrap_or_default();
                if let Some(&child) = children_in_skin.first() {
                    compute_bone_length(world_translations[node_idx], world_translations[child])
                } else {
                    20.0 // デフォルト
                }
            };

            // オフセット
            let offset = if let Some(parent_idx) = parent_bone_idx {
                let parent_node = nodes.iter()
                    .find(|n| node_idx_to_bone_idx.get(&n.index) == Some(&parent_idx))
                    .map(|n| n.index)
                    .unwrap_or(0);
                let parent_mm = yup_to_zup_mm(world_translations[parent_node]);
                let child_mm = yup_to_zup_mm(world_translations[node_idx]);
                [
                    child_mm[0] - parent_mm[0],
                    child_mm[1] - parent_mm[1],
                    child_mm[2] - parent_mm[2],
                ]
            } else {
                let wt = world_translations.get(node_idx)
                    .copied()
                    .unwrap_or([0.0; 3]);
                let mm = yup_to_zup_mm(wt);
                [mm[0], mm[1], mm[2]]
            };

            // ジョイント名からパラメータ推定（Mixamo互換名にも対応）
            let normalized_name = gltf_joint_to_vrm_name(joint_name);
            let joint_type = vrm_bone_to_joint_type(&normalized_name);
            let mass = vrm_bone_mass(&normalized_name, bone_length);
            let radius = vrm_bone_radius(&normalized_name, bone_length);
            let (pose_stiffness, pose_damping) = vrm_bone_stiffness(&normalized_name);

            bones.push(BoneDef {
                name: joint_name.clone(),
                parent: parent_bone_idx.map(BoneId),
                offset,
                length: bone_length.max(1.0),
                radius,
                mass,
                joint_type,
                pose_stiffness,
                pose_damping,
                use_direct_offset: true,
            });

            node_idx_to_bone_idx.insert(node_idx, bone_idx);
        }

        tracing::info!("glTF Skeleton (skin order): {} bones", bones.len());
        for (i, bone) in bones.iter().enumerate() {
            tracing::debug!("  [{}] {} (parent={:?}, length={:.1}mm, mass={:.2}kg)",
                i, bone.name, bone.parent, bone.length, bone.mass);
        }

        Self { bones }
    }

    /// VRM 0.x + スキン順序
    pub fn from_vrm0_skin_order(
        humanoid: &Vrm0Humanoid,
        nodes: &[GltfNodeInfo],
        skin_joint_names: &[String],
    ) -> Self {
        let mut human_bones = HashMap::new();
        for bone in &humanoid.human_bones {
            let vrm1_name = vrm0_to_vrm1_bone_name(&bone.bone);
            human_bones.insert(vrm1_name, VrmHumanBone { node: bone.node });
        }
        let vrm1_humanoid = VrmcHumanoid { human_bones };
        Self::from_vrm_skin_order(&vrm1_humanoid, nodes, skin_joint_names)
    }
}

/// VRM 0.xボーン名 → VRM 1.0ボーン名に変換
fn vrm0_to_vrm1_bone_name(vrm0_name: &str) -> String {
    match vrm0_name {
        // そのまま（大部分は一致）
        "hips" => "hips",
        "spine" => "spine",
        "chest" => "chest",
        "upperChest" => "upperChest",
        "neck" => "neck",
        "head" => "head",
        "jaw" => "jaw",
        "leftShoulder" => "leftShoulder",
        "rightShoulder" => "rightShoulder",
        "leftUpperArm" => "leftUpperArm",
        "rightUpperArm" => "rightUpperArm",
        "leftLowerArm" => "leftLowerArm",
        "rightLowerArm" => "rightLowerArm",
        "leftHand" => "leftHand",
        "rightHand" => "rightHand",
        "leftUpperLeg" => "leftUpperLeg",
        "rightUpperLeg" => "rightUpperLeg",
        "leftLowerLeg" => "leftLowerLeg",
        "rightLowerLeg" => "rightLowerLeg",
        "leftFoot" => "leftFoot",
        "rightFoot" => "rightFoot",
        "leftToes" => "leftToes",
        "rightToes" => "rightToes",
        // 指: VRM 0.x PascalCase → VRM 1.0 camelCase
        "leftThumbProximal" => "leftThumbMetacarpal",
        "leftThumbIntermediate" => "leftThumbProximal",
        "leftThumbDistal" => "leftThumbDistal",
        "leftIndexProximal" => "leftIndexProximal",
        "leftIndexIntermediate" => "leftIndexIntermediate",
        "leftIndexDistal" => "leftIndexDistal",
        "leftMiddleProximal" => "leftMiddleProximal",
        "leftMiddleIntermediate" => "leftMiddleIntermediate",
        "leftMiddleDistal" => "leftMiddleDistal",
        "leftRingProximal" => "leftRingProximal",
        "leftRingIntermediate" => "leftRingIntermediate",
        "leftRingDistal" => "leftRingDistal",
        "leftLittleProximal" => "leftLittleProximal",
        "leftLittleIntermediate" => "leftLittleIntermediate",
        "leftLittleDistal" => "leftLittleDistal",
        "rightThumbProximal" => "rightThumbMetacarpal",
        "rightThumbIntermediate" => "rightThumbProximal",
        "rightThumbDistal" => "rightThumbDistal",
        "rightIndexProximal" => "rightIndexProximal",
        "rightIndexIntermediate" => "rightIndexIntermediate",
        "rightIndexDistal" => "rightIndexDistal",
        "rightMiddleProximal" => "rightMiddleProximal",
        "rightMiddleIntermediate" => "rightMiddleIntermediate",
        "rightMiddleDistal" => "rightMiddleDistal",
        "rightRingProximal" => "rightRingProximal",
        "rightRingIntermediate" => "rightRingIntermediate",
        "rightRingDistal" => "rightRingDistal",
        "rightLittleProximal" => "rightLittleProximal",
        "rightLittleIntermediate" => "rightLittleIntermediate",
        "rightLittleDistal" => "rightLittleDistal",
        // フォールバック: そのまま
        other => other,
    }.to_string()
}

/// Mixamo/RPM等のglTFジョイント名 → VRM相当のボーン名に正規化
/// vrm_bone_to_joint_type等のパラメータ推定に使う
fn gltf_joint_to_vrm_name(joint_name: &str) -> String {
    // Mixamo: "Hips", "Spine", "Spine1", "Spine2", "Neck", "Head"
    // RPM: "Hips", "Spine", "Spine1", "Spine2", "Neck", "Head"
    // 大文字小文字を正規化してVRM名に変換
    match joint_name {
        "Hips" | "mixamorig:Hips" => "hips",
        "Spine" | "mixamorig:Spine" => "spine",
        "Spine1" | "mixamorig:Spine1" => "chest",
        "Spine2" | "mixamorig:Spine2" => "upperChest",
        "Neck" | "mixamorig:Neck" => "neck",
        "Head" | "mixamorig:Head" => "head",
        "LeftShoulder" | "mixamorig:LeftShoulder" => "leftShoulder",
        "RightShoulder" | "mixamorig:RightShoulder" => "rightShoulder",
        "LeftArm" | "mixamorig:LeftArm" => "leftUpperArm",
        "RightArm" | "mixamorig:RightArm" => "rightUpperArm",
        "LeftForeArm" | "mixamorig:LeftForeArm" => "leftLowerArm",
        "RightForeArm" | "mixamorig:RightForeArm" => "rightLowerArm",
        "LeftHand" | "mixamorig:LeftHand" => "leftHand",
        "RightHand" | "mixamorig:RightHand" => "rightHand",
        "LeftUpLeg" | "mixamorig:LeftUpLeg" => "leftUpperLeg",
        "RightUpLeg" | "mixamorig:RightUpLeg" => "rightUpperLeg",
        "LeftLeg" | "mixamorig:LeftLeg" => "leftLowerLeg",
        "RightLeg" | "mixamorig:RightLeg" => "rightLowerLeg",
        "LeftFoot" | "mixamorig:LeftFoot" => "leftFoot",
        "RightFoot" | "mixamorig:RightFoot" => "rightFoot",
        "LeftToeBase" | "mixamorig:LeftToeBase" => "leftToes",
        "RightToeBase" | "mixamorig:RightToeBase" => "rightToes",
        // 指（Mixamo/RPM形式）
        "LeftHandThumb1" | "mixamorig:LeftHandThumb1" => "leftThumbMetacarpal",
        "LeftHandThumb2" | "mixamorig:LeftHandThumb2" => "leftThumbProximal",
        "LeftHandThumb3" | "mixamorig:LeftHandThumb3" => "leftThumbDistal",
        "LeftHandIndex1" | "mixamorig:LeftHandIndex1" => "leftIndexProximal",
        "LeftHandIndex2" | "mixamorig:LeftHandIndex2" => "leftIndexIntermediate",
        "LeftHandIndex3" | "mixamorig:LeftHandIndex3" => "leftIndexDistal",
        "LeftHandMiddle1" | "mixamorig:LeftHandMiddle1" => "leftMiddleProximal",
        "LeftHandMiddle2" | "mixamorig:LeftHandMiddle2" => "leftMiddleIntermediate",
        "LeftHandMiddle3" | "mixamorig:LeftHandMiddle3" => "leftMiddleDistal",
        "LeftHandRing1" | "mixamorig:LeftHandRing1" => "leftRingProximal",
        "LeftHandRing2" | "mixamorig:LeftHandRing2" => "leftRingIntermediate",
        "LeftHandRing3" | "mixamorig:LeftHandRing3" => "leftRingDistal",
        "LeftHandPinky1" | "mixamorig:LeftHandPinky1" => "leftLittleProximal",
        "LeftHandPinky2" | "mixamorig:LeftHandPinky2" => "leftLittleIntermediate",
        "LeftHandPinky3" | "mixamorig:LeftHandPinky3" => "leftLittleDistal",
        "RightHandThumb1" | "mixamorig:RightHandThumb1" => "rightThumbMetacarpal",
        "RightHandThumb2" | "mixamorig:RightHandThumb2" => "rightThumbProximal",
        "RightHandThumb3" | "mixamorig:RightHandThumb3" => "rightThumbDistal",
        "RightHandIndex1" | "mixamorig:RightHandIndex1" => "rightIndexProximal",
        "RightHandIndex2" | "mixamorig:RightHandIndex2" => "rightIndexIntermediate",
        "RightHandIndex3" | "mixamorig:RightHandIndex3" => "rightIndexDistal",
        "RightHandMiddle1" | "mixamorig:RightHandMiddle1" => "rightMiddleProximal",
        "RightHandMiddle2" | "mixamorig:RightHandMiddle2" => "rightMiddleIntermediate",
        "RightHandMiddle3" | "mixamorig:RightHandMiddle3" => "rightMiddleDistal",
        "RightHandRing1" | "mixamorig:RightHandRing1" => "rightRingProximal",
        "RightHandRing2" | "mixamorig:RightHandRing2" => "rightRingIntermediate",
        "RightHandRing3" | "mixamorig:RightHandRing3" => "rightRingDistal",
        "RightHandPinky1" | "mixamorig:RightHandPinky1" => "rightLittleProximal",
        "RightHandPinky2" | "mixamorig:RightHandPinky2" => "rightLittleIntermediate",
        "RightHandPinky3" | "mixamorig:RightHandPinky3" => "rightLittleDistal",
        // Armature/その他 → デフォルト
        other => other,
    }.to_string()
}

/// glTFノード階層をたどってVRMボーンの親を見つける
fn find_vrm_parent(
    node_idx: usize,
    nodes: &[GltfNodeInfo],
    node_to_vrm: &HashMap<usize, String>,
    vrm_name_to_bone_idx: &HashMap<String, usize>,
) -> Option<usize> {
    let mut current = node_idx;
    loop {
        let parent = nodes[current].parent?;
        if let Some(vrm_name) = node_to_vrm.get(&parent) {
            if let Some(&bone_idx) = vrm_name_to_bone_idx.get(vrm_name) {
                return Some(bone_idx);
            }
        }
        current = parent;
    }
}

/// VRMボーンの長さを計算（最も近いVRM子ボーンまでの距離）
fn compute_vrm_bone_length(
    vrm_name: &str,
    node_idx: usize,
    world_translations: &[[f32; 3]],
    human_bones: &HashMap<String, VrmHumanBone>,
    nodes: &[GltfNodeInfo],
) -> f32 {
    // 自身のノードの子孫の中からVRMボーンであるノードを探す
    let mut min_distance = f32::MAX;
    let my_wt = world_translations[node_idx];

    // 全VRMボーンの中から、ノード階層上で直接の子孫であるものを探す
    for (_, child_bone) in human_bones {
        if child_bone.node == node_idx || child_bone.node >= nodes.len() {
            continue;
        }
        // child_bone.nodeの先祖にnode_idxがいるか
        let mut ancestor = nodes[child_bone.node].parent;
        let mut is_descendant = false;
        while let Some(a) = ancestor {
            if a == node_idx {
                is_descendant = true;
                break;
            }
            ancestor = nodes[a].parent;
        }
        if is_descendant {
            let dist = compute_bone_length(my_wt, world_translations[child_bone.node]);
            min_distance = min_distance.min(dist);
        }
    }

    if min_distance < f32::MAX {
        min_distance
    } else {
        // 子ボーンがない末端: デフォルト長
        match vrm_name {
            "head" | "Head" => 200.0,
            n if n.contains("Distal") => 15.0,
            n if n.contains("Toes") || n.contains("toes") => 30.0,
            _ => 50.0,
        }
    }
}

/// ボディタイプ分類結果
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BoneBodyType {
    /// キネマティック（位置直指定）
    Kinematic,
    /// ダイナミック（物理駆動）
    Dynamic,
    /// 固定（ルート）
    Fixed,
}

/// VRMスケルトンのボーンをKinematic/Dynamic/Fixedに分類
pub fn classify_body_types(skeleton: &Skeleton) -> Vec<BoneBodyType> {
    skeleton.bones.iter().enumerate().map(|(i, bone)| {
        if i == 0 || bone.parent.is_none() {
            BoneBodyType::Fixed
        } else if is_kinematic_bone(&bone.name) {
            BoneBodyType::Kinematic
        } else {
            BoneBodyType::Dynamic
        }
    }).collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    fn sample_nodes() -> Vec<GltfNodeInfo> {
        // 簡易的なVRMノード階層（hips → spine → chest → head）
        vec![
            GltfNodeInfo {
                name: "Root".into(),
                index: 0,
                parent: None,
                children: vec![1],
                translation: [0.0, 0.0, 0.0],
                rotation: [0.0, 0.0, 0.0, 1.0],
                scale: [1.0, 1.0, 1.0],
            },
            GltfNodeInfo {
                name: "Hips".into(),
                index: 1,
                parent: Some(0),
                children: vec![2, 5, 7],
                translation: [0.0, 1.0, 0.0], // Y-up, 1m
                rotation: [0.0, 0.0, 0.0, 1.0],
                scale: [1.0, 1.0, 1.0],
            },
            GltfNodeInfo {
                name: "Spine".into(),
                index: 2,
                parent: Some(1),
                children: vec![3],
                translation: [0.0, 0.1, 0.0],
                rotation: [0.0, 0.0, 0.0, 1.0],
                scale: [1.0, 1.0, 1.0],
            },
            GltfNodeInfo {
                name: "Chest".into(),
                index: 3,
                parent: Some(2),
                children: vec![4],
                translation: [0.0, 0.15, 0.0],
                rotation: [0.0, 0.0, 0.0, 1.0],
                scale: [1.0, 1.0, 1.0],
            },
            GltfNodeInfo {
                name: "Head".into(),
                index: 4,
                parent: Some(3),
                children: vec![],
                translation: [0.0, 0.3, 0.0],
                rotation: [0.0, 0.0, 0.0, 1.0],
                scale: [1.0, 1.0, 1.0],
            },
            GltfNodeInfo {
                name: "LeftUpperLeg".into(),
                index: 5,
                parent: Some(1),
                children: vec![6],
                translation: [0.1, -0.05, 0.0],
                rotation: [0.0, 0.0, 0.0, 1.0],
                scale: [1.0, 1.0, 1.0],
            },
            GltfNodeInfo {
                name: "LeftLowerLeg".into(),
                index: 6,
                parent: Some(5),
                children: vec![],
                translation: [0.0, -0.4, 0.0],
                rotation: [0.0, 0.0, 0.0, 1.0],
                scale: [1.0, 1.0, 1.0],
            },
            GltfNodeInfo {
                name: "RightUpperLeg".into(),
                index: 7,
                parent: Some(1),
                children: vec![8],
                translation: [-0.1, -0.05, 0.0],
                rotation: [0.0, 0.0, 0.0, 1.0],
                scale: [1.0, 1.0, 1.0],
            },
            GltfNodeInfo {
                name: "RightLowerLeg".into(),
                index: 8,
                parent: Some(7),
                children: vec![],
                translation: [0.0, -0.4, 0.0],
                rotation: [0.0, 0.0, 0.0, 1.0],
                scale: [1.0, 1.0, 1.0],
            },
        ]
    }

    fn sample_humanoid() -> VrmcHumanoid {
        let mut bones = HashMap::new();
        bones.insert("hips".into(), VrmHumanBone { node: 1 });
        bones.insert("spine".into(), VrmHumanBone { node: 2 });
        bones.insert("chest".into(), VrmHumanBone { node: 3 });
        bones.insert("head".into(), VrmHumanBone { node: 4 });
        bones.insert("leftUpperLeg".into(), VrmHumanBone { node: 5 });
        bones.insert("leftLowerLeg".into(), VrmHumanBone { node: 6 });
        bones.insert("rightUpperLeg".into(), VrmHumanBone { node: 7 });
        bones.insert("rightLowerLeg".into(), VrmHumanBone { node: 8 });
        VrmcHumanoid { human_bones: bones }
    }

    #[test]
    fn test_parse_vrmc_vrm() {
        let json: serde_json::Value = serde_json::json!({
            "VRMC_vrm": {
                "humanoid": {
                    "humanBones": {
                        "hips": { "node": 1 },
                        "spine": { "node": 2 },
                        "head": { "node": 4 }
                    }
                }
            }
        });
        let result = parse_vrmc_vrm(&json).unwrap();
        assert_eq!(result.human_bones.len(), 3);
        assert_eq!(result.human_bones["hips"].node, 1);
    }

    #[test]
    fn test_parse_vrm0() {
        let json: serde_json::Value = serde_json::json!({
            "VRM": {
                "humanoid": {
                    "humanBones": [
                        { "bone": "hips", "node": 1 },
                        { "bone": "spine", "node": 2 },
                        { "bone": "head", "node": 4 }
                    ]
                }
            }
        });
        let result = parse_vrm0(&json).unwrap();
        assert_eq!(result.human_bones.len(), 3);
        assert_eq!(result.human_bones[0].bone, "hips");
    }

    #[test]
    fn test_skeleton_from_vrm() {
        let humanoid = sample_humanoid();
        let nodes = sample_nodes();
        let skeleton = Skeleton::from_vrm(&humanoid, &nodes);

        assert!(skeleton.bone_count() >= 5, "Expected at least 5 bones, got {}", skeleton.bone_count());

        // hips should be root
        let hips = skeleton.find_bone("hips");
        assert!(hips.is_some(), "hips not found");
        assert!(skeleton.bones[hips.unwrap().0].parent.is_none(), "hips should be root");

        // spine should have hips as parent
        if let Some(spine_id) = skeleton.find_bone("spine") {
            let parent = skeleton.bones[spine_id.0].parent;
            assert!(parent.is_some(), "spine should have parent");
            assert_eq!(skeleton.bones[parent.unwrap().0].name, "hips");
        }

        // bone lengths should be positive
        for bone in &skeleton.bones {
            assert!(bone.length > 0.0, "Bone {} has zero/negative length", bone.name);
        }
    }

    #[test]
    fn test_classify_body_types() {
        let humanoid = sample_humanoid();
        let nodes = sample_nodes();
        let skeleton = Skeleton::from_vrm(&humanoid, &nodes);
        let body_types = classify_body_types(&skeleton);

        assert_eq!(body_types.len(), skeleton.bone_count());

        // hips = Fixed
        let hips_idx = skeleton.find_bone("hips").unwrap().0;
        assert_eq!(body_types[hips_idx], BoneBodyType::Fixed);

        // spine = Kinematic（体幹は安定した土台）
        if let Some(spine_id) = skeleton.find_bone("spine") {
            assert_eq!(body_types[spine_id.0], BoneBodyType::Kinematic);
        }

        // leftUpperLeg = Dynamic
        if let Some(leg_id) = skeleton.find_bone("leftUpperLeg") {
            assert_eq!(body_types[leg_id.0], BoneBodyType::Dynamic);
        }
    }

    #[test]
    fn test_is_kinematic_bone() {
        // 体幹はキネマティック（安定した土台）
        assert!(is_kinematic_bone("hips"));
        assert!(is_kinematic_bone("spine"));
        assert!(is_kinematic_bone("chest"));
        assert!(is_kinematic_bone("upperChest"));
        assert!(is_kinematic_bone("neck"));
        assert!(is_kinematic_bone("head"));
        assert!(is_kinematic_bone("leftShoulder"));
        assert!(is_kinematic_bone("Spine"));  // Mixamo名
        assert!(is_kinematic_bone("Head"));

        // 手・足・つま先もキネマティック
        assert!(is_kinematic_bone("leftHand"));
        assert!(is_kinematic_bone("LeftFoot"));
        assert!(is_kinematic_bone("LeftToeBase"));

        // 腕・脚はダイナミック
        assert!(!is_kinematic_bone("leftUpperArm"));
        assert!(!is_kinematic_bone("leftLowerLeg"));

        // 指ボーンはキネマティック（物理で暴れるのを防止）
        assert!(is_kinematic_bone("leftIndexProximal"));
        assert!(is_kinematic_bone("RightHandIndex1"));
    }

    #[test]
    fn test_vrm_bone_to_joint_type() {
        match vrm_bone_to_joint_type("leftLowerArm") {
            JointType::Hinge { .. } => {}
            _ => panic!("Expected Hinge for leftLowerArm"),
        }
        match vrm_bone_to_joint_type("leftUpperArm") {
            JointType::Ball { .. } => {}
            _ => panic!("Expected Ball for leftUpperArm"),
        }
        match vrm_bone_to_joint_type("hips") {
            JointType::Fixed => {}
            _ => panic!("Expected Fixed for hips"),
        }
    }

    #[test]
    fn test_yup_to_zup_mm() {
        // Y-up (0, 1, 0) → Z-up (0, 0, 1) * 1000
        let result = yup_to_zup_mm([0.0, 1.0, 0.0]);
        assert!((result[0] - 0.0).abs() < 0.01);
        assert!((result[1] - 0.0).abs() < 0.01);
        assert!((result[2] - 1000.0).abs() < 0.01);
    }

    #[test]
    fn test_no_nan_in_skeleton() {
        let humanoid = sample_humanoid();
        let nodes = sample_nodes();
        let skeleton = Skeleton::from_vrm(&humanoid, &nodes);

        for bone in &skeleton.bones {
            assert!(!bone.offset[0].is_nan(), "NaN in offset[0] for {}", bone.name);
            assert!(!bone.offset[1].is_nan(), "NaN in offset[1] for {}", bone.name);
            assert!(!bone.offset[2].is_nan(), "NaN in offset[2] for {}", bone.name);
            assert!(!bone.length.is_nan(), "NaN in length for {}", bone.name);
            assert!(!bone.mass.is_nan(), "NaN in mass for {}", bone.name);
        }
    }
}
