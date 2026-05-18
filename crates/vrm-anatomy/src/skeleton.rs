//! 人体スケルトン定義（142ボーン解剖学モデル）
//!
//! ボーンの長さ・太さ・関節の種類と可動域を定義する。
//! 座標系: Z-up、単位mm

use serde::{Deserialize, Serialize};

/// ボーンID
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct BoneId(pub usize);

/// 関節の種類
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum JointType {
    /// ヒンジ（1軸回転）: 肘・膝
    Hinge {
        axis: [f32; 3],
        min_angle: f32,
        max_angle: f32,
    },
    /// ボール（3軸回転）: 肩・股関節
    Ball {
        swing_limit: f32,
        twist_limit: f32,
    },
    /// バネ付き関節（肋骨用）
    Spring {
        stiffness: f32,
        damping: f32,
        swing_limit: f32,
    },
    /// 固定（動かない）: ルート
    Fixed,
}

/// ボーン（骨）定義
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BoneDef {
    pub name: String,
    /// 親ボーン（Noneならルート）
    pub parent: Option<BoneId>,
    /// 親の接続点からの相対オフセット
    pub offset: [f32; 3],
    /// ボーンの長さ（ローカルZ軸方向）
    pub length: f32,
    /// カプセルコライダーの半径
    pub radius: f32,
    /// 質量 (kg)
    pub mass: f32,
    /// 親との関節
    pub joint_type: JointType,
    /// ポーズモーターの剛性（0.0なら無効）
    pub pose_stiffness: f32,
    /// ポーズモーターの減衰
    pub pose_damping: f32,
    /// trueならoffsetをそのまま使い、chain判定をスキップする（glTFスケルトン用）
    pub use_direct_offset: bool,
}

/// スケルトン定義
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Skeleton {
    pub bones: Vec<BoneDef>,
}

// ボーンインデックス定数（参照用）
// 体幹: 0=Pelvis, 1=Sacrum, 2-6=L5-L1, 7-18=T12-T1, 19-25=C7-C1, 26=Head, 27=Jaw
// 肋骨: 28-39=Rib1L..Rib12L, 40-51=Rib1R..Rib12R
// 左肩帯: 52=ClavicleL, 53=ScapulaL
// 右肩帯: 54=ClavicleR, 55=ScapulaR
// 左腕: 56=UpperArmL, 57=RadiusL, 58=UlnaL
// 右腕: 59=UpperArmR, 60=RadiusR, 61=UlnaR
// 左手: 62-80 (MC1-5, 各指骨)
// 右手: 81-99
// 左脚: 100=FemurL, 101=TibiaL, 102=FibulaL
// 右脚: 103=FemurR, 104=TibiaR, 105=FibulaR
// 左足: 106-124
// 右足: 125-143

/// 有名ボーン名定数
pub mod bone_names {
    pub const PELVIS: &str = "Pelvis";
    pub const SACRUM: &str = "Sacrum";
    pub const HEAD: &str = "Head";
    pub const JAW: &str = "Jaw";
    pub const CLAVICLE_L: &str = "Clavicle_L";
    pub const CLAVICLE_R: &str = "Clavicle_R";
    pub const SCAPULA_L: &str = "Scapula_L";
    pub const SCAPULA_R: &str = "Scapula_R";
    pub const UPPER_ARM_L: &str = "UpperArm_L";
    pub const UPPER_ARM_R: &str = "UpperArm_R";
    pub const RADIUS_L: &str = "Radius_L";
    pub const RADIUS_R: &str = "Radius_R";
    pub const ULNA_L: &str = "Ulna_L";
    pub const ULNA_R: &str = "Ulna_R";
    pub const FEMUR_L: &str = "Femur_L";
    pub const FEMUR_R: &str = "Femur_R";
    pub const TIBIA_L: &str = "Tibia_L";
    pub const TIBIA_R: &str = "Tibia_R";
    pub const HAND_L: &str = "Hand_L";
    pub const HAND_R: &str = "Hand_R";
    pub const FOOT_L: &str = "Foot_L";
    pub const FOOT_R: &str = "Foot_R";
}

/// ベクトル演算ヘルパー（nalgebra非依存）
fn vec3_norm(v: [f32; 3]) -> f32 {
    (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).sqrt()
}

fn vec3_normalize(v: [f32; 3]) -> [f32; 3] {
    let n = vec3_norm(v);
    if n < 1e-10 {
        [0.0, 0.0, 0.0]
    } else {
        [v[0] / n, v[1] / n, v[2] / n]
    }
}

fn vec3_sub(a: [f32; 3], b: [f32; 3]) -> [f32; 3] {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}

fn vec3_add(a: [f32; 3], b: [f32; 3]) -> [f32; 3] {
    [a[0] + b[0], a[1] + b[1], a[2] + b[2]]
}

fn vec3_scale(v: [f32; 3], s: f32) -> [f32; 3] {
    [v[0] * s, v[1] * s, v[2] * s]
}

impl Skeleton {
    /// 142ボーン解剖学的人体スケルトン（成人、全高約1700mm）
    ///
    /// 座標系: Z-up、単位mm
    /// ルート: 骨盤（Pelvis）を原点付近に配置
    #[cfg(feature = "humanoid")]
    pub fn humanoid() -> Self {
        let mut bones = Vec::with_capacity(144);

        // === 体幹 ===

        // 0: 骨盤（ルート）
        bones.push(BoneDef {
            name: "Pelvis".into(),
            parent: None,
            offset: [0.0, 0.0, 1000.0],
            length: 80.0,
            radius: 100.0,
            mass: 8.0,
            joint_type: JointType::Fixed,
            pose_stiffness: 0.0,
            pose_damping: 0.0,
            use_direct_offset: false,
        });

        // 1: 仙骨
        bones.push(BoneDef {
            name: "Sacrum".into(),
            parent: Some(BoneId(0)),
            offset: [0.0, 20.0, -30.0],
            length: 100.0,
            radius: 50.0,
            mass: 1.5,
            joint_type: JointType::Ball {
                swing_limit: 5.0_f32.to_radians(),
                twist_limit: 3.0_f32.to_radians(),
            },
            pose_stiffness: 500.0,
            pose_damping: 50.0,
            use_direct_offset: false,
        });

        // 2-6: 腰椎 L5→L1（下から上）
        let lumbar_heights = [30.0, 35.0, 35.0, 35.0, 35.0]; // 各椎体の高さ
        for (i, &h) in lumbar_heights.iter().enumerate() {
            let parent = if i == 0 { 0 } else { 1 + i }; // L5→Pelvis, L4→L5...
            bones.push(BoneDef {
                name: format!("L{}", 5 - i),
                parent: Some(BoneId(parent)),
                offset: [0.0, 0.0, if i == 0 { 80.0 } else { lumbar_heights[i - 1] }],
                length: h,
                radius: 25.0,
                mass: 0.8,
                joint_type: JointType::Ball {
                    swing_limit: 8.0_f32.to_radians(),
                    twist_limit: 5.0_f32.to_radians(),
                },
                pose_stiffness: 500.0,
                pose_damping: 50.0,
                use_direct_offset: false,
            });
        }

        // 7-18: 胸椎 T12→T1（下から上）
        let thoracic_heights = [28.0; 12];
        for (i, &h) in thoracic_heights.iter().enumerate() {
            let parent = if i == 0 { 6 } else { 6 + i }; // T12→L1, T11→T12...
            bones.push(BoneDef {
                name: format!("T{}", 12 - i),
                parent: Some(BoneId(parent)),
                offset: [0.0, 0.0, if i == 0 { 35.0 } else { thoracic_heights[i - 1] }],
                length: h,
                radius: 22.0,
                mass: 0.6,
                joint_type: JointType::Ball {
                    swing_limit: 5.0_f32.to_radians(),
                    twist_limit: 5.0_f32.to_radians(),
                },
                pose_stiffness: 500.0,
                pose_damping: 50.0,
                use_direct_offset: false,
            });
        }

        // 19-25: 頸椎 C7→C1（下から上）
        let cervical_heights = [18.0, 16.0, 15.0, 15.0, 15.0, 17.0, 15.0];
        for (i, &h) in cervical_heights.iter().enumerate() {
            let parent = if i == 0 { 18 } else { 18 + i }; // C7→T1
            bones.push(BoneDef {
                name: format!("C{}", 7 - i),
                parent: Some(BoneId(parent)),
                offset: [0.0, 0.0, if i == 0 { 28.0 } else { cervical_heights[i - 1] }],
                length: h,
                radius: 15.0,
                mass: 0.3,
                joint_type: JointType::Ball {
                    swing_limit: 8.0_f32.to_radians(),
                    twist_limit: 8.0_f32.to_radians(),
                },
                pose_stiffness: 1000.0,
                pose_damping: 80.0,
                use_direct_offset: false,
            });
        }

        // 26: 頭
        bones.push(BoneDef {
            name: "Head".into(),
            parent: Some(BoneId(25)), // C1
            offset: [0.0, 0.0, 15.0],
            length: 200.0,
            radius: 95.0,
            mass: 4.5,
            joint_type: JointType::Ball {
                swing_limit: 20.0_f32.to_radians(),
                twist_limit: 10.0_f32.to_radians(),
            },
            pose_stiffness: 500.0,
            pose_damping: 50.0,
            use_direct_offset: false,
        });

        // 27: 顎
        bones.push(BoneDef {
            name: "Jaw".into(),
            parent: Some(BoneId(26)),
            offset: [0.0, 30.0, 80.0],
            length: 60.0,
            radius: 30.0,
            mass: 0.3,
            joint_type: JointType::Hinge {
                axis: [0.0, 1.0, 0.0],
                min_angle: 0.0,
                max_angle: 40.0_f32.to_radians(),
            },
            pose_stiffness: 100.0,
            pose_damping: 20.0,
            use_direct_offset: false,
        });

        // === 肋骨 ===
        // 28-39: 左肋骨 Rib1L..Rib12L (T1..T12に付く)
        for rib_num in 1..=12u32 {
            let thoracic_idx = 7 + (12 - rib_num) as usize; // T1=18, T12=7
            let rib_len = 80.0 + (rib_num.min(7) as f32) * 15.0 - ((rib_num.max(7) - 7) as f32) * 10.0;
            bones.push(BoneDef {
                name: format!("Rib{}_L", rib_num),
                parent: Some(BoneId(thoracic_idx)),
                offset: [-15.0, -5.0, 14.0],
                length: rib_len,
                radius: 6.0,
                mass: 0.08,
                joint_type: JointType::Spring {
                    stiffness: 2000.0,
                    damping: 50.0,
                    swing_limit: 5.0_f32.to_radians(),
                },
                pose_stiffness: 0.0,
                pose_damping: 0.0,
                use_direct_offset: false,
            });
        }

        // 40-51: 右肋骨 Rib1R..Rib12R
        for rib_num in 1..=12u32 {
            let thoracic_idx = 7 + (12 - rib_num) as usize;
            let rib_len = 80.0 + (rib_num.min(7) as f32) * 15.0 - ((rib_num.max(7) - 7) as f32) * 10.0;
            bones.push(BoneDef {
                name: format!("Rib{}_R", rib_num),
                parent: Some(BoneId(thoracic_idx)),
                offset: [15.0, -5.0, 14.0],
                length: rib_len,
                radius: 6.0,
                mass: 0.08,
                joint_type: JointType::Spring {
                    stiffness: 2000.0,
                    damping: 50.0,
                    swing_limit: 5.0_f32.to_radians(),
                },
                pose_stiffness: 0.0,
                pose_damping: 0.0,
                use_direct_offset: false,
            });
        }

        // === 肩帯 ===
        // 52: 左鎖骨
        bones.push(BoneDef {
            name: "Clavicle_L".into(),
            parent: Some(BoneId(18)), // T1
            offset: [-20.0, 0.0, 25.0],
            length: 150.0,
            radius: 10.0,
            mass: 0.5,
            joint_type: JointType::Ball {
                swing_limit: 15.0_f32.to_radians(),
                twist_limit: 10.0_f32.to_radians(),
            },
            pose_stiffness: 300.0,
            pose_damping: 40.0,
            use_direct_offset: false,
        });

        // 53: 左肩甲骨
        bones.push(BoneDef {
            name: "Scapula_L".into(),
            parent: Some(BoneId(52)),
            offset: [-140.0, 20.0, -10.0],
            length: 100.0,
            radius: 30.0,
            mass: 0.8,
            joint_type: JointType::Ball {
                swing_limit: 20.0_f32.to_radians(),
                twist_limit: 10.0_f32.to_radians(),
            },
            pose_stiffness: 250.0,
            pose_damping: 40.0,
            use_direct_offset: false,
        });

        // 54: 右鎖骨
        bones.push(BoneDef {
            name: "Clavicle_R".into(),
            parent: Some(BoneId(18)),
            offset: [20.0, 0.0, 25.0],
            length: 150.0,
            radius: 10.0,
            mass: 0.5,
            joint_type: JointType::Ball {
                swing_limit: 15.0_f32.to_radians(),
                twist_limit: 10.0_f32.to_radians(),
            },
            pose_stiffness: 300.0,
            pose_damping: 40.0,
            use_direct_offset: false,
        });

        // 55: 右肩甲骨
        bones.push(BoneDef {
            name: "Scapula_R".into(),
            parent: Some(BoneId(54)),
            offset: [140.0, 20.0, -10.0],
            length: 100.0,
            radius: 30.0,
            mass: 0.8,
            joint_type: JointType::Ball {
                swing_limit: 20.0_f32.to_radians(),
                twist_limit: 10.0_f32.to_radians(),
            },
            pose_stiffness: 250.0,
            pose_damping: 40.0,
            use_direct_offset: false,
        });

        // === 左腕 ===
        // 56: 左上腕
        bones.push(BoneDef {
            name: "UpperArm_L".into(),
            parent: Some(BoneId(53)), // 左肩甲骨
            offset: [-50.0, -20.0, 20.0],
            length: 300.0,
            radius: 35.0,
            mass: 2.5,
            joint_type: JointType::Ball {
                swing_limit: 60.0_f32.to_radians(),
                twist_limit: 10.0_f32.to_radians(), // ねじれ防止
            },
            pose_stiffness: 300.0,
            pose_damping: 50.0,
            use_direct_offset: false,
        });

        // 57: 左橈骨
        bones.push(BoneDef {
            name: "Radius_L".into(),
            parent: Some(BoneId(56)),
            offset: [0.0, 0.0, -300.0],
            length: 240.0,
            radius: 15.0,
            mass: 0.7,
            joint_type: JointType::Hinge {
                axis: [0.0, 1.0, 0.0],
                min_angle: 0.0,
                max_angle: 145.0_f32.to_radians(),
            },
            pose_stiffness: 100.0,
            pose_damping: 20.0,
            use_direct_offset: false,
        });

        // 58: 左尺骨
        bones.push(BoneDef {
            name: "Ulna_L".into(),
            parent: Some(BoneId(56)),
            offset: [0.0, 10.0, -300.0],
            length: 250.0,
            radius: 12.0,
            mass: 0.6,
            joint_type: JointType::Hinge {
                axis: [0.0, 1.0, 0.0],
                min_angle: 0.0,
                max_angle: 145.0_f32.to_radians(),
            },
            pose_stiffness: 100.0,
            pose_damping: 20.0,
            use_direct_offset: false,
        });

        // === 右腕 ===
        // 59: 右上腕
        bones.push(BoneDef {
            name: "UpperArm_R".into(),
            parent: Some(BoneId(55)),
            offset: [50.0, -20.0, 20.0],
            length: 300.0,
            radius: 35.0,
            mass: 2.5,
            joint_type: JointType::Ball {
                swing_limit: 60.0_f32.to_radians(),
                twist_limit: 10.0_f32.to_radians(), // ねじれ防止
            },
            pose_stiffness: 300.0,
            pose_damping: 50.0,
            use_direct_offset: false,
        });

        // 60: 右橈骨
        bones.push(BoneDef {
            name: "Radius_R".into(),
            parent: Some(BoneId(59)),
            offset: [0.0, 0.0, -300.0],
            length: 240.0,
            radius: 15.0,
            mass: 0.7,
            joint_type: JointType::Hinge {
                axis: [0.0, 1.0, 0.0],
                min_angle: 0.0,
                max_angle: 145.0_f32.to_radians(),
            },
            pose_stiffness: 100.0,
            pose_damping: 20.0,
            use_direct_offset: false,
        });

        // 61: 右尺骨
        bones.push(BoneDef {
            name: "Ulna_R".into(),
            parent: Some(BoneId(59)),
            offset: [0.0, 10.0, -300.0],
            length: 250.0,
            radius: 12.0,
            mass: 0.6,
            joint_type: JointType::Hinge {
                axis: [0.0, 1.0, 0.0],
                min_angle: 0.0,
                max_angle: 145.0_f32.to_radians(),
            },
            pose_stiffness: 100.0,
            pose_damping: 20.0,
            use_direct_offset: false,
        });

        // === 左手 (62-80): 中手骨5 + 指骨14 = 19 ===
        Self::build_hand(&mut bones, "L", 57); // 親は左橈骨

        // === 右手 (81-99) ===
        Self::build_hand(&mut bones, "R", 60); // 親は右橈骨

        // === 左脚 ===
        // 100: 左大腿骨
        bones.push(BoneDef {
            name: "Femur_L".into(),
            parent: Some(BoneId(0)),
            offset: [-100.0, 0.0, 0.0],
            length: 420.0,
            radius: 50.0,
            mass: 8.0,
            joint_type: JointType::Ball {
                swing_limit: 45.0_f32.to_radians(),
                twist_limit: 15.0_f32.to_radians(),
            },
            pose_stiffness: 600.0,
            pose_damping: 80.0,
            use_direct_offset: false,
        });

        // 101: 左脛骨
        bones.push(BoneDef {
            name: "Tibia_L".into(),
            parent: Some(BoneId(100)),
            offset: [0.0, 0.0, -420.0],
            length: 380.0,
            radius: 30.0,
            mass: 3.0,
            joint_type: JointType::Hinge {
                axis: [0.0, 1.0, 0.0],
                min_angle: 0.0,
                max_angle: 140.0_f32.to_radians(),
            },
            pose_stiffness: 200.0,
            pose_damping: 30.0,
            use_direct_offset: false,
        });

        // 102: 左腓骨
        bones.push(BoneDef {
            name: "Fibula_L".into(),
            parent: Some(BoneId(100)),
            offset: [-15.0, 0.0, -420.0],
            length: 360.0,
            radius: 10.0,
            mass: 0.5,
            joint_type: JointType::Hinge {
                axis: [0.0, 1.0, 0.0],
                min_angle: 0.0,
                max_angle: 140.0_f32.to_radians(),
            },
            pose_stiffness: 200.0,
            pose_damping: 30.0,
            use_direct_offset: false,
        });

        // === 右脚 ===
        // 103: 右大腿骨
        bones.push(BoneDef {
            name: "Femur_R".into(),
            parent: Some(BoneId(0)),
            offset: [100.0, 0.0, 0.0],
            length: 420.0,
            radius: 50.0,
            mass: 8.0,
            joint_type: JointType::Ball {
                swing_limit: 45.0_f32.to_radians(),
                twist_limit: 15.0_f32.to_radians(),
            },
            pose_stiffness: 600.0,
            pose_damping: 80.0,
            use_direct_offset: false,
        });

        // 104: 右脛骨
        bones.push(BoneDef {
            name: "Tibia_R".into(),
            parent: Some(BoneId(103)),
            offset: [0.0, 0.0, -420.0],
            length: 380.0,
            radius: 30.0,
            mass: 3.0,
            joint_type: JointType::Hinge {
                axis: [0.0, 1.0, 0.0],
                min_angle: 0.0,
                max_angle: 140.0_f32.to_radians(),
            },
            pose_stiffness: 200.0,
            pose_damping: 30.0,
            use_direct_offset: false,
        });

        // 105: 右腓骨
        bones.push(BoneDef {
            name: "Fibula_R".into(),
            parent: Some(BoneId(103)),
            offset: [15.0, 0.0, -420.0],
            length: 360.0,
            radius: 10.0,
            mass: 0.5,
            joint_type: JointType::Hinge {
                axis: [0.0, 1.0, 0.0],
                min_angle: 0.0,
                max_angle: 140.0_f32.to_radians(),
            },
            pose_stiffness: 200.0,
            pose_damping: 30.0,
            use_direct_offset: false,
        });

        // === 左足 (106-124): 中足骨5 + 趾骨14 = 19 ===
        Self::build_foot(&mut bones, "L", 101); // 親は左脛骨

        // === 右足 (125-143) ===
        Self::build_foot(&mut bones, "R", 104); // 親は右脛骨

        Self { bones }
    }

    /// 手のボーン生成（中手骨5 + 指骨14 = 19ボーン）
    #[cfg(feature = "humanoid")]
    fn build_hand(bones: &mut Vec<BoneDef>, side: &str, wrist_parent: usize) {
        let base_offset_x = if side == "L" { -1.0 } else { 1.0 };

        // 指の定義: (名前, x_offset, mc_len, pp_len, mp_len, dp_len)
        // 拇指は中手骨+基節骨+末節骨の3本（中節骨なし）
        let fingers = [
            ("Thumb",  -30.0 * base_offset_x, 40.0, 30.0, 0.0,  22.0),
            ("Index",  -15.0 * base_offset_x, 65.0, 35.0, 22.0, 18.0),
            ("Middle", 0.0,                    70.0, 40.0, 25.0, 18.0),
            ("Ring",   15.0 * base_offset_x,   65.0, 35.0, 22.0, 18.0),
            ("Pinky",  28.0 * base_offset_x,   55.0, 28.0, 18.0, 15.0),
        ];

        let wrist_z_offset = if side == "L" { -240.0 } else { -240.0 };

        for (finger_name, x_off, mc_len, pp_len, mp_len, dp_len) in &fingers {
            let mc_idx = bones.len();
            // 中手骨 (Metacarpal)
            bones.push(BoneDef {
                name: format!("MC_{}_{}", finger_name, side),
                parent: Some(BoneId(wrist_parent)),
                offset: [*x_off, 0.0, wrist_z_offset],
                length: *mc_len,
                radius: 5.0,
                mass: 0.02,
                joint_type: JointType::Ball {
                    swing_limit: 20.0_f32.to_radians(),
                    twist_limit: 10.0_f32.to_radians(),
                },
                pose_stiffness: 100.0,
                pose_damping: 15.0,
                use_direct_offset: false,
            });

            let pp_idx = bones.len();
            // 基節骨 (Proximal Phalanx)
            bones.push(BoneDef {
                name: format!("PP_{}_{}", finger_name, side),
                parent: Some(BoneId(mc_idx)),
                offset: [0.0, 0.0, -*mc_len],
                length: *pp_len,
                radius: 4.0,
                mass: 0.01,
                joint_type: JointType::Hinge {
                    axis: [0.0, 1.0, 0.0],
                    min_angle: -10.0_f32.to_radians(),
                    max_angle: 90.0_f32.to_radians(),
                },
                pose_stiffness: 100.0,
                pose_damping: 15.0,
                use_direct_offset: false,
            });

            if *mp_len > 0.0 {
                let mp_idx = bones.len();
                // 中節骨 (Middle Phalanx) — 拇指にはない
                bones.push(BoneDef {
                    name: format!("MP_{}_{}", finger_name, side),
                    parent: Some(BoneId(pp_idx)),
                    offset: [0.0, 0.0, -*pp_len],
                    length: *mp_len,
                    radius: 3.5,
                    mass: 0.008,
                    joint_type: JointType::Hinge {
                        axis: [0.0, 1.0, 0.0],
                        min_angle: 0.0,
                        max_angle: 100.0_f32.to_radians(),
                    },
                    pose_stiffness: 20.0,
                    pose_damping: 8.0,
                    use_direct_offset: false,
                });

                // 末節骨 (Distal Phalanx)
                bones.push(BoneDef {
                    name: format!("DP_{}_{}", finger_name, side),
                    parent: Some(BoneId(mp_idx)),
                    offset: [0.0, 0.0, -*mp_len],
                    length: *dp_len,
                    radius: 3.0,
                    mass: 0.005,
                    joint_type: JointType::Hinge {
                        axis: [0.0, 1.0, 0.0],
                        min_angle: 0.0,
                        max_angle: 80.0_f32.to_radians(),
                    },
                    pose_stiffness: 20.0,
                    pose_damping: 8.0,
                    use_direct_offset: false,
                });
            } else {
                // 拇指: 基節骨→末節骨（中節骨なし）
                bones.push(BoneDef {
                    name: format!("DP_{}_{}", finger_name, side),
                    parent: Some(BoneId(pp_idx)),
                    offset: [0.0, 0.0, -*pp_len],
                    length: *dp_len,
                    radius: 3.5,
                    mass: 0.005,
                    joint_type: JointType::Hinge {
                        axis: [0.0, 1.0, 0.0],
                        min_angle: 0.0,
                        max_angle: 80.0_f32.to_radians(),
                    },
                    pose_stiffness: 20.0,
                    pose_damping: 8.0,
                    use_direct_offset: false,
                });
            }
        }
    }

    /// 足のボーン生成（中足骨5 + 趾骨14 = 19ボーン）
    #[cfg(feature = "humanoid")]
    fn build_foot(bones: &mut Vec<BoneDef>, side: &str, ankle_parent: usize) {
        let base_offset_x = if side == "L" { -1.0 } else { 1.0 };

        // 趾の定義: (名前, x_offset, mt_len, pp_len, mp_len, dp_len)
        // 母趾は中足骨+基節骨+末節骨の3本
        let toes = [
            ("Hallux",  -20.0 * base_offset_x, 60.0, 28.0, 0.0,  18.0),
            ("Second",  -10.0 * base_offset_x, 70.0, 22.0, 14.0, 12.0),
            ("Third",    0.0,                   72.0, 20.0, 13.0, 11.0),
            ("Fourth",  10.0 * base_offset_x,   68.0, 18.0, 12.0, 10.0),
            ("Fifth",   20.0 * base_offset_x,   62.0, 15.0, 10.0, 9.0),
        ];

        let ankle_z_offset = -380.0;

        for (toe_name, x_off, mt_len, pp_len, mp_len, dp_len) in &toes {
            let mt_idx = bones.len();
            // 中足骨 (Metatarsal)
            bones.push(BoneDef {
                name: format!("MT_{}_{}", toe_name, side),
                parent: Some(BoneId(ankle_parent)),
                offset: [*x_off, -30.0, ankle_z_offset],
                length: *mt_len,
                radius: 5.0,
                mass: 0.02,
                joint_type: JointType::Ball {
                    swing_limit: 15.0_f32.to_radians(),
                    twist_limit: 5.0_f32.to_radians(),
                },
                pose_stiffness: 100.0,
                pose_damping: 15.0,
                use_direct_offset: false,
            });

            let pp_idx = bones.len();
            // 基節骨
            bones.push(BoneDef {
                name: format!("PP_{}_{}", toe_name, side),
                parent: Some(BoneId(mt_idx)),
                offset: [0.0, -*mt_len, 0.0],
                length: *pp_len,
                radius: 3.0,
                mass: 0.005,
                joint_type: JointType::Hinge {
                    axis: [1.0, 0.0, 0.0],
                    min_angle: -30.0_f32.to_radians(),
                    max_angle: 60.0_f32.to_radians(),
                },
                pose_stiffness: 100.0,
                pose_damping: 15.0,
                use_direct_offset: false,
            });

            if *mp_len > 0.0 {
                let mp_idx = bones.len();
                // 中節骨 — 母趾にはない
                bones.push(BoneDef {
                    name: format!("MP_{}_{}", toe_name, side),
                    parent: Some(BoneId(pp_idx)),
                    offset: [0.0, -*pp_len, 0.0],
                    length: *mp_len,
                    radius: 2.5,
                    mass: 0.003,
                    joint_type: JointType::Hinge {
                        axis: [1.0, 0.0, 0.0],
                        min_angle: 0.0,
                        max_angle: 45.0_f32.to_radians(),
                    },
                    pose_stiffness: 20.0,
                    pose_damping: 8.0,
                    use_direct_offset: false,
                });

                // 末節骨
                bones.push(BoneDef {
                    name: format!("DP_{}_{}", toe_name, side),
                    parent: Some(BoneId(mp_idx)),
                    offset: [0.0, -*mp_len, 0.0],
                    length: *dp_len,
                    radius: 2.0,
                    mass: 0.002,
                    joint_type: JointType::Hinge {
                        axis: [1.0, 0.0, 0.0],
                        min_angle: 0.0,
                        max_angle: 40.0_f32.to_radians(),
                    },
                    pose_stiffness: 20.0,
                    pose_damping: 8.0,
                    use_direct_offset: false,
                });
            } else {
                // 母趾
                bones.push(BoneDef {
                    name: format!("DP_{}_{}", toe_name, side),
                    parent: Some(BoneId(pp_idx)),
                    offset: [0.0, -*pp_len, 0.0],
                    length: *dp_len,
                    radius: 3.0,
                    mass: 0.003,
                    joint_type: JointType::Hinge {
                        axis: [1.0, 0.0, 0.0],
                        min_angle: 0.0,
                        max_angle: 40.0_f32.to_radians(),
                    },
                    pose_stiffness: 20.0,
                    pose_damping: 8.0,
                    use_direct_offset: false,
                });
            }
        }
    }

    /// ボーンのワールド位置を再帰的に計算
    ///
    /// チェーンボーン（offsetが親の長さに近い）は `parent_end` に直接接続。
    /// ブランチボーン（それ以外）は `parent_start + offset`。
    /// bone_direction()ではなく子offsetから方向を導出し、offset系と表示が一致する。
    pub fn compute_world_positions(&self) -> Vec<([f32; 3], [f32; 3])> {
        // 各ボーンの表示方向を事前計算（子offsetベース、leaf はbone_direction fallback）
        let display_dirs = self.compute_display_directions();

        let mut positions = Vec::with_capacity(self.bones.len());

        for (i, bone) in self.bones.iter().enumerate() {
            let start = if let Some(parent_id) = bone.parent {
                let (parent_start, parent_end): ([f32; 3], [f32; 3]) =
                    positions[parent_id.0];

                if bone.use_direct_offset {
                    // glTFスケルトン: offsetは親→子のworld差分（そのまま使う）
                    vec3_add(parent_start, bone.offset)
                } else {
                    let parent_len = self.bones[parent_id.0].length;
                    // チェーンボーン判定（144ボーン解剖学モデル用）
                    let offset_z_abs = bone.offset[2].abs();
                    let offset_lateral =
                        (bone.offset[0] * bone.offset[0] + bone.offset[1] * bone.offset[1]).sqrt();
                    let is_chain =
                        offset_z_abs > parent_len * 0.5 && offset_z_abs > offset_lateral * 2.0;

                    if is_chain {
                        let lateral = [bone.offset[0], bone.offset[1], 0.0];
                        vec3_add(parent_end, lateral)
                    } else {
                        vec3_add(parent_start, bone.offset)
                    }
                }
            } else {
                bone.offset
            };

            let end = vec3_add(start, vec3_scale(display_dirs[i], bone.length));
            positions.push((start, end));
        }

        positions
    }

    /// 各ボーンの表示方向を子offsetから導出
    ///
    /// 「最もチェーンらしい子」のoffsetを方向として使う。
    /// チェーン子がなければ bone_direction() にフォールバック。
    fn compute_display_directions(&self) -> Vec<[f32; 3]> {
        let mut dirs = Vec::with_capacity(self.bones.len());

        for (i, bone) in self.bones.iter().enumerate() {
            // このボーンの子の中で、offsetの大きさが自分のlengthに最も近いものを探す
            let mut best_dir: Option<[f32; 3]> = None;
            let mut best_error = f32::INFINITY;

            for child in &self.bones {
                if let Some(parent_id) = child.parent {
                    if parent_id.0 == i {
                        let offset_norm = vec3_norm(child.offset);
                        if offset_norm > 0.01 {
                            let error = (offset_norm - bone.length).abs();
                            if error < best_error {
                                best_error = error;
                                best_dir = Some(vec3_normalize(child.offset));
                            }
                        }
                    }
                }
            }

            // 誤差が長さの50%以内ならチェーン子として採用
            let dir = if best_error < bone.length * 0.5 {
                best_dir.unwrap()
            } else {
                self.bone_direction(i)
            };

            dirs.push(dir);
        }

        dirs
    }

    /// ボーンのローカル方向ベクトル
    fn bone_direction(&self, bone_idx: usize) -> [f32; 3] {
        let name = &self.bones[bone_idx].name;

        // 肋骨: 外側斜め前下方に伸びる
        if name.starts_with("Rib") {
            if name.ends_with("_L") {
                return vec3_normalize([-0.8, -0.4, -0.2]);
            } else {
                return vec3_normalize([0.8, -0.4, -0.2]);
            }
        }

        // 鎖骨: 横方向
        if name.starts_with("Clavicle") {
            if name.ends_with("_L") {
                return vec3_normalize([-1.0, 0.0, 0.1]);
            } else {
                return vec3_normalize([1.0, 0.0, 0.1]);
            }
        }

        // 肩甲骨: 後方
        if name.starts_with("Scapula") {
            return vec3_normalize([0.0, 1.0, -0.3]);
        }

        // 上腕
        if name.starts_with("UpperArm") {
            if name.ends_with("_L") {
                return vec3_normalize([-1.0, 0.0, -0.1]);
            } else {
                return vec3_normalize([1.0, 0.0, -0.1]);
            }
        }

        // 前腕（橈骨・尺骨）、手の骨
        if name.starts_with("Radius") || name.starts_with("Ulna")
            || name.starts_with("MC_") || name.starts_with("PP_")
            || name.starts_with("MP_") || name.starts_with("DP_")
        {
            // 足の趾の骨は前方（Y負）に伸びる
            if name.contains("Hallux") || name.contains("Second")
                || name.contains("Third") || name.contains("Fourth")
                || name.contains("Fifth")
            {
                // 親がfoot系かチェック
                if let Some(parent_id) = self.bones[bone_idx].parent {
                    if self.is_foot_bone(parent_id.0) || self.is_foot_bone(bone_idx) {
                        return [0.0, -1.0, 0.0];
                    }
                }
            }

            // 手の骨: 親の方向を継承
            if let Some(parent) = self.bones[bone_idx].parent {
                return self.bone_direction(parent.0);
            }
            return [0.0, 0.0, -1.0];
        }

        // 中足骨・趾骨: 前方
        if name.starts_with("MT_") {
            return [0.0, -1.0, 0.0];
        }

        // 大腿骨・脛骨・腓骨
        if name.starts_with("Femur") || name.starts_with("Tibia") || name.starts_with("Fibula") {
            return [0.0, 0.0, -1.0];
        }

        // 顎: 前下方
        if name == "Jaw" {
            return vec3_normalize([0.0, 1.0, -0.3]);
        }

        // 仙骨: 下方
        if name == "Sacrum" {
            return [0.0, 0.0, -1.0];
        }

        // 体幹・頭: 上向き
        [0.0, 0.0, 1.0]
    }

    /// 足のボーンかどうか判定
    fn is_foot_bone(&self, idx: usize) -> bool {
        let name = &self.bones[idx].name;
        name.starts_with("MT_") || name.starts_with("Tibia") || name.starts_with("Fibula")
            || (name.starts_with("PP_") && (name.contains("Hallux") || name.contains("Second")
                || name.contains("Third") || name.contains("Fourth") || name.contains("Fifth")))
            || (name.starts_with("MP_") && (name.contains("Hallux") || name.contains("Second")
                || name.contains("Third") || name.contains("Fourth") || name.contains("Fifth")))
    }

    pub fn bone_count(&self) -> usize {
        self.bones.len()
    }

    pub fn find_bone(&self, name: &str) -> Option<BoneId> {
        // 1) 完全一致
        if let Some(pos) = self.bones.iter().position(|b| b.name == name) {
            return Some(BoneId(pos));
        }
        // 1.5a) 入力名からmixamorig:を除去してボーン名と比較
        if let Some(stripped) = name.strip_prefix("mixamorig:") {
            if let Some(pos) = self.bones.iter().position(|b| b.name == stripped) {
                return Some(BoneId(pos));
            }
        }
        // 1.5b) ボーン名からmixamorig:を除去して入力名と比較
        if let Some(pos) = self.bones.iter().position(|b| {
            b.name.strip_prefix("mixamorig:").map(|s| s == name).unwrap_or(false)
        }) {
            return Some(BoneId(pos));
        }
        // 2) エイリアステーブル: anatomical name → Mixamo/RPM/Rigify 等の別名
        //    双方向解決: 正引き（anatomical name→別名）+ 逆引き（別名→anatomical name）
        let alternatives: &[&str] = match name {
            // 体幹
            "Head" => &["head", "Head", "mixamorig:Head"],
            "C4" => &["Neck", "neck", "spine.005", "Spine.005", "mixamorig:Neck"],
            "C2" => &["spine.006", "Spine.006"],
            "T1" => &["Spine2", "spine.004", "Spine.004", "mixamorig:Spine2"],
            "T6" => &["Spine1", "spine.003", "Spine.003", "mixamorig:Spine1"],
            "T10" => &["spine.002", "Spine.002"],
            "L3" => &["spine.001", "Spine.001"],
            "L5" => &["Spine", "spine", "mixamorig:Spine"],
            "Pelvis" => &["Hips", "hips", "mixamorig:Hips"],
            // 肩
            "Clavicle_L" => &["LeftShoulder", "shoulder.L", "Shoulder.L", "mixamorig:LeftShoulder"],
            "Clavicle_R" => &["RightShoulder", "shoulder.R", "Shoulder.R", "mixamorig:RightShoulder"],
            // 腕
            "UpperArm_L" => &["LeftArm", "upper_arm.L", "UpperArm.L", "mixamorig:LeftArm"],
            "UpperArm_R" => &["RightArm", "upper_arm.R", "UpperArm.R", "mixamorig:RightArm"],
            "Radius_L" => &["LeftForeArm", "forearm.L", "LowerArm.L", "mixamorig:LeftForeArm"],
            "Radius_R" => &["RightForeArm", "forearm.R", "LowerArm.R", "mixamorig:RightForeArm"],
            "Hand_L" => &["LeftHand", "hand.L", "Hand.L", "mixamorig:LeftHand"],
            "Hand_R" => &["RightHand", "hand.R", "Hand.R", "mixamorig:RightHand"],
            // 脚
            "Femur_L" => &["LeftUpLeg", "thigh.L", "UpperLeg.L", "mixamorig:LeftUpLeg"],
            "Femur_R" => &["RightUpLeg", "thigh.R", "UpperLeg.R", "mixamorig:RightUpLeg"],
            "Tibia_L" => &["LeftLeg", "shin.L", "LowerLeg.L", "mixamorig:LeftLeg"],
            "Tibia_R" => &["RightLeg", "shin.R", "LowerLeg.R", "mixamorig:RightLeg"],
            "Foot_L" => &["LeftFoot", "foot.L", "Foot.L", "mixamorig:LeftFoot"],
            "Foot_R" => &["RightFoot", "foot.R", "Foot.R", "mixamorig:RightFoot"],
            "MT_Hallux_L" => &["LeftToeBase", "toe.L", "Toes.L", "mixamorig:LeftToeBase"],
            "MT_Hallux_R" => &["RightToeBase", "toe.R", "Toes.R", "mixamorig:RightToeBase"],
            // 右手指 (anatomical name → Mixamo/RPM名)
            "MC_Thumb_R" => &["RightHandThumb1", "thumb.01.R", "mixamorig:RightHandThumb1"],
            "PP_Thumb_R" => &["RightHandThumb2", "thumb.02.R", "mixamorig:RightHandThumb2"],
            "DP_Thumb_R" => &["RightHandThumb3", "thumb.03.R", "mixamorig:RightHandThumb3"],
            "PP_Index_R" => &["RightHandIndex1", "f_index.01.R", "mixamorig:RightHandIndex1"],
            "MP_Index_R" => &["RightHandIndex2", "f_index.02.R", "mixamorig:RightHandIndex2"],
            "DP_Index_R" => &["RightHandIndex3", "f_index.03.R", "mixamorig:RightHandIndex3"],
            "PP_Middle_R" => &["RightHandMiddle1", "f_middle.01.R", "mixamorig:RightHandMiddle1"],
            "MP_Middle_R" => &["RightHandMiddle2", "f_middle.02.R", "mixamorig:RightHandMiddle2"],
            "DP_Middle_R" => &["RightHandMiddle3", "f_middle.03.R", "mixamorig:RightHandMiddle3"],
            "PP_Ring_R" => &["RightHandRing1", "f_ring.01.R", "mixamorig:RightHandRing1"],
            "MP_Ring_R" => &["RightHandRing2", "f_ring.02.R", "mixamorig:RightHandRing2"],
            "DP_Ring_R" => &["RightHandRing3", "f_ring.03.R", "mixamorig:RightHandRing3"],
            "PP_Pinky_R" => &["RightHandPinky1", "f_pinky.01.R", "mixamorig:RightHandPinky1"],
            "MP_Pinky_R" => &["RightHandPinky2", "f_pinky.02.R", "mixamorig:RightHandPinky2"],
            "DP_Pinky_R" => &["RightHandPinky3", "f_pinky.03.R", "mixamorig:RightHandPinky3"],
            // 左手指
            "MC_Thumb_L" => &["LeftHandThumb1", "thumb.01.L", "mixamorig:LeftHandThumb1"],
            "PP_Thumb_L" => &["LeftHandThumb2", "thumb.02.L", "mixamorig:LeftHandThumb2"],
            "DP_Thumb_L" => &["LeftHandThumb3", "thumb.03.L", "mixamorig:LeftHandThumb3"],
            "PP_Index_L" => &["LeftHandIndex1", "f_index.01.L", "mixamorig:LeftHandIndex1"],
            "MP_Index_L" => &["LeftHandIndex2", "f_index.02.L", "mixamorig:LeftHandIndex2"],
            "DP_Index_L" => &["LeftHandIndex3", "f_index.03.L", "mixamorig:LeftHandIndex3"],
            "PP_Middle_L" => &["LeftHandMiddle1", "f_middle.01.L", "mixamorig:LeftHandMiddle1"],
            "MP_Middle_L" => &["LeftHandMiddle2", "f_middle.02.L", "mixamorig:LeftHandMiddle2"],
            "DP_Middle_L" => &["LeftHandMiddle3", "f_middle.03.L", "mixamorig:LeftHandMiddle3"],
            "PP_Ring_L" => &["LeftHandRing1", "f_ring.01.L", "mixamorig:LeftHandRing1"],
            "MP_Ring_L" => &["LeftHandRing2", "f_ring.02.L", "mixamorig:LeftHandRing2"],
            "DP_Ring_L" => &["LeftHandRing3", "f_ring.03.L", "mixamorig:LeftHandRing3"],
            "PP_Pinky_L" => &["LeftHandPinky1", "f_pinky.01.L", "mixamorig:LeftHandPinky1"],
            "MP_Pinky_L" => &["LeftHandPinky2", "f_pinky.02.L", "mixamorig:LeftHandPinky2"],
            "DP_Pinky_L" => &["LeftHandPinky3", "f_pinky.03.L", "mixamorig:LeftHandPinky3"],
            // Mixamo名 → VRM名（逆引き: プリセットがMixamo名を使う場合にVRM名ボーンを解決）
            // 右手指
            "RightHandThumb1" => &["rightThumbMetacarpal", "MC_Thumb_R"],
            "RightHandThumb2" => &["rightThumbProximal", "PP_Thumb_R"],
            "RightHandThumb3" => &["rightThumbDistal", "DP_Thumb_R"],
            "RightHandIndex1" => &["rightIndexProximal", "PP_Index_R"],
            "RightHandIndex2" => &["rightIndexIntermediate", "MP_Index_R"],
            "RightHandIndex3" => &["rightIndexDistal", "DP_Index_R"],
            "RightHandMiddle1" => &["rightMiddleProximal", "PP_Middle_R"],
            "RightHandMiddle2" => &["rightMiddleIntermediate", "MP_Middle_R"],
            "RightHandMiddle3" => &["rightMiddleDistal", "DP_Middle_R"],
            "RightHandRing1" => &["rightRingProximal", "PP_Ring_R"],
            "RightHandRing2" => &["rightRingIntermediate", "MP_Ring_R"],
            "RightHandRing3" => &["rightRingDistal", "DP_Ring_R"],
            "RightHandPinky1" => &["rightLittleProximal", "PP_Pinky_R"],
            "RightHandPinky2" => &["rightLittleIntermediate", "MP_Pinky_R"],
            "RightHandPinky3" => &["rightLittleDistal", "DP_Pinky_R"],
            // 左手指
            "LeftHandThumb1" => &["leftThumbMetacarpal", "MC_Thumb_L"],
            "LeftHandThumb2" => &["leftThumbProximal", "PP_Thumb_L"],
            "LeftHandThumb3" => &["leftThumbDistal", "DP_Thumb_L"],
            "LeftHandIndex1" => &["leftIndexProximal", "PP_Index_L"],
            "LeftHandIndex2" => &["leftIndexIntermediate", "MP_Index_L"],
            "LeftHandIndex3" => &["leftIndexDistal", "DP_Index_L"],
            "LeftHandMiddle1" => &["leftMiddleProximal", "PP_Middle_L"],
            "LeftHandMiddle2" => &["leftMiddleIntermediate", "MP_Middle_L"],
            "LeftHandMiddle3" => &["leftMiddleDistal", "DP_Middle_L"],
            "LeftHandRing1" => &["leftRingProximal", "PP_Ring_L"],
            "LeftHandRing2" => &["leftRingIntermediate", "MP_Ring_L"],
            "LeftHandRing3" => &["leftRingDistal", "DP_Ring_L"],
            "LeftHandPinky1" => &["leftLittleProximal", "PP_Pinky_L"],
            "LeftHandPinky2" => &["leftLittleIntermediate", "MP_Pinky_L"],
            "LeftHandPinky3" => &["leftLittleDistal", "DP_Pinky_L"],
            _ => &[],
        };
        for alt in alternatives {
            if let Some(pos) = self.bones.iter().position(|b| b.name == *alt) {
                return Some(BoneId(pos));
            }
        }
        // 3) 逆引き: 入力名がエイリアス値に含まれていればそのanatomical nameで検索
        //    例: "Hips" → Pelvis, "LeftArm" → UpperArm_L, "RightForeArm" → Radius_R
        let reverse_map: &[(&str, &[&str])] = &[
            ("Pelvis", &["Hips", "hips", "mixamorig:Hips"]),
            ("L5", &["Spine", "spine", "mixamorig:Spine"]),
            ("L3", &["spine.001", "Spine.001"]),
            ("T10", &["spine.002", "Spine.002"]),
            ("T6", &["Spine1", "spine.003", "Spine.003", "mixamorig:Spine1"]),
            ("T1", &["Spine2", "spine.004", "Spine.004", "mixamorig:Spine2"]),
            ("C4", &["Neck", "neck", "spine.005", "Spine.005", "mixamorig:Neck"]),
            ("C2", &["spine.006", "Spine.006"]),
            ("Head", &["head", "mixamorig:Head"]),
            ("Jaw", &["jaw"]),
            ("Clavicle_L", &["LeftShoulder", "shoulder.L", "Shoulder.L", "mixamorig:LeftShoulder"]),
            ("Clavicle_R", &["RightShoulder", "shoulder.R", "Shoulder.R", "mixamorig:RightShoulder"]),
            ("UpperArm_L", &["LeftArm", "upper_arm.L", "UpperArm.L", "mixamorig:LeftArm"]),
            ("UpperArm_R", &["RightArm", "upper_arm.R", "UpperArm.R", "mixamorig:RightArm"]),
            ("Radius_L", &["LeftForeArm", "forearm.L", "LowerArm.L", "mixamorig:LeftForeArm"]),
            ("Radius_R", &["RightForeArm", "forearm.R", "LowerArm.R", "mixamorig:RightForeArm"]),
            ("Hand_L", &["LeftHand", "hand.L", "Hand.L", "mixamorig:LeftHand"]),
            ("Hand_R", &["RightHand", "hand.R", "Hand.R", "mixamorig:RightHand"]),
            ("Femur_L", &["LeftUpLeg", "thigh.L", "UpperLeg.L", "mixamorig:LeftUpLeg"]),
            ("Femur_R", &["RightUpLeg", "thigh.R", "UpperLeg.R", "mixamorig:RightUpLeg"]),
            ("Tibia_L", &["LeftLeg", "shin.L", "LowerLeg.L", "mixamorig:LeftLeg"]),
            ("Tibia_R", &["RightLeg", "shin.R", "LowerLeg.R", "mixamorig:RightLeg"]),
            ("Foot_L", &["LeftFoot", "foot.L", "Foot.L", "mixamorig:LeftFoot"]),
            ("Foot_R", &["RightFoot", "foot.R", "Foot.R", "mixamorig:RightFoot"]),
            ("MT_Hallux_L", &["LeftToeBase", "toe.L", "Toes.L", "mixamorig:LeftToeBase"]),
            ("MT_Hallux_R", &["RightToeBase", "toe.R", "Toes.R", "mixamorig:RightToeBase"]),
            // 右手指
            ("MC_Thumb_R", &["RightHandThumb1", "thumb.01.R", "mixamorig:RightHandThumb1"]),
            ("PP_Thumb_R", &["RightHandThumb2", "thumb.02.R", "mixamorig:RightHandThumb2"]),
            ("DP_Thumb_R", &["RightHandThumb3", "thumb.03.R", "mixamorig:RightHandThumb3"]),
            ("PP_Index_R", &["RightHandIndex1", "f_index.01.R", "mixamorig:RightHandIndex1"]),
            ("MP_Index_R", &["RightHandIndex2", "f_index.02.R", "mixamorig:RightHandIndex2"]),
            ("DP_Index_R", &["RightHandIndex3", "f_index.03.R", "mixamorig:RightHandIndex3"]),
            ("PP_Middle_R", &["RightHandMiddle1", "f_middle.01.R", "mixamorig:RightHandMiddle1"]),
            ("MP_Middle_R", &["RightHandMiddle2", "f_middle.02.R", "mixamorig:RightHandMiddle2"]),
            ("DP_Middle_R", &["RightHandMiddle3", "f_middle.03.R", "mixamorig:RightHandMiddle3"]),
            ("PP_Ring_R", &["RightHandRing1", "f_ring.01.R", "mixamorig:RightHandRing1"]),
            ("MP_Ring_R", &["RightHandRing2", "f_ring.02.R", "mixamorig:RightHandRing2"]),
            ("DP_Ring_R", &["RightHandRing3", "f_ring.03.R", "mixamorig:RightHandRing3"]),
            ("PP_Pinky_R", &["RightHandPinky1", "f_pinky.01.R", "mixamorig:RightHandPinky1"]),
            ("MP_Pinky_R", &["RightHandPinky2", "f_pinky.02.R", "mixamorig:RightHandPinky2"]),
            ("DP_Pinky_R", &["RightHandPinky3", "f_pinky.03.R", "mixamorig:RightHandPinky3"]),
            // 左手指
            ("MC_Thumb_L", &["LeftHandThumb1", "thumb.01.L", "mixamorig:LeftHandThumb1"]),
            ("PP_Thumb_L", &["LeftHandThumb2", "thumb.02.L", "mixamorig:LeftHandThumb2"]),
            ("DP_Thumb_L", &["LeftHandThumb3", "thumb.03.L", "mixamorig:LeftHandThumb3"]),
            ("PP_Index_L", &["LeftHandIndex1", "f_index.01.L", "mixamorig:LeftHandIndex1"]),
            ("MP_Index_L", &["LeftHandIndex2", "f_index.02.L", "mixamorig:LeftHandIndex2"]),
            ("DP_Index_L", &["LeftHandIndex3", "f_index.03.L", "mixamorig:LeftHandIndex3"]),
            ("PP_Middle_L", &["LeftHandMiddle1", "f_middle.01.L", "mixamorig:LeftHandMiddle1"]),
            ("MP_Middle_L", &["LeftHandMiddle2", "f_middle.02.L", "mixamorig:LeftHandMiddle2"]),
            ("DP_Middle_L", &["LeftHandMiddle3", "f_middle.03.L", "mixamorig:LeftHandMiddle3"]),
            ("PP_Ring_L", &["LeftHandRing1", "f_ring.01.L", "mixamorig:LeftHandRing1"]),
            ("MP_Ring_L", &["LeftHandRing2", "f_ring.02.L", "mixamorig:LeftHandRing2"]),
            ("DP_Ring_L", &["LeftHandRing3", "f_ring.03.L", "mixamorig:LeftHandRing3"]),
            ("PP_Pinky_L", &["LeftHandPinky1", "f_pinky.01.L", "mixamorig:LeftHandPinky1"]),
            ("MP_Pinky_L", &["LeftHandPinky2", "f_pinky.02.L", "mixamorig:LeftHandPinky2"]),
            ("DP_Pinky_L", &["LeftHandPinky3", "f_pinky.03.L", "mixamorig:LeftHandPinky3"]),
        ];
        // mixamorig:プレフィックス除去版も照合
        let check_name = name.strip_prefix("mixamorig:").unwrap_or(name);
        for &(anatomical_name, alt_names) in reverse_map {
            for &alt in alt_names {
                let alt_check = alt.strip_prefix("mixamorig:").unwrap_or(alt);
                if check_name == alt || check_name == alt_check {
                    if let Some(pos) = self.bones.iter().position(|b| b.name == anatomical_name) {
                        return Some(BoneId(pos));
                    }
                }
            }
        }
        None
    }

    /// glTFジョイント名をanatomical bone indexにマッピング
    ///
    /// 完全一致 → エイリアステーブル → Pelvis(0)フォールバック の優先順で解決。
    /// 返り値はglTFジョイントインデックス→anatomical bone indexの変換テーブル。
    pub fn map_gltf_joint_names(&self, gltf_names: &[String]) -> Vec<u32> {
        // Blender標準名 / Rigify名 → anatomical nameのエイリアステーブル
        // 大文字・小文字両方登録（case-insensitiveルックアップの代わり）
        let aliases: &[(&str, &str)] = &[
            // === 体幹（Rigify spine.000〜005 対応）===
            ("Hips", "Pelvis"),
            ("hips", "Pelvis"),
            ("Spine", "L5"),
            ("spine", "L5"),
            // Rigify脊椎セグメント（小文字 + 大文字）
            ("spine.001", "L3"),
            ("Spine.001", "L3"),
            ("spine.002", "T10"),
            ("Spine.002", "T10"),
            ("spine.003", "T6"),
            ("Spine.003", "T6"),
            ("spine.004", "T1"),
            ("Spine.004", "T1"),
            ("spine.005", "C4"),
            ("Spine.005", "C4"),
            ("spine.006", "C2"),
            ("Spine.006", "C2"),
            // 旧形式の脊椎名
            ("Spine1", "T12"),
            ("Spine2", "T6"),
            ("Chest", "T1"),
            ("chest", "T1"),
            ("Neck", "C4"),
            ("neck", "C4"),
            ("Head", "Head"),
            ("head", "Head"),
            // === 肩帯 ===
            ("Shoulder.L", "Clavicle_L"),
            ("shoulder.L", "Clavicle_L"),
            ("Shoulder.R", "Clavicle_R"),
            ("shoulder.R", "Clavicle_R"),
            // === 腕 ===
            ("UpperArm.L", "UpperArm_L"),
            ("upper_arm.L", "UpperArm_L"),
            ("UpperArm.R", "UpperArm_R"),
            ("upper_arm.R", "UpperArm_R"),
            ("LowerArm.L", "Radius_L"),
            ("forearm.L", "Radius_L"),
            ("LowerArm.R", "Radius_R"),
            ("forearm.R", "Radius_R"),
            ("Hand.L", "Radius_L"),
            ("hand.L", "Radius_L"),
            ("Hand.R", "Radius_R"),
            ("hand.R", "Radius_R"),
            // === 脚 ===
            ("UpperLeg.L", "Femur_L"),
            ("thigh.L", "Femur_L"),
            ("UpperLeg.R", "Femur_R"),
            ("thigh.R", "Femur_R"),
            ("LowerLeg.L", "Tibia_L"),
            ("shin.L", "Tibia_L"),
            ("LowerLeg.R", "Tibia_R"),
            ("shin.R", "Tibia_R"),
            ("Foot.L", "Foot_L"),
            ("foot.L", "Foot_L"),
            ("LeftFoot", "Foot_L"),
            ("mixamorig:LeftFoot", "Foot_L"),
            ("Foot.R", "Foot_R"),
            ("foot.R", "Foot_R"),
            ("RightFoot", "Foot_R"),
            ("mixamorig:RightFoot", "Foot_R"),
            ("Toes.L", "MT_Hallux_L"),
            ("toe.L", "MT_Hallux_L"),
            ("LeftToeBase", "MT_Hallux_L"),
            ("mixamorig:LeftToeBase", "MT_Hallux_L"),
            ("Toes.R", "MT_Hallux_R"),
            ("toe.R", "MT_Hallux_R"),
            ("RightToeBase", "MT_Hallux_R"),
            ("mixamorig:RightToeBase", "MT_Hallux_R"),
            // === Rigify 股関節・踵 ===
            ("pelvis.L", "Femur_L"),
            ("pelvis.R", "Femur_R"),
            ("heel.02.L", "Tibia_L"),
            ("heel.02.R", "Tibia_R"),
            // === Rigify 指（左）===
            ("thumb.01.L", "MC_Thumb_L"),
            ("thumb.02.L", "PP_Thumb_L"),
            ("thumb.03.L", "DP_Thumb_L"),
            ("f_index.01.L", "PP_Index_L"),
            ("f_index.02.L", "MP_Index_L"),
            ("f_index.03.L", "DP_Index_L"),
            ("f_middle.01.L", "PP_Middle_L"),
            ("f_middle.02.L", "MP_Middle_L"),
            ("f_middle.03.L", "DP_Middle_L"),
            ("f_ring.01.L", "PP_Ring_L"),
            ("f_ring.02.L", "MP_Ring_L"),
            ("f_ring.03.L", "DP_Ring_L"),
            ("f_pinky.01.L", "PP_Pinky_L"),
            ("f_pinky.02.L", "MP_Pinky_L"),
            ("f_pinky.03.L", "DP_Pinky_L"),
            // === Rigify 指（右）===
            ("thumb.01.R", "MC_Thumb_R"),
            ("thumb.02.R", "PP_Thumb_R"),
            ("thumb.03.R", "DP_Thumb_R"),
            ("f_index.01.R", "PP_Index_R"),
            ("f_index.02.R", "MP_Index_R"),
            ("f_index.03.R", "DP_Index_R"),
            ("f_middle.01.R", "PP_Middle_R"),
            ("f_middle.02.R", "MP_Middle_R"),
            ("f_middle.03.R", "DP_Middle_R"),
            ("f_ring.01.R", "PP_Ring_R"),
            ("f_ring.02.R", "MP_Ring_R"),
            ("f_ring.03.R", "DP_Ring_R"),
            ("f_pinky.01.R", "PP_Pinky_R"),
            ("f_pinky.02.R", "MP_Pinky_R"),
            ("f_pinky.03.R", "DP_Pinky_R"),
        ];

        let alias_map: std::collections::HashMap<&str, &str> =
            aliases.iter().copied().collect();

        gltf_names
            .iter()
            .enumerate()
            .map(|(i, gltf_name)| {
                // 1. find_bone() — 完全一致 + エイリアス(find_bone内) + ドット変換
                if let Some(bone_id) = self.find_bone(gltf_name) {
                    return bone_id.0 as u32;
                }

                // 2. "mixamorig:" プレフィックス除去して再試行
                if let Some(stripped) = gltf_name.strip_prefix("mixamorig:") {
                    if let Some(bone_id) = self.find_bone(stripped) {
                        return bone_id.0 as u32;
                    }
                }

                // 3. ドット→アンダースコア変換
                let underscore_name = gltf_name.replace('.', "_");
                if &underscore_name != gltf_name {
                    if let Some(bone_id) = self.find_bone(&underscore_name) {
                        return bone_id.0 as u32;
                    }
                }

                // 4. レガシーエイリアステーブル（古い形式との互換）
                if let Some(&anatomical_name) = alias_map.get(gltf_name.as_str()) {
                    if let Some(bone_id) = self.find_bone(anatomical_name) {
                        return bone_id.0 as u32;
                    }
                }

                // 5. フォールバック: Pelvis(0) + 警告
                tracing::warn!(
                    "glTFジョイント[{}] '{}' がスケルトンに見つかりません → Pelvis(0)にフォールバック",
                    i, gltf_name
                );
                0
            })
            .collect()
    }

    /// ボーン名の部分一致で検索（複数ヒット可）
    pub fn find_bones_containing(&self, pattern: &str) -> Vec<BoneId> {
        self.bones
            .iter()
            .enumerate()
            .filter(|(_, b)| b.name.contains(pattern))
            .map(|(i, _)| BoneId(i))
            .collect()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[cfg(feature = "humanoid")]
    #[test]
    fn test_humanoid_bone_count() {
        let skel = Skeleton::humanoid();
        // 2(pelvis+sacrum) + 5(lumbar) + 12(thoracic) + 7(cervical) + 2(head+jaw)
        // + 24(ribs) + 4(clavicle+scapula)
        // + 2(upperarm) + 4(radius+ulna) + 38(hands) + 4(femur+tibia) + 2(fibula) + 38(feet)
        // = 28 + 24 + 4 + 6 + 38 + 6 + 38 = 144
        assert!(
            skel.bone_count() >= 140,
            "Expected ~144 bones, got {}",
            skel.bone_count()
        );
        println!("Total bones: {}", skel.bone_count());
    }

    #[cfg(feature = "humanoid")]
    #[test]
    fn test_world_positions_no_nan() {
        let skel = Skeleton::humanoid();
        let positions = skel.compute_world_positions();
        assert_eq!(positions.len(), skel.bone_count());

        for (i, (start, end)) in positions.iter().enumerate() {
            assert!(
                !start[0].is_nan() && !start[1].is_nan() && !start[2].is_nan(),
                "Bone {} ({}) start has NaN",
                i,
                skel.bones[i].name
            );
            assert!(
                !end[0].is_nan() && !end[1].is_nan() && !end[2].is_nan(),
                "Bone {} ({}) end has NaN",
                i,
                skel.bones[i].name
            );
        }
    }

    #[cfg(feature = "humanoid")]
    #[test]
    fn test_find_bone() {
        let skel = Skeleton::humanoid();
        assert!(skel.find_bone("Pelvis").is_some());
        assert!(skel.find_bone("Head").is_some());
        assert!(skel.find_bone("Femur_L").is_some());
        assert!(skel.find_bone("MC_Index_L").is_some());
        assert!(skel.find_bone("MT_Hallux_R").is_some());
    }

    #[cfg(feature = "humanoid")]
    #[test]
    fn test_parent_indices_valid() {
        let skel = Skeleton::humanoid();
        for (i, bone) in skel.bones.iter().enumerate() {
            if let Some(parent_id) = bone.parent {
                assert!(
                    parent_id.0 < i,
                    "Bone {} ({}) has parent {} which is >= self",
                    i,
                    bone.name,
                    parent_id.0
                );
            }
        }
    }

    #[cfg(feature = "humanoid")]
    #[test]
    fn test_map_gltf_joint_names_exact_match() {
        let skel = Skeleton::humanoid();
        let gltf_names = vec![
            "Pelvis".to_string(),
            "Head".to_string(),
            "Femur_L".to_string(),
        ];
        let map = skel.map_gltf_joint_names(&gltf_names);
        assert_eq!(map.len(), 3);
        assert_eq!(map[0], skel.find_bone("Pelvis").unwrap().0 as u32);
        assert_eq!(map[1], skel.find_bone("Head").unwrap().0 as u32);
        assert_eq!(map[2], skel.find_bone("Femur_L").unwrap().0 as u32);
    }

    #[cfg(feature = "humanoid")]
    #[test]
    fn test_map_gltf_joint_names_alias() {
        let skel = Skeleton::humanoid();
        let gltf_names = vec![
            "Hips".to_string(),
            "Spine".to_string(),
            "UpperArm.L".to_string(),
            "forearm.R".to_string(),
        ];
        let map = skel.map_gltf_joint_names(&gltf_names);
        assert_eq!(map[0], skel.find_bone("Pelvis").unwrap().0 as u32);
        assert_eq!(map[1], skel.find_bone("L5").unwrap().0 as u32);
        assert_eq!(map[2], skel.find_bone("UpperArm_L").unwrap().0 as u32);
        assert_eq!(map[3], skel.find_bone("Radius_R").unwrap().0 as u32);
    }

    #[cfg(feature = "humanoid")]
    #[test]
    fn test_map_gltf_joint_names_dot_to_underscore() {
        let skel = Skeleton::humanoid();
        // "Femur.L" → "Femur_L" (ドット→アンダースコア変換)
        let gltf_names = vec!["Femur.L".to_string()];
        let map = skel.map_gltf_joint_names(&gltf_names);
        assert_eq!(map[0], skel.find_bone("Femur_L").unwrap().0 as u32);
    }

    #[cfg(feature = "humanoid")]
    #[test]
    fn test_map_gltf_joint_names_fallback() {
        let skel = Skeleton::humanoid();
        let gltf_names = vec!["NonExistentBone".to_string()];
        let map = skel.map_gltf_joint_names(&gltf_names);
        assert_eq!(map[0], 0, "未知のボーンはPelvis(0)にフォールバック");
    }

    // =========================================================================
    // ボーン連結テスト: チェーンボーンが親の終端に正しく接続されること
    // =========================================================================

    /// 全チェーンボーンの開始位置が親の終端近傍にあることを検証
    #[cfg(feature = "humanoid")]
    #[test]
    fn test_chain_bones_connect_at_parent_end() {
        let skel = Skeleton::humanoid();
        let positions = skel.compute_world_positions();

        // チェーンボーン: (子ボーン名, 親ボーン名, 許容gap mm)
        let chains: &[(&str, &str, f32)] = &[
            // 腕チェーン
            ("Radius_L", "UpperArm_L", 1.0),
            ("Ulna_L", "UpperArm_L", 15.0), // Ulnaは10mm横にオフセット
            ("Radius_R", "UpperArm_R", 1.0),
            ("Ulna_R", "UpperArm_R", 15.0),
            // 脚チェーン
            ("Tibia_L", "Femur_L", 1.0),
            ("Fibula_L", "Femur_L", 20.0),
            ("Tibia_R", "Femur_R", 1.0),
            ("Fibula_R", "Femur_R", 20.0),
            // 脊椎チェーン
            ("L5", "Pelvis", 1.0),
            ("L4", "L5", 10.0),
            ("T12", "L1", 5.0),
            ("T1", "T2", 1.0),
            ("C7", "T1", 1.0),
            ("C1", "C2", 1.0),
            ("Head", "C1", 1.0),
        ];

        for &(child_name, parent_name, max_gap) in chains {
            let child_id = skel.find_bone(child_name);
            let parent_id = skel.find_bone(parent_name);
            if let (Some(c), Some(p)) = (child_id, parent_id) {
                let (child_start, _) = positions[c.0];
                let (_, parent_end) = positions[p.0];
                let gap = vec3_norm(vec3_sub(child_start, parent_end));
                assert!(
                    gap <= max_gap,
                    "{} → {} gap: {:.1}mm (max {:.0}mm)\n  parent_end=({:.1},{:.1},{:.1})\n  child_start=({:.1},{:.1},{:.1})",
                    parent_name, child_name, gap, max_gap,
                    parent_end[0], parent_end[1], parent_end[2],
                    child_start[0], child_start[1], child_start[2],
                );
            }
        }
    }

    /// 全ボーンが親ボーンの近傍にあること（バラバラ防止）
    #[cfg(feature = "humanoid")]
    #[test]
    fn test_all_bones_within_parent_reach() {
        let skel = Skeleton::humanoid();
        let positions = skel.compute_world_positions();

        for (i, bone) in skel.bones.iter().enumerate() {
            if let Some(parent_id) = bone.parent {
                let (parent_start, parent_end) = positions[parent_id.0];
                let (child_start, _) = positions[i];
                let parent_len = vec3_norm(vec3_sub(parent_end, parent_start));

                // 子の開始位置は親ボーンの開始 or 終了のどちらかの近傍にあるべき
                let dist_from_start = vec3_norm(vec3_sub(child_start, parent_start));
                let dist_from_end = vec3_norm(vec3_sub(child_start, parent_end));
                let min_dist = dist_from_start.min(dist_from_end);

                // 親の長さ+150mm以内（肩甲骨など大きなオフセットがあるため余裕を持たせる）
                let max_dist = parent_len + 150.0;
                assert!(
                    min_dist <= max_dist,
                    "Bone {} ({}) is {:.1}mm away from nearest parent endpoint (max {:.0}mm)",
                    i, bone.name, min_dist, max_dist,
                );
            }
        }
    }

    // =========================================================================
    // 脊椎連続性テスト
    // =========================================================================

    /// 脊椎が連続的に上方向(Z+)に積み上がること
    #[cfg(feature = "humanoid")]
    #[test]
    fn test_spine_stacks_upward() {
        let skel = Skeleton::humanoid();
        let positions = skel.compute_world_positions();

        let spine_names = [
            "L5", "L4", "L3", "L2", "L1",
            "T12", "T11", "T10", "T9", "T8", "T7", "T6", "T5", "T4", "T3", "T2", "T1",
            "C7", "C6", "C5", "C4", "C3", "C2", "C1",
        ];

        let mut prev_z = f32::NEG_INFINITY;
        let mut found = 0;
        for name in &spine_names {
            if let Some(bone_id) = skel.find_bone(name) {
                let (start, _) = positions[bone_id.0];
                assert!(
                    start[2] >= prev_z - 5.0,
                    "Spine {} (z={:.1}) below previous (z={:.1})",
                    name, start[2], prev_z
                );
                prev_z = start[2];
                found += 1;
            }
        }
        assert_eq!(found, 24, "Expected 24 spine bones (L5-L1 + T12-T1 + C7-C1)");
    }

    /// 脊椎全体がX=0近傍を通ること（左右にブレない）
    #[cfg(feature = "humanoid")]
    #[test]
    fn test_spine_centered_on_midline() {
        let skel = Skeleton::humanoid();
        let positions = skel.compute_world_positions();

        let spine_names = [
            "Pelvis", "L5", "L4", "L3", "L2", "L1",
            "T12", "T11", "T10", "T9", "T8", "T7", "T6", "T5", "T4", "T3", "T2", "T1",
            "C7", "C6", "C5", "C4", "C3", "C2", "C1", "Head",
        ];

        for name in &spine_names {
            if let Some(bone_id) = skel.find_bone(name) {
                let (start, _) = positions[bone_id.0];
                assert!(
                    start[0].abs() < 5.0,
                    "Spine bone {} is off-center: x={:.1}mm",
                    name, start[0]
                );
            }
        }
    }

    /// 脊椎の全高が解剖学的に妥当な範囲（600〜900mm）
    #[cfg(feature = "humanoid")]
    #[test]
    fn test_spine_total_height() {
        let skel = Skeleton::humanoid();
        let positions = skel.compute_world_positions();

        let pelvis = skel.find_bone("Pelvis").unwrap();
        let head = skel.find_bone("Head").unwrap();
        let (pelvis_start, _) = positions[pelvis.0];
        let (_, head_end) = positions[head.0];

        let spine_height = head_end[2] - pelvis_start[2];
        assert!(
            (600.0..=900.0).contains(&spine_height),
            "Spine height {:.0}mm is outside anatomical range 600-900mm",
            spine_height
        );
    }

    // =========================================================================
    // 腕チェーン連続性テスト
    // =========================================================================

    /// 左腕チェーンが隙間なく連結されること
    #[cfg(feature = "humanoid")]
    #[test]
    fn test_left_arm_connected() {
        let skel = Skeleton::humanoid();
        let positions = skel.compute_world_positions();

        let chain = [
            ("UpperArm_L", "Radius_L"),
            ("Radius_L", "MC_Middle_L"),
        ];
        for (parent_name, child_name) in &chain {
            let p = skel.find_bone(parent_name).unwrap();
            let c = skel.find_bone(child_name).unwrap();
            let (_, p_end) = positions[p.0];
            let (c_start, _) = positions[c.0];
            let gap = vec3_norm(vec3_sub(c_start, p_end));
            assert!(gap < 1.0, "{} → {} gap: {:.1}mm", parent_name, child_name, gap);
        }
        // 上腕は肩の外側（X < -100）に配置
        let ua = skel.find_bone("UpperArm_L").unwrap();
        assert!(positions[ua.0].0[0] < -100.0, "UpperArm_L should start left of center");
    }

    /// 右腕チェーンが隙間なく連結されること
    #[cfg(feature = "humanoid")]
    #[test]
    fn test_right_arm_connected() {
        let skel = Skeleton::humanoid();
        let positions = skel.compute_world_positions();

        let chain = [
            ("UpperArm_R", "Radius_R"),
            ("Radius_R", "MC_Middle_R"),
        ];
        for (parent_name, child_name) in &chain {
            let p = skel.find_bone(parent_name).unwrap();
            let c = skel.find_bone(child_name).unwrap();
            let (_, p_end) = positions[p.0];
            let (c_start, _) = positions[c.0];
            let gap = vec3_norm(vec3_sub(c_start, p_end));
            assert!(gap < 1.0, "{} → {} gap: {:.1}mm", parent_name, child_name, gap);
        }
        let ua = skel.find_bone("UpperArm_R").unwrap();
        assert!(positions[ua.0].0[0] > 100.0, "UpperArm_R should start right of center");
    }

    /// 手の骨が前腕の終端近傍にあること
    #[cfg(feature = "humanoid")]
    #[test]
    fn test_hand_bones_near_wrist() {
        let skel = Skeleton::humanoid();
        let positions = skel.compute_world_positions();

        for side in &["L", "R"] {
            let radius_name = format!("Radius_{}", side);
            let mc_middle_name = format!("MC_Middle_{}", side);

            if let (Some(r), Some(mc)) = (skel.find_bone(&radius_name), skel.find_bone(&mc_middle_name)) {
                let (_, radius_end) = positions[r.0];
                let (mc_start, _) = positions[mc.0];
                let dist = vec3_norm(vec3_sub(mc_start, radius_end));
                assert!(
                    dist < 50.0,
                    "MC_Middle_{} is {:.1}mm from Radius_{} end (max 50mm)",
                    side, dist, side
                );
            }
        }
    }

    // =========================================================================
    // 脚チェーン連続性テスト
    // =========================================================================

    /// 脚チェーンが下方向(Z-)に伸びること
    #[cfg(feature = "humanoid")]
    #[test]
    fn test_legs_extend_downward() {
        let skel = Skeleton::humanoid();
        let positions = skel.compute_world_positions();

        for side in &["L", "R"] {
            let chain = [
                format!("Femur_{}", side),
                format!("Tibia_{}", side),
            ];
            let mut prev_z = f32::INFINITY;
            for name in &chain {
                if let Some(id) = skel.find_bone(name) {
                    let (start, end) = positions[id.0];
                    assert!(end[2] < start[2], "{} should point downward", name);
                    assert!(
                        start[2] < prev_z + 10.0,
                        "{} start z={:.1} above previous end z={:.1}",
                        name, start[2], prev_z
                    );
                    prev_z = end[2];
                }
            }
        }
    }

    /// 足のZ座標が地面(0mm)付近であること
    #[cfg(feature = "humanoid")]
    #[test]
    fn test_feet_near_ground() {
        let skel = Skeleton::humanoid();
        let positions = skel.compute_world_positions();

        for side in &["L", "R"] {
            let tibia_name = format!("Tibia_{}", side);
            if let Some(id) = skel.find_bone(&tibia_name) {
                let (_, end) = positions[id.0];
                assert!(
                    end[2] < 250.0,
                    "Tibia_{} end z={:.0}mm — feet should be below 250mm",
                    side, end[2]
                );
                assert!(
                    end[2] > -100.0,
                    "Tibia_{} end z={:.0}mm — feet should not go below ground",
                    side, end[2]
                );
            }
        }
    }

    // =========================================================================
    // 左右対称性テスト
    // =========================================================================

    /// 左右対称ボーンの位置対称性（全主要ペア）
    #[cfg(feature = "humanoid")]
    #[test]
    fn test_left_right_symmetry() {
        let skel = Skeleton::humanoid();
        let positions = skel.compute_world_positions();

        let pairs = [
            ("Femur_L", "Femur_R"),
            ("Tibia_L", "Tibia_R"),
            ("Fibula_L", "Fibula_R"),
            ("UpperArm_L", "UpperArm_R"),
            ("Radius_L", "Radius_R"),
            ("Ulna_L", "Ulna_R"),
            ("Clavicle_L", "Clavicle_R"),
            ("Scapula_L", "Scapula_R"),
            ("MC_Middle_L", "MC_Middle_R"),
        ];

        for (left_name, right_name) in &pairs {
            let left_id = skel.find_bone(left_name);
            let right_id = skel.find_bone(right_name);
            if let (Some(l), Some(r)) = (left_id, right_id) {
                let (l_start, l_end) = positions[l.0];
                let (r_start, r_end) = positions[r.0];

                // X座標が符号反転
                assert!(
                    (l_start[0] + r_start[0]).abs() < 10.0,
                    "{} x={:.1} and {} x={:.1} not X-symmetric",
                    left_name, l_start[0], right_name, r_start[0]
                );

                // Y, Z座標はほぼ同じ
                assert!(
                    (l_start[1] - r_start[1]).abs() < 15.0,
                    "{} and {} Y-mismatch: {:.1} vs {:.1}",
                    left_name, right_name, l_start[1], r_start[1]
                );
                assert!(
                    (l_start[2] - r_start[2]).abs() < 10.0,
                    "{} and {} Z-mismatch: {:.1} vs {:.1}",
                    left_name, right_name, l_start[2], r_start[2]
                );

                // 終端も対称
                assert!(
                    (l_end[0] + r_end[0]).abs() < 10.0,
                    "{} and {} end X-asymmetric: {:.1} vs {:.1}",
                    left_name, right_name, l_end[0], r_end[0]
                );
            }
        }
    }

    // =========================================================================
    // 全体的な不変条件テスト
    // =========================================================================

    /// 全ボーンの長さが正でNaN/Infでないこと
    #[cfg(feature = "humanoid")]
    #[test]
    fn test_all_bone_lengths_positive() {
        let skel = Skeleton::humanoid();
        let positions = skel.compute_world_positions();

        for (i, bone) in skel.bones.iter().enumerate() {
            let (start, end) = positions[i];
            let display_len = vec3_norm(vec3_sub(end, start));
            assert!(
                display_len > 0.1,
                "Bone {} ({}) has near-zero display length: {:.3}mm",
                i, bone.name, display_len
            );
            assert!(
                !display_len.is_nan() && !display_len.is_infinite(),
                "Bone {} ({}) has NaN/Inf display length",
                i, bone.name
            );
        }
    }

    /// ボーンの表示長さがdefinition上の長さと一致すること
    #[cfg(feature = "humanoid")]
    #[test]
    fn test_display_length_matches_definition() {
        let skel = Skeleton::humanoid();
        let positions = skel.compute_world_positions();

        for (i, bone) in skel.bones.iter().enumerate() {
            let (start, end) = positions[i];
            let display_len = vec3_norm(vec3_sub(end, start));
            let def_len = bone.length;
            let error = (display_len - def_len).abs();
            assert!(
                error < 0.1,
                "Bone {} ({}) display_len={:.2} != def_len={:.2} (error={:.2}mm)",
                i, bone.name, display_len, def_len, error
            );
        }
    }

    /// 人体全高が解剖学的に妥当な範囲（1500〜2000mm）
    #[cfg(feature = "humanoid")]
    #[test]
    fn test_body_total_height() {
        let skel = Skeleton::humanoid();
        let positions = skel.compute_world_positions();

        let mut min_z = f32::INFINITY;
        let mut max_z = f32::NEG_INFINITY;
        for (start, end) in &positions {
            min_z = min_z.min(start[2]).min(end[2]);
            max_z = max_z.max(start[2]).max(end[2]);
        }
        let total_height = max_z - min_z;
        assert!(
            (1500.0..=2000.0).contains(&total_height),
            "Total body height {:.0}mm outside anatomical range 1500-2000mm",
            total_height
        );
    }

    /// 左右の上腕開始位置が左右対称に配置されること
    #[cfg(feature = "humanoid")]
    #[test]
    fn test_arm_placement_symmetric() {
        let skel = Skeleton::humanoid();
        let positions = skel.compute_world_positions();

        let ua_l = skel.find_bone("UpperArm_L").unwrap();
        let ua_r = skel.find_bone("UpperArm_R").unwrap();
        let l_start = positions[ua_l.0].0;
        let r_start = positions[ua_r.0].0;
        // X座標が左右対称
        assert!(
            (l_start[0] + r_start[0]).abs() < 5.0,
            "UpperArm L/R X not symmetric: L={:.1}, R={:.1}",
            l_start[0], r_start[0]
        );
        // Z座標がほぼ同じ
        assert!(
            (l_start[2] - r_start[2]).abs() < 5.0,
            "UpperArm L/R Z not equal: L={:.1}, R={:.1}",
            l_start[2], r_start[2]
        );
        // 左右に十分離れている（最低200mm）
        assert!(
            (r_start[0] - l_start[0]).abs() > 200.0,
            "UpperArm L/R too close: L.x={:.1}, R.x={:.1}",
            l_start[0], r_start[0]
        );
    }

    /// 肋骨が胸椎から左右に広がること
    #[cfg(feature = "humanoid")]
    #[test]
    fn test_ribs_spread_laterally() {
        let skel = Skeleton::humanoid();
        let positions = skel.compute_world_positions();

        for rib_num in 1..=12 {
            let left_name = format!("Rib{}_L", rib_num);
            let right_name = format!("Rib{}_R", rib_num);
            if let (Some(l), Some(r)) = (skel.find_bone(&left_name), skel.find_bone(&right_name)) {
                let (_, l_end) = positions[l.0];
                let (_, r_end) = positions[r.0];
                // 左肋骨はX<0、右肋骨はX>0
                assert!(
                    l_end[0] < -10.0,
                    "{} end x={:.1} should be negative",
                    left_name, l_end[0]
                );
                assert!(
                    r_end[0] > 10.0,
                    "{} end x={:.1} should be positive",
                    right_name, r_end[0]
                );
            }
        }
    }

    /// 頭がC1の上に正しく乗っていること
    #[cfg(feature = "humanoid")]
    #[test]
    fn test_head_above_neck() {
        let skel = Skeleton::humanoid();
        let positions = skel.compute_world_positions();

        let c1 = skel.find_bone("C1").unwrap();
        let head = skel.find_bone("Head").unwrap();
        let (_, c1_end) = positions[c1.0];
        let (head_start, head_end) = positions[head.0];

        // 頭はC1の終端近傍から始まる
        let gap = vec3_norm(vec3_sub(head_start, c1_end));
        assert!(gap < 5.0, "Head starts {:.1}mm from C1 end", gap);

        // 頭は上方向に伸びる
        assert!(
            head_end[2] > head_start[2] + 100.0,
            "Head should extend upward: start z={:.0}, end z={:.0}",
            head_start[2], head_end[2]
        );
    }

    // =========================================================================
    // 回帰テスト: parent_end vs parent_start バグの再発防止
    // =========================================================================

    /// 前腕が肩の位置に戻らないこと（parent_start+offsetバグの回帰検出）
    #[cfg(feature = "humanoid")]
    #[test]
    fn test_forearm_not_at_shoulder() {
        let skel = Skeleton::humanoid();
        let positions = skel.compute_world_positions();

        for side in &["L", "R"] {
            let upper = skel.find_bone(&format!("UpperArm_{}", side)).unwrap();
            let radius = skel.find_bone(&format!("Radius_{}", side)).unwrap();
            let (upper_start, upper_end) = positions[upper.0];
            let (radius_start, _) = positions[radius.0];

            // 前腕は上腕の終端付近にあるべき（肩の位置ではない）
            let dist_to_end = vec3_norm(vec3_sub(radius_start, upper_end));
            let dist_to_start = vec3_norm(vec3_sub(radius_start, upper_start));
            assert!(
                dist_to_end < dist_to_start,
                "Radius_{} is closer to UpperArm start ({:.0}mm) than end ({:.0}mm) — regression!",
                side, dist_to_start, dist_to_end
            );
            assert!(
                dist_to_end < 5.0,
                "Radius_{} is {:.1}mm from UpperArm_{} end (expected <5mm)",
                side, dist_to_end, side
            );
        }
    }

    /// 手の骨が肩付近ではなく手首付近にあること
    #[cfg(feature = "humanoid")]
    #[test]
    fn test_hand_not_at_shoulder() {
        let skel = Skeleton::humanoid();
        let positions = skel.compute_world_positions();

        let shoulder_l = skel.find_bone("Clavicle_L").unwrap();
        let mc_l = skel.find_bone("MC_Middle_L").unwrap();
        let (shoulder_start, _) = positions[shoulder_l.0];
        let (mc_start, _) = positions[mc_l.0];

        // 手の骨は肩から400mm以上離れているべき
        let dist = vec3_norm(vec3_sub(mc_start, shoulder_start));
        assert!(
            dist > 400.0,
            "MC_Middle_L is only {:.0}mm from shoulder (expected >400mm)",
            dist
        );
    }
}
