//! キーフレームアニメーションシステム
//!
//! JSON形式でアニメーションクリップを定義し、AI（Opus等）が直接生成可能。
//! ボーン名ベースで柔軟にスケルトンにマッピングする。

use serde::{Deserialize, Serialize};

/// ループモード
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
pub enum LoopMode {
    /// 1回再生で停止
    Once,
    /// ループ再生
    Loop,
    /// 行って戻る（ピンポン）
    PingPong,
}

/// イージング関数
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
pub enum Easing {
    Linear,
    EaseIn,
    EaseOut,
    EaseInOut,
}

impl Easing {
    /// t (0..1) を変換
    pub fn apply(&self, t: f32) -> f32 {
        let t = t.clamp(0.0, 1.0);
        match self {
            Easing::Linear => t,
            Easing::EaseIn => t * t,
            Easing::EaseOut => 1.0 - (1.0 - t) * (1.0 - t),
            Easing::EaseInOut => {
                if t < 0.5 {
                    2.0 * t * t
                } else {
                    1.0 - (-2.0 * t + 2.0).powi(2) / 2.0
                }
            }
        }
    }
}

/// キーフレーム
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Keyframe {
    /// 時刻（秒）
    pub time: f32,
    /// オイラー角 [roll, pitch, yaw] (rad)
    pub rotation: [f32; 3],
    /// イージング関数
    #[serde(default = "default_easing")]
    pub easing: Easing,
    /// 物理モーター剛性（None = ボーンデフォルト値を使用）
    #[serde(default)]
    pub stiffness: Option<f32>,
    /// 物理モーター減衰（None = ボーンデフォルト値を使用）
    #[serde(default)]
    pub damping: Option<f32>,
}

fn default_easing() -> Easing { Easing::EaseInOut }

/// ボーントラック（1ボーン分のアニメーション）
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct BoneTrack {
    /// ボーン名（スケルトンの名前と一致させる）
    pub bone_name: String,
    /// キーフレーム列（時刻昇順）
    pub keyframes: Vec<Keyframe>,
}

/// アニメーションクリップ
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct AnimationClip {
    /// アニメーション名
    pub name: String,
    /// 全体の長さ（秒）
    pub duration: f32,
    /// ループモード
    pub loop_mode: LoopMode,
    /// ボーントラック群
    pub tracks: Vec<BoneTrack>,
}

impl BoneTrack {
    /// 指定時刻での補間回転を取得
    pub fn sample(&self, time: f32) -> [f32; 3] {
        if self.keyframes.is_empty() {
            return [0.0; 3];
        }
        if self.keyframes.len() == 1 {
            return self.keyframes[0].rotation;
        }

        // 最初のキーフレームより前
        if time <= self.keyframes[0].time {
            return self.keyframes[0].rotation;
        }
        // 最後のキーフレームより後
        let last = &self.keyframes[self.keyframes.len() - 1];
        if time >= last.time {
            return last.rotation;
        }

        // 2つのキーフレーム間を補間
        for i in 0..self.keyframes.len() - 1 {
            let kf0 = &self.keyframes[i];
            let kf1 = &self.keyframes[i + 1];
            if time >= kf0.time && time < kf1.time {
                let dt = kf1.time - kf0.time;
                if dt < 0.0001 {
                    return kf0.rotation;
                }
                let t = (time - kf0.time) / dt;
                let t = kf1.easing.apply(t);
                return [
                    kf0.rotation[0] + (kf1.rotation[0] - kf0.rotation[0]) * t,
                    kf0.rotation[1] + (kf1.rotation[1] - kf0.rotation[1]) * t,
                    kf0.rotation[2] + (kf1.rotation[2] - kf0.rotation[2]) * t,
                ];
            }
        }
        last.rotation
    }

    /// 指定時刻での補間回転 + 物理パラメータを取得
    /// 戻り値: (rotation, Option<stiffness>, Option<damping>)
    pub fn sample_with_physics(&self, time: f32) -> ([f32; 3], Option<f32>, Option<f32>) {
        if self.keyframes.is_empty() {
            return ([0.0; 3], None, None);
        }
        if self.keyframes.len() == 1 {
            let kf = &self.keyframes[0];
            return (kf.rotation, kf.stiffness, kf.damping);
        }

        // 最初のキーフレームより前
        if time <= self.keyframes[0].time {
            let kf = &self.keyframes[0];
            return (kf.rotation, kf.stiffness, kf.damping);
        }
        // 最後のキーフレームより後
        let last = &self.keyframes[self.keyframes.len() - 1];
        if time >= last.time {
            return (last.rotation, last.stiffness, last.damping);
        }

        // 2つのキーフレーム間を補間
        for i in 0..self.keyframes.len() - 1 {
            let kf0 = &self.keyframes[i];
            let kf1 = &self.keyframes[i + 1];
            if time >= kf0.time && time < kf1.time {
                let dt = kf1.time - kf0.time;
                if dt < 0.0001 {
                    return (kf0.rotation, kf0.stiffness, kf0.damping);
                }
                let t = (time - kf0.time) / dt;
                let t = kf1.easing.apply(t);
                let rotation = [
                    kf0.rotation[0] + (kf1.rotation[0] - kf0.rotation[0]) * t,
                    kf0.rotation[1] + (kf1.rotation[1] - kf0.rotation[1]) * t,
                    kf0.rotation[2] + (kf1.rotation[2] - kf0.rotation[2]) * t,
                ];
                // stiffness/dampingも補間
                let stiffness = match (kf0.stiffness, kf1.stiffness) {
                    (Some(s0), Some(s1)) => Some(s0 + (s1 - s0) * t),
                    (Some(s), None) | (None, Some(s)) => Some(s),
                    (None, None) => None,
                };
                let damping = match (kf0.damping, kf1.damping) {
                    (Some(d0), Some(d1)) => Some(d0 + (d1 - d0) * t),
                    (Some(d), None) | (None, Some(d)) => Some(d),
                    (None, None) => None,
                };
                return (rotation, stiffness, damping);
            }
        }
        (last.rotation, last.stiffness, last.damping)
    }
}

impl AnimationClip {
    /// 指定時刻での全ボーンの回転を取得
    /// 戻り値: Vec<(bone_name, [roll, pitch, yaw])>
    pub fn sample(&self, time: f32) -> Vec<(&str, [f32; 3])> {
        let effective_time = match self.loop_mode {
            LoopMode::Once => time.min(self.duration),
            LoopMode::Loop => {
                if self.duration > 0.0 { time % self.duration } else { 0.0 }
            }
            LoopMode::PingPong => {
                if self.duration > 0.0 {
                    let cycle = time / self.duration;
                    let phase = cycle % 2.0;
                    if phase < 1.0 {
                        phase * self.duration
                    } else {
                        (2.0 - phase) * self.duration
                    }
                } else {
                    0.0
                }
            }
        };

        self.tracks.iter()
            .map(|track| (track.bone_name.as_str(), track.sample(effective_time)))
            .collect()
    }

    /// 指定時刻での全ボーンの回転 + 物理パラメータを取得
    /// 戻り値: Vec<(bone_name, rotation, Option<stiffness>, Option<damping>)>
    pub fn sample_physics(&self, time: f32) -> Vec<(&str, [f32; 3], Option<f32>, Option<f32>)> {
        let effective_time = match self.loop_mode {
            LoopMode::Once => time.min(self.duration),
            LoopMode::Loop => {
                if self.duration > 0.0 { time % self.duration } else { 0.0 }
            }
            LoopMode::PingPong => {
                if self.duration > 0.0 {
                    let cycle = time / self.duration;
                    let phase = cycle % 2.0;
                    if phase < 1.0 {
                        phase * self.duration
                    } else {
                        (2.0 - phase) * self.duration
                    }
                } else {
                    0.0
                }
            }
        };

        self.tracks.iter()
            .map(|track| {
                let (rot, stiff, damp) = track.sample_with_physics(effective_time);
                (track.bone_name.as_str(), rot, stiff, damp)
            })
            .collect()
    }

    /// JSONからロード
    pub fn from_json(json: &str) -> Result<Self, serde_json::Error> {
        serde_json::from_str(json)
    }

    /// JSONにシリアライズ
    pub fn to_json(&self) -> Result<String, serde_json::Error> {
        serde_json::to_string_pretty(self)
    }

    /// 再生が完了しているか（Once モードのみ）
    pub fn is_finished(&self, time: f32) -> bool {
        self.loop_mode == LoopMode::Once && time >= self.duration
    }

    // === プリセットモーション ===
    //
    // 回転軸の規約（Rapier3D SphericalJoint, capsule_y ボーン）:
    //   AngX = twist（ボーン長軸まわりのねじり）
    //   AngY = swing（横方向の曲げ）
    //   AngZ = swing（前後方向の曲げ）
    //
    // ポーズスライダー（RPMモデルで動作確認済み）の対応:
    //   腕前後: AngX    腕上げ: AngZ(負=上)
    //   脚上げ: AngX    頭うなずき: AngX    頭左右: AngY
    //
    // ボーン名は Mixamo/RPM 直接名を使用（find_bone のエイリアスでanatomical nameからも解決可）

    /// グーパー（右手）: 指を握って開く。1.5s Loop
    pub fn fist_pump_r() -> Self {
        // Mixamo/RPM の指ボーン名を直接使用
        let finger_names = [
            // 人差し指
            "RightHandIndex1", "RightHandIndex2", "RightHandIndex3",
            // 中指
            "RightHandMiddle1", "RightHandMiddle2", "RightHandMiddle3",
            // 薬指
            "RightHandRing1", "RightHandRing2", "RightHandRing3",
            // 小指
            "RightHandPinky1", "RightHandPinky2", "RightHandPinky3",
            // 親指
            "RightHandThumb1", "RightHandThumb2", "RightHandThumb3",
        ];
        let tracks: Vec<BoneTrack> = finger_names.iter().map(|name| {
            // 基節（1）は屈曲大、中節（2）/末節（3）はやや小さく
            let flex = if name.ends_with('1') { 1.2 }
                else if name.ends_with('2') { 1.0 }
                else { 0.8 };
            // 指屈曲は AngZ（ボーンY軸に垂直な前後スイング）
            BoneTrack {
                bone_name: name.to_string(),
                keyframes: vec![
                    Keyframe { time: 0.0, rotation: [0.0, 0.0, 0.0], easing: Easing::EaseOut,
                               stiffness: Some(200.0), damping: Some(20.0) },
                    Keyframe { time: 0.3, rotation: [0.0, 0.0, flex], easing: Easing::EaseIn,
                               stiffness: Some(300.0), damping: Some(30.0) },
                    Keyframe { time: 0.8, rotation: [0.0, 0.0, flex], easing: Easing::EaseInOut,
                               stiffness: Some(300.0), damping: Some(30.0) },
                    Keyframe { time: 1.2, rotation: [0.0, 0.0, 0.0], easing: Easing::EaseOut,
                               stiffness: Some(100.0), damping: Some(15.0) },
                    Keyframe { time: 1.5, rotation: [0.0, 0.0, 0.0], easing: Easing::EaseInOut,
                               stiffness: Some(200.0), damping: Some(20.0) },
                ],
            }
        }).collect();

        AnimationClip {
            name: "グーパー（右手）".into(),
            duration: 1.5,
            loop_mode: LoopMode::Loop,
            tracks,
        }
    }

    /// 手振り（右手）: 腕を上げて手首を左右に振る。3s Once
    pub fn wave_hello_r() -> Self {
        let mut tracks = vec![
            // 右上腕: 横に上げる（AngZ負=上、ポーズスライダー準拠）
            BoneTrack {
                bone_name: "RightArm".into(),
                keyframes: vec![
                    Keyframe { time: 0.0, rotation: [0.0, 0.0, 0.0], easing: Easing::EaseInOut,
                               stiffness: Some(300.0), damping: Some(30.0) },
                    Keyframe { time: 0.5, rotation: [0.3, 0.0, -1.5], easing: Easing::EaseOut,
                               stiffness: Some(400.0), damping: Some(40.0) },
                    Keyframe { time: 2.5, rotation: [0.3, 0.0, -1.5], easing: Easing::EaseInOut,
                               stiffness: Some(400.0), damping: Some(40.0) },
                    Keyframe { time: 3.0, rotation: [0.0, 0.0, 0.0], easing: Easing::EaseIn,
                               stiffness: Some(200.0), damping: Some(25.0) },
                ],
            },
            // 右前腕: 肘を曲げる（AngX=屈曲）
            BoneTrack {
                bone_name: "RightForeArm".into(),
                keyframes: vec![
                    Keyframe { time: 0.0, rotation: [0.0, 0.0, 0.0], easing: Easing::EaseInOut,
                               stiffness: Some(300.0), damping: Some(30.0) },
                    Keyframe { time: 0.4, rotation: [0.0, 0.0, 1.2], easing: Easing::EaseOut,
                               stiffness: Some(300.0), damping: Some(30.0) },
                    Keyframe { time: 2.5, rotation: [0.0, 0.0, 1.2], easing: Easing::EaseInOut,
                               stiffness: Some(300.0), damping: Some(30.0) },
                    Keyframe { time: 3.0, rotation: [0.0, 0.0, 0.0], easing: Easing::EaseIn,
                               stiffness: Some(200.0), damping: Some(25.0) },
                ],
            },
        ];

        // 手首左右振り（3往復、0.5s〜2.5s）— AngY=左右スイング
        let wave_kfs: Vec<Keyframe> = {
            let mut kfs = vec![
                Keyframe { time: 0.0, rotation: [0.0, 0.0, 0.0], easing: Easing::EaseInOut,
                           stiffness: Some(200.0), damping: Some(20.0) },
            ];
            let wave_start = 0.5;
            let wave_period = 2.0 / 3.0; // 3往復 in 2s
            for i in 0..6 {
                let t = wave_start + wave_period * 0.5 * i as f32;
                let angle = if i % 2 == 0 { 0.5 } else { -0.5 };
                kfs.push(Keyframe {
                    time: t, rotation: [0.0, angle, 0.0], easing: Easing::EaseInOut,
                    stiffness: Some(200.0), damping: Some(20.0),
                });
            }
            kfs.push(Keyframe {
                time: 3.0, rotation: [0.0, 0.0, 0.0], easing: Easing::EaseIn,
                stiffness: Some(150.0), damping: Some(18.0),
            });
            kfs
        };
        tracks.push(BoneTrack {
            bone_name: "RightHand".into(),
            keyframes: wave_kfs,
        });

        AnimationClip {
            name: "手振り（右手）".into(),
            duration: 3.0,
            loop_mode: LoopMode::Once,
            tracks,
        }
    }

    /// うなずき: 頭を前後に動かす。1s PingPong
    /// AngX = うなずき（ポーズスライダー head_nod 準拠）
    pub fn nod_yes() -> Self {
        AnimationClip {
            name: "うなずき".into(),
            duration: 1.0,
            loop_mode: LoopMode::PingPong,
            tracks: vec![
                BoneTrack {
                    bone_name: "Head".into(),
                    keyframes: vec![
                        Keyframe { time: 0.0, rotation: [0.0, 0.0, 0.0], easing: Easing::EaseInOut,
                                   stiffness: Some(300.0), damping: Some(30.0) },
                        Keyframe { time: 0.5, rotation: [0.35, 0.0, 0.0], easing: Easing::EaseInOut,
                                   stiffness: Some(400.0), damping: Some(35.0) },
                        Keyframe { time: 1.0, rotation: [0.0, 0.0, 0.0], easing: Easing::EaseInOut,
                                   stiffness: Some(300.0), damping: Some(30.0) },
                    ],
                },
                BoneTrack {
                    bone_name: "Neck".into(),
                    keyframes: vec![
                        Keyframe { time: 0.0, rotation: [0.0, 0.0, 0.0], easing: Easing::EaseInOut,
                                   stiffness: Some(250.0), damping: Some(25.0) },
                        Keyframe { time: 0.5, rotation: [0.15, 0.0, 0.0], easing: Easing::EaseInOut,
                                   stiffness: Some(300.0), damping: Some(28.0) },
                        Keyframe { time: 1.0, rotation: [0.0, 0.0, 0.0], easing: Easing::EaseInOut,
                                   stiffness: Some(250.0), damping: Some(25.0) },
                    ],
                },
            ],
        }
    }

    /// 首振り（いいえ）: 頭を左右に振る。1s PingPong
    /// AngY = 左右回転（ポーズスライダー head_turn 準拠）
    pub fn shake_no() -> Self {
        AnimationClip {
            name: "首振り".into(),
            duration: 1.0,
            loop_mode: LoopMode::PingPong,
            tracks: vec![
                BoneTrack {
                    bone_name: "Head".into(),
                    keyframes: vec![
                        Keyframe { time: 0.0, rotation: [0.0, 0.0, 0.0], easing: Easing::EaseInOut,
                                   stiffness: Some(300.0), damping: Some(30.0) },
                        Keyframe { time: 0.25, rotation: [0.0, 0.4, 0.0], easing: Easing::EaseInOut,
                                   stiffness: Some(350.0), damping: Some(32.0) },
                        Keyframe { time: 0.75, rotation: [0.0, -0.4, 0.0], easing: Easing::EaseInOut,
                                   stiffness: Some(350.0), damping: Some(32.0) },
                        Keyframe { time: 1.0, rotation: [0.0, 0.0, 0.0], easing: Easing::EaseInOut,
                                   stiffness: Some(300.0), damping: Some(30.0) },
                    ],
                },
                BoneTrack {
                    bone_name: "Neck".into(),
                    keyframes: vec![
                        Keyframe { time: 0.0, rotation: [0.0, 0.0, 0.0], easing: Easing::EaseInOut,
                                   stiffness: Some(250.0), damping: Some(25.0) },
                        Keyframe { time: 0.25, rotation: [0.0, 0.15, 0.0], easing: Easing::EaseInOut,
                                   stiffness: Some(280.0), damping: Some(27.0) },
                        Keyframe { time: 0.75, rotation: [0.0, -0.15, 0.0], easing: Easing::EaseInOut,
                                   stiffness: Some(280.0), damping: Some(27.0) },
                        Keyframe { time: 1.0, rotation: [0.0, 0.0, 0.0], easing: Easing::EaseInOut,
                                   stiffness: Some(250.0), damping: Some(25.0) },
                    ],
                },
            ],
        }
    }

    /// 膝屈曲トラック生成（遊脚期にのみ曲がる）
    fn knee_track(bone: &str, dur: f32, amplitude: f32, phase: f32) -> BoneTrack {
        let mut kfs = Vec::new();
        for i in 0..=8 {
            let t = dur * i as f32 / 8.0;
            let p = 2.0 * std::f32::consts::PI * (t / dur + phase);
            let bend = amplitude * (p + std::f32::consts::FRAC_PI_2).sin().max(0.0);
            kfs.push(Keyframe {
                time: t, rotation: [bend, 0.0, 0.0],
                easing: Easing::Linear, stiffness: None, damping: None,
            });
        }
        BoneTrack { bone_name: bone.into(), keyframes: kfs }
    }

    /// 複数軸sin波トラック（1ボーンに複数軸を同時に適用）
    fn multi_axis_track(
        bone: &str, dur: f32,
        axes: &[(usize, f32, f32)], // (axis, amplitude, phase)
    ) -> BoneTrack {
        let mut kfs = Vec::new();
        for i in 0..=8 {
            let t = dur * i as f32 / 8.0;
            let mut rot = [0.0_f32; 3];
            for &(axis, amp, phase) in axes {
                rot[axis] += amp * (2.0 * std::f32::consts::PI * (t / dur + phase)).sin();
            }
            kfs.push(Keyframe {
                time: t, rotation: rot,
                easing: Easing::Linear, stiffness: None, damping: None,
            });
        }
        BoneTrack { bone_name: bone.into(), keyframes: kfs }
    }

    /// 定常オフセット + sin波トラック
    fn offset_sin_track(
        bone: &str, dur: f32,
        offset: [f32; 3],
        axes: &[(usize, f32, f32)],
    ) -> BoneTrack {
        let mut kfs = Vec::new();
        for i in 0..=8 {
            let t = dur * i as f32 / 8.0;
            let mut rot = offset;
            for &(axis, amp, phase) in axes {
                rot[axis] += amp * (2.0 * std::f32::consts::PI * (t / dur + phase)).sin();
            }
            kfs.push(Keyframe {
                time: t, rotation: rot,
                easing: Easing::Linear, stiffness: None, damping: None,
            });
        }
        BoneTrack { bone_name: bone.into(), keyframes: kfs }
    }

    /// sin波トラック（単軸）
    fn sin_track(bone: &str, dur: f32, axis: usize, amp: f32, phase: f32) -> BoneTrack {
        Self::multi_axis_track(bone, dur, &[(axis, amp, phase)])
    }

    /// 歩行サイクル: 1.0sループ
    pub fn walk_cycle() -> Self {
        let d = 1.0_f32;
        let tracks = vec![
            // 脚: 前後スイング
            Self::sin_track("LeftUpLeg", d, 0, 0.45, 0.0),
            Self::sin_track("RightUpLeg", d, 0, 0.45, 0.5),
            // 膝: 遊脚期に屈曲
            Self::knee_track("LeftLeg", d, 0.8, 0.0),
            Self::knee_track("RightLeg", d, 0.8, 0.5),
            // 足首
            Self::sin_track("LeftFoot", d, 0, -0.15, 0.25),
            Self::sin_track("RightFoot", d, 0, -0.15, 0.75),
            // 腕: 控えめな自然な振り
            Self::sin_track("LeftArm", d, 0, -0.25, 0.5),
            Self::sin_track("RightArm", d, 0, -0.25, 0.0),
            Self::sin_track("LeftForeArm", d, 0, 0.15, 0.5),
            Self::sin_track("RightForeArm", d, 0, 0.15, 0.0),
            // 体幹は最小限（脚と腕で歩行感を出す）
            Self::multi_axis_track("Hips", d, &[
                (2, 0.02, 0.25),  // 微小な左右傾きのみ
            ]),
        ];
        AnimationClip { name: "歩行".into(), duration: d, loop_mode: LoopMode::Loop, tracks }
    }

    /// 走行サイクル: 0.6sループ、前傾+大きな動き
    pub fn run_cycle() -> Self {
        let d = 0.6_f32;
        let tracks = vec![
            // 脚: 大きなスイング
            Self::sin_track("LeftUpLeg", d, 0, 0.7, 0.0),
            Self::sin_track("RightUpLeg", d, 0, 0.7, 0.5),
            // 膝: 深く曲がる
            Self::knee_track("LeftLeg", d, 1.2, 0.0),
            Self::knee_track("RightLeg", d, 1.2, 0.5),
            // 足首
            Self::sin_track("LeftFoot", d, 0, -0.2, 0.25),
            Self::sin_track("RightFoot", d, 0, -0.2, 0.75),
            // 腕: 走行時は歩行より大きめ
            Self::sin_track("LeftArm", d, 0, -0.4, 0.5),
            Self::sin_track("RightArm", d, 0, -0.4, 0.0),
            Self::sin_track("LeftForeArm", d, 0, 0.3, 0.5),
            Self::sin_track("RightForeArm", d, 0, 0.3, 0.0),
            // 体幹: 微小な前傾 + 左右傾き
            Self::offset_sin_track("Hips", d,
                [0.08, 0.0, 0.0], // 前傾
                &[
                    (2, 0.03, 0.25),  // 左右
                ],
            ),
        ];
        AnimationClip { name: "走行".into(), duration: d, loop_mode: LoopMode::Loop, tracks }
    }
}

/// アニメーションプレイヤー
pub struct AnimationPlayer {
    /// 現在再生中のクリップ
    clip: Option<AnimationClip>,
    /// 再生時刻
    time: f32,
    /// ブレンドウェイト (0.0=無効, 1.0=フル適用)
    pub blend_weight: f32,
    /// 再生中か
    playing: bool,
}

impl AnimationPlayer {
    pub fn new() -> Self {
        Self {
            clip: None,
            time: 0.0,
            blend_weight: 1.0,
            playing: false,
        }
    }

    /// アニメーション再生開始
    pub fn play(&mut self, clip: AnimationClip) {
        self.clip = Some(clip);
        self.time = 0.0;
        self.playing = true;
    }

    /// 停止
    pub fn stop(&mut self) {
        self.playing = false;
        self.time = 0.0;
    }

    /// 時間更新
    pub fn update(&mut self, dt: f32) {
        if !self.playing { return; }
        self.time += dt;

        if let Some(clip) = &self.clip {
            if clip.is_finished(self.time) {
                self.playing = false;
            }
        }
    }

    /// 現在のボーン回転をサンプリング
    /// 戻り値: Vec<(bone_name, weighted_rotation)>
    pub fn sample(&self) -> Vec<(&str, [f32; 3])> {
        if !self.playing || self.blend_weight < 0.001 {
            return Vec::new();
        }
        if let Some(clip) = &self.clip {
            let samples = clip.sample(self.time);
            if (self.blend_weight - 1.0).abs() < 0.001 {
                samples
            } else {
                samples.into_iter().map(|(name, rot)| {
                    let w = self.blend_weight;
                    (name, [rot[0] * w, rot[1] * w, rot[2] * w])
                }).collect()
            }
        } else {
            Vec::new()
        }
    }

    /// 物理パラメータ付きサンプリング
    /// 戻り値: Vec<(bone_name, weighted_rotation, Option<stiffness>, Option<damping>)>
    pub fn sample_physics(&self) -> Vec<(&str, [f32; 3], Option<f32>, Option<f32>)> {
        if !self.playing || self.blend_weight < 0.001 {
            return Vec::new();
        }
        if let Some(clip) = &self.clip {
            let samples = clip.sample_physics(self.time);
            if (self.blend_weight - 1.0).abs() < 0.001 {
                samples
            } else {
                let w = self.blend_weight;
                samples.into_iter().map(|(name, rot, stiff, damp)| {
                    (name, [rot[0] * w, rot[1] * w, rot[2] * w], stiff, damp)
                }).collect()
            }
        } else {
            Vec::new()
        }
    }

    pub fn is_playing(&self) -> bool {
        self.playing
    }

    pub fn current_clip_name(&self) -> Option<&str> {
        self.clip.as_ref().map(|c| c.name.as_str())
    }

    pub fn current_time(&self) -> f32 {
        self.time
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn make_test_clip() -> AnimationClip {
        AnimationClip {
            name: "test_wave".into(),
            duration: 2.0,
            loop_mode: LoopMode::Loop,
            tracks: vec![
                BoneTrack {
                    bone_name: "UpperArm_L".into(),
                    keyframes: vec![
                        Keyframe { time: 0.0, rotation: [0.0, 0.0, 0.0], easing: Easing::Linear, stiffness: None, damping: None },
                        Keyframe { time: 1.0, rotation: [0.0, -1.5, 0.0], easing: Easing::EaseInOut, stiffness: None, damping: None },
                        Keyframe { time: 2.0, rotation: [0.0, 0.0, 0.0], easing: Easing::EaseInOut, stiffness: None, damping: None },
                    ],
                },
            ],
        }
    }

    #[test]
    fn test_animation_sample_interpolation() {
        let clip = make_test_clip();
        let samples = clip.sample(0.5);
        assert_eq!(samples.len(), 1);
        // t=0.5 で UpperArm_L.pitch は 0.0 と -1.5 の間
        let (_, rot) = &samples[0];
        assert!(rot[1] < -0.1 && rot[1] > -1.5, "Expected interpolated, got {:?}", rot);
    }

    #[test]
    fn test_animation_loop() {
        let clip = make_test_clip();
        // t=2.5 → looped to t=0.5
        let samples = clip.sample(2.5);
        let (_, rot) = &samples[0];
        assert!(rot[1] < -0.1, "Should be in motion at looped time");
    }

    #[test]
    fn test_animation_once_finished() {
        let clip = AnimationClip {
            name: "bow".into(),
            duration: 1.0,
            loop_mode: LoopMode::Once,
            tracks: vec![
                BoneTrack {
                    bone_name: "T6".into(),
                    keyframes: vec![
                        Keyframe { time: 0.0, rotation: [0.0, 0.0, 0.0], easing: Easing::Linear, stiffness: None, damping: None },
                        Keyframe { time: 1.0, rotation: [0.0, 0.5, 0.0], easing: Easing::EaseOut, stiffness: None, damping: None },
                    ],
                },
            ],
        };
        assert!(!clip.is_finished(0.5));
        assert!(clip.is_finished(1.5));
    }

    #[test]
    fn test_animation_json_roundtrip() {
        let clip = make_test_clip();
        let json = clip.to_json().unwrap();
        let parsed = AnimationClip::from_json(&json).unwrap();
        assert_eq!(parsed.name, "test_wave");
        assert_eq!(parsed.tracks.len(), 1);
        assert_eq!(parsed.tracks[0].keyframes.len(), 3);
    }

    #[test]
    fn test_animation_player() {
        let mut player = AnimationPlayer::new();
        assert!(!player.is_playing());

        player.play(make_test_clip());
        assert!(player.is_playing());

        player.update(0.5);
        let samples = player.sample();
        assert!(!samples.is_empty());

        player.stop();
        assert!(!player.is_playing());
        let samples = player.sample();
        assert!(samples.is_empty());
    }

    #[test]
    fn test_easing_functions() {
        assert!((Easing::Linear.apply(0.5) - 0.5).abs() < 0.001);
        assert!(Easing::EaseIn.apply(0.5) < 0.5); // starts slow
        assert!(Easing::EaseOut.apply(0.5) > 0.5); // starts fast
        assert!((Easing::EaseInOut.apply(0.0)).abs() < 0.001);
        assert!((Easing::EaseInOut.apply(1.0) - 1.0).abs() < 0.001);
    }

    #[test]
    fn test_pingpong_mode() {
        let clip = AnimationClip {
            name: "nod".into(),
            duration: 1.0,
            loop_mode: LoopMode::PingPong,
            tracks: vec![
                BoneTrack {
                    bone_name: "Head".into(),
                    keyframes: vec![
                        Keyframe { time: 0.0, rotation: [0.0, 0.0, 0.0], easing: Easing::Linear, stiffness: None, damping: None },
                        Keyframe { time: 1.0, rotation: [0.0, 0.3, 0.0], easing: Easing::Linear, stiffness: None, damping: None },
                    ],
                },
            ],
        };
        // t=0.5 → forward phase, pitch ~0.15
        let s1 = clip.sample(0.5);
        assert!((s1[0].1[1] - 0.15).abs() < 0.05);
        // t=1.5 → reverse phase, pitch ~0.15 (going back)
        let s2 = clip.sample(1.5);
        assert!((s2[0].1[1] - 0.15).abs() < 0.05);
    }

    #[test]
    fn test_sample_physics_stiffness() {
        let track = BoneTrack {
            bone_name: "Test".into(),
            keyframes: vec![
                Keyframe { time: 0.0, rotation: [0.0, 0.0, 0.0], easing: Easing::Linear,
                           stiffness: Some(100.0), damping: Some(10.0) },
                Keyframe { time: 1.0, rotation: [1.0, 0.0, 0.0], easing: Easing::Linear,
                           stiffness: Some(300.0), damping: Some(30.0) },
            ],
        };
        let (rot, stiff, damp) = track.sample_with_physics(0.5);
        // rotation: 0.5
        assert!((rot[0] - 0.5).abs() < 0.05);
        // stiffness: 100 + (300-100)*0.5 = 200
        assert!((stiff.unwrap() - 200.0).abs() < 5.0);
        // damping: 10 + (30-10)*0.5 = 20
        assert!((damp.unwrap() - 20.0).abs() < 2.0);
    }

    #[test]
    fn test_sample_physics_none_fallback() {
        let track = BoneTrack {
            bone_name: "Test".into(),
            keyframes: vec![
                Keyframe { time: 0.0, rotation: [0.0, 0.0, 0.0], easing: Easing::Linear,
                           stiffness: None, damping: None },
                Keyframe { time: 1.0, rotation: [1.0, 0.0, 0.0], easing: Easing::Linear,
                           stiffness: None, damping: None },
            ],
        };
        let (_rot, stiff, damp) = track.sample_with_physics(0.5);
        assert!(stiff.is_none());
        assert!(damp.is_none());
    }

    #[test]
    fn test_fist_pump_preset_valid() {
        let clip = AnimationClip::fist_pump_r();
        assert_eq!(clip.loop_mode, LoopMode::Loop);
        assert!((clip.duration - 1.5).abs() < 0.01);
        assert!(!clip.tracks.is_empty());
        // 全トラックのキーフレームが時刻昇順
        for track in &clip.tracks {
            assert!(track.keyframes.len() >= 2);
            for w in track.keyframes.windows(2) {
                assert!(w[1].time >= w[0].time, "keyframes not sorted: {} >= {}", w[1].time, w[0].time);
            }
        }
        // sample_physics動作確認
        let samples = clip.sample_physics(0.5);
        assert!(!samples.is_empty());
        // 少なくとも1つはstiffness指定がある
        assert!(samples.iter().any(|(_, _, s, _)| s.is_some()));
    }

    #[test]
    fn test_wave_hello_preset_valid() {
        let clip = AnimationClip::wave_hello_r();
        assert_eq!(clip.loop_mode, LoopMode::Once);
        assert!((clip.duration - 3.0).abs() < 0.01);
        assert!(clip.tracks.len() >= 3); // 上腕 + 前腕 + 手首
    }

    #[test]
    fn test_nod_shake_presets_valid() {
        let nod = AnimationClip::nod_yes();
        assert_eq!(nod.loop_mode, LoopMode::PingPong);
        assert!((nod.duration - 1.0).abs() < 0.01);

        let shake = AnimationClip::shake_no();
        assert_eq!(shake.loop_mode, LoopMode::PingPong);
        assert!((shake.duration - 1.0).abs() < 0.01);
    }

    #[test]
    fn test_json_roundtrip_with_physics() {
        let clip = AnimationClip {
            name: "test_phys".into(),
            duration: 1.0,
            loop_mode: LoopMode::Once,
            tracks: vec![
                BoneTrack {
                    bone_name: "Head".into(),
                    keyframes: vec![
                        Keyframe { time: 0.0, rotation: [0.0, 0.0, 0.0], easing: Easing::Linear,
                                   stiffness: Some(200.0), damping: Some(25.0) },
                        Keyframe { time: 1.0, rotation: [0.5, 0.0, 0.0], easing: Easing::EaseOut,
                                   stiffness: None, damping: Some(10.0) },
                    ],
                },
            ],
        };
        let json = clip.to_json().unwrap();
        let parsed = AnimationClip::from_json(&json).unwrap();
        assert_eq!(parsed.tracks[0].keyframes[0].stiffness, Some(200.0));
        assert_eq!(parsed.tracks[0].keyframes[0].damping, Some(25.0));
        assert_eq!(parsed.tracks[0].keyframes[1].stiffness, None);
        assert_eq!(parsed.tracks[0].keyframes[1].damping, Some(10.0));
    }

    #[test]
    fn test_json_backward_compat() {
        // stiffness/damping無しのJSON（旧形式）がパースできることを確認
        let json = r#"{
            "name": "old_clip",
            "duration": 1.0,
            "loop_mode": "Once",
            "tracks": [{
                "bone_name": "Head",
                "keyframes": [
                    {"time": 0.0, "rotation": [0.0, 0.0, 0.0]},
                    {"time": 1.0, "rotation": [0.5, 0.0, 0.0]}
                ]
            }]
        }"#;
        let clip = AnimationClip::from_json(json).unwrap();
        assert_eq!(clip.tracks[0].keyframes[0].stiffness, None);
        assert_eq!(clip.tracks[0].keyframes[0].damping, None);
    }
}
