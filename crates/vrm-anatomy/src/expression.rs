//! VRM 表情（Expression / BlendShape）パース
//!
//! VRM 1.0 の `VRMC_vrm.expressions` と VRM 0.x の `VRM.blendShapeMaster` を読み取り、
//! preset 名・MorphTarget bind・override 挙動を共通モデルに正規化する。
//!
//! レンダラ非依存：morph weight をどう適用するかは consumer 側の責務。

use std::collections::HashMap;

/// VRM 1.0 標準 preset 名（VRM 0.x は内部で 1.0 名に正規化）。
///
/// `aa` / `ih` / `ou` / `ee` / `oh` は viseme（母音口形）。
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum ExpressionPreset {
    Happy,
    Angry,
    Sad,
    Relaxed,
    Surprised,
    Neutral,
    Aa,
    Ih,
    Ou,
    Ee,
    Oh,
    Blink,
    BlinkLeft,
    BlinkRight,
    LookUp,
    LookDown,
    LookLeft,
    LookRight,
}

impl ExpressionPreset {
    pub fn as_str(self) -> &'static str {
        match self {
            ExpressionPreset::Happy => "happy",
            ExpressionPreset::Angry => "angry",
            ExpressionPreset::Sad => "sad",
            ExpressionPreset::Relaxed => "relaxed",
            ExpressionPreset::Surprised => "surprised",
            ExpressionPreset::Neutral => "neutral",
            ExpressionPreset::Aa => "aa",
            ExpressionPreset::Ih => "ih",
            ExpressionPreset::Ou => "ou",
            ExpressionPreset::Ee => "ee",
            ExpressionPreset::Oh => "oh",
            ExpressionPreset::Blink => "blink",
            ExpressionPreset::BlinkLeft => "blinkLeft",
            ExpressionPreset::BlinkRight => "blinkRight",
            ExpressionPreset::LookUp => "lookUp",
            ExpressionPreset::LookDown => "lookDown",
            ExpressionPreset::LookLeft => "lookLeft",
            ExpressionPreset::LookRight => "lookRight",
        }
    }

    /// VRM 1.0 / 0.x の preset 名（lowercase 揺れ含む）から enum を解決。
    pub fn from_any(name: &str) -> Option<Self> {
        let lc = name.to_ascii_lowercase();
        Some(match lc.as_str() {
            "happy" | "joy" => ExpressionPreset::Happy,
            "angry" => ExpressionPreset::Angry,
            "sad" | "sorrow" => ExpressionPreset::Sad,
            "relaxed" | "fun" => ExpressionPreset::Relaxed,
            "surprised" => ExpressionPreset::Surprised,
            "neutral" => ExpressionPreset::Neutral,
            "aa" | "a" => ExpressionPreset::Aa,
            "ih" | "i" => ExpressionPreset::Ih,
            "ou" | "u" => ExpressionPreset::Ou,
            "ee" | "e" => ExpressionPreset::Ee,
            "oh" | "o" => ExpressionPreset::Oh,
            "blink" => ExpressionPreset::Blink,
            "blinkleft" | "blink_l" => ExpressionPreset::BlinkLeft,
            "blinkright" | "blink_r" => ExpressionPreset::BlinkRight,
            "lookup" => ExpressionPreset::LookUp,
            "lookdown" => ExpressionPreset::LookDown,
            "lookleft" => ExpressionPreset::LookLeft,
            "lookright" => ExpressionPreset::LookRight,
            _ => return None,
        })
    }
}

/// override の挙動（他の表情を抑制するか）。VRM 1.0 規格準拠。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum OverrideMode {
    None,
    Block,
    Blend,
}

impl OverrideMode {
    fn from_str(s: &str) -> Self {
        match s {
            "block" => OverrideMode::Block,
            "blend" => OverrideMode::Blend,
            _ => OverrideMode::None,
        }
    }
}

/// MorphTarget への weight 適用。
///
/// VRM 1.0: `{node, index, weight}` の node は glTF nodes 配列の index。
/// VRM 0.x: `{mesh, index, weight}` の mesh は glTF meshes 配列の index。
/// 内部表現では VRM 0.x の mesh index も `target` に格納し、`addressing` で区別する。
#[derive(Debug, Clone)]
pub struct MorphTargetBind {
    pub addressing: MorphAddressing,
    /// VRM 1.0: glTF node index / VRM 0.x: glTF mesh index
    pub target: usize,
    /// メッシュ primitives.targets 配列の index
    pub index: usize,
    /// 0.0 〜 1.0 に正規化済み（VRM 0.x の 0-100 は読込時に /100 する）
    pub weight: f32,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MorphAddressing {
    /// VRM 1.0: target は glTF node index
    NodeIndex,
    /// VRM 0.x: target は glTF mesh index
    MeshIndex,
}

/// 1 つの表情定義。
#[derive(Debug, Clone)]
pub struct Expression {
    /// preset 名（標準 18 種のいずれか）または custom
    pub kind: ExpressionKind,
    pub morph_target_binds: Vec<MorphTargetBind>,
    /// アクティブ時に 0/1 にスナップする（中間値を取らない）
    pub is_binary: bool,
    pub override_blink: OverrideMode,
    pub override_look_at: OverrideMode,
    pub override_mouth: OverrideMode,
}

#[derive(Debug, Clone)]
pub enum ExpressionKind {
    Preset(ExpressionPreset),
    Custom(String),
}

/// VRM モデル 1 体分の表情コレクション。
#[derive(Debug, Clone, Default)]
pub struct Expressions {
    pub presets: HashMap<ExpressionPreset, Expression>,
    pub custom: HashMap<String, Expression>,
}

impl Expressions {
    pub fn get_preset(&self, preset: ExpressionPreset) -> Option<&Expression> {
        self.presets.get(&preset)
    }

    pub fn get_custom(&self, name: &str) -> Option<&Expression> {
        self.custom.get(name)
    }
}

/// VRM 1.0 (`VRMC_vrm.expressions`) パース。
pub fn parse_vrmc_expressions(extensions: &serde_json::Value) -> Option<Expressions> {
    let vrmc = extensions.get("VRMC_vrm")?;
    let expressions = vrmc.get("expressions")?;
    let mut out = Expressions::default();

    if let Some(presets) = expressions.get("preset").and_then(|v| v.as_object()) {
        for (name, body) in presets {
            let Some(preset) = ExpressionPreset::from_any(name) else { continue };
            let expr = parse_vrm1_expression_body(body, ExpressionKind::Preset(preset));
            out.presets.insert(preset, expr);
        }
    }

    if let Some(customs) = expressions.get("custom").and_then(|v| v.as_object()) {
        for (name, body) in customs {
            let expr = parse_vrm1_expression_body(body, ExpressionKind::Custom(name.clone()));
            out.custom.insert(name.clone(), expr);
        }
    }

    if out.presets.is_empty() && out.custom.is_empty() {
        return None;
    }
    Some(out)
}

fn parse_vrm1_expression_body(body: &serde_json::Value, kind: ExpressionKind) -> Expression {
    let binds = body
        .get("morphTargetBinds")
        .and_then(|v| v.as_array())
        .map(|arr| {
            arr.iter()
                .filter_map(|b| {
                    Some(MorphTargetBind {
                        addressing: MorphAddressing::NodeIndex,
                        target: b.get("node")?.as_u64()? as usize,
                        index: b.get("index")?.as_u64()? as usize,
                        weight: b.get("weight").and_then(|w| w.as_f64()).unwrap_or(1.0) as f32,
                    })
                })
                .collect()
        })
        .unwrap_or_default();

    Expression {
        kind,
        morph_target_binds: binds,
        is_binary: body.get("isBinary").and_then(|v| v.as_bool()).unwrap_or(false),
        override_blink: body
            .get("overrideBlink")
            .and_then(|v| v.as_str())
            .map(OverrideMode::from_str)
            .unwrap_or(OverrideMode::None),
        override_look_at: body
            .get("overrideLookAt")
            .and_then(|v| v.as_str())
            .map(OverrideMode::from_str)
            .unwrap_or(OverrideMode::None),
        override_mouth: body
            .get("overrideMouth")
            .and_then(|v| v.as_str())
            .map(OverrideMode::from_str)
            .unwrap_or(OverrideMode::None),
    }
}

/// VRM 0.x (`VRM.blendShapeMaster`) パース。weight は /100 して 0..1 に正規化。
pub fn parse_vrm0_blend_shapes(extensions: &serde_json::Value) -> Option<Expressions> {
    let vrm = extensions.get("VRM")?;
    let master = vrm.get("blendShapeMaster")?;
    let groups = master.get("blendShapeGroups")?.as_array()?;
    let mut out = Expressions::default();

    for group in groups {
        let name = group.get("name").and_then(|v| v.as_str()).unwrap_or("");
        let preset_name = group
            .get("presetName")
            .and_then(|v| v.as_str())
            .unwrap_or("unknown");
        let preset = ExpressionPreset::from_any(preset_name);

        let binds = group
            .get("binds")
            .and_then(|v| v.as_array())
            .map(|arr| {
                arr.iter()
                    .filter_map(|b| {
                        Some(MorphTargetBind {
                            addressing: MorphAddressing::MeshIndex,
                            target: b.get("mesh")?.as_u64()? as usize,
                            index: b.get("index")?.as_u64()? as usize,
                            weight: (b.get("weight").and_then(|w| w.as_f64()).unwrap_or(100.0)
                                as f32)
                                / 100.0,
                        })
                    })
                    .collect()
            })
            .unwrap_or_default();

        let is_binary = group
            .get("isBinary")
            .and_then(|v| v.as_bool())
            .unwrap_or(false);

        let kind = match preset {
            Some(p) => ExpressionKind::Preset(p),
            None => ExpressionKind::Custom(name.to_string()),
        };

        let expr = Expression {
            kind: kind.clone(),
            morph_target_binds: binds,
            is_binary,
            override_blink: OverrideMode::None,
            override_look_at: OverrideMode::None,
            override_mouth: OverrideMode::None,
        };

        match preset {
            Some(p) => {
                out.presets.insert(p, expr);
            }
            None => {
                out.custom.insert(name.to_string(), expr);
            }
        }
    }

    if out.presets.is_empty() && out.custom.is_empty() {
        return None;
    }
    Some(out)
}

/// 表情 weight 群を [0,1] にクランプし、`is_binary` の表情は 0.5 で 0/1 にスナップ。
///
/// `out` には Expression 1 つあたりに対する最終 weight（clamp 済み）が入る。
/// MorphTarget への落とし込みは consumer の責務。
pub fn resolve_weights(
    expressions: &Expressions,
    requested: &HashMap<ExpressionPreset, f32>,
) -> HashMap<ExpressionPreset, f32> {
    let mut out = HashMap::new();
    for (preset, &w) in requested {
        let Some(expr) = expressions.get_preset(*preset) else { continue };
        let w = w.clamp(0.0, 1.0);
        let final_w = if expr.is_binary {
            if w >= 0.5 { 1.0 } else { 0.0 }
        } else {
            w
        };
        out.insert(*preset, final_w);
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde_json::json;

    #[test]
    fn preset_name_round_trip() {
        for preset in [
            ExpressionPreset::Happy,
            ExpressionPreset::Aa,
            ExpressionPreset::BlinkLeft,
            ExpressionPreset::LookUp,
        ] {
            assert_eq!(
                ExpressionPreset::from_any(preset.as_str()),
                Some(preset)
            );
        }
    }

    #[test]
    fn vrm0_aliases_resolve() {
        assert_eq!(ExpressionPreset::from_any("joy"), Some(ExpressionPreset::Happy));
        assert_eq!(ExpressionPreset::from_any("sorrow"), Some(ExpressionPreset::Sad));
        assert_eq!(ExpressionPreset::from_any("fun"), Some(ExpressionPreset::Relaxed));
        assert_eq!(ExpressionPreset::from_any("a"), Some(ExpressionPreset::Aa));
        assert_eq!(ExpressionPreset::from_any("blink_l"), Some(ExpressionPreset::BlinkLeft));
    }

    #[test]
    fn parse_vrm1_minimal() {
        let extensions = json!({
            "VRMC_vrm": {
                "expressions": {
                    "preset": {
                        "happy": {
                            "morphTargetBinds": [
                                {"node": 5, "index": 0, "weight": 1.0}
                            ],
                            "overrideBlink": "block"
                        }
                    }
                }
            }
        });
        let exprs = parse_vrmc_expressions(&extensions).expect("expressions parsed");
        let happy = exprs.get_preset(ExpressionPreset::Happy).expect("happy present");
        assert_eq!(happy.morph_target_binds.len(), 1);
        assert_eq!(happy.morph_target_binds[0].target, 5);
        assert_eq!(happy.morph_target_binds[0].weight, 1.0);
        assert_eq!(happy.morph_target_binds[0].addressing, MorphAddressing::NodeIndex);
        assert_eq!(happy.override_blink, OverrideMode::Block);
    }

    #[test]
    fn parse_vrm0_normalizes_weight() {
        let extensions = json!({
            "VRM": {
                "blendShapeMaster": {
                    "blendShapeGroups": [
                        {
                            "name": "Joy",
                            "presetName": "joy",
                            "binds": [
                                {"mesh": 0, "index": 0, "weight": 100.0}
                            ]
                        }
                    ]
                }
            }
        });
        let exprs = parse_vrm0_blend_shapes(&extensions).expect("vrm0 parsed");
        let happy = exprs.get_preset(ExpressionPreset::Happy).expect("joy → happy");
        assert_eq!(happy.morph_target_binds[0].addressing, MorphAddressing::MeshIndex);
        assert!((happy.morph_target_binds[0].weight - 1.0).abs() < 1e-6);
    }

    #[test]
    fn resolve_weights_binary_snap() {
        let mut exprs = Expressions::default();
        exprs.presets.insert(
            ExpressionPreset::Blink,
            Expression {
                kind: ExpressionKind::Preset(ExpressionPreset::Blink),
                morph_target_binds: vec![],
                is_binary: true,
                override_blink: OverrideMode::None,
                override_look_at: OverrideMode::None,
                override_mouth: OverrideMode::None,
            },
        );

        let mut req = HashMap::new();
        req.insert(ExpressionPreset::Blink, 0.7);
        let out = resolve_weights(&exprs, &req);
        assert_eq!(out[&ExpressionPreset::Blink], 1.0);

        req.insert(ExpressionPreset::Blink, 0.3);
        let out = resolve_weights(&exprs, &req);
        assert_eq!(out[&ExpressionPreset::Blink], 0.0);
    }
}
