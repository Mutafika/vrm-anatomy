//! CPUスキニング（Linear Blend Skinning）
//!
//! glTFスキンデータからメッシュを骨格アニメーションで変形する。

/// スキン付き頂点
#[derive(Clone, Debug)]
pub struct SkinVertex {
    pub position: [f32; 3],
    pub normal: [f32; 3],
    pub uv: [f32; 2],
    pub joints: [u32; 4],
    pub weights: [f32; 4],
}

/// スキン付きメッシュ（バインドポーズ）
#[derive(Clone, Debug)]
pub struct SkinMesh {
    pub vertices: Vec<SkinVertex>,
    pub indices: Vec<u32>,
}

/// スキニング結果の頂点
#[derive(Clone, Debug)]
pub struct SkinnedVertex {
    pub position: [f32; 3],
    pub normal: [f32; 3],
    pub uv: [f32; 2],
}

// === 行列演算ヘルパー ===

type Mat4 = [[f32; 4]; 4];

fn mat4_identity() -> Mat4 {
    [
        [1.0, 0.0, 0.0, 0.0],
        [0.0, 1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]
}

fn mat4_mul(a: &Mat4, b: &Mat4) -> Mat4 {
    let mut out = [[0.0_f32; 4]; 4];
    for i in 0..4 {
        for j in 0..4 {
            out[i][j] = a[i][0] * b[0][j] + a[i][1] * b[1][j]
                      + a[i][2] * b[2][j] + a[i][3] * b[3][j];
        }
    }
    out
}

fn mat4_transform_point(m: &Mat4, p: [f32; 3]) -> [f32; 3] {
    [
        m[0][0] * p[0] + m[0][1] * p[1] + m[0][2] * p[2] + m[0][3],
        m[1][0] * p[0] + m[1][1] * p[1] + m[1][2] * p[2] + m[1][3],
        m[2][0] * p[0] + m[2][1] * p[1] + m[2][2] * p[2] + m[2][3],
    ]
}

fn mat4_transform_normal(m: &Mat4, n: [f32; 3]) -> [f32; 3] {
    let out = [
        m[0][0] * n[0] + m[0][1] * n[1] + m[0][2] * n[2],
        m[1][0] * n[0] + m[1][1] * n[1] + m[1][2] * n[2],
        m[2][0] * n[0] + m[2][1] * n[1] + m[2][2] * n[2],
    ];
    let len = (out[0] * out[0] + out[1] * out[1] + out[2] * out[2]).sqrt();
    if len > 1e-8 { [out[0] / len, out[1] / len, out[2] / len] } else { out }
}

/// オイラー角 [roll, pitch, yaw] → 回転行列（ZYX順）
pub fn euler_to_mat4(euler: [f32; 3]) -> Mat4 {
    let (sr, cr) = euler[0].sin_cos(); // roll  (X)
    let (sp, cp) = euler[1].sin_cos(); // pitch (Y)
    let (sy, cy) = euler[2].sin_cos(); // yaw   (Z)

    [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr, 0.0],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr, 0.0],
        [-sp,     cp * sr,                cp * cr,                0.0],
        [0.0,     0.0,                    0.0,                    1.0],
    ]
}

/// クォータニオン [x, y, z, w] → 回転行列
pub fn quat_to_mat4(q: [f32; 4]) -> Mat4 {
    let [x, y, z, w] = q;
    let x2 = x + x; let y2 = y + y; let z2 = z + z;
    let xx = x * x2; let xy = x * y2; let xz = x * z2;
    let yy = y * y2; let yz = y * z2; let zz = z * z2;
    let wx = w * x2; let wy = w * y2; let wz = w * z2;

    [
        [1.0 - (yy + zz), xy - wz,          xz + wy,          0.0],
        [xy + wz,          1.0 - (xx + zz),  yz - wx,          0.0],
        [xz - wy,          yz + wx,          1.0 - (xx + yy),  0.0],
        [0.0,              0.0,              0.0,               1.0],
    ]
}

/// 平行移動行列
pub fn translation_mat4(t: [f32; 3]) -> Mat4 {
    [
        [1.0, 0.0, 0.0, t[0]],
        [0.0, 1.0, 0.0, t[1]],
        [0.0, 0.0, 1.0, t[2]],
        [0.0, 0.0, 0.0, 1.0],
    ]
}

/// ボーンのワールド行列を計算
///
/// - `bone_local_rotations`: ボーンインデックス → オイラー角 [roll, pitch, yaw]
///   （アニメーションのサンプリング結果）
/// - `node_translations`: glTFノードのローカル平行移動（バインドポーズ）
/// - `node_rotations`: glTFノードのローカル回転（バインドポーズ、クォータニオン）
/// - `parents`: ボーンインデックス → 親インデックス (None=ルート)
/// - `inverse_bind_matrices`: glTFのIBM
///
/// 戻り値: ジョイント行列 (world * IBM) の配列
pub fn compute_joint_matrices(
    bone_local_rotations: &[[f32; 3]],
    node_translations: &[[f32; 3]],
    node_rotations: &[[f32; 4]],
    parents: &[Option<usize>],
    inverse_bind_matrices: &[Mat4],
) -> Vec<Mat4> {
    let count = node_translations.len();
    let mut world_matrices = vec![mat4_identity(); count];

    for i in 0..count {
        // ローカル変換: T * R_bind * R_anim
        let t = translation_mat4(node_translations[i]);
        let r_bind = quat_to_mat4(node_rotations[i]);

        let local = if i < bone_local_rotations.len() {
            let r_anim = euler_to_mat4(bone_local_rotations[i]);
            mat4_mul(&t, &mat4_mul(&r_bind, &r_anim))
        } else {
            mat4_mul(&t, &r_bind)
        };

        world_matrices[i] = match parents[i] {
            Some(p) => mat4_mul(&world_matrices[p], &local),
            None => local,
        };
    }

    // joint_matrix = world * IBM
    let mut joint_matrices = Vec::with_capacity(count);
    for i in 0..count {
        let ibm = if i < inverse_bind_matrices.len() {
            &inverse_bind_matrices[i]
        } else {
            &mat4_identity()
        };
        joint_matrices.push(mat4_mul(&world_matrices[i], ibm));
    }
    joint_matrices
}

/// CPU LBS（Linear Blend Skinning）
///
/// 各頂点を最大4ジョイントのウェイト付きブレンドで変形する。
pub fn cpu_skin_lbs(mesh: &SkinMesh, joint_matrices: &[Mat4]) -> Vec<SkinnedVertex> {
    mesh.vertices.iter().map(|v| {
        let mut pos = [0.0_f32; 3];
        let mut norm = [0.0_f32; 3];

        for k in 0..4 {
            let w = v.weights[k];
            if w < 1e-6 { continue; }
            let ji = v.joints[k] as usize;
            if ji >= joint_matrices.len() { continue; }
            let m = &joint_matrices[ji];
            let p = mat4_transform_point(m, v.position);
            let n = mat4_transform_normal(m, v.normal);
            pos[0] += p[0] * w;
            pos[1] += p[1] * w;
            pos[2] += p[2] * w;
            norm[0] += n[0] * w;
            norm[1] += n[1] * w;
            norm[2] += n[2] * w;
        }

        // 法線を正規化
        let nlen = (norm[0] * norm[0] + norm[1] * norm[1] + norm[2] * norm[2]).sqrt();
        if nlen > 1e-8 {
            norm[0] /= nlen;
            norm[1] /= nlen;
            norm[2] /= nlen;
        }

        SkinnedVertex { position: pos, normal: norm, uv: v.uv }
    }).collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_identity_skinning() {
        let mesh = SkinMesh {
            vertices: vec![SkinVertex {
                position: [1.0, 2.0, 3.0],
                normal: [0.0, 0.0, 1.0],
                uv: [0.5, 0.5],
                joints: [0, 0, 0, 0],
                weights: [1.0, 0.0, 0.0, 0.0],
            }],
            indices: vec![0],
        };
        let matrices = vec![mat4_identity()];
        let result = cpu_skin_lbs(&mesh, &matrices);
        assert_eq!(result.len(), 1);
        assert!((result[0].position[0] - 1.0).abs() < 1e-5);
        assert!((result[0].position[1] - 2.0).abs() < 1e-5);
        assert!((result[0].position[2] - 3.0).abs() < 1e-5);
    }

    #[test]
    fn test_euler_identity() {
        let m = euler_to_mat4([0.0, 0.0, 0.0]);
        let id = mat4_identity();
        for i in 0..4 {
            for j in 0..4 {
                assert!((m[i][j] - id[i][j]).abs() < 1e-6);
            }
        }
    }

    #[test]
    fn test_translation_skinning() {
        let mesh = SkinMesh {
            vertices: vec![SkinVertex {
                position: [0.0, 0.0, 0.0],
                normal: [0.0, 1.0, 0.0],
                uv: [0.0, 0.0],
                joints: [0, 0, 0, 0],
                weights: [1.0, 0.0, 0.0, 0.0],
            }],
            indices: vec![0],
        };
        let mut m = mat4_identity();
        m[0][3] = 10.0; // translate x by 10
        let result = cpu_skin_lbs(&mesh, &vec![m]);
        assert!((result[0].position[0] - 10.0).abs() < 1e-5);
    }
}
