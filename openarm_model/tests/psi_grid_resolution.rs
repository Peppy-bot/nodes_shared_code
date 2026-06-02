//! Quantifies the known limitation of the fixed-step arm-angle (ψ) grid in
//! `ik::solve`'s `FromSeed` fallback (see `psi_sweep`). `round_trip` seeds at
//! the true configuration, so its ψ candidate #0 is the exact solution and the
//! grid is never exercised. This test instead seeds with an *unrelated* config,
//! forcing the coarse grid, and asserts the rate of reachable targets it fails
//! to solve stays within a known bound (deterministic: fixed RNG seed).
//!
//! Measured ~0.3% at the current 10° step. If a change to `psi_sweep` regresses
//! this materially, this test fails; if a future continuous/adaptive search
//! removes it, tighten `MAX_MISS_RATE`.

use openarm_model::description::{ArmSide, Description, Version};
use openarm_model::ik::{self, ArmAnglePolicy};
use rand::rngs::StdRng;
use rand::{Rng, SeedableRng};

const MAX_MISS_RATE: f64 = 0.01; // 1%: ~3x headroom over the measured ~0.3%

#[test]
fn coarse_psi_grid_solves_reachable_targets_from_a_bad_seed() {
    let desc = Description::new(Version::V1, ArmSide::Left);
    let mut fk = desc.forward_kinematics().expect("fk");
    let m = desc.model().expect("model");
    let mut rng = StdRng::seed_from_u64(0xBEEF);
    let lim = m.limits;
    let sample = |rng: &mut StdRng| -> [f64; 7] {
        std::array::from_fn(|i| rng.gen_range(lim[i].lo..lim[i].hi))
    };

    let n = 3000;
    let (mut miss, mut solved) = (0u32, 0u32);
    for _ in 0..n {
        let q = sample(&mut rng);
        if q[3] < 0.05 {
            continue; // skip the straight-arm singular boundary
        }
        solved += 1;
        let target = fk.at(&q).ee_pose();
        let r = target.rotation.to_rotation_matrix();
        let p = target.translation.vector;
        // Decorrelated seed: its arm angle is not the target's, so solve() must
        // fall back to the ψ grid. The target is reachable (q itself solves it).
        let bad_seed = sample(&mut rng);
        if ik::solve(&m, &r, &p, ArmAnglePolicy::FromSeed, &bad_seed).is_none() {
            miss += 1;
        }
    }
    let rate = miss as f64 / solved as f64;
    assert!(
        rate < MAX_MISS_RATE,
        "coarse ψ grid missed {miss}/{solved} reachable targets ({:.3}%), over the {:.1}% bound",
        100.0 * rate,
        100.0 * MAX_MISS_RATE
    );
}
