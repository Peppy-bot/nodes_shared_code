//! Capsules fitted at construction must conservatively contain the meshes
//! they came from: containment of every vertex and sampled face point
//! implies the capsule distance is a lower bound on true mesh distance, so
//! the model can alarm early, never late. A single capsule is convex, so it
//! holds a mesh's faces once it holds the vertices; where a body carries
//! several capsules (a wrist plus its fingers) the union is not convex, so the
//! dense face scan guards against a face leaking between them.

use bimanual_collision_model::geometry::Capsule;
use bimanual_collision_model::nalgebra::Point3;
use bimanual_collision_model::urdf_collision::UrdfCollisions;
use bimanual_collision_model::{BimanualCollisionModel, GovernorBand, MarginPolicy};

const FIXTURES: &str = concat!(env!("CARGO_MANIFEST_DIR"), "/tests/fixtures");

/// Allow only float round-off at the surface.
const TOL: f64 = 1e-9;

fn fixture() -> (UrdfCollisions, BimanualCollisionModel, String) {
    let urdf = UrdfCollisions::from_file(&format!("{FIXTURES}/openarm_v10.urdf")).expect("fixture urdf");
    let model = BimanualCollisionModel::from_urdf_file(
        &format!("{FIXTURES}/openarm_v10.urdf"),
        &format!("{FIXTURES}/meshes"),
        "openarm_left_link0",
        "openarm_right_link0",
        &MarginPolicy { band: GovernorBand::new(0.01, 0.03).expect("valid band"), references: vec![[0.0; 7]] },
    )
    .expect("fixture model");
    (urdf, model, format!("{FIXTURES}/meshes"))
}

/// Dense uniform barycentric scan: every face that no single capsule already
/// contains is sampled on an `n`-by-`n` grid. A sparse fixed sample set is not
/// an upper bound on a non-convex union (a face dips into the gap between two
/// capsules anywhere between samples), so this scans finely enough to catch a
/// multi-millimetre leak independently of how the fit certifies coverage.
const GRID: usize = 24;

fn assert_contained(vertices: &[Point3<f64>], capsules: &[Capsule], what: &str) {
    let union_escape = |p: &Point3<f64>| {
        capsules
            .iter()
            .map(|c| bimanual_collision_model::point_segment_distance(p, &c.a, &c.b) - c.radius)
            .fold(f64::INFINITY, f64::min)
    };
    let mut worst = f64::NEG_INFINITY;
    for tri in vertices.chunks_exact(3) {
        // Convexity early-exit: one capsule containing all three vertices
        // contains the whole face.
        if capsules.iter().any(|c| {
            tri.iter().all(|v| bimanual_collision_model::point_segment_distance(v, &c.a, &c.b) <= c.radius + TOL)
        }) {
            continue;
        }
        for i in 0..=GRID {
            for j in 0..=(GRID - i) {
                let (a, b) = (i as f64 / GRID as f64, j as f64 / GRID as f64);
                let p = Point3::from(tri[0].coords * a + tri[1].coords * b + tri[2].coords * (1.0 - a - b));
                worst = worst.max(union_escape(&p));
            }
        }
    }
    assert!(worst <= TOL, "{what}: a mesh face point sticks {worst:.2e} m out of the capsule union");
}

#[test]
fn fitted_bodies_contain_their_meshes_and_cover_every_collision_link() {
    let (urdf, mut model, meshes) = fixture();
    // Bodies at the home configuration; moving links are checked in their
    // own frame via the world placement at home.
    let home = [0.0, 0.0, 0.0, 0.05, 0.0, 0.0, 0.0];
    let placed = model.world_capsules(&home, &home).expect("placement");
    assert_eq!(placed.len(), 17, "torso + two mounts + 14 chain links");

    let chain = |name: &str| name.contains("link") && !name.ends_with("link0") && !name.contains("body");
    for (name, _) in &placed {
        // Fixed bodies are fit in the root frame; verify them directly.
        if !chain(name) {
            let vertices = urdf.fixed_vertices_in_root(name, &meshes).expect("fixed vertices");
            let capsules: Vec<Capsule> =
                placed.iter().find(|(n, _)| n == name).map(|(_, c)| c.clone()).expect("present");
            assert_contained(&vertices, &capsules, name);
        }
    }
    // Moving links: their capsules are link-local; comparing in any one
    // frame is equivalent, so bake both sides at identical world poses by
    // construction at home and check the wrist's finger coverage across
    // travel separately below.
}

#[test]
fn wrist_capsules_contain_fingers_across_full_travel() {
    let (urdf, mut model, meshes) = fixture();
    let home = [0.0, 0.0, 0.0, 0.05, 0.0, 0.0, 0.0];
    let _ = model.world_capsules(&home, &home).expect("placement");
    for side in ["left", "right"] {
        for finger in ["left_finger", "right_finger"] {
            let name = format!("openarm_{side}_{finger}");
            let joint = urdf.parent_joint(&name).expect("finger joint");
            for q in [joint.lower_limit, (joint.lower_limit + joint.upper_limit) / 2.0, joint.upper_limit] {
                let vertices = urdf.child_vertices_in_parent(&name, q, &meshes).expect("finger vertices");
                // Fingers are baked into the wrist's LOCAL capsules.
                let wrist_local = model
                    .local_capsules(&format!("openarm_{side}_link7"))
                    .expect("wrist body exists");
                assert_contained(&vertices, &wrist_local, &format!("{name}@{q:.3}"));
            }
        }
    }
}

#[test]
fn wrist_union_covers_the_unmodeled_palm_crossbar() {
    // The upstream description gives the palm (hand.stl) no collision entry
    // and no authored placement for this robot. Its only physically possible
    // slot is the carriage plane (z = 0.1025 off the wrist) under the
    // fingers' shared -0.673001 export offset; placed there, it must sit
    // inside the wrist capsule union or the gripper body is unguarded.
    let (_, model, meshes) = fixture();
    let raw = bimanual_collision_model::stl::load_stl(&format!("{meshes}/hand.stl")).expect("vendored palm mesh");
    let placed: Vec<Point3<f64>> = raw
        .iter()
        .map(|v| Point3::new(v.x * 0.001, v.y * 0.001, v.z * 0.001 - 0.673001 + 0.1025))
        .collect();
    for side in ["left", "right"] {
        let capsules = model.local_capsules(&format!("openarm_{side}_link7")).expect("wrist body");
        assert_contained(&placed, &capsules, &format!("palm crossbar ({side})"));
    }
}

#[test]
fn moving_links_contain_their_meshes_in_link_frame() {
    let (urdf, model, meshes) = fixture();
    for side in ["left", "right"] {
        for i in 1..=7 {
            let name = format!("openarm_{side}_link{i}");
            let vertices = urdf.link_vertices(&name, &meshes).expect("link vertices");
            let capsules = model.local_capsules(&name).expect("link body exists");
            assert_contained(&vertices, &capsules, &name);
        }
    }
}
