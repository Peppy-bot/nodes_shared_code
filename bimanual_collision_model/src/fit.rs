//! Capsule fitting: one strictly-bounding capsule per collision mesh, run at
//! model construction to turn each URDF mesh into its runtime proxy.
//!
//! A capsule is a line-swept sphere (LSS) fitted by the construction of Larsen,
//! Gottschalk, Lin & Manocha, "Fast Proximity Queries with Swept Sphere
//! Volumes" (UNC TR99-018; IEEE ICRA 2000, the basis of the PQP distance
//! library): the axis is the dominant principal axis of the vertex covariance,
//! the radius encloses every vertex projected onto the perpendicular plane, and
//! the caps cover the axial extremes. Containment is exact and needs no repair
//! step: the radius is the maximum distance of any vertex to the segment, so
//! every vertex lies inside, and a capsule is convex, so every triangle face (a
//! convex combination of its vertices) lies inside too. Capsule distance is
//! therefore a true lower bound on mesh distance, and the model alarms early,
//! never late.
//!
//! One capsule per link is the standard proxy for distance-based safety (Balan
//! & Bone, "Safe human-robot interaction based on dynamic sphere-swept line
//! bounding volumes", RCIM 26(5), 2010; the capsule proxies in MoveIt and
//! HPP-FCL). It is loose on a compound shape (a forked elbow yoke, the torso),
//! where one radius must span the whole girth, but the governor band and the
//! reference-pose rebasing absorb that looseness: a link that rests inside a fat
//! torso proxy reads the band's `d_safe`, not an alarm. Tightening a compound
//! link with several primitives (medial-axis sphere sets, as in NVIDIA cuRobo,
//! or per-segment capsules) is deferred; it would need a face-coverage pass,
//! since a union of capsules is not convex and can leak a face between two of
//! them. See the README's future-work note.

use srs_model::nalgebra::{Matrix3, Point3, Vector3};

use crate::geometry::{Capsule, point_segment_distance};

/// Fit a single capsule (line-swept sphere) containing every point of
/// `points`: principal axis, perpendicular enclosing radius, extremal caps
/// (Larsen et al., see the module docs). Containment is exact.
pub fn fit_capsule(points: &[Point3<f64>]) -> Result<Capsule, String> {
    if points.is_empty() {
        return Err("cannot fit a capsule to zero points".into());
    }
    if points.iter().any(|p| !(p.x.is_finite() && p.y.is_finite() && p.z.is_finite())) {
        return Err("cannot fit a capsule to non-finite points".into());
    }

    let axis = principal_axis(points);
    let centroid = centroid(points);

    // Extent along the axis and the largest perpendicular distance.
    let mut t_min = f64::INFINITY;
    let mut t_max = f64::NEG_INFINITY;
    let mut r_perp: f64 = 0.0;
    for p in points {
        let d = p.coords - centroid;
        let t = d.dot(&axis);
        t_min = t_min.min(t);
        t_max = t_max.max(t);
        r_perp = r_perp.max((d - axis * t).norm());
    }

    // How far to pull the endpoints inward so the spherical caps cover the
    // ends is shape-dependent: zero is best for flat-ended cylinders (any
    // shrink pays a corner penalty on the rim), the full perpendicular radius
    // for rounded ends. Try a few candidates and keep the smallest radius;
    // containment is exact for each because the radius is recomputed as the
    // worst vertex's distance to the candidate segment.
    let mid = (t_min + t_max) / 2.0;
    let max_shrink = r_perp.min((t_max - t_min) / 2.0);
    let candidates: Vec<Capsule> = [0.0, 0.25, 0.5, 0.75, 1.0]
        .iter()
        .map(|frac| {
            let half = (t_max - t_min) / 2.0 - max_shrink * frac;
            let a = Point3::from(centroid + axis * (mid - half));
            let b = Point3::from(centroid + axis * (mid + half));
            let radius = points.iter().map(|p| point_segment_distance(p, &a, &b)).fold(0.0, f64::max);
            Capsule { a, b, radius }
        })
        .collect();

    // Smallest radius wins; among near-ties (e.g. blob clouds, where every
    // shrink gives the same radius up to sampling noise) prefer the shortest
    // segment, which adds the least phantom volume. The tie window is
    // relative: a radius within 0.1% is not worth a longer segment.
    let r_min = candidates.iter().map(|c| c.radius).fold(f64::INFINITY, f64::min);
    Ok(candidates
        .into_iter()
        .filter(|c| c.radius <= r_min * 1.001 + 1e-9)
        .min_by(|x, y| (x.b - x.a).norm().total_cmp(&(y.b - y.a).norm()))
        .expect("the minimum-radius candidate always survives its own tie window"))
}

/// Dominant eigenvector of the point covariance: the direction of largest
/// spread, used as the capsule axis. The Z fallback guards a numerically
/// degenerate eigenvector (a single point yields unit eigenvectors already),
/// not a reachable shape.
fn principal_axis(points: &[Point3<f64>]) -> Vector3<f64> {
    let centroid = centroid(points);
    let cov = points.iter().fold(Matrix3::zeros(), |acc, p| {
        let d = p.coords - centroid;
        acc + d * d.transpose()
    }) / points.len() as f64;

    let eigen = cov.symmetric_eigen();
    let (dominant, _) = eigen
        .eigenvalues
        .iter()
        .enumerate()
        .max_by(|a, b| a.1.total_cmp(b.1))
        .expect("a 3x3 covariance has three eigenvalues");
    let v = eigen.eigenvectors.column(dominant).into_owned();
    if v.norm_squared() < 1e-12 { Vector3::z() } else { v.normalize() }
}

fn centroid(points: &[Point3<f64>]) -> Vector3<f64> {
    points.iter().fold(Vector3::zeros(), |acc, p| acc + p.coords) / points.len() as f64
}

#[cfg(test)]
mod tests {
    use super::*;

    fn contained(points: &[Point3<f64>], c: &Capsule) -> bool {
        // Tolerate float round-off at the surface.
        points.iter().all(|p| point_segment_distance(p, &c.a, &c.b) <= c.radius + 1e-9)
    }

    /// Points on a cylinder of radius `r` around the segment `a..b`.
    fn cylinder_cloud(a: Point3<f64>, b: Point3<f64>, r: f64) -> Vec<Point3<f64>> {
        let axis = (b - a).normalize();
        let u = axis.cross(&Vector3::new(0.3, 0.7, -0.2)).normalize();
        let v = axis.cross(&u);
        let mut pts = Vec::new();
        for i in 0..40 {
            let t = i as f64 / 39.0;
            let center = a + (b - a) * t;
            for k in 0..12 {
                let ang = k as f64 * std::f64::consts::TAU / 12.0;
                pts.push(center + (u * ang.cos() + v * ang.sin()) * r);
            }
        }
        pts
    }

    #[test]
    fn fits_a_cylinder_tightly() {
        let (a, b) = (Point3::new(0.1, -0.2, 0.3), Point3::new(1.4, 0.5, -0.1));
        let cloud = cylinder_cloud(a, b, 0.05);
        let c = fit_capsule(&cloud).expect("fit");
        assert!(contained(&cloud, &c));
        // Tight: radius within 20% of the true cylinder radius.
        assert!(c.radius < 0.06, "radius {} too loose", c.radius);
        // Axis aligned with the cylinder axis.
        let fitted_axis = (c.b - c.a).normalize();
        let true_axis = (b - a).normalize();
        assert!(fitted_axis.dot(&true_axis).abs() > 0.999);
    }

    #[test]
    fn collapses_to_a_sphere_for_blob_clouds() {
        // Points on a sphere: the segment should collapse to ~the center.
        let mut pts = Vec::new();
        for i in 0..100 {
            let phi = i as f64 * 0.618 * std::f64::consts::TAU;
            let z = -1.0 + 2.0 * (i as f64 + 0.5) / 100.0;
            let r = (1.0f64 - z * z).sqrt();
            pts.push(Point3::new(r * phi.cos(), r * phi.sin(), z));
        }
        let c = fit_capsule(&pts).expect("fit");
        assert!(contained(&pts, &c));
        assert!((c.b - c.a).norm() < 0.2, "segment {} should be near-degenerate", (c.b - c.a).norm());
        assert!(c.radius < 1.1);
    }

    #[test]
    fn handles_collinear_and_single_points() {
        let line: Vec<_> = (0..10).map(|i| Point3::new(i as f64 * 0.1, 0.0, 0.0)).collect();
        let c = fit_capsule(&line).expect("fit line");
        assert!(contained(&line, &c));

        let single = [Point3::new(0.3, 0.4, 0.5)];
        let c = fit_capsule(&single).expect("fit point");
        assert!(contained(&single, &c));

        assert!(fit_capsule(&[]).is_err());
        assert!(fit_capsule(&[Point3::new(f64::NAN, 0.0, 0.0)]).is_err());
    }

    #[test]
    fn contains_random_clouds() {
        use rand::{Rng, SeedableRng};
        let mut rng = rand::rngs::StdRng::seed_from_u64(7);
        for _ in 0..20 {
            let pts: Vec<_> = (0..200)
                .map(|_| {
                    Point3::new(
                        rng.gen_range(-1.0..1.0),
                        rng.gen_range(-0.2..0.2) + 2.0 * rng.gen_range(-1.0..1.0f64).powi(3),
                        rng.gen_range(-0.5..0.5),
                    )
                })
                .collect();
            let c = fit_capsule(&pts).expect("fit");
            assert!(contained(&pts, &c));
        }
    }
}
