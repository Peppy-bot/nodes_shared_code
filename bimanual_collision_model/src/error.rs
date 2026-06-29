//! The crate's error type.

/// A failure from building or querying a
/// [`BimanualCollisionModel`](crate::BimanualCollisionModel). The query variants
/// are distinct so a caller can react to each (a velocity-barrier caller treats
/// [`WitnessesCoincide`](Self::WitnessesCoincide) as deep penetration and holds or
/// escapes, while a bad-input variant is a genuine fault to surface).
#[derive(Debug, thiserror::Error)]
pub enum CollisionError {
    /// A query was handed a non-finite joint value (or threshold).
    #[error("non-finite value in query input")]
    NonFinite,

    /// The model has no checked pairs, so there is nothing to measure.
    #[error("no checked pairs to evaluate")]
    NoPairs,

    /// Deep penetration: the nearest pair's witness points coincide, so the
    /// surface normal, and thus the distance gradient, is undefined.
    #[error("witnesses coincide (d={distance:+.4}); distance gradient undefined")]
    WitnessesCoincide { distance: f64 },

    /// Model construction failed (URDF parse, mesh load, hull fit, or a bad
    /// pair/base specification). The string is the underlying reason.
    #[error("collision model build: {0}")]
    Build(String),
}
