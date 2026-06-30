//! Shared control-loop primitives for the openarm control nodes.
//!
//! - [`Pacer`]: fixed-rate pacing for a control loop, with overrun accounting.
//!
//! The bimanual coordination hub (openarm01_backbone) and the real arm
//! (openarm01_arm) both pace their real-time control loops with [`Pacer`]; this is
//! their one tested implementation. A home for further control primitives as they
//! are factored out of the nodes.

mod pacer;

pub use pacer::Pacer;
