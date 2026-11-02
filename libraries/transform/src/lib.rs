//! Frames, calibration and rigid transforms.
//!
//! Which edges a transform uses is decided in [`frames`] and [`extrinsics`],
//! without arithmetic, so `dora validate` can check a route before launch.
//! [`rigid`] does the arithmetic.

pub mod extrinsics;
pub mod frames;
pub mod rigid;

pub use extrinsics::{Edge, Extrinsics, Route};
pub use frames::{Frame, FrameDict, Kind};

/// Upstream dora-nav normalises in float32, which leaves the norm about 1e-7
/// off; 1e-6 accepts that and still refuses a quaternion nobody normalised.
pub const QUATERNION_NORM_TOLERANCE: f64 = 1e-6;

#[derive(Debug, Clone, PartialEq, thiserror::Error)]
pub enum FrameError {
    /// A malformed dictionary, calibration or transform.
    #[error("{0}")]
    Invalid(String),
    /// The name is not in the dictionary, or is an upstream name it rejects.
    #[error("{0}")]
    UnknownFrame(String),
    /// A static path needs an edge the robot's calibration does not hold.
    #[error("{0}")]
    MissingExtrinsic(String),
    /// A static path crosses edges only a published pose can give.
    /// `edges` holds every dynamic edge as (child, parent), in route order.
    #[error("{message}")]
    DynamicEdge {
        message: String,
        edges: Vec<(String, String)>,
    },
    /// Two transforms were composed whose frames do not meet.
    #[error("{0}")]
    FrameOrder(String),
}

pub type Result<T> = std::result::Result<T, FrameError>;
