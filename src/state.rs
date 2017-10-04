#![allow(dead_code)]

use nalgebra::{self as na, Similarity3, Isometry3, IsometryMatrix3};
use animation::Animate;

pub struct DraggableFixed<R> {
    /// Where is the object located
    loc: IsometryMatrix3<f32>,
    /// The state of the object
    state: DraggableFixedState<R>,
    /// The reset factor animation following release
    reset: Animate<f32>,
    /// If the object can respawn, the animation to use
    respawn: Option<Animate<Similarity3<f32>>>,
}

pub enum DraggableFixedState<R> {
    Dead,
    Sitting,
    Spawning {
        ani: Animate<Similarity3<f32>>,
    },
    Resetting {
        start: Isometry3<f32>,
        progress: Animate<f32>,
    },
    Grabbed {
        by: R,
        at: IsometryMatrix3<f32>,
    },
}

impl<R> DraggableFixed<R> {
    fn grabber_info(&self) -> Option<R> {
        unimplemented!()
    }

    fn update(&mut self, dt: f32, grabber: Option<IsometryMatrix3<f32>>) -> bool {
        unimplemented!()
    }
}