use ncollide::query::{RayCast, RayIntersection, Ray};
use nalgebra::{Point3, Vector3, Isometry3};
use flight::vr::{Trackable, ViveController};
use std::collections::BinaryHeap;
use std::sync::{Arc, Mutex};
use std::cmp::{Ord, PartialOrd, PartialEq, Ordering};

pub struct VrGuru {
    pub primary: ControllerGuru,
    pub secondary: ControllerGuru,
}

impl VrGuru {
    pub fn new(primary: &ViveController, secondary: &ViveController) -> VrGuru {
        VrGuru {
            primary: ControllerGuru {
                data: ViveController {
                    .. *primary
                },
                queries: BinaryHeap::new(),
                blocked: false,
                laser_toi: None,
            },
            secondary: ControllerGuru {
                data: ViveController {
                    .. *secondary
                },
                queries: BinaryHeap::new(),
                blocked: false,
                laser_toi: None,
            },
        }
    }
    
    pub fn apply(self) {
        self.primary.apply();
        self.secondary.apply();
    }
}

struct ControllerQuery {
    hit: RayIntersection<Vector3<f32>>,
    reply: PointingReply,
    stop: bool,
}

impl PartialEq for ControllerQuery {
    fn eq(&self, other: &ControllerQuery) -> bool {
        Arc::ptr_eq(&self.reply.0, &other.reply.0)
    }
}

impl Eq for ControllerQuery {}

impl Ord for ControllerQuery {
    fn cmp(&self, other: &ControllerQuery) -> Ordering {
        self.partial_cmp(other).expect("TOI can't be NaN")
    }
}

impl PartialOrd for ControllerQuery {
    fn partial_cmp(&self, other: &ControllerQuery) -> Option<Ordering> {
        self.hit.toi.partial_cmp(&other.hit.toi)
    }
}

pub struct ControllerGuru {
    pub data: ViveController,
    pub laser_toi: Option<f32>,
    queries: BinaryHeap<ControllerQuery>,
    blocked: bool,
}

pub type PointingReply = Anywhere<Option<RayIntersection<Vector3<f32>>>>;
impl ControllerGuru {
    pub fn laser<S: RayCast<Point3<f32>, Isometry3<f32>>>(
        &mut self,
        pos: &Isometry3<f32>,
        shape: &S,
    ) {
        let ray = Ray::new(self.data.origin(), self.data.pointing());
        match (shape.toi_with_ray(pos, &ray, true), self.laser_toi) {
            (Some(t), Some(ref mut o)) if *o > t => *o = t,
            (Some(t), ref mut l @ None) => *l = Some(t),
            _ => (),
        }
    }

    pub fn pointing<S: RayCast<Point3<f32>, Isometry3<f32>>>(
        &mut self,
        pos: &Isometry3<f32>,
        shape: &S,
        stops: bool,
    )
        -> PointingReply 
    {
        if !self.blocked {
            let ray = Ray::new(self.data.origin(), self.data.pointing());
            if let Some(i) = shape.toi_and_normal_with_ray(pos, &ray, true) {
                let reply = Anywhere::new();
                self.queries.push(ControllerQuery {
                    hit: i,
                    reply: reply.clone(),
                    stop: stops,
                });
                return reply;
            }
        }
        Anywhere::with(None)
    }

    pub fn block_pointing(&mut self) {
        self.blocked = true;
        for q in self.queries.drain() {
            q.reply.put(None);
        }
    }

    pub fn apply(mut self) {
        while let Some(q) = self.queries.pop() {
            q.reply.put(Some(q.hit));
            if q.stop { break; }
        }
        for q in self.queries {
            q.reply.put(None);
        }
    }
}

pub struct Anywhere<T>(Arc<Mutex<Option<T>>>);

impl<T> Clone for Anywhere<T> {
    fn clone(&self) -> Anywhere<T> {
        Anywhere(self.0.clone())
    }
}

impl<T> Anywhere<T> {
    pub fn new() -> Anywhere<T> {
        From::from(None)
    }

    pub fn with(v: T) -> Anywhere<T> {
        From::from(Some(v))
    }

    pub fn take(&self) -> Option<T> {
        self.0.lock().unwrap().take()
    }

    pub fn put(&self, v: T) {
        *self.0.lock().unwrap() = Some(v);
    }

    pub fn expect(&self, msg: &str) -> T {
        self.take().expect(msg)
    }
}

impl<T> From<Option<T>> for Anywhere<T> {
    fn from(v: Option<T>) -> Anywhere<T> {
        Anywhere(Arc::new(Mutex::new(v)))  
    }
}
