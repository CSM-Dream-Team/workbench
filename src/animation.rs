#![allow(dead_code)]

use std::collections::VecDeque;
use std::sync::Arc;

/// Just an `f32`.
pub type Param = f32;
/// Just an `f32`.
pub type Time = f32;
/// Just an `f32`.
pub type DeltaTime = f32;

/// An animation state.
pub trait Animation<V>: Clone {
    /// Get the current output of the animation.
    fn now(&self) -> V;
    /// Step the animation forward.
    fn step(&mut self, dt: DeltaTime);
    /// Ensure that the animation is in a normal form and will not behave in an
    /// undefined way. If the animation is in a valid, nontrivial state, then
    /// this should do nothing.
    ///
    /// The most common invalid state is an ended (unchanging) `Animate`
    /// variant which is not `Animate::Fixed`.
    fn normalize(mut self) -> Self { self.step(0.); self }
    /// Check if this animation has reached a steady state.
    fn steady(&self) -> bool;
}

/// The main animation type. Provides a wide variety of animation functions.
#[derive(Clone)]
pub enum Animate<V: Mixable> {
    /// Hold the given value.
    Fixed(V),
    /// Move from `a→b` over the next `t` seconds, then stop.
    Slide(V, V, Time),
    /// Move from `a→b`, outputting `a` after `s` seconds then `b` after `t` seconds.
    Linear(V, V, Time, Time),
    /// Move through the curve `a→b→c`, outputting `a` after `s` seconds then `c` after `t` seconds.
    Quadratic(V, V, V, Time, Time),
    /// Move through the curve  `a→b→c→d`, outputting `a` after `s` seconds then `d` after `t` seconds.
    Cubic(V, V, V, V, Time, Time),
    /// Move from `a→b`, starting after `s` seconds then ending after `t` seconds.
    BoundedLinear(V, V, Time, Time),
    /// Move through the curve `a→b→c`, starting after `s` seconds then ending after `t` seconds.
    BoundedQuadratic(V, V, V, Time, Time),
    /// Move through the curve  `a→b→c→d`, starting after `s` seconds then ending after `t` seconds.
    BoundedCubic(V, V, V, V, Time, Time),
    /// Switch from `a` to `b` after `t` seconds.
    Switch(V, V, Time),
    /// Switch from `a` to `b` between `s` seconds and `t` seconds from now.
    SmoothSwitch(V, V, Time, Time),
    /// Soft switch from `a` to `b` using parameter `e` between `s` seconds and `t` seconds from now.
    SoftSwitch(V, V, i32, Time, Time),
    /// Outputs `f(x)` starting with `x=t`.
    Func(Arc<Fn(Time) -> V>, Time),
    /// `f(x)` mixes between `a` and `b` starting with `x=t`.
    MixFunc(Arc<Fn(Time) -> Time>, V, V, Time),
    /// `f(v, dt)` is repeatedly applied to `v` every step.
    StepFunc(Arc<Fn(V, DeltaTime) -> V>, V),
    /// Play a sequence of animations.
    Sequence(AnimateSequence<V>),
}

use self::Animate::*;

impl<V: Mixable> Animate<V> {
    pub fn bounce_soft(a: V, b: V, duration: Time) -> Animate<V> {
        MixFunc(Arc::new(move |t| {
            let t = t / duration;
            if t < 0. || t > 1. { return 0. }
            let mut x = t * 2. - 1.;
            x *= x;
            x * x - 2. * x + 1. // x^4 - 2x^2 + 1
        }), a, b, 0.)
    }

    pub fn bounce_hard(a: V, b: V, duration: Time) -> Animate<V> {
        MixFunc(Arc::new(move |t| {
            let t = t / duration;
            if t < 0. || t > 1. { return 0. }
            let x = t * 2. - 1.;
            1. - (x * x) // -x^2 + 1
        }), a, b, 0.)
    }

    #[inline]
    fn do_step(self, dt: DeltaTime) -> Self {
        match self {
            Fixed(a) => Fixed(a),
            Slide(a, b, t) => if dt > t {
                Fixed(b)
            } else {
                Slide(V::linear(&a, &b, dt / t), b, t - dt)
            },
            Linear(a, b, s, t) => Linear(a, b, s - dt, t - dt),
            Quadratic(a, b, c, s, t) => Quadratic(a, b, c, s - dt, t - dt),
            Cubic(a, b, c, d, s, t) => Cubic(a, b, c, d, s - dt, t- dt),
            BoundedLinear(a, b, s, t) => if dt > t {
                Fixed(b)
            } else {
                BoundedLinear(a, b, s - dt, t - dt)
            },
            BoundedQuadratic(a, b, c, s, t) => if dt > t {
                Fixed(c)
            } else {
                BoundedQuadratic(a, b, c, s - dt, t - dt)
            },
            BoundedCubic(a, b, c, d, s, t) => if dt > t {
                Fixed(d)
            } else {
                BoundedCubic(a, b, c, d, s - dt, t - dt)
            },
            Switch(a, b, t) => if dt > t {
                Fixed(b)
            } else {
                Switch(a, b, t - dt)
            },
            SmoothSwitch(a, b, s, t) => if dt > t {
                Fixed(b)
            } else {
                SmoothSwitch(a, b, s - dt, t - dt)
            },
            SoftSwitch(a, b, e, s, t) => if dt > t {
                Fixed(b)
            } else {
                SoftSwitch(a, b, e, s - dt, t - dt)
            },
            Func(f, s) => Func(f, s + dt),
            MixFunc(f, a, b, s) => MixFunc(f, a, b, s + dt),
            StepFunc(f, v) => { let v = f(v, dt); StepFunc(f, v) },
            Sequence(mut seq) => { seq.step(dt); Sequence(seq) },
        }
    }
}

impl<V: Mixable> Animation<V> for Animate<V> {
    fn now(&self) -> V {
        match *self {
            Fixed(ref x) => x.clone(),
            Slide(ref x, _, _) => x.clone(),
            Linear(ref a, ref b, s, t) => V::linear(a, b, s / (s - t)),
            Quadratic(ref a, ref b, ref c, s, t) => V::quadratic(a, b, c, s / (s - t)),
            Cubic(ref a, ref b, ref c, ref d, s, t) => V::cubic(a, b, c, d, s / (s - t)),
            BoundedLinear(ref a, ref b, s, t) => if s > 0. {
                a.clone()
            } else {
                V::linear(a, b, s / (s - t)) 
            },
            BoundedQuadratic(ref a, ref b, ref c, s, t) => if s > 0. {
                a.clone()
            } else {
                V::quadratic(a, b, c, s / (s - t))
            },
            BoundedCubic(ref a, ref b, ref c, ref d, s, t) => if s > 0. {
                a.clone()
            } else {
                V::cubic(a, b, c, d, s / (s - t))
            },
            Switch(ref a, _, _) => a.clone(),
            SmoothSwitch(ref a, ref b, s, t) => if s > 0. {
                a.clone()
            } else {
                let x = s / (s - t);
                let xx = x * x;
                V::linear(a, b, 3. * xx - 2. * xx * x) 
            },
            SoftSwitch(ref a, ref b, e, s, t) => if s > 0. {
                a.clone()
            } else {
                let x = s / (s - t);
                let x = 1. - x;
                V::linear(a, b, x.powi(e)) 
            },
            Func(ref f, t) => f(t),
            MixFunc(ref f, ref a, ref b, t) => V::linear(a, b, f(t)),
            StepFunc(_, ref v) => v.clone(),
            Sequence(ref seq) => seq.now(),
        }
    }

    fn step(&mut self, dt: DeltaTime) {
        let ptr = self as *mut Self;
        use std::ptr::*;
        unsafe {
            write(ptr, read(ptr).do_step(dt));
        }
    }

    fn steady(&self) -> bool {
        match self {
            &Fixed(_) => true,
            _ => false,
        }
    }
}

/// A sequence of different animations, each one lasting a given duration.
#[derive(Clone)]
pub struct AnimateSequence<V: Mixable> {
    /// A queue of animations to play, and their durations.
    pub queue: VecDeque<(Animate<V>, Time)>,
    /// The value to hold once the queue is empty.
    pub end: V,
}

impl<V: Mixable> AnimateSequence<V> {
    pub fn new(end: V) -> AnimateSequence<V> {
        AnimateSequence {
            queue: VecDeque::new(),
            end: end,
        }
    }

    pub fn before(&mut self, time: Time, anim: Animate<V>) {
        self.queue.push_front((anim, time));
    }
}

impl<V: Mixable> Animation<V> for AnimateSequence<V> {
    fn now(&self) -> V {
        self.queue.get(0).map(|&(ref v, _)| v.now()).unwrap_or(self.end.clone())
    }

    fn step(&mut self, mut dt: DeltaTime) {
        // Logic to step over multiple animations when the `dt` is large
        let mut count = 0;
        for &mut (ref mut a, ref mut t) in self.queue.iter_mut() {
            if dt > *t {
                // step into next animation, bringing residual `dt` forward
                dt -= *t;
                count += 1;
            } else {
                // step forward in current animation
                a.step(dt);
                *t -= dt;
                break;
            }
        }
        self.queue.drain(..count);
    }

    fn steady(&self) -> bool { 
        match self.queue.back() {
            Some(a) => a.0.steady(),
            None => true,
        }
    }
}

/// A type that can be animated.
pub trait Mixable: Sized + Clone {
    /// Most animations are performed by mixing together various provided values, 
    /// so a mixer is needed.
    type Mixer: Mixer<Self>;

    /// Mix together 2 values with linear interpolation.
    fn linear(a: &Self, b: &Self, t: Param) -> Self {
        let mut acc = Self::Mixer::new();
        acc.add(a, 1. - t);
        acc.add(b, t);
        acc.close()
    }
 
    /// Mix together 3 values with a quadratic bezier function.
    fn quadratic(a: &Self, b: &Self, c: &Self, t: Param) -> Self {
        let s = 1. - t;
        let mut acc = Self::Mixer::new();
        acc.add(a, s * s);
        acc.add(b, 2. * s * t);
        acc.add(c, t * t);
        acc.close()
    }

    /// Mix together 4 values with a cubic bezier function.
    fn cubic(a: &Self, b: &Self, c: &Self, d: &Self, t: Param) -> Self {
        let s = 1. - t;
        let mut acc = Self::Mixer::new();
        acc.add(a, s * s * s);
        acc.add(b, 3. * s * s * t);
        acc.add(c, 3. * s * t * t);
        acc.add(d, t * t * t);
        acc.close()
    }

    /// Mix together an iterator of values and their weights. If there are no
    /// values, or the weights do not sum to `1`, then the result is not well
    /// defined.
    fn mix<I: Iterator<Item=(Self, Param)>>(iter: I) -> Self {
        let mut acc = Self::Mixer::new();
        for (ref v, w) in iter {
            acc.add(v, w);
        }
        acc.close()
    }
}

/// Used to mix together several values with varying weights. The mixer  is
/// not responsible for normalizing the given weights, if they do not sum to
/// `1` then the behavior of the mixer is not well defined.
pub trait Mixer<V>: Sized {
    /// Instantiate a new mixer.
    fn new() -> Self;
    /// Mix a new value into the total.
    fn add(&mut self, v: &V, weight: Param);
    /// Finish mixing the values together. If no values have been  added then
    /// the result is not well defined.
    fn close(self) -> V;
}

use nalgebra as na;
use nalgebra::*;

impl Mixer<Self> for f32 {
    fn new() -> Self { 0. }
    fn add(&mut self, v: &Self, weight: Param) { *self += v * weight }
    fn close(self) -> Self { self }
}
impl Mixable for f32 { type Mixer = Self; }

impl Mixer<Self> for f64 {
    fn new() -> Self { 0. }
    fn add(&mut self, v: &Self, weight: Param) { *self += v * weight as f64 }
    fn close(self) -> Self { self }
}
impl Mixable for f64 { type Mixer = Self; }

macro_rules! impl_mix {
    (<$g:ident: $gb:path> $i:ty = $t:ty, || $n:expr, |$ep:ident| $e:expr, |$cp:ident| $c:expr) => {
        impl<$g: $gb> Mixer<$i> for $t {
            fn new() -> Self { $n }
            fn add(&mut self, v: &$i, weight: Param) {
                let $ep = v;
                *self += $e * $g::from_f32(weight).unwrap();
            }
            fn close(self) -> $i { let $cp = self; $c }
        }

        impl<$g: $gb> Mixable for $i { type Mixer = $t; }
    };
}

macro_rules! nalg_mix {
    ({$($a:path = $b:path),*$(,)*}, $g:ident, |$ep:ident| $e:expr, |$cp:ident| $c:expr) => (
        $(impl_mix!(<$g: Real> $a = $b, || na::zero(), |$ep| $e, |$cp| $c);)*
    )
}

nalg_mix!({
    Vector1<F> = Vector1<F>,
    Vector2<F> = Vector2<F>,
    Vector3<F> = Vector3<F>,
    Vector4<F> = Vector4<F>,
    Vector5<F> = Vector5<F>,
    Vector6<F> = Vector6<F>,
}, F, |v| v, |c| c);

nalg_mix!({
    Matrix2<F> = Matrix2<F>,
    Matrix3<F> = Matrix3<F>,
    Matrix4<F> = Matrix4<F>,
    Matrix5<F> = Matrix5<F>,
    Matrix6<F> = Matrix6<F>,
}, F, |v| v, |c| c);

nalg_mix!({
    Point1<F> = Vector1<F>,
    Point2<F> = Vector2<F>,
    Point3<F> = Vector3<F>,
    Point4<F> = Vector4<F>,
    Point5<F> = Vector5<F>,
    Point6<F> = Vector6<F>,
}, F, |v| v.coords, |c| Point::from_coordinates(c));

nalg_mix!({
    Translation2<F> = Vector2<F>,
    Translation3<F> = Vector3<F>,
}, F, |v| v.vector, |c| Translation::from_vector(c));

impl_mix!(
    <F: Real> Quaternion<F> = Quaternion<F>,
    || na::zero(),
    |v| v,
    |c| c);
impl_mix!(
    <F: Real> UnitQuaternion<F> = Quaternion<F>,
    || na::zero(),
    |v| v.unwrap(),
    |c| Unit::try_new(c, F::default_epsilon()).unwrap_or(Unit::new_unchecked(c)));

type Isometry3Mixer<F> = 
    (<UnitQuaternion<F> as Mixable>::Mixer, <Translation3<F> as Mixable>::Mixer);
impl<F: Real> Mixer<Isometry3<F>> for Isometry3Mixer<F> {
    fn new() -> Self { 
        (
            <Quaternion<F> as Mixer<UnitQuaternion<F>>>::new(), 
            <Vector3<F> as Mixer<Translation3<F>>>::new())
    }

    fn add(&mut self, v: &Isometry3<F>, weight: Param) {
        Mixer::add(&mut self.0, &v.rotation, weight);
        Mixer::add(&mut self.1, &v.translation, weight);
    }

    fn close(self) -> Isometry3<F> { 
        Isometry3::from_parts(Mixer::close(self.1), Mixer::close(self.0))
    }
}
impl<F: Real> Mixable for Isometry3<F> { type Mixer = Isometry3Mixer<F>; }

type Similarity3Mixer<F> = 
    (<Isometry3<F> as Mixable>::Mixer, <F as Mixable>::Mixer);
impl<F: Real + Mixable> Mixer<Similarity3<F>> for Similarity3Mixer<F> {
    fn new() -> Self { 
        (Mixer::new(), Mixer::new())
    }

    fn add(&mut self, v: &Similarity3<F>, weight: Param) {
        Mixer::add(&mut self.0, &v.isometry, weight);
        Mixer::add(&mut self.1, &v.scaling(), weight);
    }
    
    fn close(self) -> Similarity3<F> { 
        Similarity3::from_isometry(Mixer::close(self.0), Mixer::close(self.1))
    }
}
impl<F: Real + Mixable> Mixable for Similarity3<F> { type Mixer = Similarity3Mixer<F>; }