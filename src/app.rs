use std::path::Path;
use std::time::Instant;
use gfx::{self, Factory};
use gfx::traits::FactoryExt;
use nalgebra::{self as na, Point3, Point2, Translation3, IsometryMatrix3};

use lib::{Texture, Light, PbrMesh, Error};
use lib::mesh::*;
use lib::load;
use lib::draw::{DrawParams, Painter, SolidStyle, PbrStyle, PbrMaterial, UnishadeStyle};
use lib::vr::{primary, secondary, ControllerRef, VrMoment, ViveController};

pub const NEAR_PLANE: f64 = 0.1;
pub const FAR_PLANE: f64 = 1000.;
pub const BACKGROUND: [f32; 4] = [0.529, 0.808, 0.980, 1.0];
const PI: f32 = ::std::f32::consts::PI;
const PI2: f32 = 2. * PI;
const DEG: f32 = PI2 / 360.;

pub struct AppMats<R: gfx::Resources> {
    plastic: PbrMaterial<R>,
}

impl<R: gfx::Resources> AppMats<R> {
    pub fn new<F: Factory<R> + FactoryExt<R>>(f: &mut F) -> Result<Self, Error> {
        use gfx::format::*;
        Ok(AppMats {
            plastic: PbrMaterial {
                normal: Texture::<_, (R8_G8_B8_A8, Unorm)>::uniform_value(f, [0x80, 0x80, 0xFF, 0xFF])?,
                albedo: Texture::<_, (R8_G8_B8_A8, Srgb)>::uniform_value(f, [0x60, 0x60, 0x60, 0xFF])?,
                metalness: Texture::<_, (R8, Unorm)>::uniform_value(f, 0x00)?,
                roughness: Texture::<_, (R8, Unorm)>::uniform_value(f, 0x20)?,
            },
        })
    }
}

pub struct App<R: gfx::Resources> {
    solid: Painter<R, SolidStyle<R>>,
    pbr: Painter<R, PbrStyle<R>>,
    controller: PbrMesh<R>,
    cube: PbrMesh<R>,
    grab: (IsometryMatrix3<f32>, Option<ControllerRef>),
    start_time: Instant,
    primary: ViveController,
    secondary: ViveController,
    mat: AppMats<R>,
}

fn cube(rad: f32) -> MeshSource<VertN, ()> {
    fn square<V, F: Fn(f32, f32) -> V>(rad: f32, f: F, vec: &mut Vec<V>) {
        vec.push(f(-rad, -rad));
        vec.push(f( rad,  rad));
        vec.push(f(-rad,  rad));
        vec.push(f( rad,  rad));
        vec.push(f(-rad, -rad));
        vec.push(f( rad, -rad));
    }

    let mut verts = Vec::with_capacity(6 * 6);
    square(rad, |y, z| VertN { pos: [-rad, y, z], norm: [-1., 0., 0.] }, &mut verts);
    square(rad, |z, y| VertN { pos: [ rad, y, z], norm: [ 1., 0., 0.] }, &mut verts);
    square(rad, |x, z| VertN { pos: [x, -rad, z], norm: [0., -1., 0.] }, &mut verts);
    square(rad, |z, x| VertN { pos: [x,  rad, z], norm: [0.,  1., 0.] }, &mut verts);
    square(rad, |x, y| VertN { pos: [x, y, -rad], norm: [0., 0., -1.] }, &mut verts);
    square(rad, |y, x| VertN { pos: [x, y,  rad], norm: [0., 0.,  1.] }, &mut verts);

    MeshSource {
        verts: verts,
        inds: Indexing::All,
        prim: Primitive::TriangleList,
        mat: (),
    }
}

impl<R: gfx::Resources> App<R> {
    pub fn new<F: Factory<R> + FactoryExt<R>>(factory: &mut F) -> Result<Self, Error> {
        // Setup Painters
        let mut solid = Painter::new(factory)?;
        solid.setup(factory, Primitive::LineList)?;
        solid.setup(factory, Primitive::TriangleList)?;

        let mut pbr: Painter<_, PbrStyle<_>> = Painter::new(factory)?;
        pbr.setup(factory, Primitive::TriangleList)?;

        let mat = AppMats::new(factory)?;

        // Construct App
        Ok(App {
            solid: solid,
            pbr: pbr,
            controller: load::wavefront_file("assets/controller.obj")?
                .compute_tan()
                .with_material(mat.plastic.clone())
                .upload(factory),
            cube: cube(0.125)
                .with_tex(Point2::new(0., 0.))
                .compute_tan()
                .with_material(mat.plastic.clone())
                .upload(factory),
            grab: (IsometryMatrix3::from_parts(
                Translation3::new(0., 1., 0.),
                na::one(),
            ), None),
            start_time: Instant::now(),
            primary: ViveController {
                is: primary(),
                pad: Point2::new(1., 0.),
                .. Default::default()
            },
            secondary: ViveController {
                is: secondary(),
                .. Default::default()
            },
            mat: mat,
        })
    }

    pub fn draw<C: gfx::CommandBuffer<R>>(
        &mut self,
        ctx: &mut DrawParams<R, C>,
        vrm: &VrMoment,
    ) {
        let elapsed = self.start_time.elapsed();
        let t = elapsed.as_secs() as f32 + (elapsed.subsec_nanos() as f32 * 1e-9);

        match (self.primary.update(vrm), self.secondary.update(vrm)) {
            (Ok(_), Ok(_)) => (),
            _ => warn!("A not vive-like controller is connected"),
        }
        
        // Clear targets
        ctx.encoder.clear_depth(&ctx.depth, FAR_PLANE as f32);
        ctx.encoder.clear(&ctx.color, [BACKGROUND[0].powf(1. / 2.2), BACKGROUND[1].powf(1. / 2.2), BACKGROUND[2].powf(1. / 2.2), BACKGROUND[3]]);

        // Config PBR lights
        self.pbr.cfg(|s| {
            s.ambient(BACKGROUND);
            s.lights(&[
                Light {
                    pos: vrm.stage * Point3::new((0. * PI2 / 3.).sin() * 2., 4., (0. * PI2 / 3.).cos() * 2.),
                    color: [1.0, 0.8, 0.8, 85.],
                },
                Light {
                    pos: vrm.stage * Point3::new((1. * PI2 / 3.).sin() * 2., 4., (1. * PI2 / 3.).cos() * 2.),
                    color: [0.8, 1.0, 0.8, 85.],
                },
                Light {
                    pos: vrm.stage * Point3::new((2. * PI2 / 3.).sin() * 2., 4., (2. * PI2 / 3.).cos() * 2.),
                    color: [0.8, 0.8, 1.0, 85.],
                },
                Light {
                    pos: vrm.stage * Point3::new(0., -8., 0.),
                    color: [1.0, 1.0, 1.0, 200.],
                },
            ]);
        });

        self.pbr.draw(ctx, na::convert(vrm.stage * self.grab.0), &self.cube);

        // Draw controllers
        for cont in vrm.controllers() {
            self.pbr.draw(ctx, na::convert(cont.pose), &self.controller);
        }
    }
}
