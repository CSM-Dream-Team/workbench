use std::path::Path;
use std::time::Instant;
use gfx::{self, Factory};
use gfx::traits::FactoryExt;
use nalgebra::{self as na, Point3, Point2};

use lib::{Texture, Light, PbrMesh, Error};
use lib::mesh::*;
use lib::load;
use lib::draw::{DrawParams, Painter, SolidStyle, PbrStyle, PbrMaterial};
use lib::vr::{primary, secondary, VrMoment, ViveController};

pub const NEAR_PLANE: f64 = 0.1;
pub const FAR_PLANE: f64 = 1000.;
pub const BACKGROUND: [f32; 4] = [0.529, 0.808, 0.980, 1.0];
const PI: f32 = ::std::f32::consts::PI;
const PI2: f32 = 2. * PI;
const DEG: f32 = PI2 / 360.;

pub struct App<R: gfx::Resources> {
    solid: Painter<R, SolidStyle<R>>,
    pbr: Painter<R, PbrStyle<R>>,
    controller: PbrMesh<R>,
    start_time: Instant,
    primary: ViveController,
    secondary: ViveController,
}

fn load_simple_model<P, R, F>(f: &mut F, path: P, albedo: [u8; 4])
    -> Result<Mesh<R, VertNTT, PbrMaterial<R>>, Error>
    where P: AsRef<Path>, R: gfx::Resources, F: gfx::Factory<R>
{
    use gfx::format::*;
    Ok(load::wavefront_file(path)?.compute_tan().with_material(PbrMaterial {
        normal: Texture::<_, (R8_G8_B8_A8, Unorm)>::uniform_value(f, albedo)?,
        albedo: Texture::<_, (R8_G8_B8_A8, Srgb)>::uniform_value(f, [0x60, 0x60, 0x60, 0xFF])?,
        metalness: Texture::<_, (R8, Unorm)>::uniform_value(f, 0x00)?,
        roughness: Texture::<_, (R8, Unorm)>::uniform_value(f, 0x20)?,
    }).upload(f))
}

impl<R: gfx::Resources> App<R> {
    pub fn new<F: Factory<R> + FactoryExt<R>>(factory: &mut F) -> Result<Self, Error> {
        // Setup Painters
        let mut solid = Painter::new(factory)?;
        solid.setup(factory, Primitive::LineList)?;
        solid.setup(factory, Primitive::TriangleList)?;

        let mut pbr: Painter<_, PbrStyle<_>> = Painter::new(factory)?;
        pbr.setup(factory, Primitive::TriangleList)?;

        // Construct App
        Ok(App {
            solid: solid,
            pbr: pbr,
            controller: load_simple_model(factory, "assets/controller.obj", [0x80, 0x80, 0xFF, 0xFF])?,
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
                    pos: vrm.stage * Point3::new((0. * PI2 / 3.).sin(), 4., (0. * PI2 / 3.).cos()),
                    color: [1.0, 0.8, 0.8, 85.],
                },
                Light {
                    pos: vrm.stage * Point3::new((1. * PI2 / 3.).sin(), 4., (1. * PI2 / 3.).cos()),
                    color: [0.8, 1.0, 0.8, 85.],
                },
                Light {
                    pos: vrm.stage * Point3::new((2. * PI2 / 3.).sin(), 4., (2. * PI2 / 3.).cos()),
                    color: [0.8, 0.8, 1.0, 85.],
                },
                Light {
                    pos: vrm.stage * Point3::new(0., -8., 0.),
                    color: [1.0, 1.0, 1.0, 200.],
                },
            ]);
        });

        // Draw controllers
        for cont in vrm.controllers() {
            self.pbr.draw(ctx, na::convert(cont.pose), &self.controller);
        }
    }
}