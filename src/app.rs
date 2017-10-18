use std::time::Instant;
use gfx::{self, Factory};
use gfx::traits::FactoryExt;
use nalgebra::{self as na, Point3, Point2, Vector3, Similarity3, Isometry3, Translation3, UnitQuaternion};
use ncollide::shape::Cuboid3;
use ncollide::query::{Ray, PointQuery, RayCast};

use lib::{Texture, Light, PbrMesh, Error};
use lib::mesh::*;
use lib::load;
use lib::draw::{DrawParams, Painter, SolidStyle, PbrStyle, PbrMaterial};
use lib::vr::{primary, secondary, VrMoment, ViveController, Trackable};

pub const NEAR_PLANE: f64 = 0.1;
pub const FAR_PLANE: f64 = 75.;
pub const BACKGROUND: [f32; 4] = [0.529, 0.808, 0.980, 1.0];
const PI: f32 = ::std::f32::consts::PI;
const PI2: f32 = 2. * PI;

pub struct AppMats<R: gfx::Resources> {
    plastic: PbrMaterial<R>,
    dark_plastic: PbrMaterial<R>,
    blue_plastic: PbrMaterial<R>,
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
            dark_plastic: PbrMaterial {
                normal: Texture::<_, (R8_G8_B8_A8, Unorm)>::uniform_value(f, [0x80, 0x80, 0xFF, 0xFF])?,
                albedo: Texture::<_, (R8_G8_B8_A8, Srgb)>::uniform_value(f, [0x20, 0x20, 0x20, 0xFF])?,
                metalness: Texture::<_, (R8, Unorm)>::uniform_value(f, 0x00)?,
                roughness: Texture::<_, (R8, Unorm)>::uniform_value(f, 0x40)?,
            },
            blue_plastic: PbrMaterial {
                normal: Texture::<_, (R8_G8_B8_A8, Unorm)>::uniform_value(f, [0x80, 0x80, 0xFF, 0xFF])?,
                albedo: Texture::<_, (R8_G8_B8_A8, Srgb)>::uniform_value(f, [0x20, 0x20, 0xA0, 0xFF])?,
                metalness: Texture::<_, (R8, Unorm)>::uniform_value(f, 0x00)?,
                roughness: Texture::<_, (R8, Unorm)>::uniform_value(f, 0x40)?,
            },
        })
    }
}

pub struct App<R: gfx::Resources> {
    solid: Painter<R, SolidStyle<R>>,
    pbr: Painter<R, PbrStyle<R>>,
    controller: PbrMesh<R>,
    line: Mesh<R, VertC, ()>,
    line_len: f32,
    cube: PbrMesh<R>,
    objects: Vec<Grabable>,
    grab: Option<(usize, Isometry3<f32>)>,
    last_time: Instant,
    primary: ViveController,
    secondary: ViveController,
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
            line: MeshSource {
                    verts: vec![
                        VertC { pos: [0., 0., 0.], color: [0.22, 0.74, 0.94] },
                        VertC { pos: [0., 0., -1.], color: [0.2, 0.28, 0.31] },
                    ],
                    inds: Indexing::All,
                    prim: Primitive::LineList,
                    mat: (),
                }.upload(factory),
            line_len: ::std::f32::INFINITY,
            cube: cube(1.)
                .with_tex(Point2::new(0., 0.))
                .compute_tan()
                .with_material(mat.dark_plastic)
                .upload(factory),
            objects: (0i32..10).map(|i| {
                let rad = 0.2 * (1. - i as f32 / 15.);
                let theta = (i as f32) / 5. * PI;
                Grabable {
                    pos: Isometry3::from_parts(
                        Translation3::new(theta.sin() * 1., 0., theta.cos() * 1.),
                        UnitQuaternion::from_axis_angle(&Vector3::y_axis(), theta)
                    ),
                    shape: Cuboid3::new(Vector3::from_element(rad)),
                    radius: rad,
                }
            }).collect(),
            grab: None,
            last_time: Instant::now(),
            primary: ViveController {
                is: primary(),
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
        let dt = self.last_time.elapsed();
        let dt = dt.as_secs() as f32 + (dt.subsec_nanos() as f32 * 1e-9);

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

        let ray = Ray::new(self.primary.origin(), self.primary.pointing());
        if let Some((i, t)) = self.objects.iter().enumerate().filter_map(|(i, o)| {
            o.shape.toi_with_ray(&o.pos, &ray, true).map(|t| (i, t))
        }).min_by(|a, b| a.1.partial_cmp(&b.1).unwrap()) {
            self.line_len = t;
            let pull = 3f32.powf(self.primary.pad_delta[1] as f32);
            let pull = t * pull - t;
            self.objects[i].pos.translation.vector += pull * ray.dir;

            if self.primary.trigger > 0.5 && self.grab.is_none() {
                self.grab = Some((i, self.primary.pose().inverse() * self.objects[i].pos));
            }
        } else {
            self.line_len = ::std::f32::INFINITY;
        }

        if let Some((ind, off)) = self.grab {
            if self.primary.trigger > 0.5 {
                self.objects[ind].pos = self.primary.pose() * off;
            } else {
                self.grab = None
            }
        }

        for o in &self.objects {
            self.pbr.draw(ctx, na::convert(Similarity3::from_isometry(o.pos, o.radius)), &self.cube)
        }

        // Draw controllers
        for cont in vrm.controllers() {
            self.pbr.draw(ctx, na::convert(cont.pose), &self.controller);
        }

        self.solid.draw(ctx, na::convert(
            Similarity3::from_isometry(self.primary.pose(), self.line_len.max(0.01).min(FAR_PLANE as f32))
        ), &self.line)
    }
}

pub trait InteractShape: 
    PointQuery<Point3<f32>, Isometry3<f32>> + 
    RayCast<Point3<f32>, Isometry3<f32>> {}
impl<T: PointQuery<
    Point3<f32>, Isometry3<f32>> +
    RayCast<Point3<f32>, Isometry3<f32>>> 
    InteractShape for T {}

pub struct Grabable {
    pos: Isometry3<f32>,
    shape: Cuboid3<f32>,
    radius: f32,
}
