use std::time::Instant;
use std::cell::RefCell;
use gfx::{self, Factory};
use gfx::traits::FactoryExt;
use nalgebra::{self as na, Point3, Point2, Vector3, Similarity3, Isometry3, Translation3};
use ncollide::shape::Cuboid;
use ncollide::query::{Ray, PointQuery, RayCast};

use lib::{Texture, Light, PbrMesh, Error};
use lib::mesh::*;
use lib::load;
use lib::draw::{DrawParams, Painter, SolidStyle, PbrStyle, PbrMaterial};
use lib::vr::{primary, secondary, VrMoment, ViveController, Trackable};

pub const NEAR_PLANE: f64 = 0.1;
pub const FAR_PLANE: f64 = 1000.;
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
    cube: PbrMesh<R>,
    dark_cube: PbrMesh<R>,
    blue_cube: PbrMesh<R>,
    objects: Vec<RefCell<Object>>,
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
        let mut cube = cube(1.)
            .with_tex(Point2::new(0., 0.))
            .compute_tan() 
            .with_material(mat.plastic.clone())
            .upload(factory);

        // Construct App
        Ok(App {
            solid: solid,
            pbr: pbr,
            controller: load::wavefront_file("assets/controller.obj")?
                .compute_tan()
                .with_material(mat.plastic.clone())
                .upload(factory),
            cube: cube.clone(),
            dark_cube: {
                cube.mat = mat.dark_plastic;
                cube.clone()
            },
            blue_cube: {
                cube.mat = mat.blue_plastic;
                cube
            },
            objects: (0i32..10).map(|i| Object {
                pos: na::convert(Translation3::new(i as f32 * 0.5, 0., 0.)),
                radius: 0.125,
                grab: None,
            }).map(|o| RefCell::new(o)).collect(),
            last_time: Instant::now(),
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

        for o in &self.objects {
            let mut o = o.borrow_mut();
            o.update(dt, self, vrm);
            o.draw(self, ctx);
        }

        // Draw controllers
        for cont in vrm.controllers() {
            self.pbr.draw(ctx, na::convert(cont.pose), &self.controller);
        }
    }
}

pub struct Object {
    pos: Isometry3<f32>,
    radius: f32,
    grab: Option<Isometry3<f32>>,
}

impl Object {
    fn draw<R: gfx::Resources, C: gfx::CommandBuffer<R>>(
        &self,
        app: &App<R>,
        ctx: &mut DrawParams<R, C>)
    {
        let mat = na::convert(Similarity3::from_isometry(self.pos, self.radius));
        if self.grab.is_some() {
            app.pbr.draw(ctx, mat, &app.blue_cube);
        } else {
            app.pbr.draw(ctx, mat, &app.dark_cube);
        }
        
    }

    fn update<R: gfx::Resources>(&mut self, dt: f32, app: &App<R>, vrm: &VrMoment) {
        if let Some(off) = self.grab {
            if app.primary.trigger > 0.5 {
                self.pos = app.primary.pose() * off;
            } else {
                self.grab = None;
            }
        } else {
            let shape = Cuboid::new(Vector3::from_element(self.radius));
            let prim = Ray::new(app.primary.origin(), app.primary.pointing());
            if app.primary.trigger > 0.5 && shape.intersects_ray(&self.pos, &prim) {
                self.grab = Some(app.primary.pose().inverse() * self.pos);
            }
        }
    }
}
