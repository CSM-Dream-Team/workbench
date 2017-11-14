use gfx::{self, Factory};
use gfx::traits::FactoryExt;
use nalgebra::{self as na, Point3, Point2, Vector3, Similarity3, Isometry3, Translation3, UnitQuaternion};
use ncollide::shape::Cuboid3;

use flight::{Texture, Light, PbrMesh, Error};
use flight::mesh::*;
use flight::load;
use flight::draw::{DrawParams, Painter, SolidStyle, PbrStyle, PbrMaterial};
use flight::vr::{primary, secondary, VrMoment, ViveController, Trackable};

use interact::{VrGuru, PointingReply};

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

pub struct Model {
    cubes: Vec<CubeModel>,
}

pub struct CubeModel {
    grabbed: Option<Isometry3<f32>>,
    pos: Isometry3<f32>,
    radius: f32,
}

struct CubePartial {
    index: usize,
    reply: PointingReply,
}

impl CubePartial {
    fn finish<R: gfx::Resources, C: gfx::CommandBuffer<R>>(
        self,
        ctx: &mut DrawParams<R, C>,
        app: &mut App<R>,
    ) {
        let model = &mut app.model.cubes[self.index];
        if let Some(_) = self.reply.expect("pointing not applied") {
            // Grab
            if app.primary.trigger > 0.5 {
                model.grabbed = Some(app.primary.pose().inverse() * model.pos);
            }
            // TODO: speed not delta
            // Yank
            if app.primary.pad_delta[0] < 0. {
                model.grabbed = Some(Isometry3::from_parts(
                    Translation3::from_vector((model.radius + 0.05) * Vector3::z()),
                    app.primary.pose().rotation.inverse() * model.pos.rotation,
                ));
            }
        }
        // Update position
        if let Some(off) = model.grabbed {
            model.pos = app.primary.pose() * off;
            app.pbr.draw(
                ctx,
                na::convert(Similarity3::from_isometry(model.pos, model.radius)),
                &Mesh {
                    mat: app.mats.blue_plastic.clone(),
                    .. app.cube.clone()
                },
            );
        } else {
            app.pbr.draw(
                ctx,
                na::convert(Similarity3::from_isometry(model.pos, model.radius)),
                &app.cube
            );
        }
    }
}

pub struct App<R: gfx::Resources> {
    solid: Painter<R, SolidStyle<R>>,
    pbr: Painter<R, PbrStyle<R>>,
    controller: PbrMesh<R>,
    line: Mesh<R, VertC, ()>,
    cube: PbrMesh<R>,
    mats: AppMats<R>,
    primary: ViveController,
    secondary: ViveController,
    model: Model,
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
            cube: cube(1.)
                .with_tex(Point2::new(0., 0.))
                .compute_tan()
                .with_material(mat.dark_plastic.clone())
                .upload(factory),
            mats: mat,
            primary: ViveController {
                is: primary(),
                .. Default::default()
            },
            secondary: ViveController {
                is: secondary(),
                .. Default::default()
            },
            model: Model {
                cubes: (0i32..10).map(|i| {
                    let rad = 0.2 * (1. - i as f32 / 15.);
                    let theta = (i as f32) / 5. * PI;
                    CubeModel {
                        grabbed: None,
                        pos: Isometry3::from_parts(
                            Translation3::new(theta.sin() * 1., 0., theta.cos() * 1.),
                            UnitQuaternion::from_axis_angle(&Vector3::y_axis(), theta)
                        ),
                        radius: rad,
                    }
                }).collect(),
            },
        })
    }

    pub fn draw<C: gfx::CommandBuffer<R>>(
        &mut self,
        ctx: &mut DrawParams<R, C>,
        vrm: &VrMoment,
    ) {
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

        let mut guru = VrGuru::new(&self.primary, &self.secondary); 
        let cube_partials: Vec<_> = self.model.cubes
            .iter_mut()
            .enumerate()
            .map(|(i, c)| {
                if c.grabbed.is_some() && guru.primary.data.trigger > 0.5 {
                    guru.primary.block_pointing();
                } else {
                    c.grabbed = None;
                }
                let cuboid = Cuboid3::new(Vector3::from_element(c.radius));
                guru.primary.laser(&c.pos, &cuboid);
                CubePartial {
                    index: i,
                    reply: guru.primary.pointing(
                        &c.pos,
                        &cuboid,
                        true),
                }
            })
            .collect();
        let toi = guru.primary.laser_toi.unwrap_or(FAR_PLANE as f32).max(0.01);
        guru.apply();
        for p in cube_partials {
            p.finish(ctx, self);
        }

        // Draw controllers
        for cont in vrm.controllers() {
            self.pbr.draw(ctx, na::convert(cont.pose), &self.controller);
        }

        self.solid.draw(ctx, na::convert(
            Similarity3::from_isometry(self.primary.pose(), toi)
        ), &self.line)
    }
}
