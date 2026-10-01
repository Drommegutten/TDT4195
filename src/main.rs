// Uncomment these following global attributes to silence most warnings of "low" interest:
/*
#![allow(dead_code)]
#![allow(non_snake_case)]
#![allow(unreachable_code)]
#![allow(unused_mut)]
#![allow(unused_unsafe)]
#![allow(unused_variables)]
*/
extern crate nalgebra_glm as glm;
use std::{ mem, ptr, os::raw::c_void };
use std::thread;
use std::sync::{Mutex, Arc, RwLock};

mod shader;
mod util;
mod mesh;
mod scene_graph;
mod toolbox;
use scene_graph::SceneNode;
use toolbox::Heading;

use glutin::event::{Event, WindowEvent, DeviceEvent, KeyboardInput, ElementState::{Pressed, Released}, VirtualKeyCode::{self, *}};
use glutin::event_loop::ControlFlow;

// initial window size
const INITIAL_SCREEN_W: u32 = 800;
const INITIAL_SCREEN_H: u32 = 600;

// == // Helper functions to make interacting with OpenGL a little bit prettier. You *WILL* need these! // == //

// Get the size of an arbitrary array of numbers measured in bytes
// Example usage:  byte_size_of_array(my_array)
fn byte_size_of_array<T>(val: &[T]) -> isize {
    std::mem::size_of_val(&val[..]) as isize
}

// Get the OpenGL-compatible pointer to an arbitrary array of numbers
// Example usage:  pointer_to_array(my_array)
fn pointer_to_array<T>(val: &[T]) -> *const c_void {
    &val[0] as *const T as *const c_void
}

// Get the size of the given type in bytes
// Example usage:  size_of::<u64>()
fn size_of<T>() -> i32 {
    mem::size_of::<T>() as i32
}

// Get an offset in bytes for n units of type T, represented as a relative pointer
// Example usage:  offset::<u64>(4)
fn offset<T>(n: u32) -> *const c_void {
    (n * mem::size_of::<T>() as u32) as *const T as *const c_void
}

// Get a null pointer (equivalent to an offset of 0)
// ptr::null()


// Uploads a vector of floats to a new VBO and binds it to the given vertex attribute index.
// `components` is the number of floats per vertex (3 for positions/normals, 4 for RGBA).
unsafe fn create_attribute_buffer(data: &Vec<f32>, attribute_index: u32, components: i32) {
    let mut vbo = 0;
    gl::GenBuffers(1, &mut vbo);
    gl::BindBuffer(gl::ARRAY_BUFFER, vbo);
    gl::BufferData(
        gl::ARRAY_BUFFER,
        byte_size_of_array(data),
        pointer_to_array(data),
        gl::STATIC_DRAW,
    );

    // Describe the layout of the data to OpenGL
    gl::VertexAttribPointer(
        attribute_index,
        components,
        gl::FLOAT,
        gl::FALSE,
        components * size_of::<f32>(),
        offset::<f32>(0),
    );
    gl::EnableVertexAttribArray(attribute_index);
}

// Creates a Vertex Array Object holding positions (location 0), RGBA colors (location 1),
// normals (location 2) and an index buffer. Returns the VAO ID.
unsafe fn create_vao(vertices: &Vec<f32>, indices: &Vec<u32>, rgba: &Vec<f32>, normals: &Vec<f32>) -> u32 {
    // Creates a Vertex Object Array and binds it
    let mut vao = 0;
    gl::GenVertexArrays(1, &mut vao);
    gl::BindVertexArray(vao);

    create_attribute_buffer(vertices, 0, 3);
    create_attribute_buffer(rgba, 1, 4);
    create_attribute_buffer(normals, 2, 3);

    // Stored as part of the VAO's state, this tells `glDrawElements` which vertices to connect into triangles, avoiding duplicate vertex data.
    let mut ibo = 0;
    gl::GenBuffers(1, &mut ibo);
    gl::BindBuffer(gl::ELEMENT_ARRAY_BUFFER, ibo);
    gl::BufferData(
        gl::ELEMENT_ARRAY_BUFFER,
        byte_size_of_array(indices),
        pointer_to_array(indices),
        gl::STATIC_DRAW,
    );

    vao
}

unsafe fn draw_scene(
    node: &scene_graph::SceneNode,
    view_projection_matrix: &glm::Mat4,
    transformation_so_far: &glm::Mat4,
    mvp_loc: i32,
    model_loc: i32) 
    {
        // Finds relative position of node
        let relative: glm::Mat4 = 
            glm::translation(&node.position)
            * glm::translation(&node.reference_point)
            * glm::rotation(node.rotation.x, &glm::vec3(1.0,0.0,0.0))
            * glm::rotation(node.rotation.y, &glm::vec3(0.0, 1.0, 0.0)) 
            * glm::rotation(node.rotation.z, &glm::vec3(0.0, 0.0, 1.0))
            * glm::translation(&-node.reference_point);

        let model: glm::Mat4  = transformation_so_far * relative;

        // Logical check to see if the node has any indices to draw
        if node.index_count > 0 {
            let mvp: glm::Mat4 = view_projection_matrix * model;
            gl::UniformMatrix4fv(mvp_loc, 1, gl::FALSE, mvp.as_ptr());
            gl::UniformMatrix4fv(model_loc, 1, gl::FALSE, model.as_ptr());
            gl::BindVertexArray(node.vao_id);
            gl::DrawElements(
                gl::TRIANGLES,
                node.index_count,
                gl::UNSIGNED_INT,
                ptr::null()
            );
        }

        // recursivly draws all children nodes
        for &child in &node.children {

            draw_scene(
                &*child, 
                view_projection_matrix, 
                &model, 
                mvp_loc, 
                model_loc
            );
    }
}

// VAO ids and index counts for the helicopter parts, created once and shared by every helicopter
struct HelicopterVaos {
    body: (u32, i32),
    door: (u32, i32),
    main_rotor: (u32, i32),
    tail_rotor: (u32, i32),
}

unsafe fn load_helicopter_vaos() -> HelicopterVaos {
    let helicopter_mesh = mesh::Helicopter::load("./resources/helicopter.obj");

    let upload = |m: &mesh::Mesh| (create_vao(&m.vertices, &m.indices, &m.colors, &m.normals), m.index_count);

    HelicopterVaos {
        body: upload(&helicopter_mesh.body),
        door: upload(&helicopter_mesh.door),
        main_rotor: upload(&helicopter_mesh.main_rotor),
        tail_rotor: upload(&helicopter_mesh.tail_rotor),
    }
}

// Intialization code for a helicopter
// Creates nodes for all parts and attaches them hierarchicly 

fn init_helicopter(
        vaos: &HelicopterVaos,
        terrain_node: &mut SceneNode)
            -> (scene_graph::Node, scene_graph::Node, scene_graph::Node, scene_graph::Node)
        {
        let mut helicopter_root_node = SceneNode::new();
        let body_node = SceneNode::from_vao(vaos.body.0, vaos.body.1);
        let door_node = SceneNode::from_vao(vaos.door.0, vaos.door.1);
        let mut main_rotor_node = SceneNode::from_vao(vaos.main_rotor.0, vaos.main_rotor.1);
        let mut tail_rotor_node = SceneNode::from_vao(vaos.tail_rotor.0, vaos.tail_rotor.1);

            // The helicopter parts all move along with the helicopter root
        helicopter_root_node.add_child(&body_node);
        helicopter_root_node.add_child(&door_node);
        helicopter_root_node.add_child(&main_rotor_node);
        helicopter_root_node.add_child(&tail_rotor_node);

            // The helicopter is placed relative to the terrain, which is placed relative to the root
        terrain_node.add_child(&helicopter_root_node);

        tail_rotor_node.reference_point = glm::vec3(0.35, 2.3, 10.4);
        main_rotor_node.reference_point = glm::vec3(0.0, 2.3, 0.0);

        // Return the nodes that need to be accessed (animated / drawn) from the render loop
        (helicopter_root_node, main_rotor_node, tail_rotor_node, door_node)
}

// Chase_camera function: Rotates camera towards object node if it moves toward the side of the frustum.
// Not a very good function
unsafe fn chase_camera(
    node: &SceneNode,
    view_projection_matrix: &glm::Mat4,
    mut cam_angle_x: f32,
    mut cam_angle_y: f32
    ) -> (f32, f32) 
    {
        
        let position = glm::vec4(node.position.x, node.position.y, node.position.z, 1.0);
        
        let clip = view_projection_matrix * position ;

        if clip[3] > 0.0 {
            // In front of the camera: divide by w to get NDC
            let ndc = clip / clip[3];

            if ndc[0] > 0.7 {
                cam_angle_y += 0.05;
            }
            if ndc[0] < -0.7 {
                cam_angle_y -= 0.05;
            }
            if ndc[1] > 0.7 {
                cam_angle_x -= 0.05;
            }
            
            if ndc[1] < -0.7 {
                cam_angle_x += 0.05;
            }
        } else {
            // Behind the camera: dividing by w would flip the signs, so turn using the sign of clip x
            if clip[0] >= 0.0 {
                cam_angle_y += 0.05;
            } else {
                cam_angle_y -= 0.05;
            }
        }
        (cam_angle_x, cam_angle_y)
    }

fn main() { 
    // Set up the necessary objects to deal with windows and event handling
    let el = glutin::event_loop::EventLoop::new();
    let wb = glutin::window::WindowBuilder::new()
        .with_title("Gloom-rs")
        .with_resizable(true)
        .with_inner_size(glutin::dpi::LogicalSize::new(INITIAL_SCREEN_W, INITIAL_SCREEN_H));
    let cb = glutin::ContextBuilder::new()
        .with_vsync(true);
    let windowed_context = cb.build_windowed(wb, &el).unwrap();
    // Uncomment these if you want to use the mouse for controls, but want it to be confined to the screen and/or invisible.
    // windowed_context.window().set_cursor_grab(true).expect("failed to grab cursor");
    // windowed_context.window().set_cursor_visible(false);

    // Set up a shared vector for keeping track of currently pressed keys
    let arc_pressed_keys = Arc::new(Mutex::new(Vec::<VirtualKeyCode>::with_capacity(10)));
    // Make a reference of this vector to send to the render thread
    let pressed_keys = Arc::clone(&arc_pressed_keys);

    // Set up shared tuple for tracking mouse movement between frames
    let arc_mouse_delta = Arc::new(Mutex::new((0f32, 0f32)));
    // Make a reference of this tuple to send to the render thread
    let mouse_delta = Arc::clone(&arc_mouse_delta);

    // Set up shared tuple for tracking changes to the window size
    let arc_window_size = Arc::new(Mutex::new((INITIAL_SCREEN_W, INITIAL_SCREEN_H, false)));
    // Make a reference of this tuple to send to the render thread
    let window_size = Arc::clone(&arc_window_size);

    // Spawn a separate thread for rendering, so event handling doesn't block rendering
    let render_thread = thread::spawn(move || {
        // Acquire the OpenGL Context and load the function pointers.
        // This has to be done inside of the rendering thread, because
        // an active OpenGL context cannot safely traverse a thread boundary
        let context = unsafe {
            let c = windowed_context.make_current().unwrap();
            gl::load_with(|symbol| c.get_proc_address(symbol) as *const _);
            c
        };

        let mut window_aspect_ratio = INITIAL_SCREEN_W as f32 / INITIAL_SCREEN_H as f32;

        // Set up openGL
        unsafe {
            gl::Enable(gl::DEPTH_TEST);
            gl::DepthFunc(gl::LESS);
            gl::Disable(gl::CULL_FACE);
            gl::Disable(gl::MULTISAMPLE);
            gl::Enable(gl::BLEND);
            gl::BlendFunc(gl::SRC_ALPHA, gl::ONE_MINUS_SRC_ALPHA);
            gl::Enable(gl::DEBUG_OUTPUT_SYNCHRONOUS);
            gl::DebugMessageCallback(Some(util::debug_callback), ptr::null());

            // Print some diagnostics
            println!("{}: {}", util::get_gl_string(gl::VENDOR), util::get_gl_string(gl::RENDERER));
            println!("OpenGL\t: {}", util::get_gl_string(gl::VERSION));
            println!("GLSL\t: {}", util::get_gl_string(gl::SHADING_LANGUAGE_VERSION));
        }

            // Load the lunar terrain and upload it to the GPU
        let terrain_mesh = mesh::Terrain::load("./resources/lunarsurface.obj");
        let terrain_vao = unsafe {
            create_vao(&terrain_mesh.vertices, &terrain_mesh.indices, &terrain_mesh.colors, &terrain_mesh.normals)
        };

        let mut root_node = SceneNode::new();
        let mut terrain_node = SceneNode::from_vao(terrain_vao, terrain_mesh.index_count);
        root_node.add_child(&terrain_node);


            // Load the helicopter mesh and upload it to the GPU once; all helicopters share these VAOs
        let helicopter_vaos = unsafe { 
            load_helicopter_vaos() 
        };

        let mut helicopters: Vec<_> = (0..5)
            .map(|_| init_helicopter(&helicopter_vaos, &mut terrain_node))
            .collect();

            // Door state: U toggles open/closed, door_offset animates smoothly towards the target
        let mut door_open = false;
        let mut u_was_pressed = false;
        let mut door_offset: f32 = 0.0;
        

        let simple_shader = unsafe {
            shader::ShaderBuilder::new()
                .attach_file("./shaders/simple.frag")
                .attach_file("./shaders/simple.vert")
                .link()
        };

            // Location of the combined transformation matrix in simple.vert
        let mvp_loc = unsafe { simple_shader.get_uniform_location("mvp") };
        let model_loc = unsafe { simple_shader.get_uniform_location("model") };


            // Camera state: position in world space, and rotation around the X (pitch) and Y (yaw) axes.
            // Starts slightly above the terrain so the craters are visible.
        let initial_cam_position = glm::vec3(0.0, 40.0, 0.0);
        let mut cam_position = initial_cam_position;
        let mut cam_angle_x: f32 = 0.3; // pitch, positive looks down
        let mut cam_angle_y: f32 = 0.0; // yaw

            // Units per second the camera moves, and radians per second it turns
        let cam_speed: f32 = 30.0;
        let cam_turn_speed: f32 = 1.2;

        // The main rendering loop
        let first_frame_time = std::time::Instant::now();
        let mut previous_frame_time = first_frame_time;
        loop {
            // Compute time passed since the previous frame and since the start of the program
            let now = std::time::Instant::now();
            let elapsed = now.duration_since(first_frame_time).as_secs_f32();
            let delta_time = now.duration_since(previous_frame_time).as_secs_f32();
            previous_frame_time = now;

            // Handle resize events
            if let Ok(mut new_size) = window_size.lock() {
                if new_size.2 {
                    context.resize(glutin::dpi::PhysicalSize::new(new_size.0, new_size.1));
                    window_aspect_ratio = new_size.0 as f32 / new_size.1 as f32;
                    (*new_size).2 = false;
                    println!("Window was resized to {}x{}", new_size.0, new_size.1);
                    unsafe { gl::Viewport(0, 0, new_size.0 as i32, new_size.1 as i32); }
                }
            }

                // Unit vectors pointing forwards and to the right of the camera in the XZ-plane,
                // so that W/A/S/D moves relative to the direction the camera is facing.
            let forward = glm::vec3(cam_angle_y.sin(), 0.0, -cam_angle_y.cos());
            let right = glm::vec3(cam_angle_y.cos(), 0.0, cam_angle_y.sin());
            let up = glm::vec3(0.0, 1.0, 0.0);
            let step = cam_speed * delta_time;

            // Handle keyboard input
            if let Ok(keys) = pressed_keys.lock() {
                    // Toggle the door only on the frame U goes down, not every frame it is held
                let u_pressed = keys.contains(&VirtualKeyCode::U);
                if u_pressed && !u_was_pressed {
                    door_open = !door_open;
                }
                u_was_pressed = u_pressed;

                for key in keys.iter() {
                    match key {
                        // The `VirtualKeyCode` enum is defined here:
                        //    https://docs.rs/winit/0.25.0/winit/event/enum.VirtualKeyCode.html

                        // Camera movement
                        VirtualKeyCode::W      => { cam_position += forward * step; }
                        VirtualKeyCode::S      => { cam_position -= forward * step; }
                        VirtualKeyCode::D      => { cam_position += right * step; }
                        VirtualKeyCode::A      => { cam_position -= right * step; }
                        VirtualKeyCode::Space  => { cam_position += up * step; }
                        VirtualKeyCode::LShift => { cam_position -= up * step; }

                        // Camera rotation
                        VirtualKeyCode::Left  => { cam_angle_y -= cam_turn_speed * delta_time; }
                        VirtualKeyCode::Right => { cam_angle_y += cam_turn_speed * delta_time; }
                        VirtualKeyCode::Up    => { cam_angle_x -= cam_turn_speed * delta_time; }
                        VirtualKeyCode::Down  => { cam_angle_x += cam_turn_speed * delta_time; }

                        // Resets rotation and location
                        VirtualKeyCode::R => {
                            cam_position = initial_cam_position;
                            cam_angle_x = 0.3;
                            cam_angle_y = 0.0;
                        }

                        // default handler:
                        _ => { }
                    }
                }
            }
            // Handle mouse movement. delta contains the x and y movement of the mouse since last frame in pixels
            if let Ok(mut delta) = mouse_delta.lock() {

                // == // Optionally access the accumulated mouse movement between
                // == // frames here with `delta.0` and `delta.1`

                *delta = (0.0, 0.0); // reset when done
            }

                // Projection matrix, recomputed each frame so it follows the window's aspect ratio.
            let projection: glm::Mat4 = glm::perspective(
                window_aspect_ratio,
                (60.0_f32).to_radians(),
                1.0,
                1000.0,
            );
            

                // View matrix: first move the world so the camera is at the origin,
                // then rotate it around the camera (yaw first, then pitch).
            let view: glm::Mat4 =
                glm::rotation(cam_angle_x, &glm::vec3(1.0, 0.0, 0.0))
                * glm::rotation(cam_angle_y, &glm::vec3(0.0, 1.0, 0.0))
                * glm::translation(&-cam_position);

            let view_projection = projection * view;

                // Slide the door backwards along the body (+z) when open
            let door_target = 
                if door_open { 
                    2.0 
                } else { 
                    0.0 
                };
            let door_speed = 3.0 * delta_time;
            door_offset += (door_target - door_offset).clamp(-door_speed, door_speed);

            for (i, (heli_root, main_rotor, tail_rotor, door)) in helicopters.iter_mut().enumerate() {
                door.position.z = door_offset;
                let t = elapsed + i as f32 * 0.8; // tidsforskyvning så de ikke overlapper
                main_rotor.rotation.y = t * 10.0;
                tail_rotor.rotation.x = t * 10.0;

                let heading = toolbox::simple_heading_animation(t);
                heli_root.position = glm::vec3(heading.x, 20.0 + 5.0 * (i as f32* 0.7).sin(), heading.z);
                heli_root.rotation = glm::vec3(heading.pitch, heading.yaw, heading.roll);
            }

            let cam_angle_adjust = unsafe { 
                chase_camera(&helicopters[0].0, &view_projection, cam_angle_x, cam_angle_y) 
            };

            cam_angle_x = cam_angle_adjust.0;
            cam_angle_y = cam_angle_adjust.1;

            


            unsafe {
                gl::ClearColor(0.035, 0.046, 0.078, 1.0); // night sky
                gl::Clear(gl::COLOR_BUFFER_BIT | gl::DEPTH_BUFFER_BIT);

                simple_shader.activate();

                // Call recursive draw_sccene function to draw from root node
                draw_scene(&root_node, &view_projection, &glm::identity(), mvp_loc, model_loc);
            }
                

            // Display the new color buffer on the display
            context.swap_buffers().unwrap(); // we use "double buffering" to avoid artifacts
        }
    });


    // == //
    // == // From here on down there are only internals.
    // == //


    // Keep track of the health of the rendering thread
    let render_thread_healthy = Arc::new(RwLock::new(true));
    let render_thread_watchdog = Arc::clone(&render_thread_healthy);
    thread::spawn(move || {
        if !render_thread.join().is_ok() {
            if let Ok(mut health) = render_thread_watchdog.write() {
                println!("Render thread panicked!");
                *health = false;
            }
        }
    });

    // Start the event loop -- This is where window events are initially handled
    el.run(move |event, _, control_flow| {
        *control_flow = ControlFlow::Wait;

        // Terminate program if render thread panics
        if let Ok(health) = render_thread_healthy.read() {
            if *health == false {
                *control_flow = ControlFlow::Exit;
            }
        }

        match event {
            Event::WindowEvent { event: WindowEvent::Resized(physical_size), .. } => {
                println!("New window size received: {}x{}", physical_size.width, physical_size.height);
                if let Ok(mut new_size) = arc_window_size.lock() {
                    *new_size = (physical_size.width, physical_size.height, true);
                }
            }
            Event::WindowEvent { event: WindowEvent::CloseRequested, .. } => {
                *control_flow = ControlFlow::Exit;
            }
            // Keep track of currently pressed keys to send to the rendering thread
            Event::WindowEvent { event: WindowEvent::KeyboardInput {
                    input: KeyboardInput { state: key_state, virtual_keycode: Some(keycode), .. }, .. }, .. } => {

                if let Ok(mut keys) = arc_pressed_keys.lock() {
                    match key_state {
                        Released => {
                            if keys.contains(&keycode) {
                                let i = keys.iter().position(|&k| k == keycode).unwrap();
                                keys.remove(i);
                            }
                        },
                        Pressed => {
                            if !keys.contains(&keycode) {
                                keys.push(keycode);
                            }
                        }
                    }
                }

                // Handle Escape and Q keys separately
                match keycode {
                    Escape => { *control_flow = ControlFlow::Exit; }
                    Q      => { *control_flow = ControlFlow::Exit; }
                    _      => { }
                }
            }
            Event::DeviceEvent { event: DeviceEvent::MouseMotion { delta }, .. } => {
                // Accumulate mouse movement
                if let Ok(mut position) = arc_mouse_delta.lock() {
                    *position = (position.0 + delta.0 as f32, position.1 + delta.1 as f32);
                }
            }
            _ => { }
        }
    });
}
