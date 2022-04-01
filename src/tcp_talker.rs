use std::net::TcpStream;
use std::io::prelude::*;
use serde::{Deserialize, Serialize};
use rapier3d_f64::geometry::*;
use rapier3d_f64::na;
use na::{Vector3, UnitQuaternion, Isometry3, Translation3, Quaternion};
use std::error::Error;

#[derive(Debug, Serialize, Deserialize)]
enum Geometry {
    Box {
        x : [f64; 3],
        rot : [f64; 4],
        size : [f64; 3]
    },
    Sphere {
        x : [f64; 3],
        r : f64
    }
}

#[derive(Debug, Serialize, Deserialize)]
enum Message {
    InitialState {
        q : Vec<f64>,
        geometry : Vec<Geometry>
    },
    CurrentState {
        q : Vec<f64>,
        pose_x : [f64; 3],
        pose_q : [f64; 4],
    }
}

fn parse_messages(buffer : &str) -> Vec<Message> {
    let mut messages = Vec::new();
    for msg_str in buffer.split('\n').into_iter() {
        if let Ok(m) = serde_json::de::from_str(msg_str) {
            messages.push(m);
        }
    }

    messages
}


pub fn talk_on_stream<F>(stream : &mut TcpStream, mut on_message: F) -> Result<(), Box<dyn Error>> 
    where F : FnMut(Vec<f64>, &ColliderSet, &Vector3<f64>, &UnitQuaternion<f64>) -> Option<Vec<f64>> {
    
    let mut joint_angles;
    let mut buffer: [u8; 1024] = [0; 1024];
    let mut static_geometry = ColliderSet::new();
    loop {
        let length = stream.read(&mut buffer)?;
        let mut message_data = Vec::new();
        message_data.extend_from_slice(&buffer[0..length]);
        let message_str = String::from_utf8(message_data)?;
        let mut current_state = None;
        for msg in parse_messages(&message_str) {
            match msg {
                Message::InitialState {q : _, geometry} => {
                    println!("{:?}", &geometry);
                    static_geometry = ColliderSet::new();
                    for geo in geometry {
                        match geo {
                            Geometry::Box {x, rot, size} => {
                                let trans = Isometry3::<f64>::from_parts(
                                    Translation3::new(x[0], x[1], x[2]),
                                    UnitQuaternion::from_quaternion(Quaternion::new(rot[0], rot[1], rot[2], rot[3]))
                                );
                                let mut collider = ColliderBuilder::cuboid(
                                    size[0] / 2., 
                                    size[1] / 2., 
                                    size[2] / 2.
                                ).build();
                                println!("{:?}", size);
                                collider.set_position(trans);
                                static_geometry.insert(collider);
                            },
                            Geometry::Sphere {x, r} => {
                                let translation = Vector3::new(x[0], x[1], x[2]);
                                let mut collider = ColliderBuilder::ball(r).build();
                                collider.set_translation(translation);
                                static_geometry.insert(collider);
                            }
                        }
                    }
                },
                Message::CurrentState {q, pose_x, pose_q} => {
                    current_state = Some(Message::CurrentState {q, pose_x, pose_q});
                }
            }
        }

        if let Some(Message::CurrentState {q, pose_x, pose_q}) = current_state {
            joint_angles = q;
            let x = Vector3::new(pose_x[0], pose_x[1], pose_x[2]);
            let rot = UnitQuaternion::from_quaternion(Quaternion::new(pose_q[0], pose_q[1], pose_q[2], pose_q[3]));
            
            if let Some(q) = on_message(joint_angles, &static_geometry, &x, &rot) {
                stream.write_all((serde_json::to_string(&q).unwrap() + "\n").as_bytes())?;
            }
        }
    }
}