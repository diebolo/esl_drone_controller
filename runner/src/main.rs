use std::env::args;
use tudelft_serial_upload::{upload_file_or_stop, PortSelector};
use tudelft_serial_upload::serial2::SerialPort;
use std::{thread, time};
use share_lib::{Message, Mode, Command, serialize_message, serialize_messages};
use termion;
use termion::input::TermRead;
use std::time::{Duration, Instant};
use crate::keybinds::key_to_cmd;
use crate::interface::Interface;
use crate::interface::check_js;
use std::env;
use gilrs::Gilrs;
mod keybinds;
mod interface;
mod joystick;
mod gui;
mod threads;

use std::sync::{Arc, mpsc, Mutex};
use share_lib::Command::ModeChange;
use share_lib::Mode::Manual;
use crate::threads::{gui_threads, key_threads};

fn main() {
    env::set_var("RUST_BACKTRACE", "1");
    // get a filename from the command line. This filename will be uploaded to the drone
    // note that if no filename is given, the upload to the drone does not fail.
    // `upload_file_or_stop` will still try to detect the serial port on which the drone
    // is attached. This may be useful if you don't want to actually change the code on the
    // drone, but you do want to rerun your UI. In that case you simply don't provide any
    // command line parameter.
    let file = args().nth(1);
    let port = upload_file_or_stop(PortSelector::AutoManufacturer, file);

    // The code below shows a very simple start to a PC-side receiver of data from the drone.
    // You can extend this into an entire interface to the drone written in Rust. However,
    // if you are more comfortable writing such an interface in any other programming language
    // you like (for example, python isn't a bad choice), you can also call that here. The
    // commented function below gives an example of how to do that with python, but again
    // you don't need to choose python.

    let mut interface = Interface::new();

    // open the serial port we got back from `upload_file_or_stop`. This is the same port
    // as the upload occurred on, so we know that we can communicate with the drone over
    // this port.
    let mut serial = SerialPort::open(port, 115200).unwrap();
    serial.set_read_timeout(Duration::from_millis(50)).unwrap();
    let serial_port= Arc::new(Mutex::new(serial));

    //create two thread channels
    let (key_tx, key_rx) = mpsc::channel();
    let (gui_tx, gui_rx) = mpsc::channel();
    let serial_for_thread = serial_port.clone();

    let mut last = Instant::now();
    let mut last_keepalive = Instant::now();
    let  stdin = termion::async_stdin().keys();

    let mut gilrs = Gilrs::new().unwrap();

    let mut py_process = gui::start_interface();
    let stdout = py_process.stdout.take().unwrap();
    //spawn thread gui
    let _gui_handler = gui_threads(stdout,gui_tx);
    let mut real_buf = [0; 128];
    let mut count = 0;
    let mut start_flag = false;
    let mut len = 0;
    //spawn thread keyboard
    let _key_handler = key_threads(stdin,key_tx);

    thread::sleep(time::Duration::from_millis(500));

    while interface.work_flag {
        let start = Instant::now();
        let mut mes_package = Vec::new();
        //try to receive info from channel key
        let mut buf = [0; 128];
        let now = Instant::now();
        let dt = now.duration_since(last);
        let dt_keepalive = now.duration_since(last_keepalive);
        if let Ok(num) = serial_for_thread.lock().unwrap().read(&mut buf) {
            if num > 0 { // Serial communication, bytes received
                let buffer_cpy = buf.clone();
                for buf_idx in 0..num {
                    if start_flag{
                        len = buffer_cpy[buf_idx] as usize;
                        start_flag = false;
                    }
                    if buffer_cpy[buf_idx] == 0xFE {
                        count = 0;
                        start_flag = true;
                    }
                    if count > 127 {
                        count = 0;
                    }
                    real_buf[count] =buffer_cpy[buf_idx];
                    count += 1;
                    if buffer_cpy[buf_idx] == 0xFF && count > 2 && (len+5) == count {
                        // println!("{:?}",&real_buf[0..count]);
                        match Message::get_message(&real_buf[0..count]) {

                            Some(cmd) => {
                                last = now;
                                interface.idle = false;
                                interface.process_command(cmd);
                            }
                            None => {
                            }
                        };
                        count = 0;
                    }
                }
            } else {
                // Serial communication but no bytes???
            }
        } else { // No serial communication
            /*
                If there has been no output communication
                in the last share_lib::KEEPALIVE_T ms,
                set idle to true
            */
            if dt.as_millis() > share_lib::KEEPALIVE_T_MS {
                interface.idle = true;
            }
        }
        let duration1 = start.elapsed();
        // println!("Time elapsed in receive_function() is: {:?}", duration1);

        let joystick_time = Instant::now();
        // interface.js.read_joystick(&mut gilrs);
        match interface.js.read_joystick(&mut gilrs) {
            Some(cmd) => {
                mes_package.extend(serialize_messages(cmd));
                //correct value
                match interface.correct_ref(){
                    None => {}
                    Some(cmd) => {mes_package.extend(serialize_messages(cmd));}
                };
            }
            None => {

            }
        }
        let j_duration = joystick_time.elapsed();
        // println!("Time elapsed in joystick_function() is: {:?}", j_duration);
        //try to receive info from channel gui
        match gui_rx.try_recv() {
            Ok(cmd) => {
                match cmd {
                    Command::EXIT =>{interface.work_flag = false}
                    ModeChange { mode: Manual } => {
                        if check_js(&interface) {
                            mes_package.extend(serialize_message(cmd));
                        }
                    },
                    ModeChange { mode: Mode::Panic } => {
                        interface.js.throttle = 0;
                        mes_package.extend(serialize_message(cmd));
                    },
                    ModeChange { mode: Mode::Safe } => {
                        interface.js.throttle = 0;
                        mes_package.extend(serialize_message(cmd));
                    },
                    ModeChange { mode: Mode::YawControlled } => {
                        if check_js(&interface) {
                            mes_package.extend(serialize_message(cmd));
                        }
                    },
                    ModeChange { mode: Mode::Calibration } => {
                        if check_js(&interface) {
                            mes_package.extend(serialize_message(cmd));
                        }
                    },
                    ModeChange { mode: Mode::FullControl } => {
                        if check_js(&interface) {
                            mes_package.extend(serialize_message(cmd));
                        }
                    },
                    ModeChange { mode: Mode::Raw } => {
                        if check_js(&interface) {
                            mes_package.extend(serialize_message(cmd));
                        }
                    },
                    _ => {
                        mes_package.extend(serialize_message(cmd));
                    }
                }
            }
            Err(_) => {}
        }
        interface.print_interface();
        // Read input (if any)
        //try to receive info from channel serial
        match key_rx.try_recv() {
            Ok(key) =>{
                match key_to_cmd(key,&mut interface) {
                    Some(cmd) => {
                        mes_package.extend(serialize_message(cmd));
                    },
                    None => { // No next terminal stdin?
                        /*
                            If there has been no output communication
                            in the last share_lib::KEEPALIVE_TX_MS ms,
                            try to keep the UART alive
                        */
                    }
                }
            },
            Err(_) => {}
        }
        //correct value
        match interface.correct_ref(){
            None => {}
            Some(cmd) => {mes_package.extend(serialize_messages(cmd));}
        };
        //check alive
        if dt_keepalive.as_millis() > share_lib::KEEPALIVE_TX_MS {
            last_keepalive = now;
            mes_package.extend(serialize_message(Command::KeepAlive));
        }
        //send all messages to serial
        serial_for_thread.lock().unwrap().write(&mes_package).expect("can not write");
        serial_for_thread.lock().unwrap().flush().expect("flush crash host pc");
    }
    if let Err(e) = py_process.kill() {
        eprintln!("failed to kill python process: {}", e);
    }
    // gui_handler.join().unwrap();
    // key_handler.join().unwrap();

    println!("end program");
}

