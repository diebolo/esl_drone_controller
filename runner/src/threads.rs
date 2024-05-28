use std::{io, thread};
use std::io::BufRead;
use std::process::ChildStdout;
use std::sync::mpsc::Sender;
use std::thread::JoinHandle;
use std::time::{Duration, Instant};
use termion::AsyncReader;
use termion::event::Key;
use termion::input::Keys;
use termion::raw::IntoRawMode;
use share_lib::{Command, Mode};
/// Spawns a thread to read lines from a given output and send commands to a GUI via a channel.
///
/// # Parameters
///
/// * `stdout` - A `ChildStdout` from which lines are read. Typically this is stdout from a child process.
/// * `gui_tx` - A `Sender<Command>` used to send commands to the GUI component.
///
/// # Returns
///
/// Returns a `JoinHandle<()>` for the spawned thread, allowing the calling code to manage the thread .
///
pub fn gui_threads(stdout:ChildStdout,gui_tx:Sender<Command>) ->JoinHandle<()>{
    thread::spawn(move || {
        let reader = io::BufReader::new(stdout);
        for line in reader.lines() {
            let start = Instant::now();
            if let Ok(line) = line {
                match line.as_str() {
                    "Safe button pressed" => {
                        println!("safe get");
                        gui_tx.send(Command::ModeChange {mode:Mode::Safe}).expect("gui error");
                    }
                    "Panic button pressed" => {
                        gui_tx.send(Command::ModeChange {mode:Mode::Panic}).expect("gui error");
                    }
                    "Manual button pressed" => {
                        println!("Manual get");
                        gui_tx.send(Command::ModeChange {mode:Mode::Manual}).expect("gui error");
                    }
                    "Calibration button pressed" => {
                        println!("Calibration get");
                        gui_tx.send(Command::ModeChange {mode:Mode::Calibration}).expect("gui error");
                    }
                    "Yaw button pressed" => {
                        println!("Yaw get");
                        gui_tx.send(Command::ModeChange {mode:Mode::YawControlled}).expect("gui error");
                    }
                    "Raw button pressed" => {
                        println!("Full get");
                        gui_tx.send(Command::ModeChange {mode:Mode::Raw}).expect("gui error");
                    }
                    "Full button pressed" => {
                        println!("Full get");
                        gui_tx.send(Command::ModeChange {mode:Mode::FullControl}).expect("gui error");
                    }
                    "Log button pressed" => {
                        println!("Log get");
                        gui_tx.send(Command::ModeChange {mode:Mode::LogOut}).expect("gui error");
                    }
                    "exit" => {
                        println!("end program");
                        gui_tx.send(Command::EXIT).expect("gui error");
                    }
                    _ => {
                        println!("{} ",line.as_str());
                        let mut parts = line.as_str().split(':');
                        if let Some(pid_type) = parts.next() {
                            match pid_type {
                                "yaw_p" =>{
                                    if let Some(val) = parts.next() {
                                        gui_tx.send(Command::YawPSet {num: val.trim().parse::<i16>().expect("parse i16 fail at gui") }).expect("gui error");
                                        // println!("{}",val.trim().parse::<i16>().unwrap());
                                    }
                                }
                                "yaw_d" =>{
                                    if let Some(val) = parts.next() {
                                        gui_tx.send(Command::YawDSet {num: val.trim().parse::<i16>().expect("parse i16 fail at gui") }).expect("gui error");
                                        // println!("{}",val.trim().parse::<i16>().unwrap());
                                    }
                                }
                                "pitch_p" =>{
                                    if let Some(val) = parts.next() {
                                        gui_tx.send(Command::PitchPSet {num: val.trim().parse::<i16>().expect("parse i16 fail at gui") }).expect("gui error");
                                        // println!("{}",val.trim().parse::<i16>().unwrap());
                                    }
                                }
                                "pitch_d" =>{
                                    if let Some(val) = parts.next() {
                                        gui_tx.send(Command::PitchDSet {num: val.trim().parse::<i16>().expect("parse i16 fail at gui") }).expect("gui error");
                                        // println!("{}",val.trim().parse::<i16>().unwrap());
                                    }
                                }
                                "roll_p" =>{
                                    if let Some(val) = parts.next() {
                                        gui_tx.send(Command::RollPSet {num: val.trim().parse::<i16>().expect("parse i16 fail at gui") }).expect("gui error");
                                        // println!("{}",val.trim().parse::<i16>().unwrap());
                                    }
                                }
                                "roll_d" =>{
                                    if let Some(val) = parts.next() {
                                        gui_tx.send(Command::RollDSet {num: val.trim().parse::<i16>().expect("parse i16 fail at gui") }).expect("gui error");
                                        // println!("{}",val.trim().parse::<i16>().unwrap());
                                    }
                                }
                                _ => {}
                            }
                        }
                    }
                }
            }
            let duration = start.elapsed();
            // println!("Time elapsed in gui_thread_function() is: {:?}", duration);
        }

    })
}
/// Spawns a thread to read keyboard inputs and send keys via a channel.
///
/// # Parameters
///
/// * `stdin` - A mutable reference to an `AsyncReader` for reading keyboard inputs asynchronously.
/// * `key_tx` - A `Sender<Key>` used to transmit the keys to another part of the application.
///
/// # Returns
///
/// Returns a `JoinHandle<()>` for the spawned thread, allowing for management of the thread's lifecycle.
///
pub fn key_threads(mut stdin:Keys<AsyncReader>, key_tx:Sender<Key>) ->JoinHandle<()>{
    thread::spawn(move || {
        loop {
            let start = Instant::now();
            let stdout = io::stdout().into_raw_mode().unwrap();
            // Read input (if any)
            let input = stdin.next();
            drop(stdout);
            if let Some(Ok(key)) = input {
                key_tx.send(key).expect("keyboard error");
            }
            let duration = start.elapsed();
            // println!("Time elapsed in key_function() is: {:?}", duration);
            thread::sleep(Duration::from_millis(35));

        }
    })
}