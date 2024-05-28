use fixed::types::I22F10;
use tudelft_quadrupel::barometer::read_pressure;
use crate::control::drone::Drone;
use tudelft_quadrupel::battery::read_battery;
use tudelft_quadrupel::led::Led::{Blue,  Red};
use tudelft_quadrupel::motor::get_motors;
use tudelft_quadrupel::mpu::read_raw;
use tudelft_quadrupel::time::{set_tick_frequency, wait_for_next_tick, Instant};
use tudelft_quadrupel::uart::{send_bytes,receive_bytes};
use share_lib::{ Message, Command, serialize_message, Mode};
use share_lib::Command::Datalog;
use crate::control::datalog::datalog;
use crate::control::info::{send_configure_joystick_vals, send_motor_vals};

pub mod drone;
mod utils;
mod datalog;
pub mod yaw_pitch_roll;
mod pid;
mod info;
mod fsm;
const TICK_FREQ: u64 = 100; // Tick frequency in Hz
///control loop when the drone is running
pub fn control_loop() -> ! {
    set_tick_frequency(TICK_FREQ);
    let mut last_keepalive = Instant::now();
    let mut real_buffer = [0;256];
    let mut keepalive_count: u32 = 0;
    let start_point = Instant::now();
    let mut start_flag = false;
    let mut len = 0;
    let mut drone = Drone::new();
    let mut count = 0;
    for i in 0.. {
        let mut buffer = [0;32];
        // let time1 = Instant::now();
        let now = Instant::now();
        let dt_keepalive = now.duration_since(last_keepalive);
        // let dt = now.duration_since(last);
        // last = now;


        let t = receive_bytes(&mut buffer);
        if t > 0 { // Bytes received from serial communication
            last_keepalive = now;
            keepalive_count = 0;
            let buffer_cpy = buffer.clone();
            for buf_idx in 0..t {
                if start_flag{
                    len = buffer_cpy[buf_idx] as usize;
                    start_flag = false;
                }
                if buffer_cpy[buf_idx] == 0xFE {
                    count = 0;
                    start_flag = true;
                }
                if count > 255 {
                    count = 0;
                }
                real_buffer[count] = buffer_cpy[buf_idx];
                count += 1;
                if buffer_cpy[buf_idx] == 0xFF && count > 2 && (len+5) == count {
                    match Message::get_message(&real_buffer[0..count]) {
                        Some(cmd) => {
                            last_keepalive = now;
                            drone.process_command(cmd);
                        },
                        None => {

                        }
                    }
                    count = 0;
                }
            }
        }

        //log message when it in yaw_control and raw mode
        if drone.mode != Mode::LogOut {
                drone.read_sensor_ypr();
                let read_val = drone.sensor_ypr;
                drone.read_raw_sensor();
                let raw_data = drone.raw_data.current_ypr;
                let count_point = Instant::now();
                let motor_val = get_motors();
                let (_,speed) = read_raw().unwrap();
                let time = count_point.duration_since(start_point);
                let time1 = Instant::now();
                drone.current_point = datalog(Datalog {
                    mode:drone.mode,
                    ypr: [read_val.yaw.to_num(), read_val.pitch.to_num(), read_val.roll.to_num()],
                    raw_ypr: [raw_data.yaw.to_num(), raw_data.pitch.to_num(), raw_data.roll.to_num()],
                    motor: motor_val,
                    time: time.as_millis(),
                    raw_speed:(speed.y as f32*0.017)
                },drone.current_point);
            let time2 = Instant::now();
            let t1 = time2.duration_since(time1).as_micros();
            send_bytes(&serialize_message(Command::Time {num:t1}));
            //send the configuration info for joystick
            if i % 9 == 0 {
                Blue.toggle();
                send_configure_joystick_vals(&drone);
            }
            //send real-time motor value
            if i % 2 == 0 {
                Blue.toggle();
                send_motor_vals();
            }
            //send real-time height value
            if i % 6 == 0 {

                // let pr = I22F10::from_num(read_temperature()).to_bits();
                // drone.pressure = height_clac(drone.pressure);
                let pr = I22F10::from_num(read_pressure());
                // drone.pressure = h_butterworth.filter(pr);
                drone.height.current_high = drone.height.calibration_p-pr;
                send_bytes(&serialize_message(Command::Height {num:(drone.height.calibration_p-pr).to_bits()}));
            }

        }

        let dt = dt_keepalive.as_micros();
        drone.operate(dt);

        if i % 5 == 0 && i > 200 { // prevents panic in the first 200 iterations
            keepalive_count += 1;
            if keepalive_count >= 20 {
                drone.process_command(Command::ModeChange { mode: share_lib::Mode::Panic }); // Go into panic mode
            }
            let bat = read_battery();
            if bat < 1050 && bat > 500{
                drone.process_command(Command::ModeChange { mode: share_lib::Mode::Panic }); // Go into panic mode
            }
            send_bytes(&serialize_message(Command::BatteryCheck { num: bat}));
        }
        // wait until the timer interrupt goes off again
        // based on the frequency set above
        wait_for_next_tick();
    }
    unreachable!();
}

