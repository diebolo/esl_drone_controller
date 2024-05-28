use alloc::vec::Vec;
use fixed::types::I22F10;
// use tudelft_quadrupel::barometer::read_pressure;
use tudelft_quadrupel::motor::get_motors;
use tudelft_quadrupel::uart::send_bytes;
use share_lib::{Command, Mode, serialize_message};
use crate::control::drone::Drone;
use crate::control::yaw_pitch_roll::YawPitchRoll;

//send the real_time value 0f  motors
pub fn send_motor_vals(){
    let motor = get_motors();
    let mut temp = Vec::new();
    temp.extend(serialize_message(Command::Motor1 { num:motor[0] }));
    temp.extend(serialize_message(Command::Motor2 { num:motor[1] }));
    temp.extend(serialize_message(Command::Motor3 { num:motor[2] }));
    temp.extend(serialize_message(Command::Motor4 { num:motor[3] }));
    send_bytes(&temp);
}

//send the calibration value
pub fn send_calibration_vals(ypr:YawPitchRoll){
    let mut temp = Vec::new();
    temp.extend(serialize_message(Command::Trueyaw{num: ypr.yaw.to_bits()}));
    temp.extend(serialize_message(Command::Truepitch{num: ypr.pitch.to_bits()}));
    temp.extend(serialize_message(Command::Trueroll{num: ypr.roll.to_bits()}));
    send_bytes(&temp);
}

//send the configuration value for joystick
pub fn send_configure_joystick_vals(drone: &Drone){
    if drone.mode == Mode::Manual|| drone.mode == Mode::YawControlled|| drone.mode == Mode::FullControl || drone.mode == Mode::Raw{
        let mut temp = Vec::new();
        temp.extend(serialize_message(Command::YawBack { num: I22F10::from_num(drone.js_ypr.yaw).to_bits() }));
        temp.extend(serialize_message(Command::PitchBack { num: I22F10::from_num(drone.js_ypr.pitch).to_bits() }));
        temp.extend(serialize_message(Command::RollBack { num: I22F10::from_num(drone.js_ypr.roll).to_bits() }));
        temp.extend(serialize_message(Command::ThrottleBack { num: drone.js_t }));
        send_bytes(&temp);
    }
}

//height calculation: H = 44330 * [1 - (P/p0)^(1/5.255) ]
