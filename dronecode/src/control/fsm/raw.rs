use alloc::vec::Vec;
use fixed::types::I22F10;
use tudelft_quadrupel::barometer::read_pressure;
use tudelft_quadrupel::motor::set_motors;
use tudelft_quadrupel::mpu::read_raw;
use tudelft_quadrupel::uart::send_bytes;
use share_lib::{Command, serialize_message, Mode};
use crate::control::drone::Drone;
use crate::control::TICK_FREQ;
use crate::control::utils::calc_motors;
use crate::control::yaw_pitch_roll::YawPitchRoll;
use crate::filters::butterworth::ButterWorth;
use crate::filters::kalman::Kalman;
const DEG2RAD:f32 = 0.017; //ref value for degree to radian (pi/180)
pub struct RawData{
    pub prev_ypr:YawPitchRoll,
    pub current_ypr:YawPitchRoll,
    pub calibration_ypr_raw:YawPitchRoll,
    pub kalman:Kalman,
    pub butterworth:ButterWorth,
    pub roll_offset:I22F10,
}

impl RawData{
    pub fn new()->Self{
        RawData{
            prev_ypr: YawPitchRoll::new(),
            current_ypr:YawPitchRoll::new(),
            calibration_ypr_raw: YawPitchRoll::new(),
            kalman: Kalman::new(),
            butterworth: ButterWorth::new(),
            roll_offset:I22F10::from_num(0),
        }
    }
}

impl Drone {
    pub fn raw_operate(&mut self){
        self.read_raw_sensor();
        read_pressure();
        // + Yaw +

        // Calculate the sensor and reference velocity
        let sensor_velocity = self.raw_data.current_ypr.yaw;
        //                                                                              \/ CHANGE IF JOYSTICK NOT / TOO SENSITIVE 
        let ref_velocity = I22F10::from_num(3) * self.js_ypr.yaw; // radians per seconds


        // Calculate the PID value                \/ CHANGE IF P NOT BIG / SMALL ENOUGH
        self.motor_ypr.yaw = I22F10::from_num(0.7) * self.yaw_pid.p * (ref_velocity - sensor_velocity);

        if self.yaw_pid.p < 1 && self.yaw_pid.d < 1{
            self.motor_ypr.yaw = I22F10::from_num(100) * self.js_ypr.yaw;
        }

        // Send the motor values
        
        self.prev_sensor_ypr_control = self.sensor_ypr;
        // + Pitch +

        let p_ref = self.js_ypr.pitch; // radians
        let p_act = self.raw_data.current_ypr.pitch; // radians

        // PD Controller for p_ref and p_act
        let p_error = p_ref - p_act;
        let d_error = (p_error - self.prev_error_ypr.pitch) *  I22F10::from_num(TICK_FREQ/100); // rad/micros Calculate derivative error
        
        
        // Calculate the control signal              \/ CHANGE IF P NOT BIG / SMALL ENOUGH                    \/ CHANGE IF D NOT BIG / SMALL ENOUGH
        self.motor_ypr.pitch = I22F10::from_num(15) * self.pitch_pid.p * p_error + I22F10::from_num(150) * self.pitch_pid.d * d_error; // Calculate control signal

        self.prev_error_ypr.pitch = p_error; // Update last error for next iteration

        if self.pitch_pid.p < 1 && self.pitch_pid.d < 1 {
            self.motor_ypr.pitch = I22F10::from_num(500) * self.js_ypr.pitch;
        }

        // + Roll +

        let r_ref = self.js_ypr.roll; // radians
        let r_act = self.raw_data.current_ypr.roll; // radians

        // PD Controller for p_ref and p_act
        let p_error = r_ref - r_act;
        let d_error = (p_error - self.prev_error_ypr.roll) * I22F10::from_num(TICK_FREQ/100); // rad/micros Calculate derivative error
        
        
        // Calculate the PID value               \/ CHANGE IF P NOT BIG / SMALL ENOUGH                      \/ CHANGE IF D NOT BIG / SMALL ENOUGH
        self.motor_ypr.roll = I22F10::from_num(10) * self.roll_pid.p * p_error + I22F10::from_num(100) * self.roll_pid.d * d_error; // Calculate control signal

        self.prev_error_ypr.roll = p_error; // Update last error for next iteration

        if self.roll_pid.p < 1 && self.roll_pid.d < 1{
            self.motor_ypr.roll = I22F10::from_num(500) * self.js_ypr.roll;
        }

        let mut temp = Vec::new();
        temp.extend(serialize_message(Command::Trueyaw{num: self.raw_data.current_ypr.yaw.to_bits()}));
        temp.extend(serialize_message(Command::Truepitch{num: self.raw_data.kalman.pitchphi.to_bits()}));
        temp.extend(serialize_message(Command::Trueroll{num: self.raw_data.kalman.rollphi.to_bits()}));
        send_bytes(&temp);

        set_motors(calc_motors(self.motor_ypr, I22F10::from_num(self.js_t)));

    }

    pub fn read_raw_sensor(&mut self){
        self.raw_data.prev_ypr = self.raw_data.current_ypr;
        let (acc, speed) = read_raw().unwrap();//get the raw sensor data
        //transfer the raw sensor data to ideal value for calculation
        let acc_x = I22F10::from_num(acc.x);
        let acc_y = I22F10::from_num(acc.y);
        let acc_z = I22F10::from_num(acc.z);
        let speed_x = I22F10::from_num(speed.x);
        let speed_y = I22F10::from_num(speed.y);
        let mut speed_z = I22F10::from_num(speed.z)*I22F10::from_num(DEG2RAD);
        self.raw_data.kalman.filtering(acc_x,acc_y,acc_z,speed_x,speed_y);
        speed_z = self.raw_data.butterworth.filter(speed_z);
        
        // Check if the sensor has made a full rotation and add or subtract 2*PI
        if (self.raw_data.prev_ypr.roll - self.raw_data.roll_offset) > I22F10::from_num(2.5) && self.raw_data.kalman.rollphi < I22F10::from_num(-2.5) {
            self.roll_offset += I22F10::PI*2;
        }
        else if (self.raw_data.prev_ypr.roll - self.raw_data.roll_offset) < I22F10::from_num(-2.5) &&  self.raw_data.kalman.rollphi > I22F10::from_num(2.5) {
            self.roll_offset -= I22F10::PI*2;
        }
        self.raw_data.current_ypr = YawPitchRoll{
            yaw: speed_z,
            pitch: -self.raw_data.kalman.pitchphi,
            roll: self.raw_data.kalman.rollphi + self.raw_data.roll_offset,
        };

        if self.mode != Mode::Calibration {
            self.raw_data.current_ypr = self.raw_data.current_ypr.sub(&self.calibration_ypr_raw);
        }
        
    }
}