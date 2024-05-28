use alloc::vec::Vec;
use fixed::types::I22F10;
use tudelft_quadrupel::motor::set_motors;
use tudelft_quadrupel::uart::send_bytes;
use share_lib::{Command, serialize_message};
use crate::control::drone::Drone;
use crate::control::pid::PID;
use crate::control::TICK_FREQ;
use crate::control::utils::calc_motors;
use crate::filters::butterworth::ButterWorth;

pub struct Height{
    pub pid:PID,
    pub current_throttle:I22F10,
    pub prev_high:I22F10,
    pub prev_error:I22F10,
    pub current_high:I22F10,
    pub calibration_p:I22F10,
    pub butterworth:ButterWorth,
}

impl Height{
    pub fn new()->Self{
       Height{
           pid:PID::new(),
           prev_high: I22F10::from_num(0),
           current_throttle: I22F10::from_num(0),
           prev_error:I22F10::from_num(0),
           current_high: I22F10::from_num(0),
           calibration_p: I22F10::from_num(0),
           butterworth: ButterWorth::new(),
       }
    }
    //height filter
    pub fn height_update(&mut self){
        self.current_high = self.butterworth.filter(self.current_high);
    }
}

impl Drone{
    pub fn calc_high_throttle(&mut self){
        self.height.pid = PID{
            p: I22F10::from_num(20),
            i: I22F10::from_num(0),
            d: I22F10::from_num(20),
        };
        let t_ref = I22F10::from_num(-self.js_t); // target height
        let t_act = self.height.current_high*I22F10::from_num(50); // radians
        // PD Controller for t_ref and t_act
        let p_error = t_ref - t_act;
        let d_error = (p_error - self.height.prev_error) *  I22F10::from_num(TICK_FREQ/100); // rad/micros Calculate derivative error

        // Calculate the control signal              \/ CHANGE IF P NOT BIG / SMALL ENOUGH                    \/ CHANGE IF D NOT BIG / SMALL ENOUGH
        self.height.current_throttle = -I22F10::from_num(0.05) * (self.height.pid.p)  * p_error;
            // + I22F10::from_num(100) * self.height.pid.d * d_error; // Calculate control signal
        send_bytes(&serialize_message(Command::Speed {num:self.height.current_throttle.to_bits()}));
        self.height.prev_error = p_error; // Update last error for next iteration
        self.height.prev_high = self.height.current_high;
        // if self.height.pid.p < 1 && self.height.pid.d < 1{
        if self.js_t > -185{
            self.height.current_throttle = I22F10::from_num(self.js_t);
        }
    }

    pub fn height_operate(&mut self){
        //get the value of ypr
        self.read_sensor_ypr();
        //update the height value via filter
        self.height.height_update();
        //calculate the lift rate and then translate it to throttle value
        self.calc_high_throttle();
        // + Yaw +

        // Calculate the sensor and reference velocity
        let sensor_velocity = I22F10::from_num(TICK_FREQ) * (self.sensor_ypr.yaw - self.prev_sensor_ypr_control.yaw);

        //                                                                              \/ CHANGE IF JOYSTICK NOT / TOO SENSITIVE ENOUGH
        let ref_velocity = I22F10::from_num(3) * self.js_ypr.yaw; // radians per seconds


        // Calculate the PID value                \/ CHANGE IF P NOT BIG / SMALL ENOUGH
        self.motor_ypr.yaw = I22F10::from_num(1) * self.yaw_pid.p * (ref_velocity - sensor_velocity);

        if self.yaw_pid.p < 1 && self.yaw_pid.d < 1 {
            self.motor_ypr.yaw = I22F10::from_num(100) * self.js_ypr.yaw;
        }


        // + Pitch +

        let p_ref = self.js_ypr.pitch; // radians
        let p_act = self.sensor_ypr.pitch; // radians

        // PD Controller for p_ref and p_act
        let p_error = p_ref - p_act;
        let d_error = (p_error - self.prev_error_ypr.pitch) *  I22F10::from_num(TICK_FREQ/100); // rad/micros Calculate derivative error

        // Calculate the control signal              \/ CHANGE IF P NOT BIG / SMALL ENOUGH                    \/ CHANGE IF D NOT BIG / SMALL ENOUGH
        self.motor_ypr.pitch = I22F10::from_num(10) * self.pitch_pid.p * p_error + I22F10::from_num(100) * self.pitch_pid.d * d_error; // Calculate control signal

        self.prev_error_ypr.pitch = p_error; // Update last error for next iteration

        if self.pitch_pid.p < 1 && self.pitch_pid.d < 1{
            self.motor_ypr.pitch = I22F10::from_num(500) * self.js_ypr.pitch;
        }


        // + Roll +

        let r_ref = self.js_ypr.roll; // radians
        let r_act = self.sensor_ypr.roll; // radians

        // PD Controller for p_ref and p_act
        let p_error = r_ref - r_act;
        let d_error = (p_error - self.prev_error_ypr.roll)*I22F10::from_num(TICK_FREQ/100); // rad/micros Calculate derivative error

        // Calculate the PID value               \/ CHANGE IF P NOT BIG / SMALL ENOUGH                      \/ CHANGE IF D NOT BIG / SMALL ENOUGH
        self.motor_ypr.roll = I22F10::from_num(10) * self.roll_pid.p * p_error + I22F10::from_num(100) * self.roll_pid.d * d_error; // Calculate control signal

        self.prev_error_ypr.roll = p_error; // Update last error for next iteration

        if self.roll_pid.p < 1 && self.roll_pid.d < 1{
            self.motor_ypr.roll = I22F10::from_num(500) * self.js_ypr.roll;
        }


        let mut temp = Vec::new();
        temp.extend(serialize_message(Command::Trueyaw{num: self.sensor_ypr.yaw.to_bits()}));
        temp.extend(serialize_message(Command::Truepitch{num: self.sensor_ypr.pitch.to_bits()}));
        temp.extend(serialize_message(Command::Trueroll{num: self.sensor_ypr.roll.to_bits()}));
        send_bytes(&temp);

        // Send the motor values
        set_motors(calc_motors(self.motor_ypr, self.height.current_throttle));
        self.prev_sensor_ypr_control = self.sensor_ypr;
    }
}