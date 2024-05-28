use fixed::types::I22F10;
use tudelft_quadrupel::motor::set_motors;
use crate::control::drone::Drone;
use crate::control::TICK_FREQ;
use crate::control::utils::calc_motors;

impl Drone{
    pub fn yaw_operate(&mut self){
        self.read_sensor_ypr();
        // + Yaw +

        // Calculate the sensor and reference velocity
        let sensor_velocity = I22F10::from_num(TICK_FREQ) * (self.sensor_ypr.yaw - self.prev_sensor_ypr_control.yaw);
        
        //                                                                              \/ CHANGE IF JOYSTICK NOT / TOO SENSITIVE 
        let ref_velocity = I22F10::from_num(3) * self.js_ypr.yaw; // radians per seconds


        // Calculate the PID value                \/ CHANGE IF P NOT BIG / SMALL ENOUGH
        self.motor_ypr.yaw = I22F10::from_num(1) * self.yaw_pid.p * (ref_velocity - sensor_velocity);

        if self.yaw_pid.p < 1 && self.yaw_pid.d < 1 {
            self.motor_ypr.yaw = I22F10::from_num(100) * self.js_ypr.yaw;
        }

        self.motor_ypr.roll = I22F10::from_num(500) * self.js_ypr.roll;
        self.motor_ypr.pitch = I22F10::from_num(500) * self.js_ypr.pitch;

        set_motors(calc_motors(self.motor_ypr, I22F10::from_num(self.js_t)));
        self.prev_sensor_ypr_control = self.sensor_ypr;
    }        
}
