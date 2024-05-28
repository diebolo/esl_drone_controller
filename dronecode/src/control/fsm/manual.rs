use fixed::types::I22F10;
use tudelft_quadrupel::led::Led;
use tudelft_quadrupel::motor::set_motors;
use crate::control::drone::Drone;
use crate::control::utils::calc_motors;

impl Drone{
     pub fn manual_operate(&mut self){
         Led::Green.on();
         // TUNE THESE MULTIPLIERS IF MANUAL MODE IS TOO STRONG / WEAK
         self.motor_ypr.yaw = I22F10::from_num(100) * self.js_ypr.yaw;
         self.motor_ypr.pitch = I22F10::from_num(500) * self.js_ypr.pitch;
         self.motor_ypr.roll = I22F10::from_num(500) * self.js_ypr.roll;
         // Send motor values
         set_motors(calc_motors(self.motor_ypr, I22F10::from_num(self.js_t)));
     }
}