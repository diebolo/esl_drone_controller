use tudelft_quadrupel::led::{ Red, Yellow};
use tudelft_quadrupel::motor::{get_motors, set_motors};
use tudelft_quadrupel::time::assembly_delay;
use share_lib::{Command, Mode};
use crate::control::drone::Drone;
use crate::control::yaw_pitch_roll::YawPitchRoll;

impl Drone{
    pub fn panic_operate(&mut self){
        self.js_ypr = YawPitchRoll::new();
        self.js_t = 0;
        Red.on();
        let motors = get_motors();
        let stage1 = (motors[0]+motors[1]+motors[2]+motors[3])/8;
        set_motors([stage1,stage1,stage1,stage1]);//reduce motor speed
        assembly_delay(200);//keep a little whil
        let stage2 = stage1/2;
        set_motors([stage2,stage2,stage2,stage2]);
        assembly_delay(100);//keep a little whil
        let stage3 = stage2/2;
        set_motors([stage3,stage3,stage3,stage3]);
        assembly_delay(100);//keep a little whil
        let stage4 = stage3/2;
        set_motors([stage4,stage4,stage4,stage4]);
        assembly_delay(100);//keep a little whil
        set_motors([0,0,0,0]);
        Red.off();
        self.process_command(Command::ModeChange { mode: Mode::Safe })
    }

    pub fn safe_operate(&mut self){
        Yellow.on();
        set_motors([0,0,0,0]);
        self.js_ypr = YawPitchRoll::new();
        self.js_t = 0;
    }
}