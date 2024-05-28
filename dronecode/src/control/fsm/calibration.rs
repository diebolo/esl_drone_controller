use alloc::vec::Vec;
use fixed::types::I22F10;
use tudelft_quadrupel::barometer::read_pressure;
use share_lib::Command;
use crate::control::drone::Drone;
use crate::control::info::send_calibration_vals;
use crate::control::yaw_pitch_roll::YawPitchRoll;


impl Drone{
    pub fn calibration_operate(&mut self){
        self.roll_offset = I22F10::from_num(0);
        self.yaw_offset = I22F10::from_num(0);
        // create the data base of ypr and raw ypr
        let mut data_base_ypr = Vec::new();
        let mut data_base_raw = Vec::new();
        let mut data_base_height = Vec::new();
        // store ypr and raw ypr
        for _i in 0..20 {
            self.read_sensor_ypr();
            self.read_raw_sensor();
            data_base_height.push(I22F10::from_num(read_pressure()));
            data_base_ypr.push(self.sensor_ypr);
            data_base_raw.push(self.raw_data.current_ypr);
        }
        // get the average value as the calibration value
        self.calibration_ypr_raw = YawPitchRoll{
            yaw:data_base_raw.iter().map(|item| item.yaw).sum::<I22F10>() / I22F10::from_num(data_base_raw.len()),
            pitch:data_base_raw.iter().map(|item| item.pitch).sum::<I22F10>() / I22F10::from_num(data_base_raw.len()),
            roll: data_base_raw.iter().map(|item| item.roll).sum::<I22F10>() / I22F10::from_num(data_base_raw.len()) };
        self.calibration_ypr= YawPitchRoll{
            yaw:data_base_ypr.iter().map(|item| item.yaw).sum::<I22F10>() / I22F10::from_num(data_base_ypr.len()),
            pitch:data_base_ypr.iter().map(|item| item.pitch).sum::<I22F10>() / I22F10::from_num(data_base_ypr.len()),
            roll: data_base_ypr.iter().map(|item| item.roll).sum::<I22F10>() / I22F10::from_num(data_base_ypr.len()) };
        //send back the calibration value to pc
        send_calibration_vals(self.sensor_ypr);

        self.height.calibration_p = data_base_height.iter().map(|item| item).sum::<I22F10>() / I22F10::from_num(data_base_raw.len());

        //go to the safe mode
        self.process_command(Command::ModeChange { mode: share_lib::Mode::Safe });
    }
}