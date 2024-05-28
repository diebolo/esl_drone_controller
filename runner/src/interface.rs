use std::fs::{File, OpenOptions};
use std::io:: Write;
use share_lib::{Command, GyroValue, Mode, YPRT};
// use clearscreen;
use crate::joystick::Joystick;
use fixed::types::I22F10;

pub struct Interface {
    // Fields of the struct
    pub current_mode: Mode,
    pub height:i32,
    pub idle: bool,
    pub js: Joystick,
    pub configure: YPRT,
    pub yprt: GyroValue,
    motor: [u16; 4],
    pub abort: bool,
    pub pid_yaw:[i16;3],
    pub pid_pitch:[i16;3],
    pub pid_roll:[i16;3],
    pub work_flag:bool,
    pub battery: u16,
    pub velocity: i32,
}

impl Interface {
    pub fn new() -> Interface {
        let file = File::create("drone_data.txt").expect("create data file fail");
        // clean data file
        file.set_len(0).expect("fail reset file data");
        Interface{
            current_mode: Mode::Safe,
            height:0,
            idle: false,
            abort: false,
            js: Joystick::new(),
            configure:YPRT { yaw: 0.0, pitch: 0.0, roll: 0.0, throttle: 0 },
            yprt: GyroValue { yaw: 0, pitch: 0, roll: 0, throttle: 0 },
            motor: [0, 0, 0, 0],
            pid_yaw:[0,0,0],
            pid_pitch:[0,0,0],
            pid_roll:[0,0,0],
            work_flag:true,
            battery:0,
            velocity:0
        }
    }
    /// Processes various commands to manipulate drone states or configurations.
    ///
    /// # Parameters
    ///
    /// * `self` - A mutable reference to the current instance of the drone's context.
    /// * `cmd` - A `Command` variant that specifies the action to be taken.
    pub fn process_command(&mut self, cmd: Command) {
        match cmd {
            Command::Time {num}=>{
                println!("Time period is: {}",num);
            }
            Command::ModeChange { mode } => {
                self.current_mode = mode;
            }
            Command::ThrottleSet { num } => {
                self.yprt.throttle = num;

            }
            Command::Motor { motor1, motor2, motor3, motor4 } => {
                self.motor = [motor1, motor2, motor3, motor4];
            }
            Command::Motor1 { num} => {
                self.motor[0] = num;
            }
            Command::Motor2 { num} => {
                self.motor[1] = num;
            }
            Command::Motor3 { num} => {
                self.motor[2] = num;
            }
            Command::Motor4 { num} => {
                self.motor[3] = num;
            }
            Command::Trueyaw {num} =>{
                self.yprt.yaw =num;
            }
            Command::Truepitch {num} =>{
                self.yprt.pitch = num;
            }
            Command::Trueroll{num}=>{
                self.yprt.roll = num;
            }
            Command::YawSet {num}=>{
                self.yprt.yaw = num;
            }
            Command::YawPSet {num}=>{
                self.pid_yaw[0] = num;
            }
            Command::YawDSet {num}=>{
                self.pid_yaw[1] = num;
            }
            Command::PitchPSet {num}=>{
                self.pid_pitch[0] = num;
            }
            Command::PitchDSet {num}=>{
                self.pid_pitch[1] = num;
            }
            Command::RollPSet {num}=>{
                self.pid_roll[0] = num;
            }
            Command::RollDSet {num}=>{
                self.pid_roll[1] = num;
            }
            Command::YawBack {num}=>{
                self.configure.yaw = I22F10::from_bits(num).to_num();
            }
            Command::PitchBack {num}=>{
                self.configure.pitch = I22F10::from_bits(num).to_num();
            }
            Command::RollBack {num}=>{
                self.configure.roll = I22F10::from_bits(num).to_num();
            }
            Command::ThrottleBack {num}=>{
                self.configure.throttle = num;
            }
            Command::PitchSet {num}=>{
                self.yprt.pitch = num;
            }
            Command::Speed {num}=>{
                let t:i32 = I22F10::from_bits(num).to_num();
                println!("t:{}",t);
                self.velocity= num;
            }
            Command::BatteryCheck {num}=>{
                self.battery = num;
            }
            Command::Height {num}=>{
                self.height = I22F10::from_bits(num).to_num();
            }
            Command::Datalog{
                mode,
                ypr,
                raw_ypr,
                motor,
                time,
                raw_speed
            }=>{
                println!("get log");
                let mut file = OpenOptions::new()
                    .write(true)
                    .append(true)
                    .create(true)
                    .open("drone_data.txt").expect("open data file fail");

                let output = format!(
                    "mode:{}\n\
                    ypr:{},{},{}\n\
                     raw_ypr:{},{},{}\n\
                     motor:{},{},{},{}\n\
                     time:{}\n\
                     raw_speed:{}\n\
                     ",mode_format(mode),ypr[0],ypr[1],ypr[2],raw_ypr[0],raw_ypr[1],raw_ypr[2],
                    motor[0],motor[1],motor[2],motor[3],
                    time,raw_speed
                );

                match file.write_all(output.as_bytes()) {
                    Ok(_) => {},
                    Err(e) => eprintln!("write fail : {}", e),
                }
            }
            _ => {
            }
        }
    }
    fn mode_to_str(&self) -> &str {
        match self.current_mode {
            Mode::Safe => "Safe",
            Mode::Panic => "Panic",
            Mode::Manual => "Manual",
            Mode::Calibration => "Calibration",
            Mode::YawControlled => "Yaw Controlled",
            Mode::FullControl => "Full Control",
            Mode::Raw => "Raw",
            Mode::Height => "Height",
            Mode::LogOut => "LogOut"
        }
    }

    /// Prints the current interface settings to a file named `output.txt`.
    ///
    /// This method outputs the current status and configuration of the system to a text file.
    /// It captures various parameters like yaw, pitch, roll, throttle, and additional data
    pub fn print_interface(&self) {
        //open the data file
        let mut file = match OpenOptions::new().write(true).truncate(true).open("output.txt") {
            Ok(file) => file,
            Err(e) => {
                eprintln!("can not open the file: {}", e);
                return;
            }
        };
        //convert the data as print type
        let mut p1: f32 = I22F10::from_bits(self.yprt.yaw).to_num();
        let mut p2: f32 = I22F10::from_bits(self.yprt.pitch).to_num();
        let mut p3: f32 = I22F10::from_bits(self.yprt.roll).to_num();
        let mut _p4 =self.yprt.throttle;
        //select printed data
        if self.current_mode == Mode::Manual { // ||(self.current_mode == Mode::FullControl)
             p1= self.configure.yaw;
             p2 = self.configure.pitch;
             p3 = self.configure.roll;
             _p4 = self.configure.throttle;
        }
        let print = self.print_transfer();
        //package the data to txt file
        let output = format!(
            "{}\n\
             {}\n\
             {:.3}\n\
             {:.3}\n\
             {:.3}\n\
             {}\n\
             {}\n\
             {:.2}\n\
             {:.2}\n\
             {:.2}\n\
             {}\n\
             {:.3}\n\
             {:.3}\n\
             {:.3}\n\
             {}\n\
             {}\n\
             {}\n\
             {}\n\
             {}\n\
             {}\n\
             {}\n\
             {}\n\
             {}\n\
             {}\n\
             {}\n\
             {}\n\
             {}",
            self.mode_to_str(),
            self.idle,
            print[0],print[1],print[2], self.js.throttle,
            self.js.disconnect,
            p1,p2 ,p3 , self.yprt.throttle,
            self.js.y_trim, self.js.p_trim, self.js.r_trim, self.js.t_trim,
            self.motor[0], self.motor[1], self.motor[2], self.motor[3],self.pid_yaw[0],self.pid_yaw[1]
            ,self.pid_pitch[0],self.pid_pitch[1],self.pid_roll[0],self.pid_roll[1],
            self.battery,self.height
        );
        // write packages to txt file
        match file.write_all(output.as_bytes()) {
            Ok(_) => {},
            Err(e) => eprintln!("write fail : {}", e),
        }
    }


    pub fn print_transfer(&self)->[f32;3]{
        match self.current_mode {
            Mode::YawControlled => {[self.js.yaw.to_num(),0.0,0.0]}
            _ => {[self.js.yaw.to_num(),self.js.pitch.to_num(),self.js.roll.to_num()]},
        }
    }
    //correct the variable between pc and drone
    pub fn correct_ref(&self)->Option<Vec<Command>>{
        let mut temp = Vec::new();
        if self.current_mode == Mode::Manual|| self.current_mode == Mode::YawControlled||
            self.current_mode == Mode::FullControl ||self.current_mode == Mode::Raw{
            if (self.js.yaw + self.js.y_trim) != I22F10::from_num(self.configure.yaw) {
                temp.push(Command::YawSet{num: I22F10::from_num(self.js.yaw).to_bits()});
            }
            if (self.js.pitch + self.js.p_trim) != I22F10::from_num(self.configure.pitch) {
                temp.push(Command::PitchSet{num: I22F10::from_num(self.js.pitch).to_bits()});
            }
            if (self.js.roll + self.js.r_trim) != I22F10::from_num(self.configure.roll) {
                temp.push(Command::RollSet{num: I22F10::from_num(self.js.roll).to_bits()});
            }
            if (self.js.throttle + self.js.t_trim) != self.configure.throttle {
                temp.push(Command::ThrottleSet{num:self.js.throttle});
            }
        }
        if temp.is_empty() {
            None
        }else {
            Some(temp)
        }
    }
}


/// Checks if the joystick controls are in their neutral position.
///
/// # Parameters
///
/// * `interface` - A reference to the `Interface` struct that holds the joystick settings.
///
/// # Returns
///
/// Returns `true` if all joystick controls are zeroed out, indicating no input.
/// Returns `false` if any joystick control is not in the neutral position.
pub fn check_js(interface: &Interface) -> bool{
    if interface.js.throttle == 0 && interface.js.yaw == I22F10::from_num(0) && interface.js.pitch == I22F10::from_num(0) && interface.js.roll == I22F10::from_num(0) {
        return true;
    }
    else {
        return false;
    }
}
/// Converts a mode enum to its corresponding string representation.
///
/// # Parameters
///
/// * `mode` - A variant of the `Mode` enum representing the current mode of the drone
///
/// # Returns
///
/// Returns a `String` that describes the mode.
pub fn mode_format(mode:Mode) -> String {
    match mode {
        Mode::Safe => "Safe".to_string(),
        Mode::Panic => "Panic".to_string(),
        Mode::Manual => "Manual".to_string(),
        Mode::Calibration => "Calibration".to_string(),
        Mode::YawControlled => "Yaw Controlled".to_string(),
        Mode::FullControl => "Full Control".to_string(),
        Mode::Raw => "Raw".to_string(),
        Mode::Height => "Height".to_string(),
        Mode::LogOut => "LogOut".to_string()
    }
}