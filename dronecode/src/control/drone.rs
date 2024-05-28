use share_lib::{Command, serialize_message, Mode};
use tudelft_quadrupel::led::Led::{self, Yellow};
use tudelft_quadrupel::uart::send_bytes;
use tudelft_quadrupel::block;
use tudelft_quadrupel::mpu::{read_dmp_bytes};
use fixed::types::I22F10;
use tudelft_quadrupel::motor::set_motor_max;
use crate::control::pid::PID;
use crate::control::yaw_pitch_roll::YawPitchRoll;
use crate::control::fsm::raw::RawData;
use crate::control::fsm::height::Height;


pub struct Drone {
    pub mode: Mode,
    pub js_ypr: YawPitchRoll, // YPR reference from joystik
    pub js_t: i16, // Throttle reference from joystick

    pub prev_sensor_ypr: YawPitchRoll, // Previous measured YPR on drone
    pub calibration_ypr: YawPitchRoll, // Calibration YPR
    pub calibration_ypr_raw: YawPitchRoll, // RAW Calibration YPR
    pub prev_error_ypr: YawPitchRoll, // Previous error for D-control
    pub prev_sensor_ypr_control: YawPitchRoll, // Previous measured YPR on drone
    pub sensor_ypr: YawPitchRoll, // Current YPR on drone
    pub motor_ypr: YawPitchRoll, // YPRT that is send to motor control

    pub yaw_pid: PID, // PID values for the yaw control
    pub pitch_pid: PID, // PID values for the pitch control
    pub roll_pid: PID, // PID values for the roll control

    pub yaw_offset: I22F10, // Offset for the yaw control
    pub roll_offset: I22F10, // Offset for the roll control
    pub prev_time: I22F10, // Previous time
    pub pc_counter: u32,
    pub current_point:u32,
    pub raw_data: RawData,
    pub height:Height,
}

impl Drone {
    pub fn new() -> Self {
        set_motor_max(1000);
        Drone {
            mode: Mode::Safe,
            js_ypr: YawPitchRoll::new(),
            js_t: 0,
            prev_sensor_ypr: YawPitchRoll::new(),
            prev_sensor_ypr_control: YawPitchRoll::new(),
            calibration_ypr: YawPitchRoll::new(),
            calibration_ypr_raw:YawPitchRoll::new(),
            prev_error_ypr: YawPitchRoll::new(),
            sensor_ypr: YawPitchRoll::new(),
            motor_ypr: YawPitchRoll::new(),
            yaw_pid: PID::new(),
            pitch_pid: PID::new(),
            roll_pid: PID::new(),
            yaw_offset: I22F10::from_num(0),
            roll_offset: I22F10::from_num(0),
            prev_time: I22F10::from_num(0),
            pc_counter:0,
            current_point:0,
            raw_data:RawData::new(),
            height:Height::new(),
        }
    }

    pub fn process_command(&mut self, cmd:Command){
        if (self.mode == Mode::Manual)||(self.mode == Mode::YawControlled)||
            (self.mode == Mode::FullControl)||(self.mode == Mode::Raw)||(self.mode == Mode::Height){
            // self.commandmatch(cmd);
            match cmd {
                Command::ModeChange { mode:m}=> {
                    self.mode_match(m);
                }
                //message timeout test add, new situation!
                _ => {
                    self.commandmatch(cmd);
                },
            }

        }else {
            match cmd {
                Command::ModeChange { mode } => {
                    if self.mode != mode {
                        self.mode_match(mode);
                    }
                }
                _ => {},
            }
        }
    }

    pub fn mode_match(&mut self, mode:Mode){
        Led::Red.off();
        Led::Green.off();
        Yellow.off();
        self.mode_limit_check(mode);
        send_bytes(&serialize_message(Command::ModeChange { mode: self.mode }));
    }

    pub fn read_sensor_ypr(&mut self){
        let quaternion = block!(read_dmp_bytes()).unwrap();
        let sensor_ypr = YawPitchRoll::from(quaternion);
        self.prev_sensor_ypr = self.sensor_ypr;

        // Check if the sensor has made a full rotation and add or subtract 2*PI
        if (self.prev_sensor_ypr.yaw - self.yaw_offset) > I22F10::from_num(2.5) && sensor_ypr.yaw < I22F10::from_num(-2.5) {
            self.yaw_offset += I22F10::PI*2;
        }
        else if (self.prev_sensor_ypr.yaw - self.yaw_offset) < I22F10::from_num(-2.5) && sensor_ypr.yaw > I22F10::from_num(2.5) {
            self.yaw_offset -= I22F10::PI*2;
        }
        if (self.prev_sensor_ypr.roll - self.roll_offset) > I22F10::from_num(2.5) && sensor_ypr.roll < I22F10::from_num(-2.5) {
            self.roll_offset += I22F10::PI*2;
        }
        else if (self.prev_sensor_ypr.roll - self.roll_offset) < I22F10::from_num(-2.5) && sensor_ypr.roll > I22F10::from_num(2.5) {
            self.roll_offset -= I22F10::PI*2;
        }

        // Add offset to new sensor values
        self.sensor_ypr = sensor_ypr;
        self.sensor_ypr.pitch = -self.sensor_ypr.pitch;
        self.sensor_ypr.yaw += self.yaw_offset;
        self.sensor_ypr.roll += self.roll_offset;

        if self.mode != Mode::Calibration {
            self.sensor_ypr = self.sensor_ypr.sub(&self.calibration_ypr);
        }
    }

    pub fn operate(&mut self, _dt: u128){
        match self.mode {
            Mode::Safe => {
                self.safe_operate();
            },
            Mode::Panic => {
                self.panic_operate();
            },
            Mode::Manual => {
                self.manual_operate();
            },
            Mode::Calibration => {
                self.calibration_operate();
            },
            Mode::Raw => {
                self.raw_operate();
            },
            Mode::YawControlled => {
                self.yaw_operate();
            },
            Mode::FullControl=>{
                self.full_operate();
            },
            Mode::Height=>{
                self.height_operate();
            },
            Mode::LogOut=>{
                if self.pc_counter < 0x01FFFE {
                    match self.pick_up_message() {
                        None => {}
                        Some(cmd) => {
                            Yellow.toggle();
                            send_bytes(&serialize_message(cmd));
                        }
                    }
                    self.pc_counter += 64;
                }else {
                    self.pc_counter = 0x000000
                }
            }
        }

    }


    fn commandmatch(&mut self, cmd: Command){
        match cmd {
            Command::ThrottleSet{num}=>{
                self.js_t = num;
                send_bytes(&serialize_message(Command::ThrottleSet {num}));
            }
            Command::YawSet{num}=>{
                self.js_ypr.yaw = I22F10::from_bits(num);
                let mut mes = share_lib::Message::new(Command::PitchSet {num});
                let serialized = mes.build_message();
                send_bytes(&serialized);
            }
            Command::PitchSet{num}=>{
                self.js_ypr.pitch = I22F10::from_bits(num);
            }
            Command::RollSet{num}=>{
                self.js_ypr.roll = I22F10::from_bits(num);
            },
            Command::YawPSet{num}=>{
                self.yaw_pid.p = I22F10::from_num(num);
                send_bytes(&serialize_message(Command::YawPSet {num:self.yaw_pid.p.to_num()}));
            }
            Command::YawDSet{num}=>{
                self.yaw_pid.d = I22F10::from_num(num);
                send_bytes(&serialize_message(Command::YawDSet {num:self.yaw_pid.d.to_num()}));
            }
            Command::PitchPSet{num}=>{
                self.pitch_pid.p = I22F10::from_num(num);
                send_bytes(&serialize_message(Command::PitchPSet {num:self.pitch_pid.p.to_num()}));
            }
            Command::PitchDSet{num}=>{
                self.pitch_pid.d = I22F10::from_num(num);
                send_bytes(&serialize_message(Command::PitchDSet {num:self.pitch_pid.d.to_num()}));
            }
            Command::RollPSet{num}=>{
                self.roll_pid.p = I22F10::from_num(num);
                send_bytes(&serialize_message(Command::RollPSet {num:self.roll_pid.p.to_num()}));
            }
            Command::RollDSet{num}=>{
                self.roll_pid.d = I22F10::from_num(num);
                send_bytes(&serialize_message(Command::RollDSet {num:self.roll_pid.d.to_num()}));
            }
            _=> {

            },
        }

    }
    pub fn mode_limit_check(&mut self,mode:Mode) {
        //if we are in operation modes, we can only enter the panic or safe mode
        if self.mode == Mode::Calibration || self.mode == Mode::Manual || self.mode
            == Mode::YawControlled || self.mode == Mode::Raw || self.mode == Mode::FullControl{
            if mode == Mode::Safe || mode == Mode::Panic{
                self.mode = mode;
            }
        }else {
            self.mode = mode;
        }
    }
}







