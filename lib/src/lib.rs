#![cfg_attr(not(test), no_std)]
#[cfg(test)]
extern crate std;
extern crate alloc;
use serde::{Serialize, Deserialize};
use alloc::vec::Vec;
use postcard::{to_allocvec, from_bytes};
use fixed::types::I22F10;
use crc_any::CRCu16;
// Keepalive timers
pub const KEEPALIVE_T_MS: u128 = 100; // Keepalive timer in ms
pub const KEEPALIVE_TX_MS: u128 = KEEPALIVE_T_MS-50; // Keepalive timer for sending in ns
pub const YAW_SCALE: f32 = 200.0; // Scale the val form joystick to motor cal
pub const PITCH_SCALE: f32 = 800.0; // Scale the val form joystick to motor cal
pub const ROLL_SCALE: f32 = 800.0; // Scale the val form joystick to motor cal
pub const THROTTLE_SCALE: f32 = 1000.0; // Scale the val form joystick to motor cal
pub const MESSAGE_LEN: usize = 64;


// drone mode
#[derive(Serialize, Deserialize, PartialEq,Clone,Copy)]
pub enum Mode {
    Safe,
    Panic,
    Manual,
    Calibration,
    YawControlled,
    FullControl,
    Raw,
    Height,
    LogOut
}

//communication and log  command
#[derive(Serialize, Deserialize, PartialEq,Clone)]
pub enum Command {
    EXIT,
    KeepAlive,
    ModeChange{mode:Mode},
    YawPSet{num: i16},
    YawDSet{num: i16},
    PitchPSet{num: i16},
    PitchDSet{num: i16},
    RollPSet{num: i16},
    RollDSet{num: i16},
    Height{num:i32},
    Time{num:u128},
    ThrottleSet{num: i16},
    YawBack{num: i32},
    PitchBack{num:i32},
    RollBack{num:i32},
    ThrottleBack{num:i16},
    YawSet{num: i32},
    PitchSet{num:i32},
    RollSet{num:i32},
    Trueyaw{num : i32},
    Truepitch{num : i32},
    Trueroll{num:i32},
    Speed{num:i32},
    Datalog{
      mode: Mode,
      ypr:[f32;3],
      raw_ypr:[f32;3],
      motor:[u16;4],
      time: u128,
      raw_speed:f32
    },
    Motor{
        motor1: u16,
        motor2: u16,
        motor3: u16,
        motor4: u16,
    },
    Motor1{
        num: u16,
    },
    Motor2{
        num: u16,
    },
    Motor3{
        num: u16,
    },
    Motor4{
        num: u16,
    },
    BatteryCheck{num: u16},
}

impl Command {
    //serialize the command as bits
    pub fn serialize(&self) -> Vec<u8> {
        to_allocvec(self).unwrap()
    }
}

#[derive(Serialize, Deserialize, PartialEq)]
pub struct Message {
    start_byte: u8,
    command: Command,
    end_byte: u8,
}

impl Message {
    pub fn new(command: Command) -> Self{
        Self {
            start_byte: 0xFE,
            command,
            end_byte: 0xFF,
        }
    }
    //serialize the command as a message according to our protocol
    pub fn build_message(&mut self) -> Vec<u8> {
        //get serialized command
        let serialized_command = self.command.serialize();

        //get command length
        let len = serialized_command.len() as u8;

        // Create CRC instance
        let mut crc = CRCu16::crc16ccitt_false();

        // Update CRC with data
        crc.digest(&serialized_command);

        // Get CRC value
        let temp = crc.get_crc();
        let check_sum:[u8;2] = (temp%511).to_be_bytes();

        //package all info to protocol format
        let mut buffer = Vec::new();
        buffer.push(self.start_byte);
        buffer.push(len);
        buffer.extend_from_slice(serialized_command.as_slice());
        // buffer.push(0x20);   // test for unmatched check_sum
        let t1 = check_sum[0]%253;
        let t2 = check_sum[1]%253;
        buffer.push(t1);
        buffer.push(t2);
        buffer.push(self.end_byte);
        buffer
    }

    pub fn get_message(received_message: &[u8]) -> Option<Command>{
        let length = received_message.len();
        if received_message.starts_with(&[0xFE]) && received_message.ends_with(&[0xFF]) {
            let expected_len = received_message[1] as usize ;
            let expected_check_sum = received_message[length - 3] ;
            let expected_check_sum1 = received_message[length - 2] ;
            let received_command = &received_message[2..length - 3];

            // Create CRC instance
            let mut crc = CRCu16::crc16ccitt_false();

            // Update CRC with data
            crc.digest(&received_command);

            // Get CRC value
            let calculate_check_sum = crc.get_crc();
            let verify_sum:[u8;2] = (calculate_check_sum%511).to_be_bytes();
            if (expected_len == received_command.len())&&(expected_check_sum == verify_sum[0]%253) && (expected_check_sum1 == verify_sum[1]%253) {
                let command: Command = from_bytes(received_command).unwrap();
                return Some(command)
            }
        }
        None
    }

    pub fn build_message_log(&mut self) -> Vec<u8> {
        let serialized_command = self.command.serialize();
        let mes_len = serialized_command.len() as u8;
        // Pad serialized command to fixed length
        let padded_serialized_command =  {
            let mut padded = serialized_command.to_vec();
            padded.resize(59, 0); // Pad with zeros
            padded
        };

        let temp: u16 = padded_serialized_command.iter().map(|&b| b as u16).sum();
        let check_sum: [u8; 2] = (temp%256).to_be_bytes();
        let mut buffer = Vec::new();
        buffer.push(self.start_byte);
        buffer.push(mes_len);
        buffer.extend_from_slice(&padded_serialized_command);
        let t1 = check_sum[0];
        let t2 = check_sum[1];
        buffer.push(t1);
        buffer.push(t2);
        buffer.push(self.end_byte);
        buffer
    }

    pub fn get_message_log(received_message: &[u8]) -> Option<Command>{
        let length = received_message.len();
        if received_message.starts_with(&[0xFE]) && received_message.ends_with(&[0xFF]) {
            let expected_length = received_message[1];
            let expected_check_sum = received_message[length - 3];
            let expected_check_sum1 = received_message[length - 2];
            let received_command = &received_message[2..(2+expected_length) as usize];
            let calculate_check_sum: u16 = received_command.iter().map(|&b| b as u16).sum();
            let verify_sum:[u8;2] = (calculate_check_sum%256).to_be_bytes();
            if (expected_check_sum == verify_sum[0]) && (expected_check_sum1 == verify_sum[1]) {
                let command: Command = from_bytes(received_command).unwrap();
                return Some(command)
            }
        }
        None
    }

}

pub fn serialize_message(command: Command) -> Vec<u8>{
    let mut mes = Message::new(command);
    mes.build_message()
}
pub fn serialize_message_log(command: Command) -> Vec<u8>{
    let mut mes = Message::new(command);
    mes.build_message_log()
}

pub fn serialize_messages(commands: Vec<Command>) -> Vec<u8>{
    let mut temp = Vec::new();
    for command in commands {
        let mut mes = Message::new(command);
        temp.extend(mes.build_message());
    }
    temp
}


#[derive(Debug, Copy, Clone)]
pub struct YPRT {
    // Yaw, Pitch, Roll, Throttle
    // To convert from f64 to i32, multiply by 1000 
    pub yaw: f32,
    pub pitch: f32,
    pub roll: f32,
    pub throttle: i16,
}

#[derive(Debug, Copy, Clone)]
pub struct GyroValue {
    // Yaw, Pitch, Roll, Throttle
    // To convert from f64 to i32, multiply by 1000
    pub yaw: i32,
    pub pitch: i32,
    pub roll: i32,
    pub throttle: i16,
}

impl GyroValue {
    pub fn initial(&mut self){
        self.yaw = 0;
        self.pitch = 0;
        self.roll = 0;
    }
}

#[derive(Debug, Copy, Clone)]
pub struct Datalog {
    pub ypr:[I22F10;3],
    pub time: u128,
    pub yaw_speed_error: I22F10,
}

