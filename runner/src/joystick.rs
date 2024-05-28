use gilrs::{ Gilrs};
use share_lib::{Command, Mode, THROTTLE_SCALE};
use fixed::types::I22F10;
#[allow(unused)]
const JS_YAW_SCALE: f32 = 0.8; // radians per second
const JS_ROLL_SCALE: f32 = 0.4;
const JS_PITCH_SCALE: f32 = JS_ROLL_SCALE;

#[derive(Debug, Clone)]
pub struct Joystick {
    pub disconnect: bool,
    pub yaw: I22F10,
    pub pitch: I22F10,
    pub roll: I22F10,
    pub throttle: i16,
    pub y_trim: I22F10,
    pub p_trim: I22F10,
    pub r_trim: I22F10,
    pub t_trim: i16,

}

impl Joystick {
    pub fn new() -> Self {
        Joystick{
            disconnect: true,
            yaw: I22F10::from_num(0),
            pitch: I22F10::from_num(0),
            roll: I22F10::from_num(0),
            throttle: 0,
            y_trim: I22F10::from_num(0),
            p_trim: I22F10::from_num(0),
            r_trim: I22F10::from_num(0),
            t_trim: 0
        }
    }
    /// Processes joystick events and updates the internal state of joystick controls.
    ///
    /// # Parameters
    ///
    /// * `self` - A mutable reference to the current instance of the context holding joystick states.
    /// * `gilrs` - A mutable reference to a `Gilrs` instance, used for fetching the next event
    ///   from the gamepad.
    ///
    /// # Returns
    ///
    /// Returns an `Option<Vec<share_lib::Command>>`. If there are changes in joystick positions, it returns
    /// a vector of commands to be executed; otherwise, it returns `None`. The function returns immediately
    /// with a command for mode change to `Panic` if the joystick is disconnected.
    pub fn read_joystick(&mut self, gilrs: &mut Gilrs) -> Option<Vec<share_lib::Command>> {
        self.disconnect = true;
        for (_id, _gamepad) in gilrs.gamepads() {
            self.disconnect = false;
            break;
        }

        let last_js = self.clone();
        let mut changed = Vec::new();
        while let Some(event) = gilrs.next_event() {
            // Handle connected events
            match event.event {
                gilrs::EventType::ButtonChanged(gilrs::Button::Unknown, 1.0, _code) => { // Panic
                    return Some(vec![share_lib::Command::ModeChange {mode: Mode::Panic}])
                },
                //**********************************controller******************************************//
                gilrs::EventType::AxisChanged(gilrs::Axis::RightStickY, val, _code) => { // Throttle
                    let mut throttle = 0;
                    if val > 0.01 {
                        throttle = -(val * 2000.0) as i16;
                    }
                    self.throttle = throttle + self.t_trim;
                },
                gilrs::EventType::AxisChanged(gilrs::Axis::RightStickX, val, _code) => { // Yaw
                    let yaw = I22F10::from_num(val * JS_YAW_SCALE);
                    self.yaw = yaw + self.y_trim;
                }
                //****************************************************************************//

                //**********************************joystick******************************************//
                gilrs::EventType::AxisChanged(gilrs::Axis::Unknown, val, _code) => { // Throttle
                    let mut throttle = 0;
                    if val < 0.95 {
                        // throttle = ((1.0 - val)*THROTTLE_SCALE as f32) as i16;
                        throttle = -(( 1.0-val)*THROTTLE_SCALE as f32) as i16;
                    }
                    //need a cap for trim
                    //throttle = (val*1000.0) as i16;
                    self.throttle = throttle + self.t_trim;
                    // return Some(share_lib::Command::ThrottleSet{num: temp})
                }
                gilrs::EventType::AxisChanged(gilrs::Axis::RightZ, val, _code) => { // Yaw
                    self.yaw = I22F10::from_num(val) + self.y_trim;
                }
                //**************************************************************************************
                gilrs::EventType::AxisChanged(gilrs::Axis::LeftStickY, val, _code) => { // Pitch
                    let pitch = I22F10::from_num(val * JS_PITCH_SCALE);
                    self.pitch = pitch + self.p_trim;
                }
                gilrs::EventType::AxisChanged(gilrs::Axis::LeftStickX, val, _code) => { // Roll
                    let roll = I22F10::from_num(val * JS_ROLL_SCALE);
                    self.roll = roll + self.r_trim;
                }
                gilrs::EventType::Disconnected => {
                    self.disconnect = true;
                    return Some(vec![share_lib::Command::ModeChange {mode: Mode::Panic}])
                }
                gilrs::EventType::Connected => {
                    self.disconnect = false;
                    // return None
                }
                _ => {
                    // return None
                }
            }
        }

        if last_js.yaw != self.yaw {
            changed.push(Command::YawSet{num: (self.yaw+self.y_trim).to_bits()});
        }
        if last_js.pitch != self.pitch {
            changed.push(share_lib::Command::PitchSet{num: (self.pitch+self.p_trim).to_bits()});
        }
        if last_js.roll != self.roll {
            changed.push(share_lib::Command::RollSet{num: (self.roll+self.r_trim).to_bits()});
        }
        if last_js.throttle != self.throttle {
            changed.push(share_lib::Command::ThrottleSet{num: (self.throttle+self.t_trim)});
        }
        if changed.len() > 0 {
            return Some(changed);
        }
        None

    }

}
