use share_lib::Mode;
use crate::interface::{check_js, Interface};
use fixed::types::I22F10;
/// Maps keyboard inputs to corresponding drone control commands.
///
/// # Parameters
///
/// * `key` - A `termion::event::Key` representing the key that was pressed.
/// * `interface` - A mutable reference to the `Interface` struct which holds current settings
///   and state of the drone's interface.
///
/// # Returns
///
/// Returns an `Option<share_lib::Command>` which contains the command to be executed based on the
/// key pressed.
///
pub fn key_to_cmd(key: termion::event::Key, interface: &mut Interface) -> Option<share_lib::Command> {
    match key {
        // Exit (safe mode) through panic mode
        termion::event::Key::Esc|termion::event::Key::Char(' ') => {
            Some(share_lib::Command::ModeChange {mode: Mode::Panic})
        },
        termion::event::Key::Char('g')=> {
            interface.work_flag = false;
            Some(share_lib::Command::ModeChange {mode: Mode::Panic})
        },
        termion::event::Key::Char('0') => {
            Some(share_lib::Command::ModeChange {mode: Mode::Safe})
        },
        termion::event::Key::Char('1') => {
            Some(share_lib::Command::ModeChange {mode: Mode::Panic})
        },
        termion::event::Key::Char('2') => {
            // if check_js(&interface) {
            if check_js(&interface) && interface.current_mode == Mode::Safe {
                Some(share_lib::Command::ModeChange { mode: Mode::Manual })
            } else { None }
        },
        termion::event::Key::Char('3') => {
            if check_js(&interface) &&interface.current_mode == Mode::Safe {
                Some(share_lib::Command::ModeChange { mode: Mode::Calibration })
            } else { None }
        },
        termion::event::Key::Char('4') => {
            if check_js(&interface) && interface.current_mode == Mode::Safe{
                Some(share_lib::Command::ModeChange {mode: Mode::YawControlled})
            } else { None }
        },
        termion::event::Key::Char('5') => {
            if check_js(&interface) && interface.current_mode == Mode::Safe{
                Some(share_lib::Command::ModeChange {mode: Mode::FullControl})
            } else { None }
        },
        termion::event::Key::Char('6') => {
            if check_js(&interface) && interface.current_mode == Mode::Safe{
                Some(share_lib::Command::ModeChange {mode: Mode::Height})
            } else { None }
        },
        termion::event::Key::Char('a') => {
            interface.js.t_trim += 30;
            None
        },
        termion::event::Key::Char('z') => {
            interface.js.t_trim -= 30;
            None
        },
        termion::event::Key::Left => {
            interface.js.r_trim += I22F10::from_num(0.1);
            None
        },
        termion::event::Key::Right => {
            interface.js.r_trim -= I22F10::from_num(0.1);
            None
        },
        termion::event::Key::Up => {
            interface.js.p_trim += I22F10::from_num(0.1);
            None
        },
        termion::event::Key::Down => {
            interface.js.p_trim -= I22F10::from_num(0.1);
            None
        },
        termion::event::Key::Char('q') => {
            interface.js.y_trim += I22F10::from_num(0.1);
            None
        },
        termion::event::Key::Char('w') => {
            interface.js.y_trim -= I22F10::from_num(0.1);
            None
        },
        termion::event::Key::Char('u') => {
            Some(share_lib::Command::YawPSet {num:interface.pid_yaw[0]+1})
        },
        termion::event::Key::Char('j') => {
            Some(share_lib::Command::YawPSet {num:interface.pid_yaw[0]-1})
        },
        termion::event::Key::Char('i') => {
            Some(share_lib::Command::PitchPSet {num:interface.pid_pitch[0]+1})
            // Some(share_lib::Command::PControlUp)
        },
        termion::event::Key::Char('k') => {
            Some(share_lib::Command::PitchPSet {num:interface.pid_pitch[0]-1})
            // Some(share_lib::Command::PControlDown)
        },
        termion::event::Key::Char('o') => {
            Some(share_lib::Command::PitchDSet {num:interface.pid_pitch[1]+1})
            // Some(share_lib::Command::DControlUp)
        },
        termion::event::Key::Char('l') => {
            Some(share_lib::Command::PitchDSet {num:interface.pid_pitch[1]-1})
            // Some(share_lib::Command::DControlDown)
        },
        termion::event::Key::Char(',') => {
            Some(share_lib::Command::RollPSet {num:interface.pid_roll[0]+1})
            // Some(share_lib::Command::DControlUp)
        },
        termion::event::Key::Char('.') => {
            Some(share_lib::Command::RollPSet {num:interface.pid_roll[0]-1})
            // Some(share_lib::Command::DControlDown)
        },
        termion::event::Key::Char('b') => {
            Some(share_lib::Command::RollDSet {num:interface.pid_roll[1]+1})
            // Some(share_lib::Command::DControlUp)
        },
        termion::event::Key::Char('n') => {
            Some(share_lib::Command::RollDSet {num:interface.pid_roll[1]-1})
            // Some(share_lib::Command::DControlDown)
        },
        // Else print the pressed key
        _ => {
            None
        }
    }

}