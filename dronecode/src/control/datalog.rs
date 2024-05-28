use tudelft_quadrupel::flash::{flash_chip_erase, flash_read_bytes, flash_write_bytes};
use share_lib::{Command, Message};
use crate::control::drone::Drone;

//data log write
pub fn datalog(cmd:Command, pc_counter: u32) ->u32{
    let mut mes = Message::new(cmd);
    let serialized = mes.build_message_log();
    let (point,new_pont) = full_check(pc_counter,64);
    flash_write_bytes(point, &serialized).expect("log fail");
    new_pont
}

//erase the flash
pub fn full_check(pc_counter: u32, len: u32) -> (u32,u32){
    let mut result = pc_counter;
    if (pc_counter + len) > 0x01FFFE{
        result = 0x000000;
        flash_chip_erase().expect("erase fail");
    }
    (result, result+len)
}
impl Drone {
    pub fn pick_up_message(&mut self) -> Option<Command>{
        let mut buf= [0;64];
        flash_read_bytes(self.pc_counter, &mut buf).expect("read flash fail");
        Message::get_message_log(&buf)
    }
}
