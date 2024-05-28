use fixed::types::I22F10;
pub struct PID{
    pub p:I22F10,
    pub i:I22F10,
    pub d:I22F10
}
impl PID {
    pub fn new()->Self{
        PID{
            p: I22F10::from_num(0),
            i: I22F10::from_num(0),
            d: I22F10::from_num(0),
        }
    }
}

impl PID {
    #[allow(unused)]
    pub fn clac_pid(&mut self,last_error:I22F10,val_ref:I22F10,val_act:I22F10,
                    dt:I22F10,p_scale:I22F10,d_scale:I22F10) -> (I22F10,I22F10) {
        //calculate the current error
        let error =  val_ref - val_act;
        //the feedback of P controller
        let p_gain = self.p * p_scale * error;
        //the feedback of D controller
        let d_gian = self.d * d_scale *(error - last_error)/dt;
        //get the yaw_speed after the PID control
        let mut calc_yaw = p_gain + d_gian;
        if self.p < 1{
            calc_yaw = I22F10::from_num(1500) * val_ref;
        }
        //return cal_value and error
        (calc_yaw, error)
    }
}




