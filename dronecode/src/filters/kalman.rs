use cordic::{atan2};
use fixed::types::I22F10;
const DEG2RAD:f32 = 0.017; //ref value for degree to radian
pub struct Kalman{
    pub pitchp: I22F10, // the position of pitch
    pub pitchb: I22F10, // the bias of pitch
    pub pitchphi: I22F10, // the angular of pitch
    pub pitch_c1: I22F10, // the coefficient1 of pitch
    pub pitch_c2: I22F10, // the coefficient2 of pitch

    pub rollp: I22F10, // the position of roll
    pub rollb: I22F10, // the bias of roll
    pub rollphi: I22F10, // the angular of roll
    pub roll_c1: I22F10, // the coefficient1 of roll
    pub roll_c2: I22F10, // the coefficient2 of roll
}
impl Kalman{
    pub fn new() ->Self{
        Kalman{
            pitchp: I22F10::from_num(0.0),
            pitchb: I22F10::from_num(0.0),
            pitchphi: I22F10::from_num(0.0),
            pitch_c1: I22F10::from_num(1.0),
            pitch_c2: I22F10::from_num(1000),

            rollp: I22F10::from_num(0.0),
            rollb: I22F10::from_num(0.0),
            rollphi: I22F10::from_num(0.0),
            roll_c1: I22F10::from_num(1.0),
            roll_c2: I22F10::from_num(1000)
        }
    }
    pub fn filtering(&mut self,ax:I22F10,ay:I22F10,az:I22F10,spx:I22F10,spy:I22F10){
        // get the angular of roll and pitch from the Accelerator
        let pitchsphi = atan2(ax, az);
        let rollsphi = atan2(ay, az);

        // get the velocity of pitch
        self.pitchp = I22F10::from_num(spy)*I22F10::from_num(DEG2RAD)-self.pitchb;
        // calculate the angular of pitch from the gyro
        let  pitchphi= self.pitchphi+self.pitchp/I22F10::from_num(100); //dt
        // calculate the error
        let e = pitchphi-pitchsphi;
        // get the angle of pitch
        self.pitchphi -= e/self.pitch_c1;
        // update the bias of pitch
        self.pitchb += (e/I22F10::from_num(0.01))/self.pitch_c2;

        // get the velocity of roll
        self.rollp = I22F10::from_num(spx)*I22F10::from_num(DEG2RAD)-self.rollb;
        // calculate the angular of roll from the gyro
        let  rollphi= self.rollphi+self.rollp/I22F10::from_num(100); //dt
        // calculate the error
        let e = rollphi-rollsphi;
        // get the angle of roll
        self.rollphi -= e/self.roll_c1;
        // update the bias of roll
        self.rollb += (e/I22F10::from_num(0.01))/self.roll_c2;
    }
}
