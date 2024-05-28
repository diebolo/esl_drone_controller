use fixed::types::I22F10;
use super::yaw_pitch_roll::YawPitchRoll;
pub fn calc_motors(ypr: YawPitchRoll, throttle: I22F10) -> [u16; 4] {
    // input array
    // lift, roll, pitch, yaw
    //      Z,      L,      M,      N
    //ae1 -0.25	    0	    0.5	    -0.25
    //ae2 -0.25	    -0.5	0	    0.25
    //ae3 -0.25	    0	    -0.5	-0.25
    //ae4 -0.25	    0.5	    0	    0.25

    // Max output RPM is 1445, after that we get funky bit behaviour.

    let inp = [throttle, ypr.roll, ypr.pitch, ypr.yaw];
    let qrtr = I22F10::from_num(0.25);
    let half = I22F10::from_num(0.50);
    let min_throttle = I22F10::from_num(-50);

    if throttle > min_throttle {
        return [0, 0, 0, 0];
    }

    // RESTRICTED MODE (400 RPM)
    // let b = I22F10::from_num(300);
    // let d = I22F10::from_num(2000);

    // UNRESTRICTED MODE (1000 RPM)
    let b = I22F10::from_num(1000);
    let d = I22F10::from_num(7000);


    // Matrix above except pitch is inverted
    let a1 = (-qrtr * inp[0] - half * inp[2]) * b - (qrtr * inp[3]) * d;
    let a2 = (-qrtr * inp[0] - half * inp[1]) * b + (qrtr * inp[3]) * d;
    let a3 = (-qrtr * inp[0] + half * inp[2]) * b - (qrtr * inp[3]) * d;
    let a4 = (-qrtr * inp[0] + half * inp[1]) * b + (qrtr * inp[3]) * d;

    let mut ae1 = I22F10::from_num(0);
    let mut ae2 = I22F10::from_num(0);
    let mut ae3 = I22F10::from_num(0);
    let mut ae4 = I22F10::from_num(0);

    // REMOVED MAX RPM CHECK, set_motors() already limits max rpm, so no need to check here.
    if a1 > 0 {
        ae1 = a1.sqrt();
    }
    if a2 > 0 {
        ae2 = a2.sqrt();
    }
    if a3 > 0 {
        ae3 = a3.sqrt();
    }
    if a4 > 0 {
        ae4 = a4.sqrt();
    }

    // If throttle is on, keep all motors above 180 RPM
    if throttle < I22F10::from_num(min_throttle) {
        if ae1 < I22F10::from_num(180) {
            ae1 = I22F10::from_num(180);
        }
        if ae2 < I22F10::from_num(180) {
            ae2 = I22F10::from_num(180);
        }
        if ae3 < I22F10::from_num(180) {
            ae3 = I22F10::from_num(180);
        }
        if ae4 < I22F10::from_num(180) {
            ae4 = I22F10::from_num(180);
        }
    }


    [ae1.to_num::<u16>(), ae2.to_num::<u16>(), ae3.to_num::<u16>(), ae4.to_num::<u16>()]
}
