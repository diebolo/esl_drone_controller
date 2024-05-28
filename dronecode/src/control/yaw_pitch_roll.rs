use tudelft_quadrupel::mpu::structs::Quaternion;
use fixed::types::I22F10;
use cordic::atan2;
/// This struct holds the yaw, pitch, and roll that the drone things it is in.
/// The struct is currently implemented using `f32`, you may want to change this to use fixed point arithmetic.
#[derive(Debug, Copy, Clone)]
pub struct YawPitchRoll {
    pub yaw: I22F10,
    pub pitch: I22F10,
    pub roll: I22F10,
}

impl YawPitchRoll {
    pub fn new() -> Self {
        YawPitchRoll{
            yaw: I22F10::from_num(0),
            pitch: I22F10::from_num(0),
            roll: I22F10::from_num(0)
        }
    }
}

impl From<Quaternion> for YawPitchRoll {
    /// Creates a YawPitchRoll from a Quaternion
    fn from(q: Quaternion) -> Self {
        let Quaternion { w, x, y, z } = q;
        let w = I22F10::from_num(w);
        let x= I22F10::from_num(x);
        let y = I22F10::from_num(y);
        let z = I22F10::from_num(z);

        let gx = 2 * (x * z - w * y);
        let gy = 2 * (w * x + y * z);
        let gz = w * w - x * x - y * y + z * z;
        let fixedone = I22F10::from_num(1);
        // yaw: (about Z axis)
        let yaw = atan2(2*x*y-2*w*z, (2*w*w+2*x*x)-fixedone);
        // pitch: (nose up/down, about Y axis)
        let pitch = atan2(gx,(gy*gy+gz*gz).sqrt());
        // roll: (tilt left/right, about X axis)
        let roll = atan2(gy,gz);
        Self { yaw, pitch, roll }
    }
}

impl YawPitchRoll {
    pub fn sub(&self, other: &YawPitchRoll) -> YawPitchRoll {
        YawPitchRoll {
            yaw: self.yaw - other.yaw,
            pitch: self.pitch - other.pitch,
            roll: self.roll - other.roll,
        }
    }
}
