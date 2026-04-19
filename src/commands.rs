use vqm::Vector4df32;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MotorMixerCommands {
    // roll, pitch, and yaw commands are in the range [-1.0, 1.0]
    pub roll: f32,
    pub pitch: f32,
    pub yaw: f32,
    // throttle commands are in the range [0.0, 1.0]
    pub throttle: f32,
}

impl MotorMixerCommands {
    fn new() -> Self {
        Self { roll: 0.0, pitch: 0.0, yaw: 0.0, throttle: 0.0 }
    }
}

impl Default for MotorMixerCommands {
    fn default() -> Self {
        Self::new()
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MotorMixerCommandsDps {
    // roll, pitch, and yaw commands are in the degrees per second range ie approx [-1000.0, 1000.0]
    pub roll_dps: f32,
    pub pitch_dps: f32,
    pub yaw_dps: f32,
    // throttle commands are in the range [-1.0, 1.0]
    pub throttle: f32,
}

impl From<Vector4df32> for MotorMixerCommandsDps {
    fn from(v: Vector4df32) -> Self {
        MotorMixerCommandsDps { roll_dps: v.x, pitch_dps: v.y, yaw_dps: v.z, throttle: v.t }
    }
}

impl MotorMixerCommandsDps {
    fn new() -> Self {
        Self { roll_dps: 0.0, pitch_dps: 0.0, yaw_dps: 0.0, throttle: 0.0 }
    }
}

impl Default for MotorMixerCommandsDps {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    #![allow(clippy::float_cmp)]
    use super::*;

    #[allow(unused)]
    fn is_normal<T: Sized + Send + Sync + Unpin>() {}
    fn is_full<T: Sized + Send + Sync + Unpin + Copy + Clone + Default + PartialEq>() {}

    #[test]
    fn normal_types() {
        is_full::<MotorMixerCommands>();
        is_full::<MotorMixerCommandsDps>();
    }
    #[test]
    fn default() {
        let commands = MotorMixerCommandsDps::default();
        assert_eq!(0.0, commands.roll_dps);
    }
}
