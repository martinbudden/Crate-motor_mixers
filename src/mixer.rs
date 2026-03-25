use crate::{MixerConfig, MixerType, MotorConfig, MotorMixerCommandsDps, MotorMixerParameters};

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MotorMixerCommon {
    mixer_type: u8,
    output_denominator: u8,
    output_count: u8,
    mixer_config: MixerConfig,
    motor_config: MotorConfig,
    mixer_parameters: MotorMixerParameters,
    throttle_command: f32, // used for blackbox recording
    motors_is_on: bool,
    motors_is_armed: bool,
    motors_is_reversed: bool, //reversed motors typically used to flip multi-rotor after a crash
}

impl MotorMixerCommon {
    fn new() -> Self {
        Self {
            mixer_type: MixerType::QuadX as u8,
            output_denominator: 1,
            output_count: 0,
            mixer_config: MixerConfig::default(),
            motor_config: MotorConfig::default(),
            mixer_parameters: MotorMixerParameters::default(),
            throttle_command: 0.0, // used for blackbox recording
            motors_is_on: false,
            motors_is_armed: false,
            motors_is_reversed: false, //reversed motors typically used to flip multi-rotor after a crash
        }
    }
}

impl Default for MotorMixerCommon {
    fn default() -> Self {
        Self::new()
    }
}

pub trait MotorMixer {
    fn common(&self) -> &MotorMixerCommon;
    fn common_mut(&mut self) -> &mut MotorMixerCommon;

    fn output_to_motors(&mut self, commands_dps: MotorMixerCommandsDps);

    fn motors_is_on(&self) -> bool {
        self.common().motors_is_on
    }
    fn motors_switch_off(&mut self) {
        self.common_mut().motors_is_on = false;
    }
    fn motors_switch_on(&mut self) {
        self.common_mut().motors_is_on = true;
    }
    fn motors_is_armed(&self) -> bool {
        self.common().motors_is_armed
    }
    /// Switch off motors and disarm.
    fn disarm_motors(&mut self) {
        self.motors_switch_off();
        self.common_mut().motors_is_armed = false;
    }
    /// Arm motors, ensuring they are switched off first.
    fn arm_motors(&mut self) {
        self.motors_switch_off();
        self.common_mut().motors_is_armed = true;
    }
    fn throttle_command(&self) -> f32 {
        self.common().throttle_command
    }
    fn set_throttle_command(&mut self, throttle_command: f32) {
        self.common_mut().throttle_command = throttle_command;
    }
    fn output_this_cycle(&mut self) -> bool {
        // TODO: check the logic of this
        self.common_mut().output_count += 1;
        if self.common().output_count < self.common().output_denominator {
            return false;
        }
        self.common_mut().output_count = 0;
        true
    }
}

pub trait MotorMixerDriver {
    fn write_to_motor(&mut self, index: u8, value: f32);
}

#[cfg(test)]
mod tests {
    use super::*;

    fn _is_normal<T: Sized + Send + Sync + Unpin>() {}
    fn is_full<T: Sized + Send + Sync + Unpin + Copy + Clone + Default + PartialEq>() {}

    #[test]
    fn normal_types() {
        is_full::<MotorMixerCommon>();
    }
    #[test]
    fn new() {
        let state = MotorMixerCommon::new();
        assert_eq!(3, state.mixer_type);
    }
}
