use crate::{MixerConfig, MixerType, MotorConfig, MotorMixerCommandsDps, MotorMixerParameters};

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MotorMixerState {
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

impl MotorMixerState {
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

impl Default for MotorMixerState {
    fn default() -> Self {
        Self::new()
    }
}

pub trait MotorMixer {
    fn state(&self) -> &MotorMixerState;
    fn state_mut(&mut self) -> &mut MotorMixerState;

    fn output_to_motors(&mut self, commands_dps: MotorMixerCommandsDps, delta_t: f32);

    fn motors_is_on(&self) -> bool {
        self.state().motors_is_on
    }
    fn motors_switch_off(&mut self) {
        self.state_mut().motors_is_on = false;
    }
    fn motors_switch_on(&mut self) {
        self.state_mut().motors_is_on = true;
    }
    fn motors_is_armed(&self) -> bool {
        self.state().motors_is_armed
    }
    /// Switch off motors and disarm.
    fn disarm_motors(&mut self) {
        self.motors_switch_off();
        self.state_mut().motors_is_armed = false;
    }
    /// Arm motors, ensuring they are switched off first.
    fn arm_motors(&mut self) {
        self.motors_switch_off();
        self.state_mut().motors_is_armed = true;
    }
    fn throttle_command(&self) -> f32 {
        self.state().throttle_command
    }
    fn set_throttle_command(&mut self, throttle_command: f32) {
        self.state_mut().throttle_command = throttle_command;
    }
    fn output_this_cycle(&mut self) -> bool {
        // TODO: check the logic of this
        self.state_mut().output_count += 1;
        if self.state().output_count < self.state().output_denominator {
            return false;
        }
        self.state_mut().output_count = 0;
        true
    }
}

pub trait MotorMixerDriver {
    fn write_to_motor(&mut self, index: u8, value: f32);
}

#[cfg(test)]
mod tests {
    use super::*;

    fn is_normal<T: Sized + Send + Sync + Unpin+ Copy + Clone + Default+ PartialEq>() {}

    #[test]
    fn normal_types() {
        is_normal::<MotorMixerState>();
    }
    #[test]
    fn new() {
        let state = MotorMixerState::new();
        assert_eq!(3, state.mixer_type);
    }
}