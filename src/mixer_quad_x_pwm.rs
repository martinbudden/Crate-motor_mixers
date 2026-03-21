use crate::{
    MotorMixer, MotorMixerCommands, MotorMixerCommandsDps, MotorMixerDriver, MotorMixerParameters, MotorMixerState,
    mix_quad_x,
};

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MotorMixerQuadXPwm {
    state: MotorMixerState,
    max_duty: u32,
    motor_count: u8,
    outputs: [f32; 4],
}

impl MotorMixerQuadXPwm {
    pub fn new() -> Self {
        Self {
            state: MotorMixerState::default(), // more idiomatic than calling new
            max_duty: 255,
            motor_count: 4,
            outputs: [0.0, 0.0, 0.0, 0.0],
        }
    }
}

// it is idiomatic to implement default in terms of new
impl Default for MotorMixerQuadXPwm {
    fn default() -> Self {
        Self::new()
    }
}

impl MotorMixerDriver for MotorMixerQuadXPwm {
    // TODO: implement write_to_motor for MotorMixerQuadXPwm
    fn write_to_motor(&mut self, _motor_index: u8, motor_output: f32) {
        let _duty = (motor_output * self.max_duty as f32) as u16;
        //self.pwm.set_duty(Channel::from_index(motor_index), duty);
    }
}

impl MotorMixer for MotorMixerQuadXPwm {
    fn state(&self) -> &MotorMixerState {
        &self.state
    }
    fn state_mut(&mut self) -> &mut MotorMixerState {
        &mut self.state
    }

    // Calculate and output motor mix.
    // Called by the scheduler when the updateOutputsUsingPIDs function running in the AHRS task SIGNALs that output data is available.
    // It is typically called at frequency of between 1000Hz and 8000Hz, so it has to be FAST.
    fn output_to_motors(&mut self, commands_dps: MotorMixerCommandsDps) {
        // ALWAYS write 0.0 to the motors if they are not switched on, as a safety precaution
        if !self.motors_is_on() || !self.motors_is_armed() {
            for ii in 0..self.outputs.len() {
                self.write_to_motor(ii as u8, 0.0);
            }
            return;
        }
        if !self.output_this_cycle() {
            return;
        }
        const MIXER_OUTPUT_SCALE_FACTOR: f32 = 1000.0;
        let mut mix_params = MotorMixerParameters::default();
        let commands = MotorMixerCommands {
            throttle: commands_dps.throttle,
            // scale roll, pitch, and yaw from DPS range to [-1.0F, 1.0F]
            roll: commands_dps.roll_dps * MIXER_OUTPUT_SCALE_FACTOR,
            pitch: commands_dps.pitch_dps * MIXER_OUTPUT_SCALE_FACTOR,
            yaw: commands_dps.yaw_dps * MIXER_OUTPUT_SCALE_FACTOR,
        };
        self.outputs = mix_quad_x(commands, &mut mix_params);
        self.set_throttle_command(mix_params.throttle);

        for ii in 0..self.outputs.len() {
            self.write_to_motor(ii as u8, self.outputs[ii]);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn is_normal<T: Sized + Send + Sync + Unpin>() {}

    #[test]
    fn normal_types() {
        is_normal::<MotorMixerQuadXPwm>();
    }
    #[test]
    fn new() {
        let quadx = MotorMixerQuadXPwm::new();
        assert_eq!(4, quadx.motor_count);
    }
}
