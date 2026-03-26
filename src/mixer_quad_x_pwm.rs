use crate::{
    MotorMixer, MotorMixerCommands, MotorMixerCommandsDps, MotorMixerCommon, MotorMixerDriver, MotorMixerParameters,
    mix_quad_x,
};

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MotorMixerQuadXPwm {
    common: MotorMixerCommon,
    outputs: [f32; Self::MOTOR_COUNT],
    max_duty: u32,
}

impl Default for MotorMixerQuadXPwm {
    fn default() -> Self {
        Self::new()
    }
}

impl MotorMixerQuadXPwm {
    const MOTOR_COUNT:usize = 4;

    pub fn new() -> Self {
        Self {
            common: MotorMixerCommon::default(), // more idiomatic than calling new
            outputs: <[f32; Self::MOTOR_COUNT]>::default(),
            max_duty: 255,
        }
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
    fn common(&self) -> &MotorMixerCommon {
        &self.common
    }
    fn common_mut(&mut self) -> &mut MotorMixerCommon {
        &mut self.common
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

    fn _is_normal<T: Sized + Send + Sync + Unpin>() {}
    fn is_full<T: Sized + Send + Sync + Unpin + Copy + Clone + Default + PartialEq>() {}

    #[test]
    fn normal_types() {
        is_full::<MotorMixerQuadXPwm>();
    }
    #[test]
    fn new() {
        let quadx = MotorMixerQuadXPwm::new();
        assert_eq!(255, quadx.max_duty);
    }
}
