use crate::mixer::MotorOutputs;
use crate::{
    MotorFrequencies, MotorMixer, MotorMixerCommands, MotorMixerCommandsDps, MotorMixerCommon, MotorMixerDriver,
    MotorMixerParameters, RpmFilterBank, RpmFilterBankConfig, mix_quad_x,
};

impl MotorMixerDriver for MotorMixerQuadXDshot {
    fn write_to_motors(&mut self, _motor_outputs: MotorOutputs) {
        // TODO: implement write_to_motors for MotorMixerQuadXShot
    }
    fn read_motor_frequencies_hz(&mut self) -> MotorFrequencies {
        // TODO: implement read_motor_frequencies_hz for MotorMixerQuadDshot
        MotorFrequencies::default()
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MotorMixerQuadXDshot {
    common: MotorMixerCommon,
    config: RpmFilterBankConfig,
    rpm_filters: RpmFilterBank,
    motor_frequencies_hz: MotorFrequencies,
    rpm_filter_iteration_count: usize,
}

impl Default for MotorMixerQuadXDshot {
    fn default() -> Self {
        Self::new()
    }
}

impl MotorMixerQuadXDshot {
    const MOTOR_COUNT: usize = 4;

    pub fn new() -> Self {
        Self {
            common: MotorMixerCommon::default(),
            config: RpmFilterBankConfig::default(),
            rpm_filters: RpmFilterBank::default(),
            motor_frequencies_hz: MotorFrequencies::default(),
            rpm_filter_iteration_count: 0,
        }
    }
}

impl MotorMixer for MotorMixerQuadXDshot {
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
            self.common.outputs = MotorOutputs::default();
            self.write_to_motors(self.common.outputs);
            return;
        }

        // TODO: read the motor frequencies from the Dshot driver.
        // We retain the values, so they can be displayed on an OSD or recorded in blackbox, if required.
        self.motor_frequencies_hz = self.read_motor_frequencies_hz();
        self.rpm_filters.start(self.motor_frequencies_hz);

        if self.output_this_cycle() {
            const MIXER_OUTPUT_SCALE_FACTOR: f32 = 1000.0;
            let mut mix_params = MotorMixerParameters::default();
            let commands = MotorMixerCommands {
                throttle: commands_dps.throttle,
                // scale roll, pitch, and yaw from DPS range to [-1.0F, 1.0F]
                roll: commands_dps.roll_dps * MIXER_OUTPUT_SCALE_FACTOR,
                pitch: commands_dps.pitch_dps * MIXER_OUTPUT_SCALE_FACTOR,
                yaw: commands_dps.yaw_dps * MIXER_OUTPUT_SCALE_FACTOR,
            };
            self.common.outputs[..Self::MOTOR_COUNT].copy_from_slice(&mix_quad_x(commands, &mut mix_params));
            self.set_throttle_command(mix_params.throttle);

            self.write_to_motors(self.common.outputs);
        }

        // We need to complete the rpm_filter iterations before the next time rpm_filter.start() is called.
        // So, for example, if there are 2 harmonics and 4 motors that gives 8 iterations in total.
        // So if output_denominator is 2, then we need to do 4 iterations.
        // If output denominator is 3, then we need to do 3 iterations.
        // TODO: move the calculation of iteration_count into set_config.
        let iteration_count =
            (self.config.rpm_filter_harmonics as usize * Self::MOTOR_COUNT).div_ceil(self.output_denominator());
        for _ in 0..iteration_count {
            self.rpm_filters.update();
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
        is_full::<MotorMixerQuadXDshot>();
    }
    #[test]
    fn new() {
        let _quadx = MotorMixerQuadXDshot::new();
    }
}
