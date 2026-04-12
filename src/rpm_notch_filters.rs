//use defmt::debug;
//use embassy_time::{Instant, Timer};
use crate::{
    mixer::MAX_MOTOR_COUNT,
    rpm_notch_filters_state_machine::{FUNDAMENTAL, RpmFilterMotorStates, SECOND_HARMONIC, State, THIRD_HARMONIC},
};
use filters::{BiquadFilterVector3df32, Pt1Filterf32};
use serde::{Deserialize, Serialize};

use vector_quaternion_matrix::Vector3df32;

pub const RPM_FILTER_HARMONICS_COUNT: usize = 3;
pub type MotorFrequencies = [f32; MAX_MOTOR_COUNT];

#[derive(Clone, Copy, Debug, PartialEq, Deserialize, Serialize)]
pub struct RpmNotchFilterBankConfig {
    pub rpm_filter_fade_range_hz: u16, // range in which notch filters fade down to min_hz
    pub rpm_filter_q_x100: u16,        // Q of the notch filters * 100
    pub rpm_filter_lpf_hz: u16,        // LPF cutoff (from motor rpm converted to Hz)
    pub rpm_filter_weights_x100: [u16; RPM_FILTER_HARMONICS_COUNT], // weight as a percentage for each harmonic
    pub rpm_filter_harmonics: u8,      // number of harmonics, zero means filters off
    pub rpm_filter_min_hz: u8,         // minimum notch frequency for fundamental harmonic
    pub motor_count: u8,
}

impl Default for RpmNotchFilterBankConfig {
    fn default() -> Self {
        Self::new()
    }
}

impl RpmNotchFilterBankConfig {
    pub fn new() -> Self {
        Self {
            rpm_filter_fade_range_hz: 50,           // range in which notch filters fade down to min_hz
            rpm_filter_q_x100: 500,                 // Q of the notch filters
            rpm_filter_lpf_hz: 150,                 // LPF cutoff (from motor rpm converted to Hz)
            rpm_filter_weights_x100: [100, 0, 100], // weight as a percentage for each harmonic
            rpm_filter_harmonics: 3,                // number of harmonics, zero means filters off
            rpm_filter_min_hz: 100,                 // minimum notch frequency for fundamental harmonic
            motor_count: 4,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RpmNotchFilterFrequencies {
    pub motor_frequencies_hz: MotorFrequencies,
    pub min_hz: f32,
    pub max_hz: f32,
    pub half_of_max_hz: f32,
    pub third_of_max_hz: f32,
    pub fade_range_hz: f32,
}

impl Default for RpmNotchFilterFrequencies {
    fn default() -> Self {
        Self::new(50.0)
    }
}

impl RpmNotchFilterFrequencies {
    pub fn new(fade_range_hz: f32) -> Self {
        Self {
            motor_frequencies_hz: MotorFrequencies::default(),
            min_hz: 100.0,
            max_hz: 0.0,
            half_of_max_hz: 0.0,
            third_of_max_hz: 0.0,
            fade_range_hz,
        }
    }
}
type NotchFilters = [[BiquadFilterVector3df32; RPM_FILTER_HARMONICS_COUNT]; MAX_MOTOR_COUNT];
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct RpmNotchFilterBankContext {
    pub motor_rpm_filters: [Pt1Filterf32; MAX_MOTOR_COUNT],
    pub notch_filters: NotchFilters,
    pub motor_states: RpmFilterMotorStates,
    pub weights: [f32; RPM_FILTER_HARMONICS_COUNT],
}

/// Bank of `RpmFilters`, one for each motor.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RpmNotchFilterBank {
    config: RpmNotchFilterBankConfig,
    frequencies: RpmNotchFilterFrequencies,
    state: State,
    ctx: RpmNotchFilterBankContext,
    // all the notch filters have the same q and looptime
    looptime_seconds: f32,
    q: f32,
}

impl Default for RpmNotchFilterBank {
    fn default() -> Self {
        Self::new(0.001)
    }
}

impl RpmNotchFilterBank {
    pub fn new(looptime_seconds: f32) -> Self {
        Self {
            config: RpmNotchFilterBankConfig::default(),
            frequencies: RpmNotchFilterFrequencies::default(),
            state: State::default(),
            ctx: RpmNotchFilterBankContext::default(),
            looptime_seconds,
            q: 0.0,
        }
    }

    pub fn set_config(&mut self, config: RpmNotchFilterBankConfig) {
        self.config = config;

        self.q = f32::from(config.rpm_filter_q_x100) * 0.01;

        self.state = State::Stopped;
        // just under  Nyquist frequency (ie just under half sampling rate)
        // for 8kHz loop this is 3840Hz
        self.frequencies.max_hz = 480_000.0 / self.looptime_seconds;

        // pre-calculate frequencies for speed in iteration steps
        self.frequencies.half_of_max_hz = self.frequencies.max_hz / 2.0;
        self.frequencies.third_of_max_hz = self.frequencies.max_hz / 3.0;
        self.frequencies.min_hz = f32::from(config.rpm_filter_min_hz);
        self.frequencies.fade_range_hz = f32::from(config.rpm_filter_fade_range_hz);

        #[allow(clippy::cast_precision_loss)]
        for harmonic in 0..config.rpm_filter_harmonics as usize {
            for motor in 0..config.motor_count as usize {
                self.ctx.notch_filters[motor][harmonic].init_notch(
                    self.frequencies.min_hz * (harmonic + 1) as f32,
                    self.looptime_seconds,
                    self.q,
                );
            }
        }

        if config.rpm_filter_lpf_hz == 0 {
            for mut rpm_filter in self.ctx.motor_rpm_filters {
                rpm_filter.set_to_passthrough();
            }
        } else {
            for mut rpm_filter in self.ctx.motor_rpm_filters {
                rpm_filter.set_cutoff_frequency_and_reset(f32::from(config.rpm_filter_lpf_hz), self.looptime_seconds);
            }
        }
    }

    /// Start the filter state machine
    /// This is called from `MotorMixer::output_to_motors` and so needs to be FAST.
    pub fn start_updating_filter_frequencies(&mut self, motor_frequencies_hz: MotorFrequencies) {
        if self.config.rpm_filter_lpf_hz == 0 {
            return;
        }
        self.frequencies.motor_frequencies_hz = motor_frequencies_hz;
        self.state.start();
    }

    pub fn update_filter_frequencies_step(&mut self) {
        //self.state = self.state.update(self);
        self.state.update(&self.config, self.frequencies, &mut self.ctx);
    }

    /// Apply the notch filters for all selected harmonics for the given motor
    pub fn update(ctx: &mut RpmNotchFilterBankContext, input: Vector3df32, motor_index: usize) -> Vector3df32 {
        let mut ret = ctx.notch_filters[motor_index][FUNDAMENTAL].update_notch_weighted(input);

        if ctx.weights[SECOND_HARMONIC] != 0.0 {
            ret = ctx.notch_filters[motor_index][SECOND_HARMONIC].update_notch_weighted(ret);
        }
        if ctx.weights[THIRD_HARMONIC] != 0.0 {
            ret = ctx.notch_filters[motor_index][THIRD_HARMONIC].update_notch_weighted(ret);
        }
        ret
    }
}

pub trait RpmNotchFilters {
    fn common(&self) -> &RpmNotchFilterBank;
    fn common_mut(&mut self) -> &mut RpmNotchFilterBank;
    fn config(&self) -> &RpmNotchFilterBankConfig;

    fn update(&mut self, value: Vector3df32, motor_index: usize) -> Vector3df32;
}

impl RpmNotchFilters for RpmNotchFilterBank {
    fn common(&self) -> &RpmNotchFilterBank {
        self
    }
    fn common_mut(&mut self) -> &mut RpmNotchFilterBank {
        self
    }
    fn config(&self) -> &RpmNotchFilterBankConfig {
        &self.common().config
    }

    fn update(&mut self, value: Vector3df32, _motor_index: usize) -> Vector3df32 {
        value
    }
}
#[cfg(test)]
mod tests {
    use super::*;

    fn _is_normal<T: Sized + Send + Sync + Unpin>() {}
    fn is_full<T: Sized + Send + Sync + Unpin + Copy + Clone + Default + PartialEq>() {}
    fn is_config<
        T: Sized + Send + Sync + Unpin + Copy + Clone + Default + PartialEq + Serialize + for<'a> Deserialize<'a>,
    >() {
    }

    #[test]
    fn normal_types() {
        is_config::<RpmNotchFilterBankConfig>();
        is_full::<RpmNotchFilterBank>();
    }
    #[test]
    fn new() {
        let config = RpmNotchFilterBankConfig::new();
        assert_eq!(50, config.rpm_filter_fade_range_hz);
    }
}
