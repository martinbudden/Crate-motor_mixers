#![allow(unused)]
use filters::{BiquadFilter, FilterPt1};
use vector_quaternion_matrix::Vector3df32;

pub const RPM_FILTER_HARMONICS_COUNT: usize = 3;
pub const MAX_MOTOR_COUNT: usize = 8;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RpmFiltersConfig {
    pub rpm_filter_fade_range_hz: u16, // range in which notch filters fade down to min_hz
    pub rpm_filter_q: u16,             // Q of the notch filters
    pub rpm_filter_lpf_hz: u16,        // LPF cutoff (from motor rpm converted to Hz)
    pub rpm_filter_weights: [u16; RPM_FILTER_HARMONICS_COUNT], // weight as a percentage for each harmonic
    pub rpm_filter_harmonics: u8,      // number of harmonics, zero means filters off
    pub rpm_filter_min_hz: u8,         // minimum notch frequency for fundamental harmonic
}

impl RpmFiltersConfig {
    pub fn new() -> Self {
        Self {
            rpm_filter_fade_range_hz: 50,       // range in which notch filters fade down to min_hz
            rpm_filter_q: 500,                  // Q of the notch filters
            rpm_filter_lpf_hz: 150,             // LPF cutoff (from motor rpm converted to Hz)
            rpm_filter_weights: [1000, 0, 100], // weight as a percentage for each harmonic
            rpm_filter_harmonics: 3,            // number of harmonics, zero means filters off
            rpm_filter_min_hz: 100,             // minimum notch frequency for fundamental harmonic
        }
    }
}

impl Default for RpmFiltersConfig {
    fn default() -> Self {
        Self::new()
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct RpmFiltersMotorState {
    frequency_hz_unclamped: f32,
    weight_multiplier: f32,
    omega: f32,
    sin_omega: f32,
    cos_omega: f32,
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
struct StateMachineState {
    motor_index: usize,
    motor_states: [RpmFiltersMotorState; MAX_MOTOR_COUNT],
}

impl StateMachineState {
    pub fn new() -> Self {
        Self::default()
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RpmFiltersState {
    config: RpmFiltersConfig,
    weights: [f32; RPM_FILTER_HARMONICS_COUNT],
    min_frequency_hz: f32,
    max_frequency_hz: f32,
    half_of_max_frequency_hz: f32,
    third_of_max_frequency_hz: f32,
    fade_range_hz: f32,
    q: f32,
    filters: [[BiquadFilter<Vector3df32>; RPM_FILTER_HARMONICS_COUNT]; MAX_MOTOR_COUNT],
    motor_rpm_filters: [FilterPt1<f32>; MAX_MOTOR_COUNT],
}

enum State {
    Stopped,
    Fundamental,
    SecondHarmonic,
    ThirdHarmonic,
}

impl RpmFiltersState {
    pub fn new() -> Self {
        Self {
            config: RpmFiltersConfig::default(),
            weights: <[f32; RPM_FILTER_HARMONICS_COUNT]>::default(),
            min_frequency_hz: 100.0,
            max_frequency_hz: 0.0,
            half_of_max_frequency_hz: 0.0,
            third_of_max_frequency_hz: 0.0,
            fade_range_hz: 50.0,
            q: 0.0,
            filters: <[[BiquadFilter<Vector3df32>; RPM_FILTER_HARMONICS_COUNT]; MAX_MOTOR_COUNT]>::default(),
            motor_rpm_filters: <[FilterPt1<f32>; MAX_MOTOR_COUNT]>::default(),
        }
    }
    pub fn set_config(&mut self, config: RpmFiltersConfig) {
        self.config = config;
    }
}

impl Default for RpmFiltersState {
    fn default() -> Self {
        Self::new()
    }
}

pub trait RpmFilters {
    fn state(&self) -> &RpmFiltersState;
    fn state_mut(&mut self) -> &mut RpmFiltersState;
    fn config(&self) -> &RpmFiltersConfig;

    fn filter(&mut self, value: Vector3df32, motor_index: usize) -> Vector3df32;
}

impl RpmFilters for RpmFiltersState {
    fn state(&self) -> &RpmFiltersState {
        self
    }
    fn state_mut(&mut self) -> &mut RpmFiltersState {
        self
    }
    fn config(&self) -> &RpmFiltersConfig {
        &self.state().config
    }

    fn filter(&mut self, value: Vector3df32, _motor_index: usize) -> Vector3df32 {
        value
    }
}

#[cfg(test)]
mod tests {
    use approx::assert_abs_diff_eq;

    use super::*;

    fn is_normal<T: Sized + Send + Sync + Unpin+ Copy + Clone + Default+ PartialEq>() {}

    #[test]
    fn normal_types() {
        is_normal::<RpmFiltersConfig>();
        is_normal::<RpmFiltersMotorState>();
        is_normal::<StateMachineState>();
        is_normal::<RpmFiltersState>();
    }
    #[test]
    fn new() {
        let config = RpmFiltersConfig::new();
        assert_eq!(50, config.rpm_filter_fade_range_hz);
    }
}