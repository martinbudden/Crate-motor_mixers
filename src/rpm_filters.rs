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
    pub motor_count: u8,
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
            motor_count: 4,
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

#[repr(u8)]
enum State {
    Stopped,
    Fundamental,
    SecondHarmonic,
    ThirdHarmonic,
}
#[derive(Clone, Copy, Debug, Default, PartialEq)]
struct StateMachineState {
    state: u8,
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
    looptime_seconds: f32,
    state: StateMachineState,
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

impl RpmFiltersState {
    pub fn new() -> Self {
        Self {
            config: RpmFiltersConfig::default(),
            looptime_seconds: 0.001,
            state: StateMachineState::default(),
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

        self.q = config.rpm_filter_q as f32 * 0.01;

        self.state.state = State::Stopped as u8;
        // just under  Nyquist frequency (ie just under half sampling rate)
        // for 8kHz loop this is 3840Hz
        self.max_frequency_hz = 480000.0 / self.looptime_seconds as f32;
        self.half_of_max_frequency_hz = self.max_frequency_hz / 2.0;
        self.third_of_max_frequency_hz = self.max_frequency_hz / 3.0;
        self.min_frequency_hz = config.rpm_filter_min_hz as f32;
        self.fade_range_hz = config.rpm_filter_fade_range_hz as f32;

        for harmonic in 0..config.rpm_filter_harmonics as usize {
            for motor in 0..config.motor_count as usize {
                self.filters[motor][harmonic].init_notch(
                    self.min_frequency_hz * (harmonic + 1) as f32,
                    self.looptime_seconds,
                    self.q,
                );
            }
        }

        if config.rpm_filter_lpf_hz == 0 {
            for mut rpm_filter in self.motor_rpm_filters {
                rpm_filter.set_to_passthrough();
            }
        } else {
            for mut rpm_filter in self.motor_rpm_filters {
                rpm_filter.set_cutoff_frequency_and_reset(config.rpm_filter_lpf_hz as f32, self.looptime_seconds);
            }
        }
    }

    ///This is called from MotorMixer::output_to_motors and so needs to be FAST.
    fn set_frequency_hz_iteration_start(&mut self, motor_index: usize, frequency_hz: f32) {
        if self.config.rpm_filter_lpf_hz == 0 {
            return;
        }
        if (self.state.state == State::Stopped as u8) {
            self.state.state = State::Fundamental as u8;
            self.state.motor_index = 0;
        }
        let mut motor_state = self.state.motor_states[motor_index]; // NOLINT(cppcoreguidelines-pro-bounds-constant-array-index)

        motor_state.frequency_hz_unclamped = self.motor_rpm_filters[motor_index].filter(frequency_hz);
        let frequency_hz = motor_state.frequency_hz_unclamped.clamp(self.min_frequency_hz, self.max_frequency_hz);

        let margin_frequency_hz = frequency_hz - self.min_frequency_hz;
        motor_state.weight_multiplier =
            if margin_frequency_hz < self.fade_range_hz { margin_frequency_hz / self.fade_range_hz } else { 1.0 };

        let rpm_filter = self.filters[motor_index][State::Fundamental as usize];
        motor_state.omega = rpm_filter.calculate_omega(frequency_hz);
    }
    //This is called from MotorMixer::rpm_filter_set_frequency_hz_iteration_step and so needs to be FAST.
    fn set_frequency_hz_iteration_step() {}
    fn filter(&mut self, mut input: Vector3df32, motor_index: usize) {
        input = self.filters[motor_index][State::Fundamental as usize].filter_weighted(input); // NOLINT(cppcoreguidelines-pro-bounds-constant-array-index)

        if self.weights[State::SecondHarmonic as usize] != 0.0 {
            input = self.filters[motor_index][State::SecondHarmonic as usize].filter_weighted(input); // NOLINT(cppcoreguidelines-pro-bounds-constant-array-index)
        };
        if self.weights[State::ThirdHarmonic as usize] != 0.0 {
            input = self.filters[motor_index][State::ThirdHarmonic as usize].filter_weighted(input);
        };
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

    fn is_normal<T: Sized + Send + Sync + Unpin + Copy + Clone + Default + PartialEq>() {}

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
