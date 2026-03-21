use embassy_time::{Instant, Timer};
use filters::{BiquadFilterf32, FilterPt1f32};
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

// NOTE: I have considered the typestate patter an have elected not to use it.
#[repr(u8)]
#[derive(Clone, Copy, Default, Debug, PartialEq)]
enum State {
    #[default]
    Stopped,
    Fundamental,
    SecondHarmonic,
    ThirdHarmonic,
}

impl From<u8> for State {
    fn from(value: u8) -> Self {
        match value {
            0 => State::Stopped,
            1 => State::Fundamental,
            2 => Self::SecondHarmonic,
            3 => Self::ThirdHarmonic,
            _ => State::Stopped,
        }
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
struct StateMachineState {
    state: State,
    motor_index: usize,
    motor_states: [RpmFiltersMotorState; MAX_MOTOR_COUNT],
}

impl StateMachineState {
    #[allow(dead_code)]
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
    filters: [[BiquadFilterf32<Vector3df32>; RPM_FILTER_HARMONICS_COUNT]; MAX_MOTOR_COUNT],
    motor_rpm_filters: [FilterPt1f32<f32>; MAX_MOTOR_COUNT],
}

const FUNDAMENTAL: usize = 0;
const SECOND_HARMONIC: usize = 1;
const THIRD_HARMONIC: usize = 2;

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
            filters: <[[BiquadFilterf32<Vector3df32>; RPM_FILTER_HARMONICS_COUNT]; MAX_MOTOR_COUNT]>::default(),
            motor_rpm_filters: <[FilterPt1f32<f32>; MAX_MOTOR_COUNT]>::default(),
        }
    }
    pub fn set_config(&mut self, config: RpmFiltersConfig) {
        self.config = config;

        self.q = config.rpm_filter_q as f32 * 0.01;

        self.state.state = State::Stopped;
        // just under  Nyquist frequency (ie just under half sampling rate)
        // for 8kHz loop this is 3840Hz
        self.max_frequency_hz = 480000.0 / self.looptime_seconds;

        // pre-calculate frequencies for speed in iteration steps
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

    /// Start the filter state machine
    /// This is called from MotorMixer::output_to_motors and so needs to be FAST.
    pub fn set_frequency_hz_iteration_start(&mut self, motor_index: usize, frequency_hz: f32) {
        if self.config.rpm_filter_lpf_hz == 0 {
            return;
        }
        if self.state.state == State::Stopped {
            self.state.state = State::Fundamental;
            self.state.motor_index = 0;
        }
        let mut motor_state = self.state.motor_states[motor_index];

        motor_state.frequency_hz_unclamped = self.motor_rpm_filters[motor_index].filter(frequency_hz);
        let frequency_hz = motor_state.frequency_hz_unclamped.clamp(self.min_frequency_hz, self.max_frequency_hz);

        let margin_frequency_hz = frequency_hz - self.min_frequency_hz;
        motor_state.weight_multiplier =
            if margin_frequency_hz < self.fade_range_hz { margin_frequency_hz / self.fade_range_hz } else { 1.0 };

        let rpm_filter = self.filters[motor_index][FUNDAMENTAL];
        motor_state.omega = rpm_filter.calculate_omega(frequency_hz);

        self.state.motor_states[motor_index] = motor_state;
    }

    /// Perform one step of the state machine
    /// This is called from MotorMixer::rpm_filter_set_frequency_hz_iteration_step and so needs to be FAST.
    pub fn set_frequency_hz_iteration_step(&mut self) {
        // state machine sets notch filter for one harmonic of one motor on each iteration.

        match self.state.state {
            State::Stopped => {}
            State::Fundamental => {
                let mut rpm_filter = self.filters[self.state.motor_index][FUNDAMENTAL]; // NOLINT(cppcoreguidelines-pro-bounds-constant-array-index)
                // omega = frequency * _2PiLoopTimeSeconds
                // max_frequency < 0.5 / looptime_seconds
                // max_omega = (0.5 / looptime_seconds) * 2PiLooptimeSeconds = 0.5 * 2PI = PI;
                // so omega is in range [0, PI]
                let mut motor_state = self.state.motor_states[self.state.motor_index];
                (motor_state.sin_omega, motor_state.cos_omega) = motor_state.omega.sin_cos();
                rpm_filter.set_notch_frequency_weighted_from_sin_cos_assuming_q(
                    motor_state.sin_omega,
                    motor_state.cos_omega * 2.0,
                    self.weights[FUNDAMENTAL] * motor_state.weight_multiplier,
                );
                self.state.motor_index += 1;
                if self.state.motor_index == self.config.motor_count as usize {
                    // we have set the notch frequency for all motors, so move onto the next harmonic if there is one, otherwise we are finished
                    self.state.motor_index = 0;
                    if self.config.rpm_filter_harmonics >= 2 {
                        if self.config.rpm_filter_weights[SECOND_HARMONIC] != 0 {
                            self.state.state = State::SecondHarmonic;
                        } else if self.config.rpm_filter_harmonics >= 3
                            && self.config.rpm_filter_weights[THIRD_HARMONIC] != 0
                        {
                            self.state.state = State::ThirdHarmonic;
                        } else {
                            self.state.state = State::Stopped;
                        }
                    } else {
                        self.state.state = State::Stopped;
                    }
                }
            }
            State::SecondHarmonic => {
                let motor_state = self.state.motor_states[self.state.motor_index];
                if motor_state.frequency_hz_unclamped > self.half_of_max_frequency_hz {
                    // ie 2.0 * frequency_hz_unclamped > _max_frequency_hz
                    // no point filtering the second harmonic if it is above the Nyquist frequency
                    self.weights[SECOND_HARMONIC] = 0.0;
                } else {
                    self.weights[SECOND_HARMONIC] = self.config.rpm_filter_weights[SECOND_HARMONIC] as f32 * 0.01;
                    let mut rpm_filter = self.filters[self.state.motor_index][SECOND_HARMONIC]; // NOLINT(cppcoreguidelines-pro-bounds-constant-array-index)
                    // sin(2θ) = 2 * sin(θ) * cos(θ)
                    // cos(2θ) = 2 * cos^2(θ) - 1
                    let sin_2_omega = motor_state.sin_omega * motor_state.cos_omega * 2.0;
                    let two_cos_2_omega = motor_state.cos_omega * motor_state.cos_omega * 4.0 - 2.0;
                    rpm_filter.set_notch_frequency_weighted_from_sin_cos_assuming_q(
                        sin_2_omega,
                        two_cos_2_omega,
                        self.weights[SECOND_HARMONIC] * motor_state.weight_multiplier,
                    );
                }
                self.state.motor_index += 1;
                if self.state.motor_index == self.config.motor_count as usize {
                    // we have set the notch frequency for all motors, so move onto the next harmonic if there is one, otherwise we are finished
                    self.state.motor_index = 0;
                    if self.config.rpm_filter_harmonics >= 3 && self.config.rpm_filter_weights[THIRD_HARMONIC] != 0 {
                        self.state.state = State::ThirdHarmonic;
                    } else {
                        self.state.state = State::Stopped;
                    }
                }
            }
            State::ThirdHarmonic => {
                let motor_state = self.state.motor_states[self.state.motor_index]; // NOLINT(cppcoreguidelines-pro-bounds-constant-array-index)
                if motor_state.frequency_hz_unclamped > self.third_of_max_frequency_hz {
                    // ie 3.0 * frequency_hz_unclamped > _max_frequency_hz
                    // no point filtering the third harmonic if it is above the Nyquist frequency
                    self.weights[THIRD_HARMONIC] = 0.0;
                } else {
                    self.weights[THIRD_HARMONIC] = self.config.rpm_filter_weights[THIRD_HARMONIC] as f32 * 0.01;
                    let mut rpm_filter = self.filters[self.state.motor_index][THIRD_HARMONIC]; // NOLINT(cppcoreguidelines-pro-bounds-constant-array-index)
                    // sin(3θ) = 3 * sin(θ)   - 4 * sin^3(θ)
                    //         = sin(θ) * ( 3 - 4 * sin^2(θ) )
                    //         = sin(θ) * ( 3 - 4 * (1 - cos^2(θ)) )
                    //         = sin(θ) * ( 4 * cos^2(θ) - 1)
                    // cos(3θ) = 4 * cos^3(θ) - 3 * cos(θ)
                    //         = cos(θ) * ( 4 * cos^2(θ) - 3 )
                    let four_cos_squared_omega = 4.0 * motor_state.cos_omega * motor_state.cos_omega;
                    let sin_3_omega = motor_state.sin_omega * (four_cos_squared_omega - 1.0);
                    let two_cos_3_omega = 2.0 * motor_state.cos_omega * (four_cos_squared_omega - 3.0);
                    rpm_filter.set_notch_frequency_weighted_from_sin_cos_assuming_q(
                        sin_3_omega,
                        two_cos_3_omega,
                        self.weights[THIRD_HARMONIC] * motor_state.weight_multiplier,
                    );
                }
                self.state.motor_index += 1;
                if self.state.motor_index == self.config.motor_count as usize {
                    // we have set the notch frequency for all motors, so we are finished
                    self.state.motor_index = 0;
                    self.state.state = State::Stopped;
                }
            }
        }
    }

    pub async fn set_frequency_hz_iteration_step_async(&mut self) {
        self.set_frequency_hz_iteration_step();
        Timer::at(Instant::now()).await;
    }

    pub fn filter(&mut self, input: Vector3df32, motor_index: usize) -> Vector3df32 {
        let mut ret = self.filters[motor_index][FUNDAMENTAL].filter_weighted(input); // NOLINT(cppcoreguidelines-pro-bounds-constant-array-index)

        if self.weights[SECOND_HARMONIC] != 0.0 {
            ret = self.filters[motor_index][SECOND_HARMONIC].filter_weighted(input); // NOLINT(cppcoreguidelines-pro-bounds-constant-array-index)
        };
        if self.weights[THIRD_HARMONIC] != 0.0 {
            ret = self.filters[motor_index][THIRD_HARMONIC].filter_weighted(input);
        };
        ret
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
