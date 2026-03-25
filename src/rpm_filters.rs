//use embassy_time::{Instant, Timer};
use filters::{BiquadFilterf32, FilterSignal, Pt1Filterf32};

use vector_quaternion_matrix::Vector3df32;

pub const RPM_FILTER_HARMONICS_COUNT: usize = 3;
pub const MAX_MOTOR_COUNT: usize = 8;
const FUNDAMENTAL: usize = 0;
const SECOND_HARMONIC: usize = 1;
const THIRD_HARMONIC: usize = 2;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RpmFilterBankConfig {
    pub rpm_filter_fade_range_hz: u16, // range in which notch filters fade down to min_hz
    pub rpm_filter_q: u16,             // Q of the notch filters
    pub rpm_filter_lpf_hz: u16,        // LPF cutoff (from motor rpm converted to Hz)
    pub rpm_filter_weights: [u16; RPM_FILTER_HARMONICS_COUNT], // weight as a percentage for each harmonic
    pub rpm_filter_harmonics: u8,      // number of harmonics, zero means filters off
    pub rpm_filter_min_hz: u8,         // minimum notch frequency for fundamental harmonic
    pub motor_count: u8,
}

impl RpmFilterBankConfig {
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

impl Default for RpmFilterBankConfig {
    fn default() -> Self {
        Self::new()
    }
}

// NOTE: I have considered the typestate pattern for the state machine and have elected not to use it.
/// State enum to drive state machine.
#[repr(u8)]
#[derive(Clone, Copy, Default, Debug, PartialEq)]
enum State {
    #[default]
    Stopped,
    Fundamental(usize),
    SecondHarmonic(usize),
    ThirdHarmonic(usize),
}

/*impl From<u8> for State {
    fn from(value: u8) -> Self {
        match value {
            0 => State::Stopped,
            1 => State::Fundamental(0),
            2 => Self::SecondHarmonic(0),
            3 => Self::ThirdHarmonic(0),
            _ => State::Stopped,
        }
    }
}*/

impl State {
    /// External trigger to kick off the sequence
    pub fn start(&mut self) {
        if let State::Stopped = self {
            *self = State::Fundamental(0);
        }
    }

    /// Perform one step of the state machine
    /// This is called from MotorMixer::rpm_filter_set_frequency_hz_iteration_step and so needs to be FAST.
    pub fn update(
        &mut self,
        config: &RpmFilterBankConfig,
        frequencies: RpmFilterFrequencies,
        ctx: &mut RpmFilterBankContext,
    ) {
        // state machine sets notch filter for one harmonic of one motor on each iteration.

        match core::mem::take(self) {
            State::Stopped => {
                // If we are stopped, we stay stopped until start() is called
                // Explicitly setting *self = State::Stopped defends against a change in the default.
                *self = State::Stopped;
            }
            State::Fundamental(motor_index) => {
                // omega = frequency * _2PiLoopTimeSeconds
                // max_frequency < 0.5 / looptime_seconds
                // max_omega = (0.5 / looptime_seconds) * 2PiLooptimeSeconds = 0.5 * 2PI = PI;
                // so omega is in range [0, PI]
                (ctx.motor_states[motor_index].sin_omega, ctx.motor_states[motor_index].cos_omega) =
                    ctx.motor_states[motor_index].omega.sin_cos();
                ctx.filters[motor_index][FUNDAMENTAL].set_notch_frequency_weighted_from_sin_cos_assuming_q(
                    ctx.motor_states[motor_index].sin_omega,
                    ctx.motor_states[motor_index].cos_omega,
                    ctx.weights[FUNDAMENTAL] * ctx.motor_states[motor_index].weight_multiplier,
                );
                *self = State::Fundamental(motor_index + 1);
                if motor_index == config.motor_count as usize {
                    // we have set the notch frequency for all motors, so move onto the next harmonic if there is one, otherwise we are finished
                    if config.rpm_filter_harmonics >= 2 {
                        if config.rpm_filter_weights[SECOND_HARMONIC] != 0 {
                            *self = State::SecondHarmonic(0);
                        } else if config.rpm_filter_harmonics >= 3 && config.rpm_filter_weights[THIRD_HARMONIC] != 0 {
                            *self = State::ThirdHarmonic(0);
                        } else {
                            *self = State::Stopped;
                        }
                    } else {
                        *self = State::Stopped;
                    }
                }
            }
            State::SecondHarmonic(motor_index) => {
                let motor_state = ctx.motor_states[motor_index];
                if motor_state.frequency_hz_unclamped > frequencies.half_of_max_hz {
                    // ie 2.0 * frequency_hz_unclamped > _max_frequency_hz
                    // no point filtering the second harmonic if it is above the Nyquist frequency
                    ctx.weights[SECOND_HARMONIC] = 0.0;
                } else {
                    ctx.weights[SECOND_HARMONIC] = config.rpm_filter_weights[SECOND_HARMONIC] as f32 * 0.01;
                    // sin(2θ) = 2 * sin(θ) * cos(θ)
                    // cos(2θ) = 2 * cos^2(θ) - 1
                    let sin_2_omega = motor_state.sin_omega * motor_state.cos_omega * 2.0;
                    let cos_2_omega = motor_state.cos_omega * motor_state.cos_omega * 2.0 - 1.0;
                    ctx.filters[motor_index][SECOND_HARMONIC].set_notch_frequency_weighted_from_sin_cos_assuming_q(
                        sin_2_omega,
                        cos_2_omega,
                        ctx.weights[SECOND_HARMONIC] * ctx.motor_states[motor_index].weight_multiplier,
                    );
                }
                *self = State::SecondHarmonic(motor_index + 1);
                if motor_index == config.motor_count as usize {
                    // we have set the notch frequency for all motors, so move onto the next harmonic if there is one, otherwise we are finished
                    if config.rpm_filter_harmonics >= 3 && config.rpm_filter_weights[THIRD_HARMONIC] != 0 {
                        *self = State::ThirdHarmonic(0);
                    } else {
                        *self = State::Stopped;
                    }
                }
            }
            State::ThirdHarmonic(motor_index) => {
                let motor_state = ctx.motor_states[motor_index];
                if motor_state.frequency_hz_unclamped > frequencies.third_of_max_hz {
                    // ie 3.0 * frequency_hz_unclamped > _max_frequency_hz
                    // no point filtering the third harmonic if it is above the Nyquist frequency
                    ctx.weights[THIRD_HARMONIC] = 0.0;
                } else {
                    ctx.weights[THIRD_HARMONIC] = config.rpm_filter_weights[THIRD_HARMONIC] as f32 * 0.01;
                    // sin(3θ) = 3 * sin(θ)   - 4 * sin^3(θ)
                    //         = sin(θ) * ( 3 - 4 * sin^2(θ) )
                    //         = sin(θ) * ( 3 - 4 * (1 - cos^2(θ)) )
                    //         = sin(θ) * ( 4 * cos^2(θ) - 1)
                    // cos(3θ) = 4 * cos^3(θ) - 3 * cos(θ)
                    //         = cos(θ) * ( 4 * cos^2(θ) - 3 )
                    let four_cos_squared_omega = 4.0 * motor_state.cos_omega * motor_state.cos_omega;
                    let sin_3_omega = motor_state.sin_omega * (four_cos_squared_omega - 1.0);
                    let cos_3_omega = motor_state.cos_omega * (four_cos_squared_omega - 3.0);
                    ctx.filters[motor_index][THIRD_HARMONIC].set_notch_frequency_weighted_from_sin_cos_assuming_q(
                        sin_3_omega,
                        cos_3_omega,
                        ctx.weights[THIRD_HARMONIC] * motor_state.weight_multiplier,
                    );
                }
                *self = State::ThirdHarmonic(motor_index + 1);
                if motor_index == config.motor_count as usize {
                    // we have set the notch frequency for all motors, so we are finished
                    *self = State::Stopped;
                }
            }
        }
    }
}

type BiquadFilters = [[BiquadFilterf32<Vector3df32>; RPM_FILTER_HARMONICS_COUNT]; MAX_MOTOR_COUNT];

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct RpmFilterMotorState {
    frequency_hz_unclamped: f32,
    weight_multiplier: f32,
    omega: f32,
    sin_omega: f32,
    cos_omega: f32,
}

type RpmFilterMotorStates = [RpmFilterMotorState; MAX_MOTOR_COUNT];

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RpmFilterFrequencies {
    min_hz: f32,
    max_hz: f32,
    half_of_max_hz: f32,
    third_of_max_hz: f32,
}

impl Default for RpmFilterFrequencies {
    fn default() -> Self {
        Self::new()
    }
}

impl RpmFilterFrequencies {
    pub fn new() -> Self {
        Self { min_hz: 100.0, max_hz: 0.0, half_of_max_hz: 0.0, third_of_max_hz: 0.0 }
    }
}
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct RpmFilterBankContext {
    filters: BiquadFilters,
    motor_states: RpmFilterMotorStates,
    weights: [f32; RPM_FILTER_HARMONICS_COUNT],
}

/// Bank of RpmFilters, one for each motor.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RpmFilterBank {
    motor_rpm_filters: [Pt1Filterf32<f32>; MAX_MOTOR_COUNT],
    config: RpmFilterBankConfig,
    frequencies: RpmFilterFrequencies,
    state: State,
    ctx: RpmFilterBankContext,
    looptime_seconds: f32,
    fade_range_hz: f32,
    q: f32,
}

impl Default for RpmFilterBank {
    fn default() -> Self {
        Self::new()
    }
}

impl RpmFilterBank {
    pub fn new() -> Self {
        Self {
            motor_rpm_filters: <[Pt1Filterf32<f32>; MAX_MOTOR_COUNT]>::default(),
            config: RpmFilterBankConfig::default(),
            frequencies: RpmFilterFrequencies::default(),
            state: State::default(),
            ctx: RpmFilterBankContext::default(),
            looptime_seconds: 0.001,
            fade_range_hz: 50.0,
            q: 0.0,
        }
    }

    pub fn set_config(&mut self, config: RpmFilterBankConfig) {
        self.config = config;

        self.q = config.rpm_filter_q as f32 * 0.01;

        self.state = State::Stopped;
        // just under  Nyquist frequency (ie just under half sampling rate)
        // for 8kHz loop this is 3840Hz
        self.frequencies.max_hz = 480000.0 / self.looptime_seconds;

        // pre-calculate frequencies for speed in iteration steps
        self.frequencies.half_of_max_hz = self.frequencies.max_hz / 2.0;
        self.frequencies.third_of_max_hz = self.frequencies.max_hz / 3.0;
        self.frequencies.min_hz = config.rpm_filter_min_hz as f32;
        self.fade_range_hz = config.rpm_filter_fade_range_hz as f32;

        for harmonic in 0..config.rpm_filter_harmonics as usize {
            for motor in 0..config.motor_count as usize {
                self.ctx.filters[motor][harmonic].init_notch(
                    self.frequencies.min_hz * (harmonic + 1) as f32,
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
    pub fn start(&mut self, motor_index: usize, frequency_hz: f32) {
        if self.config.rpm_filter_lpf_hz == 0 {
            return;
        }
        let mut motor_state = self.ctx.motor_states[motor_index];

        motor_state.frequency_hz_unclamped = self.motor_rpm_filters[motor_index].apply(frequency_hz);
        let frequency_hz = motor_state.frequency_hz_unclamped.clamp(self.frequencies.min_hz, self.frequencies.max_hz);

        let margin_frequency_hz = frequency_hz - self.frequencies.min_hz;
        motor_state.weight_multiplier =
            if margin_frequency_hz < self.fade_range_hz { margin_frequency_hz / self.fade_range_hz } else { 1.0 };

        let rpm_filter = self.ctx.filters[motor_index][FUNDAMENTAL];
        motor_state.omega = rpm_filter.calculate_omega(frequency_hz);

        self.ctx.motor_states[motor_index] = motor_state;

        self.state.start();
    }

    pub fn update(&mut self) {
        //self.state = self.state.update(self);
        self.state.update(&self.config, self.frequencies, &mut self.ctx);
    }

    pub fn filter(ctx: &mut RpmFilterBankContext, input: Vector3df32, motor_index: usize) -> Vector3df32 {
        let mut ret = ctx.filters[motor_index][FUNDAMENTAL].filter_weighted(input);

        if ctx.weights[SECOND_HARMONIC] != 0.0 {
            ret = ctx.filters[motor_index][SECOND_HARMONIC].filter_weighted(input);
        };
        if ctx.weights[THIRD_HARMONIC] != 0.0 {
            ret = ctx.filters[motor_index][THIRD_HARMONIC].filter_weighted(input);
        };
        ret
    }
}

pub trait RpmFilters {
    fn common(&self) -> &RpmFilterBank;
    fn common_mut(&mut self) -> &mut RpmFilterBank;
    fn config(&self) -> &RpmFilterBankConfig;

    fn filter(&mut self, value: Vector3df32, motor_index: usize) -> Vector3df32;
}

impl RpmFilters for RpmFilterBank {
    fn common(&self) -> &RpmFilterBank {
        self
    }
    fn common_mut(&mut self) -> &mut RpmFilterBank {
        self
    }
    fn config(&self) -> &RpmFilterBankConfig {
        &self.common().config
    }

    fn filter(&mut self, value: Vector3df32, _motor_index: usize) -> Vector3df32 {
        value
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn _is_normal<T: Sized + Send + Sync + Unpin>() {}
    fn is_full<T: Sized + Send + Sync + Unpin + Copy + Clone + Default + PartialEq>() {}

    #[test]
    fn normal_types() {
        is_full::<RpmFilterBankConfig>();
        is_full::<RpmFilterMotorState>();
        is_full::<State>();
        is_full::<RpmFilterBank>();
    }
    #[test]
    fn new() {
        let config = RpmFilterBankConfig::new();
        assert_eq!(50, config.rpm_filter_fade_range_hz);
    }
}
