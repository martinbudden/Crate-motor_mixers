#![no_std]
#![deny(clippy::unwrap_used)]
#![deny(clippy::expect_used)]
#![deny(clippy::panic)]
#![deny(unused_must_use)]

pub mod dshot_codec;
pub mod dynamic_idle_controller;

pub mod mixer;
pub mod mixer_calculations;
pub mod mixer_config;
pub mod mixer_quad_x_dshot;
pub mod mixer_quad_x_pwm;
pub mod mixer_quad_x_pwm_drivers;

pub mod rpm_filters;
mod rpm_filters_state_machine;

pub use mixer::{MotorMixer, MotorMixerCommon, MotorMixerDriver};

pub use mixer_config::{
    MixerConfig, MixerType, MotorConfig, MotorDeviceConfig, MotorMixerCommands, MotorMixerCommandsDps,
    MotorMixerParameters, ServoConfig, ServoDeviceConfig,
};

pub use mixer_calculations::{mix_airplane, mix_bicopter, mix_hex_x, mix_quad_x, mix_tricopter, mix_wing};

pub use mixer_quad_x_pwm::MotorMixerQuadXPwm;

pub use rpm_filters::{MotorFrequencies, RpmFilterBank, RpmFilterBankConfig, RpmFilters};
