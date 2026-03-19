#![no_std]
#![deny(clippy::unwrap_used)]
#![deny(clippy::expect_used)]
#![deny(clippy::panic)]
#![deny(unused_must_use)]

pub mod dshot_codec;
pub mod dynamic_idle_controller;
pub mod mixers;
pub mod motor_mixer_quad_x_pwm;
pub mod motor_mixer_quad_x_pwm_drivers;
pub mod motor_mixers;

pub use mixers::{
    MotorMixer, MotorMixerCommands, MotorMixerCommandsDps, MotorMixerDriver, MotorMixerParameters, MotorMixerState,
};

pub use motor_mixers::{mix_airplane, mix_bicopter, mix_hex_x, mix_quad_x, mix_tricopter, mix_wing};

pub use motor_mixer_quad_x_pwm::MotorMixerQuadXPwm;
