use serde::{Deserialize, Serialize};

#[derive(Clone, Copy, Debug, PartialEq, Default)]
pub struct MotorMixerCommands {
    // throttle commands are in the range [0.0, 1.0]
    pub throttle: f32,
    // roll, pitch, and yaw commands are in the range [-1.0, 1.0]
    pub roll: f32,
    pub pitch: f32,
    pub yaw: f32,
}

#[derive(Clone, Copy, Debug, PartialEq, Default)]
pub struct MotorMixerCommandsDps {
    // throttle commands are in the range [0.0, 1.0]
    pub throttle: f32,
    // roll, pitch, and yaw commands are in the degrees per second range ie approx [-1000.0, 1000.0]
    pub roll_dps: f32,
    pub pitch_dps: f32,
    pub yaw_dps: f32,
}

// parameters to mix function
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MotorMixerParameters {
    // minimum motor output, typically set to 5.5% to avoid ESC desynchronization,
    // may be set to zero if using dynamic idle control or brushed motors
    pub motor_output_min: f32,
    pub motor_output_max: f32,
    pub max_servo_angle_radians: f32, // used by tricopter
    pub throttle: f32,                // possibly adjusted throttle value for recording by blackbox
    pub undershoot: f32,              // used by test code
    pub overshoot: f32,               // used by test code
}

impl MotorMixerParameters {
    fn new() -> Self {
        Self {
            motor_output_min: 0.0,
            motor_output_max: 1.0,
            max_servo_angle_radians: 0.0, // used by tricopter
            throttle: 0.0,                // possibly adjusted throttle value for recording by blackbox
            undershoot: 0.0,              // used by test code
            overshoot: 0.0,               // used by test code
        }
    }
}

impl Default for MotorMixerParameters {
    fn default() -> Self {
        Self::new()
    }
}

#[repr(u8)]
pub enum MixerType {
    Tricopter = 1,
    QuadP = 2,
    QuadX = 3,
    Bicopter = 4,
    Gimbal = 5,
    Y6 = 6,
    HexP = 7,
    FlyingWingSinglePropeller = 8,
    Y4 = 9,
    HexX = 10,
    OctoQuadX = 11,
    OctoFlatP = 12,
    OctoFlatX = 13,
    AirplaneSinglePropeller = 14,
    Heli120Ccpm = 15,
    Heli90Deg = 16,
    Vtail4 = 17,
    HexH = 18,
    PpmToServo = 19, // PPM -> servo relay
    DualCopter = 20,
    SingleCopter = 21,
    Atail4 = 22,
    Custom = 23,
    CustomAirplane = 24,
    CustomTri = 25,
    QuadX1234 = 26,
    OctoXp = 27,
}

#[derive(Clone, Copy, Debug, PartialEq, Deserialize, Serialize)]
pub struct MixerConfig {
    // constants compatible with Betaflight mixerMode_e enums.
    pub mixer_type: u8,
    pub yaw_motors_reversed: bool,
}

impl MixerConfig {
    fn new() -> Self {
        Self { mixer_type: MixerType::QuadX as u8, yaw_motors_reversed: true }
    }
}

impl Default for MixerConfig {
    fn default() -> Self {
        Self::new()
    }
}

#[repr(u8)]
pub enum ProtocolFamily {
    Unknown = 0,
    Pwm = 1,
    Dshot = 2,
}

#[repr(u8)]
pub enum MotorProtocol {
    Pwm = 0,
    Oneshot125 = 1,
    Oneshot42 = 2,
    Multishot = 3,
    Brushed = 4,
    Dshot150 = 5,
    Dshot300 = 6,
    Dshot600 = 7,
    Proshot1000 = 8,
    Disabled = 9,
    //Count = 10,
}

#[derive(Clone, Copy, Debug, PartialEq, Deserialize, Serialize)]
pub struct MotorDeviceConfig {
    pub motor_pwm_rate: u16, // The update rate of motor outputs (50-498Hz)
    pub motor_protocol: u8,
    pub motor_inversion: bool, // Active-High vs Active-Low. Useful for brushed FCs converted for brushless operation
    pub use_continuous_update: bool,
    pub use_burst_dshot: bool,
    pub use_dshot_telemetry: bool,
    pub use_dshot_edt: bool,
}

impl MotorDeviceConfig {
    fn new() -> Self {
        Self {
            motor_pwm_rate: 480, // 16000 for brushed
            motor_protocol: MotorProtocol::Dshot300 as u8,
            motor_inversion: false,
            use_continuous_update: true,
            use_burst_dshot: false,
            use_dshot_telemetry: false,
            use_dshot_edt: false,
        }
    }
}

impl Default for MotorDeviceConfig {
    fn default() -> Self {
        Self::new()
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Deserialize, Serialize)]
pub struct MotorConfig {
    pub device: MotorDeviceConfig,
    pub motor_idle: u16, // percentage of the motor range added to the disarmed value to give the idle value
    pub max_throttle: u16, // value of throttle at full power, can be set up to 2000
    pub min_command: u16, // value for ESCs when they are not armed. For some specific ESCs this value must be lowered to 900
    pub kv: u16,          // Motor constant estimate RPM under no load
    pub motor_pole_count: u8, // Number of motor poles, used to calculate actual RPM from eRPM
}

impl MotorConfig {
    fn new() -> Self {
        Self {
            device: MotorDeviceConfig::default(),
            motor_idle: 550, // 700 for brushed
            max_throttle: 2000,
            min_command: 1000,
            kv: 1960,
            motor_pole_count: 14,
        }
    }
}

impl Default for MotorConfig {
    fn default() -> Self {
        Self::new()
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Deserialize, Serialize)]
pub struct ServoDeviceConfig {
    // PWM values, in milliseconds, common range is 1000-2000 (1ms to 2ms)
    pub servo_center_pulse: u16, // This is the value for servos when they should be in the middle. e.g. 1500.
    pub servo_pwm_rate: u16,     // The update rate of servo outputs (50-498Hz)
}

impl ServoDeviceConfig {
    fn new() -> Self {
        Self { servo_center_pulse: 1500, servo_pwm_rate: 50 }
    }
}

impl Default for ServoDeviceConfig {
    fn default() -> Self {
        Self::new()
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Deserialize, Serialize)]
pub struct ServoConfig {
    pub device: ServoDeviceConfig,
    pub servo_lowpass_freq: u16, // lowpass servo filter frequency selection; 1/1000ths of loop freq
    pub tri_unarmed_servo: bool, // send tail servo correction pulses even when unarmed
    pub channel_forwarding_start_channel: u8,
}

impl ServoConfig {
    fn new() -> Self {
        Self {
            device: ServoDeviceConfig::default(),
            servo_lowpass_freq: 0,
            tri_unarmed_servo: false,
            channel_forwarding_start_channel: 0,
        }
    }
}

impl Default for ServoConfig {
    fn default() -> Self {
        Self::new()
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
        is_full::<MotorMixerCommands>();
        is_full::<MotorMixerCommandsDps>();
        is_full::<MotorMixerParameters>();
        is_config::<MixerConfig>();
        is_config::<MotorDeviceConfig>();
        is_config::<MotorConfig>();
        is_config::<ServoDeviceConfig>();
        is_config::<ServoConfig>();
    }
    #[test]
    fn new() {
        let config = MixerConfig::new();
        assert_eq!(3, config.mixer_type);
    }
}
