#[derive(Clone, Copy, Debug, PartialEq, Default)]
pub struct MotorMixerCommands {
    // throttle commands are in the range [0.0, 1.0]
    pub throttle: f32,
    // roll, pitch, and yaw commands are in the range [-1.0, 1.0]
    pub roll: f32,
    pub pitch: f32,
    pub yaw: f32,
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

#[derive(Clone, Copy, Debug, PartialEq, Default)]
pub struct MotorMixerCommandsDps {
    // throttle commands are in the range [0.0, 1.0]
    pub throttle: f32,
    // roll, pitch, and yaw commands are in the degrees per second range ie approx [-1000.0, 1000.0]
    pub roll_dps: f32,
    pub pitch_dps: f32,
    pub yaw_dps: f32,
}

pub const TRICOPTER: u8 = 1;
pub const QUAD_P: u8 = 2;
pub const QUAD_X: u8 = 3;
pub const BICOPTER: u8 = 4;
pub const GIMBAL: u8 = 5;
pub const Y6: u8 = 6;
pub const HEX_P: u8 = 7;
pub const FLYING_WING_SINGLE_PROPELLER: u8 = 8;
pub const Y4: u8 = 9;
pub const HEX_X: u8 = 10;
pub const OCTO_QUAD_X: u8 = 11;
pub const OCTO_FLAT_P: u8 = 12;
pub const OCTO_FLAT_X: u8 = 13;
pub const AIRPLANE_SINGLE_PROPELLER: u8 = 14;
pub const HELI_120_CCPM: u8 = 15;
pub const HELI_90_DEG: u8 = 16;
pub const VTAIL4: u8 = 17;
pub const HEX_H: u8 = 18;
pub const PPM_TO_SERVO: u8 = 19; // PPM -> servo relay
pub const DUALCOPTER: u8 = 20;
pub const SINGLECOPTER: u8 = 21;
pub const ATAIL4: u8 = 22;
pub const CUSTOM: u8 = 23;
pub const CUSTOM_AIRPLANE: u8 = 24;
pub const CUSTOM_TRI: u8 = 25;
pub const QUAD_X_1234: u8 = 26;
pub const OCTO_XP: u8 = 27;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MixerConfig {
    // constants compatible with Betaflight mixerMode_e enums.
    pub mixer_type: u8,
    pub yaw_motors_reversed: bool,
}

impl MixerConfig {
    fn new() -> Self {
        Self {
            mixer_type: 0,
            yaw_motors_reversed: false,
        }
    }
}

impl Default for MixerConfig {
    fn default() -> Self {
        Self::new()
    }
}

pub const PROTOCOL_FAMILY_UNKNOWN: u8 = 0;
pub const PROTOCOL_FAMILY_PWM: u8 = 1;
pub const PROTOCOL_FAMILY_DSHOT: u8 = 2;

pub const MOTOR_PROTOCOL_PWM: u8 = 0;
pub const MOTOR_PROTOCOL_ONESHOT125: u8 = 1;
pub const MOTOR_PROTOCOL_ONESHOT42: u8 = 2;
pub const MOTOR_PROTOCOL_MULTISHOT: u8 = 3;
pub const MOTOR_PROTOCOL_BRUSHED: u8 = 4;
pub const MOTOR_PROTOCOL_DSHOT150: u8 = 5;
pub const MOTOR_PROTOCOL_DSHOT300: u8 = 6;
pub const MOTOR_PROTOCOL_DSHOT600: u8 = 7;
pub const MOTOR_PROTOCOL_PROSHOT1000: u8 = 8;
pub const MOTOR_PROTOCOL_DISABLED: u8 = 9;
pub const MOTOR_PROTOCOL_COUNT: u8 = 10;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MotorDeviceConfig {
    motor_pwm_rate: u16, // The update rate of motor outputs (50-498Hz)
    motor_protocol: u8,
    motor_inversion: bool, // Active-High vs Active-Low. Useful for brushed FCs converted for brushless operation
    use_continuous_update: bool,
    use_burst_dshot: bool,
    use_dshot_telemetry: bool,
    use_dshot_edt: bool,
}

impl MotorDeviceConfig {
    fn new() -> Self {
        Self {
            motor_pwm_rate: 480, // 16000 for brushed
            motor_protocol: MOTOR_PROTOCOL_DSHOT300,
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

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MotorConfig {
    device: MotorDeviceConfig,
    motor_idle: u16,   // percentage of the motor range added to the disarmed value to give the idle value
    max_throttle: u16, // value of throttle at full power, can be set up to 2000
    min_command: u16, // value for ESCs when they are not armed. For some specific ESCs this value must be lowered to 900
    kv: u16,          // Motor constant estimate RPM under no load
    motor_pole_count: u8, // Number of motor poles, used to calculate actual RPM from eRPM
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

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ServoDeviceConfig {
    // PWM values, in milliseconds, common range is 1000-2000 (1ms to 2ms)
    servo_center_pulse: u16, // This is the value for servos when they should be in the middle. e.g. 1500.
    servo_pwm_rate: u16,     // The update rate of servo outputs (50-498Hz)
}

impl ServoDeviceConfig {
    fn new() -> Self {
        Self {
            servo_center_pulse: 1500,
            servo_pwm_rate: 50,
        }
    }
}

impl Default for ServoDeviceConfig {
    fn default() -> Self {
        Self::new()
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ServoConfig {
    device: ServoDeviceConfig,
    servo_lowpass_freq: u16, // lowpass servo filter frequency selection; 1/1000ths of loop freq
    tri_unarmed_servo: bool, // send tail servo correction pulses even when unarmed
    channel_forwarding_start_channel: u8,
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

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MotorMixerState {
    mixer_type: u8,
    output_denominator: u8,
    output_count: u8,
    mixer_config: MixerConfig,
    motor_config: MotorConfig,
    mixer_parameters: MotorMixerParameters,
    throttle_command: f32, // used for blackbox recording
    motors_is_on: bool,
    motors_is_armed: bool,
    motors_is_reversed: bool, //reversed motors typically used to flip multi-rotor after a crash
}

impl MotorMixerState {
    fn new() -> Self {
        Self {
            mixer_type: QUAD_X,
            output_denominator: 1,
            output_count: 0,
            mixer_config: MixerConfig::default(),
            motor_config: MotorConfig::default(),
            mixer_parameters: MotorMixerParameters::default(),
            throttle_command: 0.0, // used for blackbox recording
            motors_is_on: false,
            motors_is_armed: false,
            motors_is_reversed: false, //reversed motors typically used to flip multi-rotor after a crash
        }
    }
}

impl Default for MotorMixerState {
    fn default() -> Self {
        Self::new()
    }
}

pub trait MotorMixer {
    fn state(&self) -> &MotorMixerState;
    fn state_mut(&mut self) -> &mut MotorMixerState;

    fn output_to_motors(&mut self, commands_dps: MotorMixerCommandsDps, delta_t: f32);

    fn motors_is_on(&self) -> bool {
        self.state().motors_is_on
    }
    fn motors_switch_off(&mut self) {
        self.state_mut().motors_is_on = false;
    }
    fn motors_switch_on(&mut self) {
        self.state_mut().motors_is_on = true;
    }
    fn motors_is_armed(&self) -> bool {
        self.state().motors_is_armed
    }
    /// Switch off motors and disarm.
    fn disarm_motors(&mut self) {
        self.motors_switch_off();
        self.state_mut().motors_is_armed = false;
    }
    /// Arm motors, ensuring they are switched off first.
    fn arm_motors(&mut self) {
        self.motors_switch_off();
        self.state_mut().motors_is_armed = true;
    }
    fn throttle_command(&self) -> f32 {
        self.state().throttle_command
    }
    fn set_throttle_command(&mut self, throttle_command: f32) {
        self.state_mut().throttle_command = throttle_command;
    }
    fn output_this_cycle(&mut self) -> bool {
        // TODO: check the logic of this
        self.state_mut().output_count += 1;
        if self.state().output_count < self.state().output_denominator {
            return false;
        }
        self.state_mut().output_count = 0;
        true
    }
}

pub trait MotorMixerDriver {
    fn write_to_motor(&mut self, index: u8, value: f32);
}
