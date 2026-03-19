pub use filters::FilterPt1;
pub use pid_controller::{PidConstants, PidController, PidError};

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct DynamicIdleControllerConfig {
    pub dyn_idle_min_rpm_100: u8, // multiply this by 100 to get the actual min RPM
    pub dyn_idle_p_gain: u8,
    pub dyn_idle_i_gain: u8,
    pub dyn_idle_d_gain: u8,
    pub dyn_idle_max_increase: u8,
}

impl DynamicIdleControllerConfig {
    pub fn new() -> Self {
        Self {
            dyn_idle_min_rpm_100: 0,
            dyn_idle_p_gain: 50,
            dyn_idle_i_gain: 50,
            dyn_idle_d_gain: 50,
            dyn_idle_max_increase: 150,
        }
    }
}

// it is idiomatic to implement default in terms of new
impl Default for DynamicIdleControllerConfig {
    fn default() -> Self {
        Self::new()
    }
}

/// Dynamic Idle: use PID controller to boost motor speeds so that slowest motor does not go below minimum allowed RPM
///
/// A minimum RPM is required because the ESC will desynchronize if the motors turn too slowly (since they won't generate
/// enough back EMF for the ESC know the position of the rotor relative to the windings).
///
/// Note that a simple minimum output value is not sufficient: consider the case where the throttle is cut while hovering,
/// the quad will start to fall and this falling will generate a reverse torque on the motors which will eventually
/// overcome the fixed output value. Many types of maneuver can generate this reverse torque.
///
/// Instead we have a PID controller that increases output to the motors as the slowest motor nears the minimum allowed RPM.
///
pub trait DynamicIdleController {
    fn calculate_speed_increase(&mut self, slowest_motor_hz: f32, delta_t: f32) -> f32;
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct DynamicIdleControllerState {
    task_interval_microseconds: u32,
    minimum_allowed_motor_hz: f32, // minimum motor Hz, dynamically controlled
    max_increase: f32,
    // dynamic_idle_max_increase_delay_k :f32,
    pid: PidController<f32>, // PID to dynamic idle, ie to ensure slowest motor does not go below min RPS
    dterm_filter: FilterPt1<f32>,
    config: DynamicIdleControllerConfig,
}

impl DynamicIdleControllerState {
    fn new(task_interval_microseconds: u32) -> Self {
        Self {
            task_interval_microseconds,
            minimum_allowed_motor_hz: 0.0, // minimum motor Hz, dynamically controlled
            max_increase: 0.0,
            // dynamic_idle_max_increase_delay_k :f32,
            pid: PidController::new(1.0, 0.0, 0.0), // PID to dynamic idle, ie to ensure slowest motor does not go below min RPS
            dterm_filter: FilterPt1::default(),
            config: DynamicIdleControllerConfig::default(),
        }
    }
    pub fn config(&self) -> DynamicIdleControllerConfig {
        self.config
    }
    pub fn set_config(&mut self, config: DynamicIdleControllerConfig) {
        self.config = config;

        self.max_increase = self.config.dyn_idle_max_increase as f32 * 0.001;

        self.minimum_allowed_motor_hz = self.config.dyn_idle_min_rpm_100 as f32 * 100.0 / 60.0;
        self.pid.set_setpoint(self.minimum_allowed_motor_hz);

        // use Betaflight multiplier for compatibility with Betaflight Configurator
        self.pid.set_kp(self.config.dyn_idle_p_gain as f32 * 0.00015);

        let delta_t = self.task_interval_microseconds as f32 * 0.000001;

        self.pid.set_ki(self.config.dyn_idle_i_gain as f32 * 0.01 * delta_t);
        // limit Iterm to range [0, _max_increase]
        self.pid.set_integral_max(self.max_increase);
        self.pid.set_integral_min(0.0);

        self.pid.set_kd(self.config.dyn_idle_i_gain as f32 * 0.0000003 / delta_t);
        self.dterm_filter.set_k(800.0 * delta_t / 20.0); //approx 20ms D delay, arbitrarily suits many motors
    }

    pub fn minimum_allowed_motor_hz(&self) -> f32 {
        self.minimum_allowed_motor_hz
    }
    pub fn set_minimum_allowed_motor_hz(&mut self, minimum_allowed_motor_hz: f32) {
        self.minimum_allowed_motor_hz = minimum_allowed_motor_hz;
        self.pid.set_setpoint(self.minimum_allowed_motor_hz);
    }
}

impl Default for DynamicIdleControllerState {
    fn default() -> Self {
        Self::new(1000)
    }
}

impl DynamicIdleController for DynamicIdleControllerState {
    fn calculate_speed_increase(&mut self, slowest_motor_hz: f32, delta_t: f32) -> f32 {
        if self.minimum_allowed_motor_hz == 0.0 {
            // if motors are allowed to stop, then no speed increase is needed
            return 0.0;
        }

        let slowest_motor_hz_delta_filtered =
            self.dterm_filter.filter(slowest_motor_hz - self.pid.previous_measurement());
        let speed_increase = self.pid.update_delta(slowest_motor_hz, slowest_motor_hz_delta_filtered, delta_t);

        /*if (debug.get_mode() == DEBUG_DYN_IDLE) {
            const pid_error_t error = _pid.get_error();
            debug.set(0, static_cast<int16_t>(std::max(-1000L, std::lroundf(error.p * 10000))));
            debug.set(1, static_cast<int16_t>(std::lroundf(error.i * 10000)));
            debug.set(2, static_cast<int16_t>(std::lroundf(error.d * 10000)));
        }*/

        speed_increase.clamp(0.0, self.max_increase)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use assert_float_eq::assert_f32_near;

    fn is_normal<T: Sized + Send + Sync + Unpin>() {}

    #[test]
    fn normal_types() {
        is_normal::<DynamicIdleControllerConfig>();
        is_normal::<DynamicIdleControllerState>();
    }
    #[test]
    fn dynamic_idle_controller() {
        const TASK_INTERVAL_MICROSECONDS: u32 = 1000;
        let dynamic_idle_controller_config = DynamicIdleControllerConfig {
            dyn_idle_min_rpm_100: 0,
            dyn_idle_p_gain: 50,
            dyn_idle_i_gain: 50,
            dyn_idle_d_gain: 50,
            dyn_idle_max_increase: 150,
        };
        let mut dynamic_idle_controller = DynamicIdleControllerState::new(TASK_INTERVAL_MICROSECONDS);
        dynamic_idle_controller.set_config(dynamic_idle_controller_config);
        const DELTA_T: f32 = TASK_INTERVAL_MICROSECONDS as f32 * 0.000001;

        assert_eq!(0, dynamic_idle_controller.config().dyn_idle_min_rpm_100);

        assert_eq!(0.0, dynamic_idle_controller.calculate_speed_increase(0.0, DELTA_T));
        const SLOWEST_MOTOR_HZ: f32 = 1000.0 / 60.0; // 1000 RPM
        assert_eq!(0.0, dynamic_idle_controller.calculate_speed_increase(SLOWEST_MOTOR_HZ, DELTA_T));
    }

    #[test]
    fn dynamic_idle_controller_p_only() {
        const MOTOR_HZ_500_RPM: f32 = 500.0 / 60.0;
        const MOTOR_HZ_750_RPM: f32 = 750.0 / 60.0;
        const MOTOR_HZ_1000_RPM: f32 = 1000.0 / 60.0;
        const MOTOR_HZ_2000_RPM: f32 = 2000.0 / 60.0;

        const TASK_INTERVAL_MICROSECONDS: u32 = 1000;
        let dynamic_idle_controller_config = DynamicIdleControllerConfig {
            dyn_idle_min_rpm_100: 10, // 10*100 = 1000 rpm
            dyn_idle_p_gain: 50,
            dyn_idle_i_gain: 0,
            dyn_idle_d_gain: 0,
            dyn_idle_max_increase: 150,
        };
        let mut dynamic_idle_controller = DynamicIdleControllerState::new(TASK_INTERVAL_MICROSECONDS);
        dynamic_idle_controller.set_config(dynamic_idle_controller_config);
        let delta_t = TASK_INTERVAL_MICROSECONDS as f32 * 0.000001;

        assert_eq!(10, dynamic_idle_controller.config().dyn_idle_min_rpm_100);
        assert_eq!(MOTOR_HZ_1000_RPM, dynamic_idle_controller.minimum_allowed_motor_hz());

        // slowest motor faster than 1000 RPM, so no speed increase
        assert_eq!(0.0, dynamic_idle_controller.calculate_speed_increase(MOTOR_HZ_2000_RPM, delta_t));
        assert_eq!(0.0, dynamic_idle_controller.calculate_speed_increase(MOTOR_HZ_1000_RPM, delta_t));

        // slowest motor slower than 1000 RPM, speed increase
        assert_eq!(0.0625, dynamic_idle_controller.calculate_speed_increase(MOTOR_HZ_500_RPM, delta_t));
        assert_eq!(0.0625, dynamic_idle_controller.calculate_speed_increase(MOTOR_HZ_500_RPM, delta_t));
        // half the speed difference from 1000, so half the output, since PID is P-Term only
        assert_f32_near!(0.03125, dynamic_idle_controller.calculate_speed_increase(MOTOR_HZ_750_RPM, delta_t));
        assert_f32_near!(0.03125, dynamic_idle_controller.calculate_speed_increase(MOTOR_HZ_750_RPM, delta_t));
    }
}
