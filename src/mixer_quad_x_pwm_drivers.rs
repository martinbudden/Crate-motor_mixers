use cfg_if::cfg_if;

cfg_if! {
if #[cfg(feature = "rp2040")] {
use embassy_rp::pwm::{Config, Pwm};
//#[cfg(feature = "stm32")]
//type PwmType = SimplePwm<'static, embassy_stm32::peripherals::TIM1>;
//#[cfg(feature = "esp32")]
//type PwmType = SimplePwm<'static, embassy_esp32::peripherals::LED_PWM>;
//#[cfg(feature = "rp2040")]
//type PwmType = SimplePwm<'static, embassy_rp::peripherals::PWM_SLICE0>;

pub struct MotorDriver {
    pwm0: Pwm<'static>,
    pwm1: Pwm<'static>,
    config0: Config,
    config1: Config,
    top: u16,
}

impl MotorDriver {
    pub fn new(pwm0: Pwm<'static>, pwm1: Pwm<'static>) -> Self {
        let config0 = Config::default();
        let config1 = Config::default();
        let top = config0.top;

        Self {
            pwm0,
            pwm1,
            config0: config0,
            config1: config1,
            top,
        }
    }

    pub fn write_motor(&mut self, motor_index: u8, motor_output: f32) {
        let pulse_ms = 1.0 + motor_output;
        let duty = ((pulse_ms / 20.0) * self.top as f32) as u16;

        match motor_index {
            0 => self.config0.compare_a = duty,
            1 => self.config0.compare_b = duty,
            2 => self.config1.compare_a = duty,
            3 => self.config1.compare_b = duty,
            _ => return,
        }

        let (pwm, config) = match motor_index {
            0 | 1 => (&mut self.pwm0, &self.config0),
            2 | 3 => (&mut self.pwm1, &self.config1),
            _ => return,
        };
        pwm.set_config(&config);
    }
}

/*
let pwm0 = Pwm::new_output_ab(p.PWM_SLICE0, p.PIN_0, p.PIN_1, Config::default());
let pwm1 = Pwm::new_output_ab(p.PWM_SLICE1, p.PIN_2, p.PIN_3, Config::default());
*/

} else if #[cfg(feature = "stm32")] {

use embassy_stm32::timer::simple_pwm::{SimplePwm, SimplePwmChannel};
use embassy_stm32::timer::GeneralInstance4Channel;

pub struct MotorDriver<T>
where
    T: GeneralInstance4Channel,
{
    ch1: SimplePwmChannel<'static, T>,
    ch2: SimplePwmChannel<'static, T>,
    ch3: SimplePwmChannel<'static, T>,
    ch4: SimplePwmChannel<'static, T>,
}

impl<T> MotorDriver<T>
where
    T: GeneralInstance4Channel,
{
    pub fn new(pwm: SimplePwm<'static, T>) -> Self {
        let channels = pwm.split();
        Self {
            ch1: channels.ch1,
            ch2: channels.ch2,
            ch3: channels.ch3,
            ch4: channels.ch4,
        }
    }
    pub fn write_motor(&mut self, motor_index: u8, motor_output: f32) {
        let pulse_ms = 1.0 + motor_output;
        let duty = (pulse_ms * 1000.0 / 20.0) as u32;

        let channel = match motor_index {
            0 => &mut self.ch1,
            1 => &mut self.ch2,
            2 => &mut self.ch3,
            3 => &mut self.ch4,
            _ => return,
        };

        channel.set_duty_cycle(duty);
        channel.enable();
    }
}
/*
let p = embassy_stm32::init(Default::default());
let ch1 = PwmPin::new_ch1(p.PA8); // TIM1_CH1
let ch2 = PwmPin::new_ch2(p.PA9);
let pwm = SimplePwm::new(p.TIM1, Some(ch1), Some(ch2), None, None, khz(1));
let mut driver = MotorDriver::new(pwm);
*/

} else if #[cfg(feature = "esp32")] {
/*use esp_idf_hal::ledc::{config::TimerConfig, LedcDriver, LedcTimerDriver, SpeedMode};
use esp_idf_hal::gpio::PinDriver;

let pin = PinDriver::output(p.GPIO0).unwrap();
let timer = LedcTimerDriver::new(&ledc, SpeedMode::Low, &TimerConfig::default()).unwrap();
let mut channel = LedcDriver::new(&ledc, SpeedMode::Low, &timer, pin).unwrap();

channel.set_duty(1023).unwrap(); // 10-bit duty   }
*/
use esp_idf_hal::ledc::{LedcDriver, LedcTimerDriver, SpeedMode, Channel};

pub struct MotorDriver {
    channels: [LedcDriver<'static>; 4],
}

impl MotorDriver {
    pub fn new(
        ch0: LedcDriver<'static>,
        ch1: LedcDriver<'static>,
        ch2: LedcDriver<'static>,
        ch3: LedcDriver<'static>,
    ) -> Self {
        Self {
            channels: [ch0, ch1, ch2, ch3],
        }
    }
    pub fn write_motor(&mut self, motor_index: u8, motor_output: f32) {
        // Convert [0.0, 1.0] to pulse width (1-2ms for ESCs)
        let pulse_ms = 1.0 + motor_output;
        // Scale to duty cycle based on timer resolution
        let max_duty = self.driver.get_max_duty();
        let duty = ((pulse_ms / 20.0) * max_duty as f32) as u32;

        // Set duty for the appropriate channel
        match motor_index {
            0 => self.driver.set_duty(Channel::CH0, duty).unwrap(),
            1 => self.driver.set_duty(Channel::CH1, duty).unwrap(),
            2 => self.driver.set_duty(Channel::CH2, duty).unwrap(),
            3 => self.driver.set_duty(Channel::CH3, duty).unwrap(),
            _ => return,
        }
        self.driver.update_duty().unwrap();
    }
}

}
//#[cfg(feature = "esp32")]
//type PwmType = SimplePwm<'static, embassy_esp32::peripherals::LED_PWM>;

}
