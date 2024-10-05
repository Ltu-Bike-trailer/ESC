//! Defines a simple pin map for this specific board version.
#![deny(warnings, missing_docs)]
#![no_std]

use constants::{adc_gain, raw_adc_to_current_factor};
use nrf52840_hal::{
    gpio::{
        self,
        p0::{self, P0_29, P0_30, P0_31},
        p1, Floating, Input, Output, Pin, PullUp, PushPull,
    },
    ppi::{ConfigurablePpi, Ppi},
    saadc::{SaadcConfig, Time},
    Saadc,
};
pub mod constants;
pub mod events;

/// The pins required to drive a single phase of the motor.
pub struct PhasePins {
    /// The high side mosfet.
    pub high_side: Pin<Output<PushPull>>,
    /// The lowside mosfet.
    pub low_side: Pin<Output<PushPull>>,
    /// The hal effect feedback pin.
    pub hal_effect: Pin<Input<PullUp>>,
}

/// A simple current measurment tool.
///
/// For more details see [`CurrentManager::sample`]
pub struct CurrentManager {
    // Current sensing.
    p1cs: P0_31<Input<Floating>>,
    // Current sensing.
    p2cs: P0_30<Input<Floating>>,
    // Current sensing.
    p3cs: P0_29<Input<Floating>>,
    adc: Saadc,
}

/// The defualt pin configuration of the board.
pub struct PinConfig<const P1: bool, const P2: bool, const P3: bool, const ADC: bool> {
    p1: Option<PhasePins>,
    p2: Option<PhasePins>,
    p3: Option<PhasePins>,
    // Current sensing.
    p1cs: Option<P0_31<Input<Floating>>>,
    // Current sensing.
    p2cs: Option<P0_30<Input<Floating>>>,
    // Current sensing.
    p3cs: Option<P0_29<Input<Floating>>>,
    /// Manages gpiote events and makes them pretty.
    event_manager: events::Manager,
}

impl PinConfig<false, false, false, false> {
    /// Creates  a new [`PinConfig`]
    #[must_use]
    pub fn new(
        p0: p0::Parts,
        _p1: p1::Parts,
        ppi: nrf52840_hal::ppi::Parts,
        gpiote: nrf52840_hal::pac::GPIOTE,
    ) -> Self {
        let p1h = p0.p0_17.into_pullup_input().degrade();
        let p2h = p0.p0_14.into_pullup_input().degrade();
        let p3h = p0.p0_13.into_pullup_input().degrade();

        // Start events on the specified channels.
        let gpiote = nrf52840_hal::gpiote::Gpiote::new(gpiote);
        let conf = (ppi, gpiote);
        let (_ppi, gpiote) = conf
            .enable_event_c0::<_, true, true>(&p1h)
            .enable_event_c1::<_, true, true>(&p2h)
            .enable_event_c2::<_, true, true>(&p3h);

        let p1 = PhasePins {
            high_side: p0
                .p0_19
                .into_push_pull_output(gpio::Level::Low)
                .degrade()
                .into(),
            low_side: p0
                .p0_20
                .into_push_pull_output(gpio::Level::Low)
                .degrade()
                .into(),
            // TODO: Check if we actually need a pullup here.
            hal_effect: p1h,
        };
        let p2 = PhasePins {
            high_side: p0
                .p0_21
                .into_push_pull_output(gpio::Level::Low)
                .degrade()
                .into(),
            low_side: p0
                .p0_22
                .into_push_pull_output(gpio::Level::Low)
                .degrade()
                .into(),
            // TODO: Check if we actually need a pullup here.
            hal_effect: p2h,
        };
        let p3 = PhasePins {
            high_side: p0
                .p0_24
                .into_push_pull_output(gpio::Level::Low)
                .degrade()
                .into(),
            low_side: p0
                .p0_23
                .into_push_pull_output(gpio::Level::Low)
                .degrade()
                .into(),
            // TODO: Check if we actually need a pullup here.
            hal_effect: p3h,
        };

        let p3cs = Some(p0.p0_29.into_floating_input());
        let p2cs = Some(p0.p0_30.into_floating_input());
        let p1cs = Some(p0.p0_31.into_floating_input());

        gpiote.port().enable_interrupt();
        let event_manager = events::Manager::new(gpiote);

        Self {
            p1: Some(p1),
            p2: Some(p2),
            p3: Some(p3),
            p1cs,
            p2cs,
            p3cs,
            event_manager,
        }
    }
}

#[allow(dead_code)]
trait EnableEvent {
    /// Enables events on channel c0.
    ///
    /// If low to high is set this triggers on rising edge other wise it triggers on falling edge.
    fn enable_event_c0<
        T: nrf52840_hal::gpiote::GpioteInputPin,
        const LOW_TO_HIGH: bool,
        const HIGH_TO_LOW: bool,
    >(
        self,
        pin: &T,
    ) -> Self;
    /// Enables events on channel c1.
    ///
    /// If low to high is set this triggers on rising edge other wise it triggers on falling edge.
    fn enable_event_c1<
        T: nrf52840_hal::gpiote::GpioteInputPin,
        const LOW_TO_HIGH: bool,
        const HIGH_TO_LOW: bool,
    >(
        self,
        pin: &T,
    ) -> Self;
    /// Enables events on channel c2.
    ///
    /// If low to high is set this triggers on rising edge other wise it triggers on falling edge.
    fn enable_event_c2<
        T: nrf52840_hal::gpiote::GpioteInputPin,
        const LOW_TO_HIGH: bool,
        const HIGH_TO_LOW: bool,
    >(
        self,
        pin: &T,
    ) -> Self;

    /// Enables events on channel c3.
    ///
    /// If low to high is set this triggers on rising edge other wise it triggers on falling edge.
    fn enable_event_c3<
        T: nrf52840_hal::gpiote::GpioteInputPin,
        const LOW_TO_HIGH: bool,
        const HIGH_TO_LOW: bool,
    >(
        self,
        pin: &T,
    ) -> Self;

    /// Enables events on channel c4.
    ///
    /// If low to high is set this triggers on rising edge other wise it triggers on falling edge.
    fn enable_event_c4<
        T: nrf52840_hal::gpiote::GpioteInputPin,
        const LOW_TO_HIGH: bool,
        const HIGH_TO_LOW: bool,
    >(
        self,
        pin: &T,
    ) -> Self;
}

impl<const P2: bool, const P3: bool, const ADC: bool> PinConfig<false, P2, P3, ADC> {
    /// Configures the first phase of the motor.
    pub fn configure_p1(mut self) -> (PinConfig<true, P2, P3, ADC>, PhasePins) {
        let ret = unsafe { self.p1.take().unwrap_unchecked() };
        (
            PinConfig {
                p1: None,
                p2: self.p2.take(),
                p3: self.p3.take(),
                p1cs: self.p1cs.take(),
                p2cs: self.p2cs.take(),
                p3cs: self.p3cs.take(),
                event_manager: self.event_manager,
            },
            ret,
        )
    }
}

impl<const P1: bool, const P3: bool, const ADC: bool> PinConfig<P1, false, P3, ADC> {
    /// Configures the second phase of the motor.
    pub fn configure_p2(mut self) -> (PinConfig<P1, true, P3, ADC>, PhasePins) {
        let ret = unsafe { self.p2.take().unwrap_unchecked() };
        (
            PinConfig {
                p1: self.p1.take(),
                p2: None,
                p3: self.p3.take(),
                p1cs: self.p1cs.take(),
                p2cs: self.p2cs.take(),
                p3cs: self.p3cs.take(),
                event_manager: self.event_manager,
            },
            ret,
        )
    }
}

impl<const P1: bool, const P2: bool, const ADC: bool> PinConfig<P1, P2, false, ADC> {
    /// Configures the second phase of the motor.
    pub fn configure_p3(mut self) -> (PinConfig<P1, P2, true, ADC>, PhasePins) {
        let ret = unsafe { self.p3.take().unwrap_unchecked() };
        (
            PinConfig {
                p1: self.p1.take(),
                p2: self.p2.take(),
                p3: None,
                p1cs: self.p1cs.take(),
                p2cs: self.p2cs.take(),
                p3cs: self.p3cs.take(),
                event_manager: self.event_manager,
            },
            ret,
        )
    }
}

impl<const P1: bool, const P2: bool, const P3: bool> PinConfig<P1, P2, P3, false> {
    /// Configures the second phase of the motor.
    pub fn configure_adc(
        mut self,
        adc: nrf52840_hal::pac::SAADC,
    ) -> (PinConfig<P1, P2, P3, false>, CurrentManager) {
        let mut cfg = SaadcConfig::default();
        cfg.time = Time::_3US;
        cfg.gain = adc_gain();
        // TODO: Change this to a DMA version as to not block for 3us.
        let adc = Saadc::new(adc, cfg);

        let current_manager = unsafe {
            CurrentManager {
                p1cs: self.p1cs.take().unwrap_unchecked(),
                p2cs: self.p2cs.take().unwrap_unchecked(),
                p3cs: self.p3cs.take().unwrap_unchecked(),
                adc,
            }
        };

        (
            PinConfig {
                p1: self.p1.take(),
                p2: self.p2.take(),
                p3: None,
                p1cs: None,
                p2cs: None,
                p3cs: None,
                event_manager: self.event_manager,
            },
            current_manager,
        )
    }
}

impl<const P1: bool, const P2: bool, const P3: bool, const ADC: bool> PinConfig<P1, P2, P3, ADC> {
    #[must_use]
    /// Configures the second phase of the motor.
    pub fn complete(self) -> events::Manager {
        self.event_manager
    }
}

impl CurrentManager {
    /// Returns the current measurements in phase order.
    pub fn sample(&mut self) -> Result<[f32; 3], ()> {
        Ok([
            f32::from(self.adc.read_channel(&mut self.p1cs)?) * raw_adc_to_current_factor(),
            f32::from(self.adc.read_channel(&mut self.p2cs)?) * raw_adc_to_current_factor(),
            f32::from(self.adc.read_channel(&mut self.p3cs)?) * raw_adc_to_current_factor(),
        ])
    }
}

impl EnableEvent for (nrf52840_hal::ppi::Parts, nrf52840_hal::gpiote::Gpiote) {
    fn enable_event_c0<
        T: nrf52840_hal::gpiote::GpioteInputPin,
        const LOW_TO_HIGH: bool,
        const HIGH_TO_LOW: bool,
    >(
        mut self,
        pin: &T,
    ) -> Self {
        if LOW_TO_HIGH && HIGH_TO_LOW {
            self.1
                .channel0()
                .input_pin(pin)
                .lo_to_hi()
                .hi_to_lo()
                .enable_interrupt();
        } else if LOW_TO_HIGH {
            self.1
                .channel0()
                .input_pin(pin)
                .lo_to_hi()
                .enable_interrupt();
        } else {
            self.1
                .channel0()
                .input_pin(pin)
                .hi_to_lo()
                .enable_interrupt();
        }
        self.1.port().input_pin(pin).high();
        self.1.port().input_pin(pin).low();

        let ppi0 = &mut self.0.ppi0;

        ppi0.set_event_endpoint(self.1.channel0().event());
        ppi0.set_task_endpoint(self.1.channel0().task_out());

        ppi0.enable();
        self
    }
    fn enable_event_c1<
        T: nrf52840_hal::gpiote::GpioteInputPin,
        const LOW_TO_HIGH: bool,
        const HIGH_TO_LOW: bool,
    >(
        mut self,
        pin: &T,
    ) -> Self {
        if LOW_TO_HIGH && HIGH_TO_LOW {
            self.1
                .channel1()
                .input_pin(pin)
                .lo_to_hi()
                .hi_to_lo()
                .enable_interrupt();
        } else if LOW_TO_HIGH {
            self.1
                .channel1()
                .input_pin(pin)
                .lo_to_hi()
                .hi_to_lo()
                .enable_interrupt();
        } else {
            self.1
                .channel1()
                .input_pin(pin)
                .hi_to_lo()
                .enable_interrupt();
        }
        self.1.port().input_pin(pin).high();
        self.1.port().input_pin(pin).low();

        let ppi = &mut self.0.ppi1;

        ppi.set_event_endpoint(self.1.channel1().event());
        ppi.set_task_endpoint(self.1.channel1().task_out());

        ppi.enable();
        self
    }
    fn enable_event_c2<
        T: nrf52840_hal::gpiote::GpioteInputPin,
        const LOW_TO_HIGH: bool,
        const HIGH_TO_LOW: bool,
    >(
        mut self,
        pin: &T,
    ) -> Self {
        if LOW_TO_HIGH && HIGH_TO_LOW {
            self.1
                .channel2()
                .input_pin(pin)
                .lo_to_hi()
                .hi_to_lo()
                .enable_interrupt();
        } else if LOW_TO_HIGH {
            self.1
                .channel2()
                .input_pin(pin)
                .lo_to_hi()
                .enable_interrupt();
        } else {
            self.1
                .channel2()
                .input_pin(pin)
                .hi_to_lo()
                .enable_interrupt();
        }
        self.1.port().input_pin(pin).high();
        self.1.port().input_pin(pin).low();

        let ppi = &mut self.0.ppi2;

        ppi.set_event_endpoint(self.1.channel2().event());
        ppi.set_task_endpoint(self.1.channel2().task_out());

        ppi.enable();
        self
    }
    fn enable_event_c3<
        T: nrf52840_hal::gpiote::GpioteInputPin,
        const LOW_TO_HIGH: bool,
        const HIGH_TO_LOW: bool,
    >(
        mut self,
        pin: &T,
    ) -> Self {
        if LOW_TO_HIGH && HIGH_TO_LOW {
            self.1
                .channel3()
                .input_pin(pin)
                .lo_to_hi()
                .hi_to_lo()
                .enable_interrupt();
        } else if LOW_TO_HIGH {
            self.1
                .channel3()
                .input_pin(pin)
                .lo_to_hi()
                .enable_interrupt();
        } else {
            self.1
                .channel3()
                .input_pin(pin)
                .hi_to_lo()
                .enable_interrupt();
        }
        self.1.port().input_pin(pin).high();
        self.1.port().input_pin(pin).low();

        let ppi = &mut self.0.ppi3;

        ppi.set_event_endpoint(self.1.channel3().event());
        ppi.set_task_endpoint(self.1.channel3().task_out());
        ppi.enable();
        self
    }
    fn enable_event_c4<
        T: nrf52840_hal::gpiote::GpioteInputPin,
        const LOW_TO_HIGH: bool,
        const HIGH_TO_LOW: bool,
    >(
        mut self,
        pin: &T,
    ) -> Self {
        if LOW_TO_HIGH && HIGH_TO_LOW {
            self.1
                .channel4()
                .input_pin(pin)
                .lo_to_hi()
                .hi_to_lo()
                .enable_interrupt();
        } else if LOW_TO_HIGH {
            self.1
                .channel4()
                .input_pin(pin)
                .lo_to_hi()
                .enable_interrupt();
        } else {
            self.1
                .channel4()
                .input_pin(pin)
                .hi_to_lo()
                .enable_interrupt();
        }
        self.1.port().input_pin(pin).high();
        self.1.port().input_pin(pin).low();

        let ppi = &mut self.0.ppi4;

        ppi.set_event_endpoint(self.1.channel4().event());
        ppi.set_task_endpoint(self.1.channel4().task_out());

        ppi.enable();
        self
    }
}
