//! Defines a simple pin map for this specific board version.
#![deny(warnings, missing_docs)]
#![no_std]

use constants::adc_gain;
use nrf52840_hal::{
    gpio::{
        self,
        p0::{self, P0_29, P0_30, P0_31},
        p1, Disconnected, Input, Output, Pin, PullUp, PushPull,
    },
    ppi::{ConfigurablePpi, Ppi},
    saadc::{Channel, SaadcConfig, SaadcTask, Time},
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
/// For more details see
/// [`CurrentManager::start_sample`],
/// [`CurrentManager::complete_sample`] or
/// [`CurrentManager::sample_blocking`]
pub struct CurrentManager {
    // Current sensing.
    _p1cs: P0_31<Disconnected>,
    // Current sensing.
    _p2cs: P0_30<Disconnected>,
    // Current sensing.
    _p3cs: P0_29<Disconnected>,
    adc: SaadcTask<3>,
}

/// The defualt pin configuration of the board.
pub struct PinConfig<const P1: bool, const P2: bool, const P3: bool, const ADC: bool> {
    p1: Option<PhasePins>,
    p2: Option<PhasePins>,
    p3: Option<PhasePins>,
    // Current sensing.
    p1cs: Option<P0_31<Disconnected>>,
    // Current sensing.
    p2cs: Option<P0_30<Disconnected>>,
    // Current sensing.
    p3cs: Option<P0_29<Disconnected>>,
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
            .enable_event_c0::<_, true, false>(&p1h)
            .enable_event_c1::<_, false, true>(&p1h)
            .enable_event_c2::<_, true, false>(&p2h)
            .enable_event_c3::<_, false, true>(&p2h)
            .enable_event_c4::<_, true, false>(&p3h)
            .enable_event_c5::<_, false, true>(&p3h);

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

        let p3cs = Some(p0.p0_29);
        let p2cs = Some(p0.p0_30);
        let p1cs = Some(p0.p0_31);

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

    /// Enables events on channel c5.
    ///
    /// If low to high is set this triggers on rising edge other wise it triggers on falling edge.
    fn enable_event_c5<
        T: nrf52840_hal::gpiote::GpioteInputPin,
        const LOW_TO_HIGH: bool,
        const HIGH_TO_LOW: bool,
    >(
        self,
        pin: &T,
    ) -> Self;

    /// Enables events on channel c6.
    ///
    /// If low to high is set this triggers on rising edge other wise it triggers on falling edge.
    fn enable_event_c6<
        T: nrf52840_hal::gpiote::GpioteInputPin,
        const LOW_TO_HIGH: bool,
        const HIGH_TO_LOW: bool,
    >(
        self,
        pin: &T,
    ) -> Self;

    /// Enables events on channel c7.
    ///
    /// If low to high is set this triggers on rising edge other wise it triggers on falling edge.
    fn enable_event_c7<
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
    ///
    /// This sets the adc to sample for ~1ms. This means that the control system should be ran at ~1khz
    pub fn configure_adc(
        mut self,
        adc: nrf52840_hal::pac::SAADC,
    ) -> (PinConfig<P1, P2, P3, true>, CurrentManager) {
        let mut cfg = SaadcConfig::default();
        cfg.time = Time::_40US;
        cfg.gain = adc_gain();
        cfg.oversample = nrf52840_hal::saadc::Oversample::OVER256X;
        cfg.reference = nrf52840_hal::saadc::Reference::INTERNAL;

        let p1cs = unsafe { self.p1cs.take().unwrap_unchecked() };
        let p2cs = unsafe { self.p2cs.take().unwrap_unchecked() };
        let p3cs = unsafe { self.p3cs.take().unwrap_unchecked() };
        // TODO: Change this to a DMA version as to not block for 3us.
        let adc = SaadcTask::new(
            adc,
            cfg,
            &[
                P0_31::<Disconnected>::channel(),
                P0_30::<Disconnected>::channel(),
                P0_29::<Disconnected>::channel(),
            ],
            [0; 3],
        );

        let current_manager = CurrentManager {
            adc,
            _p1cs: p1cs,
            _p2cs: p2cs,
            _p3cs: p3cs,
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
    #[inline(always)]
    pub fn start_sample(&mut self) {
        self.adc.start_sample();
    }

    /// Waits for the target value.
    pub fn sample_blocking(&mut self) -> Option<[f32; 3]> {
        self.adc.sample_blocking(Self::conv)
    }

    /// Gets the latest current measurements from the motor.
    ///
    /// Returns the measurements in mA.
    #[inline(always)]
    pub fn complete_sample(&mut self) -> [f32; 3] {
        self.adc.complete_sample(Self::conv)
    }

    /// Converts the adc value in to mA.
    #[inline(always)]
    fn conv(val: u16) -> f32 {
        2_500. / constants::shunt_factor()
            - f32::from(val) * constants::raw_adc_to_current_factor() / constants::shunt_factor()
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

    fn enable_event_c5<
        T: nrf52840_hal::gpiote::GpioteInputPin,
        const LOW_TO_HIGH: bool,
        const HIGH_TO_LOW: bool,
    >(
        mut self,
        pin: &T,
    ) -> Self {
        if LOW_TO_HIGH && HIGH_TO_LOW {
            self.1
                .channel5()
                .input_pin(pin)
                .lo_to_hi()
                .hi_to_lo()
                .enable_interrupt();
        } else if LOW_TO_HIGH {
            self.1
                .channel5()
                .input_pin(pin)
                .lo_to_hi()
                .enable_interrupt();
        } else {
            self.1
                .channel5()
                .input_pin(pin)
                .hi_to_lo()
                .enable_interrupt();
        }
        self.1.port().input_pin(pin).high();
        self.1.port().input_pin(pin).low();

        let ppi = &mut self.0.ppi5;

        ppi.set_event_endpoint(self.1.channel5().event());
        ppi.set_task_endpoint(self.1.channel5().task_out());

        ppi.enable();
        self
    }

    fn enable_event_c6<
        T: nrf52840_hal::gpiote::GpioteInputPin,
        const LOW_TO_HIGH: bool,
        const HIGH_TO_LOW: bool,
    >(
        mut self,
        pin: &T,
    ) -> Self {
        if LOW_TO_HIGH && HIGH_TO_LOW {
            self.1
                .channel6()
                .input_pin(pin)
                .lo_to_hi()
                .hi_to_lo()
                .enable_interrupt();
        } else if LOW_TO_HIGH {
            self.1
                .channel6()
                .input_pin(pin)
                .lo_to_hi()
                .enable_interrupt();
        } else {
            self.1
                .channel6()
                .input_pin(pin)
                .hi_to_lo()
                .enable_interrupt();
        }
        self.1.port().input_pin(pin).high();
        self.1.port().input_pin(pin).low();

        let ppi = &mut self.0.ppi6;

        ppi.set_event_endpoint(self.1.channel6().event());
        ppi.set_task_endpoint(self.1.channel6().task_out());

        ppi.enable();
        self
    }

    fn enable_event_c7<
        T: nrf52840_hal::gpiote::GpioteInputPin,
        const LOW_TO_HIGH: bool,
        const HIGH_TO_LOW: bool,
    >(
        mut self,
        pin: &T,
    ) -> Self {
        if LOW_TO_HIGH && HIGH_TO_LOW {
            self.1
                .channel7()
                .input_pin(pin)
                .lo_to_hi()
                .hi_to_lo()
                .enable_interrupt();
        } else if LOW_TO_HIGH {
            self.1
                .channel7()
                .input_pin(pin)
                .lo_to_hi()
                .enable_interrupt();
        } else {
            self.1
                .channel7()
                .input_pin(pin)
                .hi_to_lo()
                .enable_interrupt();
        }
        self.1.port().input_pin(pin).high();
        self.1.port().input_pin(pin).low();

        let ppi = &mut self.0.ppi7;

        ppi.set_event_endpoint(self.1.channel7().event());
        ppi.set_task_endpoint(self.1.channel7().task_out());

        ppi.enable();
        self
    }
}
