//! Provides a neat way to manage interrupts.

use nrf52840_hal::gpiote::Gpiote;

/// The events that can trigger gpiote events.
#[derive(Clone)]
pub enum GpioEvents {
    /// The first phases hal effect sensor was triggered.
    P1Hal,
    /// The second phases hal effect sensor was triggered.
    P2Hal,
    /// The thrid phases hal effect sensor was triggered.
    P3Hal,
    /// The can module has something to say.
    CAN,
}

/// Manages events and provides access to the [`Iter`]
pub struct Manager {
    pub(crate) gpiote: Gpiote,
}

/// Itterates over the generated events, and clears the previous one
/// automatically after each iteration
pub struct Iter<'a> {
    idx: usize,
    manager: &'a mut Manager,
}

impl Manager {
    /// Constructs a new event manager.
    pub(crate) const fn new(gpiote: Gpiote) -> Self {
        Self { gpiote }
    }
}

impl Manager {
    /// Returns an iterator of the events that have been triggered.
    pub fn events(&mut self) -> Iter<'_> {
        Iter {
            idx: 0,
            manager: self,
        }
    }

    /// Clears all channel events
    pub fn clear(&mut self) {
        self.gpiote.reset_events();
        self.gpiote.channel0().clear();
        self.gpiote.channel1().clear();
        self.gpiote.channel2().clear();
        self.gpiote.channel3().clear();
        self.gpiote.channel4().clear();
    }
}

impl<'a> Iterator for Iter<'a> {
    type Item = GpioEvents;

    fn next(&mut self) -> Option<Self::Item> {
        self.idx += 1;

        let gpiote = &mut self.manager.gpiote;
        match self.idx {
            1 => {
                if gpiote.channel0().is_event_triggered() {
                    // info!("Event 0 triggered");
                    gpiote.channel0().clear();
                    Some(GpioEvents::P1Hal)
                } else {
                    self.next()
                }
            }
            2 => {
                if gpiote.channel1().is_event_triggered() {
                    // info!("Event 0 triggered");
                    gpiote.channel1().clear();
                    Some(GpioEvents::P2Hal)
                } else {
                    self.next()
                }
            }
            3 => {
                if gpiote.channel3().is_event_triggered() {
                    // info!("Event 0 triggered");
                    gpiote.channel3().clear();
                    Some(GpioEvents::P3Hal)
                } else {
                    self.next()
                }
            }
            4 => {
                if gpiote.channel4().is_event_triggered() {
                    // info!("Event 0 triggered");
                    gpiote.channel4().clear();
                    Some(GpioEvents::CAN)
                } else {
                    self.next()
                }
            }
            _ => {
                gpiote.reset_events();
                // gpiote.port().reset_events();
                None
            }
        }
    }
}
