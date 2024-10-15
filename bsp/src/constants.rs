//! Defines a few board specific constants.

use nrf52840_hal::saadc;
/// Resistance to ground for the current sense pins.
pub const R_PHASE_SENSE_TO_GROUND: f32 = 10_000.;

/// Resistance to vin for the current sense pins.
pub const R_PHASE_SENSE_TO_VIN: f32 = 4_700.;

/// The shunt resistor that the current sense measures voltage across.
///
/// The Sense resistor is 10 mOhm on the esc.
pub const R_SHUNT: f32 = 0.01;

/// The gain that the sensor applies to the differential measurement.
pub const SENSE_GAIN: f32 = 50.;

/// The system voltage.
pub const V_REF: f32 = 3.3;

/// The gain in the nrf chip.
pub const ADC_BITS: usize = 14;

/// The gain of the adc on the nrf chip.
pub const ADC_GAIN: f32 = 1.0;

/// The voltage divider used for current sensing.
/// $I_shunt = (2.5-V_sense *  ( R_5+R_4 ) / R_5) / ( R_shunt  * NågonGain ) $
pub const fn raw_adc_to_current_factor() -> f32 {
    let v_sense = (V_REF / ((1 << ADC_BITS) as f32)) / ADC_GAIN;
    (2.5 - v_sense * (R_PHASE_SENSE_TO_GROUND + R_PHASE_SENSE_TO_VIN) / R_PHASE_SENSE_TO_VIN)
        / (R_SHUNT * SENSE_GAIN)
}

/// Converts the adc gain to the type representation at compile time.
pub const fn adc_gain() -> saadc::Gain {
    if ADC_GAIN == 1. / 6. {
        saadc::Gain::GAIN1_6
    } else if ADC_GAIN == 1. / 5. {
        saadc::Gain::GAIN1_5
    } else if ADC_GAIN == 1. / 4. {
        saadc::Gain::GAIN1_4
    } else if ADC_GAIN == 1. / 3. {
        saadc::Gain::GAIN1_3
    } else if ADC_GAIN == 1. / 2. {
        saadc::Gain::GAIN1_2
    } else if ADC_GAIN == 1. {
        saadc::Gain::GAIN1
    } else if ADC_GAIN == 2. {
        saadc::Gain::GAIN2
    } else {
        panic!("Invalid ADC_GAIN")
    }
}
