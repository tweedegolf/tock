// Licensed under the Apache License, Version 2.0 or the MIT License.
// SPDX-License-Identifier: Apache-2.0 OR MIT
// Copyright Tock Contributors 2022.

use kernel::hil::time::Alarm;
use nrf52::chip::Nrf52DefaultPeripherals;

/// This struct, when initialized, instantiates all peripheral drivers for the nrf52840.
/// If a board wishes to use only a subset of these peripherals, this
/// should not be used or imported, and a modified version should be
/// constructed manually in main.rs.
//create all base nrf52 peripherals
pub struct Nrf52840DefaultPeripherals<'a> {
    pub nrf52: Nrf52DefaultPeripherals<'a>,
    pub ieee802154_radio: crate::ieee802154_radio::Radio<'a>,
    pub usbd: crate::usbd::Usbd<'static>,
    pub gpio_port: &'static crate::gpio::Port<'static, { crate::gpio::NUM_PINS }>,
}

impl<'a> Nrf52840DefaultPeripherals<'a> {
    pub unsafe fn new() -> Self {
        Self {
            nrf52: Nrf52DefaultPeripherals::new(),
            ieee802154_radio: crate::ieee802154_radio::Radio::new(),
            usbd: crate::usbd::Usbd::new(),
            gpio_port: &crate::gpio::NRF52840_GPIO_CREATE,
        }
    }
    // Necessary for setting up circular dependencies
    pub fn init(&'static self) {
        self.ieee802154_radio.set_timer_ref(&self.nrf52.timer0);
        self.nrf52.timer0.set_alarm_client(&self.ieee802154_radio);
        self.nrf52.init();
    }
}
impl<'a> kernel::platform::chip::InterruptService for Nrf52840DefaultPeripherals<'a> {
    unsafe fn service_interrupt(&self, interrupt: u32) -> bool {
        self.nrf52.service_interrupt(interrupt)
    }
}
