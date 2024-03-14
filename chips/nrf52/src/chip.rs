// Licensed under the Apache License, Version 2.0 or the MIT License.
// SPDX-License-Identifier: Apache-2.0 OR MIT
// Copyright Tock Contributors 2022.

use core::fmt::Write;
use cortexm4::{nvic, CortexM4, CortexMVariant};
use kernel::platform::chip::InterruptService;

pub struct NRF52<'a, I: InterruptService + 'a> {
    mpu: cortexm4::mpu::MPU,
    userspace_kernel_boundary: cortexm4::syscall::SysCall,
    interrupt_service: &'a I,
}

impl<'a, I: InterruptService + 'a> NRF52<'a, I> {
    pub unsafe fn new(interrupt_service: &'a I) -> Self {
        Self {
            mpu: cortexm4::mpu::MPU::new(),
            userspace_kernel_boundary: cortexm4::syscall::SysCall::new(),
            interrupt_service,
        }
    }
}

/// This struct, when initialized, instantiates all peripheral drivers for the nrf52.
///
/// If a board wishes to use only a subset of these peripherals, this
/// should not be used or imported, and a modified version should be
/// constructed manually in main.rs.
pub struct Nrf52DefaultPeripherals<'a> {
    pub uarte0: crate::uart::Uarte<'a>,
    pub rtc: crate::rtc::Rtc<'a>,
    pub timer0: crate::timer::TimerAlarm<'a>,
    pub timer1: crate::timer::TimerAlarm<'a>,
    pub timer2: crate::timer::Timer,
}

impl<'a> Nrf52DefaultPeripherals<'a> {
    pub fn new() -> Self {
        Self {
            uarte0: crate::uart::Uarte::new(crate::uart::UARTE0_BASE),
            rtc: crate::rtc::Rtc::new(),
            timer0: crate::timer::TimerAlarm::new(0),
            timer1: crate::timer::TimerAlarm::new(1),
            timer2: crate::timer::Timer::new(2),
        }
    }
    // Necessary for setting up circular dependencies
    pub fn init(&'static self) {
        // kernel::deferred_call::DeferredCallClient::register(&self.nvmc);
    }
}
impl<'a> kernel::platform::chip::InterruptService for Nrf52DefaultPeripherals<'a> {
    unsafe fn service_interrupt(&self, interrupt: u32) -> bool {
        match interrupt {
            crate::peripheral_interrupts::UART0 => self.uarte0.handle_interrupt(),
            crate::peripheral_interrupts::RTC1 => self.rtc.handle_interrupt(),
            crate::peripheral_interrupts::TIMER0 => self.timer0.handle_interrupt(),
            crate::peripheral_interrupts::TIMER1 => self.timer1.handle_interrupt(),
            crate::peripheral_interrupts::TIMER2 => self.timer2.handle_interrupt(),
            _ => return false,
        }
        true
    }
}

impl<'a, I: InterruptService + 'a> kernel::platform::chip::Chip for NRF52<'a, I> {
    type MPU = cortexm4::mpu::MPU;
    type UserspaceKernelBoundary = cortexm4::syscall::SysCall;

    fn mpu(&self) -> &Self::MPU {
        &self.mpu
    }

    fn userspace_kernel_boundary(&self) -> &Self::UserspaceKernelBoundary {
        &self.userspace_kernel_boundary
    }

    fn service_pending_interrupts(&self) {
        unsafe {
            loop {
                if let Some(interrupt) = nvic::next_pending() {
                    if !self.interrupt_service.service_interrupt(interrupt) {
                        panic!("unhandled interrupt {}", interrupt);
                    }
                    let n = nvic::Nvic::new(interrupt);
                    n.clear_pending();
                    n.enable();
                } else {
                    break;
                }
            }
        }
    }

    fn has_pending_interrupts(&self) -> bool {
        unsafe { nvic::has_pending() }
    }

    fn sleep(&self) {
        unsafe {
            cortexm4::support::wfi();
        }
    }

    unsafe fn atomic<F, R>(&self, f: F) -> R
    where
        F: FnOnce() -> R,
    {
        cortexm4::support::atomic(f)
    }

    unsafe fn print_state(&self, write: &mut dyn Write) {
        CortexM4::print_cortexm_state(write);
    }
}
