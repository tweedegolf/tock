// Licensed under the Apache License, Version 2.0 or the MIT License.
// SPDX-License-Identifier: Apache-2.0 OR MIT
// Copyright Tock Contributors 2022.

//! Tock kernel for the Nordic Semiconductor nRF52840 development kit (DK).
//!
//! It is based on nRF52840 SoC (Cortex M4 core with a BLE transceiver) with
//! many exported I/O and peripherals.
//!
//! Pin Configuration
//! -------------------
//!
//! ### `GPIO`
//!
//! | #  | Pin   | Ix | Header | Arduino |
//! |----|-------|----|--------|---------|
//! | 0  | P1.01 | 33 | P3 1   | D0      |
//! | 1  | P1.02 | 34 | P3 2   | D1      |
//! | 2  | P1.03 | 35 | P3 3   | D2      |
//! | 3  | P1.04 | 36 | P3 4   | D3      |
//! | 4  | P1.05 | 37 | P3 5   | D4      |
//! | 5  | P1.06 | 38 | P3 6   | D5      |
//! | 6  | P1.07 | 39 | P3 7   | D6      |
//! | 7  | P1.08 | 40 | P3 8   | D7      |
//! | 8  | P1.10 | 42 | P4 1   | D8      |
//! | 9  | P1.11 | 43 | P4 2   | D9      |
//! | 10 | P1.12 | 44 | P4 3   | D10     |
//! | 11 | P1.13 | 45 | P4 4   | D11     |
//! | 12 | P1.14 | 46 | P4 5   | D12     |
//! | 13 | P1.15 | 47 | P4 6   | D13     |
//! | 14 | P0.26 | 26 | P4 9   | D14     |
//! | 15 | P0.27 | 27 | P4 10  | D15     |
//!
//! ### `GPIO` / Analog Inputs
//!
//! | #  | Pin        | Header | Arduino |
//! |----|------------|--------|---------|
//! | 16 | P0.03 AIN1 | P2 1   | A0      |
//! | 17 | P0.04 AIN2 | P2 2   | A1      |
//! | 18 | P0.28 AIN4 | P2 3   | A2      |
//! | 19 | P0.29 AIN5 | P2 4   | A3      |
//! | 20 | P0.30 AIN6 | P2 5   | A4      |
//! | 21 | P0.31 AIN7 | P2 6   | A5      |
//! | 22 | P0.02 AIN0 | P4 8   | AVDD    |
//!
//! ### Onboard Functions
//!
//! | Pin   | Header | Function |
//! |-------|--------|----------|
//! | P0.05 | P6 3   | UART RTS |
//! | P0.06 | P6 4   | UART TXD |
//! | P0.07 | P6 5   | UART CTS |
//! | P0.08 | P6 6   | UART RXT |
//! | P0.11 | P24 1  | Button 1 |
//! | P0.12 | P24 2  | Button 2 |
//! | P0.13 | P24 3  | LED 1    |
//! | P0.14 | P24 4  | LED 2    |
//! | P0.15 | P24 5  | LED 3    |
//! | P0.16 | P24 6  | LED 4    |
//! | P0.18 | P24 8  | Reset    |
//! | P0.19 | P24 9  | SPI CLK  |
//! | P0.20 | P24 10 | SPI MOSI |
//! | P0.21 | P24 11 | SPI MISO |
//! | P0.22 | P24 12 | SPI CS   |
//! | P0.24 | P24 14 | Button 3 |
//! | P0.25 | P24 15 | Button 4 |
//! | P0.26 | P24 16 | I2C SDA  |
//! | P0.27 | P24 17 | I2C SCL  |

#![no_std]
// Disable this attribute when documenting, as a workaround for
// https://github.com/rust-lang/rust/issues/62184.
#![cfg_attr(not(doc), no_main)]
#![deny(missing_docs)]

use capsules_core::virtualizers::virtual_alarm::{MuxAlarm, VirtualMuxAlarm};
use capsules_core::virtualizers::virtual_uart::UartDevice;
use capsules_extra::net::ieee802154::MacAddress;
use capsules_extra::net::ipv6::ip_utils::IPAddr;
use cortexm4::nvic::Nvic;
use kernel::component::Component;
use kernel::hil::led::LedLow;
use kernel::hil::time::{Alarm, AlarmClient, Counter, Ticks};
use kernel::hil::uart::Configure;
#[allow(unused_imports)]
use kernel::hil::usb::Client;
use kernel::platform::{KernelResources, SyscallDriverLookup};
use kernel::scheduler::round_robin::RoundRobinSched;
#[allow(unused_imports)]
use kernel::{capabilities, create_capability, debug, debug_gpio, debug_verbose, static_init};
use nrf52840::gpio::Pin;
use nrf52840::interrupt_service::Nrf52840DefaultPeripherals;
use nrf52840::rtc::Rtc;
use nrf52_components::{UartChannel, UartPins};

#[allow(dead_code)]
mod test;

// The nRF52840DK LEDs (see back of board)
const LED1_PIN: Pin = Pin::P0_13;
const LED2_PIN: Pin = Pin::P0_14;
const LED3_PIN: Pin = Pin::P0_15;
const LED4_PIN: Pin = Pin::P0_16;

// The nRF52840DK buttons (see back of board)
const BUTTON1_PIN: Pin = Pin::P0_11;
const BUTTON2_PIN: Pin = Pin::P0_12;
const BUTTON3_PIN: Pin = Pin::P0_24;
const BUTTON4_PIN: Pin = Pin::P0_25;
const BUTTON_RST_PIN: Pin = Pin::P0_18;

const UART_RTS: Option<Pin> = Some(Pin::P0_05);
const UART_TXD: Pin = Pin::P0_06;
const UART_CTS: Option<Pin> = Some(Pin::P0_07);
const UART_RXD: Pin = Pin::P0_08;

const SPI_MOSI: Pin = Pin::P0_20;
const SPI_MISO: Pin = Pin::P0_21;
const SPI_CLK: Pin = Pin::P0_19;
const SPI_CS: Pin = Pin::P0_22;

const SPI_MX25R6435F_CHIP_SELECT: Pin = Pin::P0_17;
const SPI_MX25R6435F_WRITE_PROTECT_PIN: Pin = Pin::P0_22;
const SPI_MX25R6435F_HOLD_PIN: Pin = Pin::P0_23;

/// I2C pins
const I2C_SDA_PIN: Pin = Pin::P0_26;
const I2C_SCL_PIN: Pin = Pin::P0_27;

// Constants related to the configuration of the 15.4 network stack
const PAN_ID: u16 = 0xABCD;
const DST_MAC_ADDR: capsules_extra::net::ieee802154::MacAddress =
    capsules_extra::net::ieee802154::MacAddress::Short(49138);
const DEFAULT_CTX_PREFIX_LEN: u8 = 8; //Length of context for 6LoWPAN compression
const DEFAULT_CTX_PREFIX: [u8; 16] = [0x0_u8; 16]; //Context for 6LoWPAN Compression

/// Debug Writer
pub mod io;

// Whether to use UART debugging or Segger RTT (USB) debugging.
// - Set to false to use UART.
// - Set to true to use Segger RTT over USB.
const USB_DEBUGGING: bool = false;

// State for loading and holding applications.
// How should the kernel respond when a process faults.
const FAULT_RESPONSE: kernel::process::PanicFaultPolicy = kernel::process::PanicFaultPolicy {};

// Number of concurrent processes this platform supports.
const NUM_PROCS: usize = 0;

static mut PROCESSES: [Option<&'static dyn kernel::process::Process>; NUM_PROCS] =
    [None; NUM_PROCS];

static mut CHIP: Option<&'static nrf52840::chip::NRF52<Nrf52840DefaultPeripherals>> = None;

/// Dummy buffer that causes the linker to reserve enough space for the stack.
#[no_mangle]
#[link_section = ".stack_buffer"]
pub static mut STACK_MEMORY: [u8; 0x2000] = [0; 0x2000];

//------------------------------------------------------------------------------
// SYSCALL DRIVER TYPE DEFINITIONS
//------------------------------------------------------------------------------

type AlarmDriver = components::alarm::AlarmDriverComponentType<nrf52840::rtc::Rtc<'static>>;
type RngDriver = components::rng::RngComponentType<nrf52840::trng::Trng<'static>>;

// TicKV
type Mx25r6435f = components::mx25r6435f::Mx25r6435fComponentType<
    nrf52840::spi::SPIM<'static>,
    nrf52840::gpio::GPIOPin<'static>,
    nrf52840::rtc::Rtc<'static>,
>;
const TICKV_PAGE_SIZE: usize =
    core::mem::size_of::<<Mx25r6435f as kernel::hil::flash::Flash>::Page>();
type Siphasher24 = components::siphash::Siphasher24ComponentType;
type TicKVDedicatedFlash =
    components::tickv::TicKVDedicatedFlashComponentType<Mx25r6435f, Siphasher24, TICKV_PAGE_SIZE>;
type TicKVKVStore = components::kv::TicKVKVStoreComponentType<
    TicKVDedicatedFlash,
    capsules_extra::tickv::TicKVKeyType,
>;
type KVStorePermissions = components::kv::KVStorePermissionsComponentType<TicKVKVStore>;
type VirtualKVPermissions = components::kv::VirtualKVPermissionsComponentType<KVStorePermissions>;
type KVDriver = components::kv::KVDriverComponentType<VirtualKVPermissions>;

// Temperature
type TemperatureDriver =
    components::temperature::TemperatureComponentType<nrf52840::temperature::Temp<'static>>;

type Ieee802154MacDevice = components::ieee802154::Ieee802154ComponentMacDeviceType<
    nrf52840::ieee802154_radio::Radio<'static>,
    nrf52840::aes::AesECB<'static>,
>;
type Ieee802154Driver = components::ieee802154::Ieee802154ComponentType<
    nrf52840::ieee802154_radio::Radio<'static>,
    nrf52840::aes::AesECB<'static>,
>;

/// Supported drivers by the platform
pub struct Platform {
    scheduler: &'static RoundRobinSched<'static>,
    systick: cortexm4::systick::SysTick,
}

impl SyscallDriverLookup for Platform {
    fn with_driver<F, R>(&self, driver_num: usize, f: F) -> R
    where
        F: FnOnce(Option<&dyn kernel::syscall::SyscallDriver>) -> R,
    {
        match driver_num {
            _ => f(None),
        }
    }
}

impl KernelResources<nrf52840::chip::NRF52<'static, Nrf52840DefaultPeripherals<'static>>>
    for Platform
{
    type SyscallDriverLookup = Self;
    type SyscallFilter = ();
    type ProcessFault = ();
    type CredentialsCheckingPolicy = ();
    type Scheduler = RoundRobinSched<'static>;
    type SchedulerTimer = cortexm4::systick::SysTick;
    type WatchDog = ();
    type ContextSwitchCallback = ();

    fn syscall_driver_lookup(&self) -> &Self::SyscallDriverLookup {
        self
    }
    fn syscall_filter(&self) -> &Self::SyscallFilter {
        &()
    }
    fn process_fault(&self) -> &Self::ProcessFault {
        &()
    }
    fn credentials_checking_policy(&self) -> &'static Self::CredentialsCheckingPolicy {
        &()
    }
    fn scheduler(&self) -> &Self::Scheduler {
        self.scheduler
    }
    fn scheduler_timer(&self) -> &Self::SchedulerTimer {
        &self.systick
    }
    fn watchdog(&self) -> &Self::WatchDog {
        &()
    }
    fn context_switch_callback(&self) -> &Self::ContextSwitchCallback {
        &()
    }
}

/// This is in a separate, inline(never) function so that its stack frame is
/// removed when this function returns. Otherwise, the stack space used for
/// these static_inits is wasted.
#[inline(never)]
pub unsafe fn start() -> (
    &'static kernel::Kernel,
    Platform,
    &'static nrf52840::chip::NRF52<'static, Nrf52840DefaultPeripherals<'static>>,
) {
    //--------------------------------------------------------------------------
    // INITIAL SETUP
    //--------------------------------------------------------------------------

    // Apply errata fixes and enable interrupts.
    nrf52840::init();

    use nrf52840::clock::{Clock, LowClockSource};
    let clock = Clock::new();
    clock.low_set_source(LowClockSource::RC);
    clock.low_start();
    while !clock.low_started() {}

    // Set up peripheral drivers. Called in separate function to reduce stack
    // usage.
    let ieee802154_ack_buf = static_init!(
        [u8; nrf52840::ieee802154_radio::ACK_BUF_SIZE],
        [0; nrf52840::ieee802154_radio::ACK_BUF_SIZE]
    );
    // Initialize chip peripheral drivers
    let nrf52840_peripherals = static_init!(
        Nrf52840DefaultPeripherals,
        Nrf52840DefaultPeripherals::new(ieee802154_ack_buf)
    );

    // Set up circular peripheral dependencies.
    nrf52840_peripherals.init();
    let base_peripherals = &nrf52840_peripherals.nrf52;

    // Configure kernel debug GPIOs as early as possible.
    kernel::debug::assign_gpios(
        Some(&nrf52840_peripherals.gpio_port[LED1_PIN]),
        Some(&nrf52840_peripherals.gpio_port[LED2_PIN]),
        Some(&nrf52840_peripherals.gpio_port[LED3_PIN]),
    );

    // Setup space to store the core kernel data structure.
    let board_kernel = static_init!(kernel::Kernel, kernel::Kernel::new(&PROCESSES));

    // Create (and save for panic debugging) a chip object to setup low-level
    // resources (e.g. MPU, systick).
    let chip = static_init!(
        nrf52840::chip::NRF52<Nrf52840DefaultPeripherals>,
        nrf52840::chip::NRF52::new(nrf52840_peripherals)
    );
    CHIP = Some(chip);

    //--------------------------------------------------------------------------
    // CAPABILITIES
    //--------------------------------------------------------------------------

    // Create capabilities that the board needs to call certain protected kernel
    // functions.
    let process_management_capability =
        create_capability!(capabilities::ProcessManagementCapability);
    let gpio_port = &nrf52840_peripherals.gpio_port;

    //--------------------------------------------------------------------------
    // TIMER
    //--------------------------------------------------------------------------

    let rtc = &base_peripherals.rtc;
    let _ = rtc.start().unwrap();

    Nvic::new(11).enable();

    base_peripherals.uarte0.initialize(
        nrf52840::pinmux::Pinmux::new(Pin::P0_06 as u32),
        nrf52840::pinmux::Pinmux::new(Pin::P0_08 as u32),
        Some(nrf52840::pinmux::Pinmux::new(Pin::P0_07 as u32)),
        Some(nrf52840::pinmux::Pinmux::new(Pin::P0_05 as u32)),
    );

    base_peripherals
        .uarte0
        .configure(kernel::hil::uart::Parameters {
            baud_rate: 115200,
            width: kernel::hil::uart::Width::Eight,
            stop_bits: kernel::hil::uart::StopBits::One,
            parity: kernel::hil::uart::Parity::None,
            hw_flow_control: false,
        });

    let hello_buffer = static_init!(
        [u8; 11],
        [b'H', b'e', b'l', b'l', b'o', b' ', b'W', b'o', b'r', b'l', b'd']
    );
    let hello_buffer2 = static_init!(
        [u8; 11],
        [b'W', b'e', b'l', b'l', b'o', b' ', b'H', b'o', b'r', b'l', b'd']
    );
    let hello = static_init!(
        hello_world::HelloWorld<'static, Rtc<'static>, nrf52840::uart::Uarte<'static>>,
        hello_world::HelloWorld::new(
            &base_peripherals.rtc,
            &base_peripherals.uarte0,
            hello_buffer,
        )
    );
    hello.start();

    //--------------------------------------------------------------------------
    // TESTS
    //--------------------------------------------------------------------------

    // let alarm_test_component =
    //     components::test::multi_alarm_test::MultiAlarmTestComponent::new(&mux_alarm).finalize(
    //         components::multi_alarm_test_component_buf!(nrf52840::rtc::Rtc),
    //     );

    //--------------------------------------------------------------------------
    // USB EXAMPLES
    //--------------------------------------------------------------------------
    // Uncomment to experiment with this.

    // // Create the strings we include in the USB descriptor.
    // let strings = static_init!(
    //     [&str; 3],
    //     [
    //         "Nordic Semiconductor", // Manufacturer
    //         "nRF52840dk - TockOS",  // Product
    //         "serial0001",           // Serial number
    //     ]
    // );

    // CTAP Example
    //
    // let (ctap, _ctap_driver) = components::ctap::CtapComponent::new(
    //     board_kernel,
    //     capsules_extra::ctap::DRIVER_NUM,
    //     &nrf52840_peripherals.usbd,
    //     0x1915, // Nordic Semiconductor
    //     0x503a, // lowRISC generic FS USB
    //     strings,
    // )
    // .finalize(components::ctap_component_static!(nrf52840::usbd::Usbd));

    // ctap.enable();
    // ctap.attach();

    // // Keyboard HID Example
    // type UsbHw = nrf52840::usbd::Usbd<'static>;
    // let usb_device = &nrf52840_peripherals.usbd;

    // let (keyboard_hid, keyboard_hid_driver) = components::keyboard_hid::KeyboardHidComponent::new(
    //     board_kernel,
    //     capsules_core::driver::NUM::KeyboardHid as usize,
    //     usb_device,
    //     0x1915, // Nordic Semiconductor
    //     0x503a,
    //     strings,
    // )
    // .finalize(components::keyboard_hid_component_static!(UsbHw));

    // keyboard_hid.enable();
    // keyboard_hid.attach();

    //--------------------------------------------------------------------------
    // PLATFORM SETUP, SCHEDULER, AND START KERNEL LOOP
    //--------------------------------------------------------------------------

    let scheduler = components::sched::round_robin::RoundRobinComponent::new(&PROCESSES)
        .finalize(components::round_robin_component_static!(NUM_PROCS));

    let platform = Platform {
        scheduler,
        systick: cortexm4::systick::SysTick::new_with_calibration(64000000),
    };

    // test::aes_test::run_aes128_ctr(&base_peripherals.ecb);
    // test::aes_test::run_aes128_cbc(&base_peripherals.ecb);
    // test::aes_test::run_aes128_ecb(&base_peripherals.ecb);

    debug!("Initialization complete. Entering main loop\r");
    debug!("{}", &nrf52840::ficr::FICR_INSTANCE);

    // alarm_test_component.run();

    // These symbols are defined in the linker script.
    extern "C" {
        /// Beginning of the ROM region containing app images.
        static _sapps: u8;
        /// End of the ROM region containing app images.
        static _eapps: u8;
        /// Beginning of the RAM region for app memory.
        static mut _sappmem: u8;
        /// End of the RAM region for app memory.
        static _eappmem: u8;
    }

    kernel::process::load_processes(
        board_kernel,
        chip,
        core::slice::from_raw_parts(
            core::ptr::addr_of!(_sapps),
            core::ptr::addr_of!(_eapps) as usize - core::ptr::addr_of!(_sapps) as usize,
        ),
        core::slice::from_raw_parts_mut(
            core::ptr::addr_of_mut!(_sappmem),
            core::ptr::addr_of!(_eappmem) as usize - core::ptr::addr_of!(_sappmem) as usize,
        ),
        &mut PROCESSES,
        &FAULT_RESPONSE,
        &process_management_capability,
    )
    .unwrap_or_else(|err| {
        debug!("Error loading processes!");
        debug!("{:?}", err);
    });

    // hello.transmit_now(hello_buffer2);

    // for _ in 0..10000000 {
    //     core::hint::black_box(core::arch::asm!("NOP"));
    // }

    // use kernel::hil::time::Time;
    // let now = rtc.now().into_u32();
    // let buffer = static_init!([u8; 4], [0xAA; 4]);
    // buffer.copy_from_slice(&now.to_be_bytes());
    // hello.transmit_now(buffer);

    (board_kernel, platform, chip)
}

/// Main function called after RAM initialized.
#[no_mangle]
pub unsafe fn main() {
    let main_loop_capability = create_capability!(capabilities::MainLoopCapability);

    let (board_kernel, platform, chip) = start();
    board_kernel.kernel_loop(
        &platform,
        chip,
        None::<&kernel::ipc::IPC<{ NUM_PROCS as u8 }>>,
        &main_loop_capability,
    );
}

mod hello_world {
    use core::cell::Cell;

    use kernel::hil::{
        time::{Alarm, AlarmClient, Frequency, Ticks},
        uart::{TransmitClient, UartData},
    };

    enum State {
        Transmitting,
        Waiting(&'static mut [u8]),
    }

    pub struct HelloWorld<'a, A: Alarm<'a>, U: UartData<'a>> {
        alarm: &'a A,
        uart: &'a U,
        state: Cell<State>,
    }

    impl<'a, A: Alarm<'a>, U: UartData<'a>> HelloWorld<'a, A, U> {
        pub fn new(alarm: &'a A, uart: &'a U, buffer: &'static mut [u8]) -> Self {
            HelloWorld {
                alarm,
                uart,
                state: Cell::new(State::Waiting(buffer)),
            }
        }

        pub fn transmit_now(&self, buffer: &'static mut [u8]) {
            self.uart.transmit_buffer(buffer, buffer.len());
        }

        pub fn start(&'a self) {
            use kernel::hil::time::ConvertTicks;

            self.alarm.set_alarm_client(self);
            self.uart.set_transmit_client(self);

            let now = self.alarm.now();
            self.alarm.set_alarm(now, self.alarm.ticks_from_ms(1000));
        }
    }

    impl<'a, A: Alarm<'a>, U: UartData<'a>> AlarmClient for HelloWorld<'a, A, U> {
        fn alarm(&self) {
            match self.state.replace(State::Transmitting) {
                State::Transmitting => {
                    // Shouldn't happen, but just ignore
                }
                State::Waiting(buffer) => {
                    self.uart.transmit_buffer(buffer, buffer.len());

                    let now = self.alarm.now();
                    self.alarm.set_alarm(
                        now,
                        <A::Ticks>::from_or_max(<A::Frequency>::frequency() as u64),
                    );
                }
            }
        }
    }

    impl<'a, A: Alarm<'a>, U: UartData<'a>> TransmitClient for HelloWorld<'a, A, U> {
        fn transmitted_buffer(
            &self,
            tx_buffer: &'static mut [u8],
            tx_len: usize,
            rval: Result<(), kernel::ErrorCode>,
        ) {
        }
    }
}
