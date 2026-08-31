/*
* Main entrypoint
* Sets up the clock, spawns tasks, and handles general business logic
*/

#![no_std]
#![no_main]

use core::future;

use defmt::*;
use embassy_executor::Spawner;
use embassy_stm32::{
    Peri,
    gpio::{Level, Output, Speed},
    peripherals::PA5,
};
use embassy_time::{Duration, Ticker};
use mints::{
    can::{get_can_reader_and_writer, handle_can_rx, handle_can_tx},
    cmd::handle_cmds,
};
use {defmt_rtt as _, panic_probe as _};

// TODO: Error handling. Grep for 'panic', 'unwrap' and fix
#[embassy_executor::main]
async fn main(spawner: Spawner) {
    info!(
        "PSAS Minimal Test Stand Firmware v{}",
        env!("CARGO_PKG_VERSION")
    );
    info!("Configuring clock...");
    let mut config = embassy_stm32::Config::default();
    {
        use embassy_stm32::rcc::*;
        config.rcc.hsi48 = Some(Hsi48Config::default());
        config.rcc.pll = Some(Pll {
            source: PllSource::HSI,
            prediv: PllPreDiv::DIV1,
            mul: PllMul::MUL8,         // Mult factor for VCO
            divp: None,                // ADC
            divq: Some(PllQDiv::DIV2), // USB, I2S23, SAI1, FDCAN, QSPI
            divr: Some(PllRDiv::DIV2), // SYSCLK
        });
        config.rcc.mux.fdcansel = mux::Fdcansel::PLL1_Q;
        config.rcc.sys = Sysclk::PLL1_R;
    }

    let p = embassy_stm32::init(config);

    let (can_reader, can_writer) = get_can_reader_and_writer(p.FDCAN1, p.PA11, p.PA12);

    // TODO: Set node_id by reading gpios
    let node_id: u8 = 0x01;
    info!("Spawning tasks");
    spawner.spawn(unwrap!(blink_led(p.PA5)));
    spawner.spawn(unwrap!(handle_can_rx(can_reader, node_id)));
    spawner.spawn(unwrap!(handle_can_tx(can_writer, node_id)));
    spawner.spawn(unwrap!(handle_cmds()));

    // TODO: Emergency shutdown routine, main loop Watch? Signal?

    // Keep main alive indefinitely
    future::pending::<()>().await;
}

#[embassy_executor::task]
async fn blink_led(led: Peri<'static, PA5>) {
    let mut led = Output::new(led, Level::High, Speed::Low);
    let mut ticker = Ticker::every(Duration::from_millis(300));

    loop {
        led.toggle();
        ticker.next().await;
    }
}
