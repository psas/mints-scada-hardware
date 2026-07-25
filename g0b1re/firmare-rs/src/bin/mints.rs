/*
* Main entrypoint
* Sets up the clock, spawns tasks, and handles general business logic
*/

#![no_std]
#![no_main]

use core::future;

use defmt::*;
use embassy_executor::Spawner;
use mints::can::handle_can;
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

    // TODO: Set node_id by reading gpios
    info!("Spawning tasks...");
    spawner.spawn(handle_can(p.FDCAN1, p.PA11, p.PA12, 0x01).unwrap());

    info!("Entering main loop");
    future::pending::<()>().await;
}
