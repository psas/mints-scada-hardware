#![no_std]
#![no_main]

use defmt::*;
use embassy_executor::Spawner;
use embassy_stm32::{bind_interrupts, can, peripherals::FDCAN1, time::Hertz};
use embassy_time::Timer;
use {defmt_rtt as _, panic_probe as _};

bind_interrupts!(struct Irqs {
    TIM16_FDCAN_IT0 => can::IT0InterruptHandler<FDCAN1>;
    TIM17_FDCAN_IT1 => can::IT1InterruptHandler<FDCAN1>;
});

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let mut config = embassy_stm32::Config::default();
    info!("creating peripherals config");
    {
        // TODO: Check these values. They work but are likely not ideal
        use embassy_stm32::rcc::*;
        info!("hsi");
        config.rcc.hsi48 = Some(Hsi48Config::default());
        info!("pll");
        config.rcc.pll = Some(Pll {
            source: PllSource::HSI,
            prediv: PllPreDiv::DIV1,
            mul: PllMul::MUL8, // Mult factor for VCO
            divp: None, // ADC
            divq: Some(PllQDiv::DIV2), // USB, I2S23, SAI1, FDCAN, QSPI
            divr: Some(PllRDiv::DIV2), // SYSCLK
        });
        info!("fdcansel");
        config.rcc.mux.fdcansel = mux::Fdcansel::PLL1_Q;
        info!("sys");
        config.rcc.sys = Sysclk::PLL1_R;
    }

    info!("configuring peripherals");
    let peripherals = embassy_stm32::init(config);

    info!("Configuratoring CAN");
    let mut can =
        can::CanConfigurator::new(peripherals.FDCAN1, peripherals.PA11, peripherals.PA12, Irqs);

    info!("setting filters");
    can.properties().set_extended_filter(
        can::filter::ExtendedFilterSlot::_0,
        can::filter::ExtendedFilter::accept_all_into_fifo1(),
    );

    // FIXME: No FDCAN - unsupported by vulCAN
    can.set_bitrate(1_000_000);
    can.set_fd_data_bitrate(1_000_000, false);
    info!("Configured CAN");

    // TODO: Test normal operating mode, not just loopback
    let mut can = can.start(can::OperatingMode::InternalLoopbackMode);

    let mut i = 0;
    let mut last_read_ts = embassy_time::Instant::now();

    loop {
        let frame = can::frame::FdFrame::new_extended(0x123456F, &[i; 16]).unwrap();
        info!("Writing frame");

        _ = can.write_fd(&frame).await;

        match can.read_fd().await {
            Ok(envelope) => {
                let (ts, rx_frame) = (envelope.ts, envelope.frame);
                let delta = (ts - last_read_ts).as_millis();
                last_read_ts = ts;
                info!(
                    "Rx: {} {:02x} --- using FD API {}ms",
                    rx_frame.header().len(),
                    rx_frame.data()[0..rx_frame.header().len() as usize],
                    delta,
                )
            }
            Err(_err) => error!("Error in frame"),
        }

        Timer::after_millis(250).await;

        i += 1;
    }
}
