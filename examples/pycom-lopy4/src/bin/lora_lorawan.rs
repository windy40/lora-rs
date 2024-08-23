//! This example runs on the Pycom LoPy4 board containing a Semtech Sx1276 radio.
//! It demonstrates LoRaWAN join functionality.

#![no_std]
#![no_main]

use esp_backtrace as _;
use esp_println as _;
use defmt::{info, debug};

use esp_hal::{
    clock::ClockControl,
    dma::*,
    dma_buffers,
    gpio::{Input, Io, Level, Output, Pull, NO_PIN},
    peripherals::Peripherals,
    prelude::*,
    spi::{
        master::Spi,
        SpiMode,
    },
    system::SystemControl,
    timer::{timg::TimerGroup, ErasedTimer, OneShotTimer},
    rng::Rng,
};

use embassy_executor::Spawner;
use embassy_time::{Delay,Timer};
use embedded_hal_bus::spi::ExclusiveDevice;

use lora_phy::iv::GenericSx127xInterfaceVariant;
use lora_phy::lorawan_radio::LorawanRadio;
use lora_phy::sx127x::{self, Sx1276, Sx127x};
use lora_phy::LoRa;
use lorawan_device::async_device::{region, Device, EmbassyTimer, JoinMode, JoinResponse};
use lorawan_device::default_crypto::DefaultFactory as Crypto;
use lorawan_device::{AppEui, AppKey, DevEui};


use static_cell::StaticCell;
macro_rules! mk_static {
    ($t:ty,$val:expr) => {{
        static STATIC_CELL: StaticCell<$t> = StaticCell::new();
        STATIC_CELL.uninit().write(($val))
    }};
}

//const DMA_BUF: usize = 256;
//type SafeSpiDma = FlashSafeDma<
//    SpiDma<'static, esp_hal::peripherals::SPI2, Spi2DmaChannel, FullDuplexMode, Async>,
//    DMA_BUF,
//>;
//type SpiDmaT = SpiDma<'static, esp_hal::peripherals::SPI2, Spi2DmaChannel, FullDuplexMode, Async>;

// warning: set these appropriately for the region
const LORAWAN_REGION: region::Region = region::Region::EU868;
const MAX_TX_POWER: u8 = 14;



//#[embassy_executor::main]
#[esp_hal_embassy::main]
async fn main(_spawner: Spawner) {
    debug!("Init:");

    let peripherals = Peripherals::take();
    let system = SystemControl::new(peripherals.SYSTEM);
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    let timg0 = TimerGroup::new(peripherals.TIMG0, &clocks);
    let timer0: ErasedTimer = timg0.timer0.into();
    let timers = [OneShotTimer::new(timer0)];
    let timers = mk_static!([OneShotTimer<ErasedTimer>; 1], timers);

    //log::debug!("Init clocks!");
    debug!("Init clocks!");

    esp_hal_embassy::init(&clocks, timers);

    let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);
    // Init SPI and LoRa
    let dma = Dma::new(peripherals.DMA);
    let dma_channel = dma.spi2channel;

    let (tx_buffer, tx_descriptors, rx_buffer, rx_descriptors) = dma_buffers!(32000);
    let dma_tx_buf = DmaTxBuf::new(tx_descriptors, tx_buffer).unwrap();
    let dma_rx_buf = DmaRxBuf::new(rx_descriptors, rx_buffer).unwrap();

    let sclk = io.pins.gpio5;
    let miso = io.pins.gpio19;
    let mosi = io.pins.gpio27;
    let cs = Output::new(io.pins.gpio18, Level::Low); // gpio18 Level::Low

    let spi = Spi::new(peripherals.SPI2, 100.kHz(), SpiMode::Mode0, &clocks)
        // use NO_PIN for CS as we'll going to be using the SpiDevice trait
        // via ExclusiveSpiDevice as we don't (yet) want to pull in embassy-sync
        .with_pins(Some(sclk), Some(mosi), Some(miso), NO_PIN)
        .with_dma(dma_channel.configure_for_async(false, DmaPriority::Priority0))        
        .with_buffers(dma_tx_buf, dma_rx_buf);

//    let spi: SafeSpiDma = FlashSafeDma::new(spi);

    let spi_dev = ExclusiveDevice::new(spi, cs, Delay).unwrap();

    let lora_reset = Output::new(io.pins.gpio17, Level::Low); //lilygo gpio22
    let lora_dio0 = Input::new(io.pins.gpio23, Pull::None); //gpio26
    let iv = GenericSx127xInterfaceVariant::new(lora_reset, lora_dio0, None, None).unwrap();

    let config = sx127x::Config {
        chip: Sx1276,
        tcxo_used: false,
        rx_boost: false,
        tx_boost: false,
    };

    info!("Initializing LoRa");
    let lora = {
        match LoRa::new(Sx127x::new(spi_dev, iv, config), true, Delay).await {
            Ok(r) => r,
            Err(r) => panic!("Fail: {:?}", r),
        }
    };

    let radio: LorawanRadio<_, _, MAX_TX_POWER> = lora.into();
    let region: region::Configuration = region::Configuration::new(LORAWAN_REGION);
    // following code does not work, change constant in lorawan-device/src/region/constants.rs
    //let mut region = region::Configuration::new(LORAWAN_REGION);
    //region.set_join_accept_delay1(5000);
    //region.set_join_accept_delay2(8000);

    let mut device: Device<_, Crypto, _, _> =
        Device::new(region, radio, EmbassyTimer::new(), Rng::new(peripherals.RNG)); //embassy_rp::clocks::RoscRng

    info!("Joining LoRaWAN network");

    // TODO: Adjust the EUI and Keys according to your network credentials
    loop{
    if let Ok(JoinResponse::JoinSuccess) = device
        .join(&JoinMode::OTAA {
            deveui: DevEui::from([0x9d, 0xd0, 0x0e, 0x9f, 0x49, 0xd5, 0xb3, 0x70]),
            appeui: AppEui::from([0, 0, 0, 0, 0, 0, 0, 0]),
            appkey: AppKey::from([0x2c,0xc1,0x72,0x96,0x9d,0x5c,0xc2,0x63,0x82,0xe0,0xad,0x05,0x45,0x68,0xce,0x3e]),
   //         appkey: AppKey::from([0x3e,0xce,0x68,0x45,0x05,0xad,0xe0,0x82,0x63,0xc2,0x5c,0x9d,0x96,0x72,0xc1,0x2c]),
        })
        .await
 //       .unwrap()
        {
            info!("LoRaWAN network joined");
            break;
        }
        Timer::after_secs(5).await;
        info!("LoRaWAN network join failed. Retrying ...");
    }
    
}