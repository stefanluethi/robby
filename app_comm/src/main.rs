//! HTTP/WebSocket Server with contexts

use core::convert::TryInto;

use embedded_svc::{
    wifi::{self, AuthMethod},
};

use esp_idf_svc::{
    eventloop::EspSystemEventLoop, http::server::EspHttpServer, nvs::EspDefaultNvsPartition, wifi::{BlockingWifi, EspWifi}
};
use esp_idf_svc::hal::peripherals::Peripherals;
use esp_idf_svc::sys::EspError;

use log::*;

use std::{
    str,
    sync::{Arc, Mutex},
    thread::sleep,
    time::Duration,
};

const WIFI_SSID: &str = "robby";
const WIFI_PASSWORD: &str = "robby1234";

const STACK_SIZE: usize = 10240;

fn main() -> anyhow::Result<()> {
    esp_idf_svc::sys::link_patches();
    esp_idf_svc::log::EspLogger::initialize_default();

    let peripherals = Peripherals::take()?;

    let mut led_conn = esp_idf_hal::gpio::PinDriver::output(peripherals.pins.gpio15).unwrap();
    led_conn.set_high().ok();

    // configure web server
    let mut server = create_server(peripherals.modem)?;
    led_conn.set_low().ok();

    let websocket_sender = Arc::new(Mutex::new(None));
    {
        let socket_sender = websocket_sender.clone();
        server.ws_handler("/ws/robby", None,  move |ws| {
            if ws.is_new() {
                if let Ok(mut socket) = socket_sender.try_lock() {
                    *socket = Some(ws.create_detached_sender().unwrap());
                }

                info!("New WebSocket session");
                return Ok(());
            } else if ws.is_closed() {
                if let Ok(mut socket) = socket_sender.try_lock() {
                    *socket = None;
                }

                info!("Closed WebSocket session");
                return Ok(());
            }
            Ok::<(), EspError>(())
        })?;
    }

    let mut led_div_counter = 0_u16;
    loop {

        if led_div_counter >= 10 {
            led_conn.toggle().ok();
            led_div_counter = 0;
        } else {
            led_div_counter += 1;
        }
        sleep(Duration::from_millis(10));
    }
}

fn create_server(modem: esp_idf_hal::modem::Modem) -> anyhow::Result<EspHttpServer<'static>> {
    let sys_loop = EspSystemEventLoop::take()?;
    let nvs = EspDefaultNvsPartition::take()?;

    let mut wifi = BlockingWifi::wrap(
        EspWifi::new(modem, sys_loop.clone(), Some(nvs))?,
        sys_loop,
    )?;

    let wifi_configuration = wifi::Configuration::AccessPoint(
        wifi::AccessPointConfiguration {
            ssid: WIFI_SSID.try_into().unwrap(),
            ssid_hidden: false,
            auth_method: AuthMethod::WPA2Personal,
            password: WIFI_PASSWORD.try_into().unwrap(),
            ..Default::default()
    });

    wifi.set_configuration(&wifi_configuration)?;
    wifi.start()?;
    wifi.wait_netif_up()?;

    info!("Created Wi-Fi with WIFI_SSID `{WIFI_SSID}` and WIFI_PASS `{WIFI_PASSWORD}`");

    let server_configuration = esp_idf_svc::http::server::Configuration {
        stack_size: STACK_SIZE,
        ..Default::default()
    };

    // Keep wifi running beyond when this function returns (forever)
    // Do not call this if you ever want to stop or access it later.
    // Otherwise it should be returned from this function and kept somewhere
    // so it does not go out of scope.
    // https://doc.rust-lang.org/stable/core/mem/fn.forget.html
    core::mem::forget(wifi);

    Ok(EspHttpServer::new(&server_configuration)?)
}

