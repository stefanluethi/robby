//! HTTP/WebSocket Server with contexts

use core::convert::TryInto;

use embedded_svc::wifi::{self, AuthMethod};
use esp_idf_hal::uart::*;

use esp_idf_svc::hal::peripherals::Peripherals;
use esp_idf_svc::sys::EspError;
use esp_idf_svc::{
    eventloop::EspSystemEventLoop,
    http::server::EspHttpServer,
    nvs::EspDefaultNvsPartition,
    wifi::{BlockingWifi, EspWifi},
    ws::FrameType,
};

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
const MAX_RECEIVE_LEN: usize = 128;

fn main() -> anyhow::Result<()> {
    esp_idf_svc::sys::link_patches();
    esp_idf_svc::log::EspLogger::initialize_default();

    let peripherals = Peripherals::take()?;

    let mut led_conn = esp_idf_hal::gpio::PinDriver::output(peripherals.pins.gpio15).unwrap();
    led_conn.set_high().ok();

    // configure uart
    let tx = peripherals.pins.gpio16;
    let rx = peripherals.pins.gpio17;
    let config = config::Config::new()
        .baudrate(esp_idf_hal::units::Hertz(1_000_000))
        .rx_fifo_size(2048);
    let mut uart = UartDriver::new(
        peripherals.uart1,
        tx,
        rx,
        Option::<esp_idf_hal::gpio::Gpio0>::None,
        Option::<esp_idf_hal::gpio::Gpio1>::None,
        &config,
    )?;

    // configure web server
    let mut server = create_server(peripherals.modem)?;
    led_conn.set_low().ok();

    let (uart_tx, uart_rx) = uart.split();
    let uart_tx_cell = Mutex::new(uart_tx);

    let websocket_sender = Arc::new(Mutex::new(None));
    {
        let socket_sender = websocket_sender.clone();
        let _ = server.ws_handler("/ws/robby", None, move |ws| {
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
            
            // NOTE: Due to the way the underlying C implementation works, ws.recv()
            // may only be called with an empty buffer exactly once to receive the
            // incoming buffer size, then must be called exactly once to receive the
            // actual payload.
            let (_frame_type, len) = match ws.recv(&mut []) {
                Ok(frame) => frame,
                Err(e) => return Err(e),
            };
        
            if len > MAX_RECEIVE_LEN {
                ws.send(FrameType::Text(false), "Request too big".as_bytes())?;
                ws.send(FrameType::Close, &[])?;
                return Ok::<(), EspError>(());
            }
        
            let mut buf = [0; MAX_RECEIVE_LEN]; // Small digit buffer can go on the stack
            ws.recv(buf.as_mut())?;
            info!("received {} bytes on ws -> forwarding to UART", len);
            uart_tx_cell.lock().unwrap().write(&buf[..len]).ok();
        
            Ok::<(), EspError>(())
        });
    }

    let mut led_div_counter = 0_u16;
    let mut buf = [0_u8; 2048];
    let mut received_accumulated: usize = 0;
    loop {
        // receive on UART side
        while let Some((event, _)) = uart_rx.event_queue().unwrap().recv_front(0) {
            // info!("uart event {:?}", event.payload());
            let evt = event.payload();
            match evt {
                UartEventPayload::Data { size, timeout } => {
                    received_accumulated += size;
                    if timeout {
                        info!("uart received {}B", received_accumulated);

                        let result = uart_rx.read(&mut buf[..received_accumulated], 0);
                        if let Ok(bytes_read) = result {
                            info!("forwarding {}B from UART to websocket", bytes_read);
                        }
                        if buf[0] != 0xa1 {
                            warn!("receiver out of sync, flushing");
                            uart_rx.clear().ok();
                            continue;
                        }

                        match (result, websocket_sender.try_lock()) {
                            (Ok(bytes_read), Ok(mut socket)) => {
                                if let Some(ws) = socket.as_mut() {
                                    if ws
                                        .send(FrameType::Binary(false), &buf[..bytes_read])
                                        .is_err()
                                    {
                                        warn!("Failed to send message");
                                    }
                                }
                            }
                            (_, Err(_)) => warn!("websocket lock error"),
                            (_, _) => (),
                        };


                        received_accumulated = 0;
                    }
                }
                _ => (),
            };
        };

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

    //let mut wifi = BlockingWifi::wrap(EspWifi::new(modem, sys_loop.clone(), Some(nvs))?, sys_loop)?;

    // let wifi_configuration = wifi::Configuration::AccessPoint(wifi::AccessPointConfiguration {
    //     ssid: WIFI_SSID.try_into().unwrap(),
    //     ssid_hidden: false,
    //     auth_method: AuthMethod::WPA2Personal,
    //     password: WIFI_PASSWORD.try_into().unwrap(),
    //     ..Default::default()
    // });

    let wifi = EspWifi::wrap_all(
        esp_idf_svc::wifi::WifiDriver::new(modem, sys_loop.clone(), Some(nvs)).unwrap(),
        esp_idf_svc::netif::EspNetif::new_with_conf(&esp_idf_svc::netif::NetifConfiguration {
            ip_configuration: Some(esp_idf_svc::ipv4::Configuration::Client(esp_idf_svc::ipv4::ClientConfiguration::DHCP(
                esp_idf_svc::ipv4::DHCPClientSettings {
                    hostname: Some("esp-tap".try_into().unwrap()),
                },
            ))),
            ..esp_idf_svc::netif::NetifConfiguration::wifi_default_client()
        })?,
        esp_idf_svc::netif::EspNetif::new(esp_idf_svc::netif::NetifStack::Ap)?,
    )?;

    let mut wifi = BlockingWifi::wrap(wifi, sys_loop)?;

    let wifi_configuration = wifi::Configuration::Client(wifi::ClientConfiguration {
        ssid: WIFI_SSID.try_into().unwrap(),
        auth_method: AuthMethod::WPA2Personal,
        password: WIFI_PASSWORD.try_into().unwrap(),
        ..Default::default()
    });


    wifi.set_configuration(&wifi_configuration)?;
    wifi.start()?;
    wifi.connect()?;
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
