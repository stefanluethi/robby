#![no_std]
#![no_main]

extern crate alloc;
use alloc::string::String;

use embassy_executor::Spawner;
use embassy_net::{
    tcp::TcpSocket, Config, Stack, StackResources,
};
use embassy_time::{Duration, Timer};
use embassy_sync::{
    blocking_mutex::raw::CriticalSectionRawMutex,
    channel::Channel,
};
use embedded_io_async::{Write};

use esp_backtrace as _;
use log::{debug, info, warn};

use esp_hal::{
    clock::CpuClock,
    gpio::{Level, Output},
    interrupt::{software::SoftwareInterruptControl, Priority},
    timer::timg::TimerGroup,
    uart::{UartRx, UartTx, Config as UartConfig, Uart},
    Async,
};

use esp_rtos::embassy::InterruptExecutor;
use static_cell::StaticCell;

esp_bootloader_esp_idf::esp_app_desc!();

const WIFI_SSID: &str = env!("WIFI_SSID");
const WIFI_PASSWORD: &str = env!("WIFI_PASSWORD");

const MAX_RECEIVE_LEN: usize = 128;
/// Largest frame from the target: a sonde batch of app_core (its trace storage is 1024 bytes)
const MAX_FRAME_LEN: usize = 1024;

fn init_heap() {
    const HEAP_SIZE: usize = 64 * 1024;
    static mut HEAP: [core::mem::MaybeUninit<u8>; HEAP_SIZE] = [core::mem::MaybeUninit::uninit(); HEAP_SIZE];

    unsafe {
        esp_alloc::HEAP.add_region(esp_alloc::HeapRegion::new(
            (&raw mut HEAP) as *mut u8,
            HEAP_SIZE,
            esp_alloc::MemoryCapability::Internal.into(),
        ));
    }
}

// The frames are static, not on the heap: the queue fills up with sonde batches of about 1 kB
// while no client is connected, which exhausted the heap the radio needs too.
type UartFrame = heapless::Vec<u8, MAX_FRAME_LEN>;
type WsFrame = heapless::Vec<u8, MAX_RECEIVE_LEN>;

// Channel to send messages from UART to WS, ~0.5 s of traces to bridge WiFi hiccups
static UART_TO_WS: Channel<CriticalSectionRawMutex, UartFrame, 32> = Channel::new();
// Channel to send messages from WS to UART
static WS_TO_UART: Channel<CriticalSectionRawMutex, WsFrame, 16> = Channel::new();

#[esp_rtos::main]
async fn main(spawner: Spawner) -> ! {
    init_heap();
    esp_println::logger::init_logger_from_env();

    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    let sw_int = SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);
    let timg0 = TimerGroup::new(peripherals.TIMG0);
    
    esp_rtos::start(timg0.timer0, sw_int.software_interrupt0);

    let led_conn = Output::new(peripherals.GPIO15, Level::High, esp_hal::gpio::OutputConfig::default());
    let mut rf_switch_power = Output::new(peripherals.GPIO3, Level::Low, esp_hal::gpio::OutputConfig::default());
    let mut rf_switch_external = Output::new(peripherals.GPIO14, Level::High, esp_hal::gpio::OutputConfig::default());

    let _ = rf_switch_power.set_low();
    let _ = rf_switch_external.set_high();

    let (controller, interfaces) = esp_radio::wifi::new(
        peripherals.WIFI, 
        Default::default(),
    ).unwrap();

    // Started after the radio init, which masks interrupts long enough to overflow the UART FIFO.
    // UART runs in a higher priority interrupt executor, so the network stack can't delay it
    // long enough to overflow the HW FIFO
    static UART_EXECUTOR: StaticCell<InterruptExecutor<1>> = StaticCell::new();
    let uart_executor = UART_EXECUTOR.init(InterruptExecutor::new(sw_int.software_interrupt1));
    let uart_spawner = uart_executor.start(Priority::Priority2);
    uart_spawner.spawn(
        uart_task(peripherals.UART1, peripherals.GPIO17, peripherals.GPIO16, led_conn).unwrap(),
    );

    let wifi = interfaces.station;

    let net_config = Config::dhcpv4(Default::default());

    let seed = 1234; 
    static STACK: static_cell::StaticCell<Stack<'static>> = static_cell::StaticCell::new();
    static RESOURCES: static_cell::StaticCell<StackResources<3>> = static_cell::StaticCell::new();

    let (stack, runner) = embassy_net::new(
        wifi,
        net_config,
        RESOURCES.init(StackResources::<3>::new()),
        seed,
    );
    let stack = STACK.init(stack);

    spawner.spawn(connection(controller).unwrap());
    spawner.spawn(net_task(runner).unwrap());

    loop {
        if stack.is_link_up() {
            break;
        }
        Timer::after(Duration::from_millis(500)).await;
    }

    info!("Waiting to get IP address...");
    loop {
        if let Some(config) = stack.config_v4() {
            info!("Got IP: {}", config.address);
            break;
        }
        Timer::after(Duration::from_millis(500)).await;
    }

    spawner.spawn(ws_server_task(stack).unwrap());
    loop {
        Timer::after(Duration::from_secs(10)).await;
    }
}

#[embassy_executor::task]
async fn connection(mut controller: esp_radio::wifi::WifiController<'static>) {
    let client_config = esp_radio::wifi::Config::Station(
        esp_radio::wifi::sta::StationConfig::default()
            .with_ssid(WIFI_SSID)
            .with_password(String::from(WIFI_PASSWORD))
    );
    controller.set_config(&client_config).unwrap();

    loop {
        match controller.connect_async().await {
            Ok(_) => {
                info!("Wifi connected!");
                // only reconnect once the connection is actually lost
                let _ = controller.wait_for_disconnect_async().await;
                warn!("Wifi disconnected");
            }
            Err(e) => {
                warn!("Failed to connect to wifi: {:?}", e);
            }
        }
        Timer::after(Duration::from_millis(5000)).await;
    }
}

#[embassy_executor::task]
async fn net_task(mut runner: embassy_net::Runner<'static, esp_radio::wifi::Interface<'static>>) {
    runner.run().await
}

#[embassy_executor::task]
async fn ws_server_task(stack: &'static Stack<'static>) {
    let mut rx_buffer = [0; 4096];
    // the traces stream at ~65 kB/s, a larger window keeps up with the round trips of WiFi
    let mut tx_buffer = [0; 16384];

    loop {
        let mut socket = TcpSocket::new(*stack, &mut rx_buffer, &mut tx_buffer);
        socket.set_timeout(Some(embassy_time::Duration::from_secs(10)));

        info!("Listening on TCP:80...");
        if let Err(e) = socket.accept(80).await {
            warn!("Accept error: {:?}", e);
            continue;
        }
        info!("Connection from {:?}", socket.remote_endpoint());

        let mut buf = [0; 1024];
        let mut pos = 0;
        
        while pos < buf.len() {
            match socket.read(&mut buf[pos..pos+1]).await {
                Ok(0) => break,
                Ok(_) => {
                    pos += 1;
                    if pos >= 4 && &buf[pos-4..pos] == b"\r\n\r\n" {
                        break;
                    }
                }
                Err(_) => break,
            }
        }

        let request = core::str::from_utf8(&buf[..pos]).unwrap_or("");
        if !request.starts_with("GET /ws/robby") {
            warn!("Rejecting request: {:?}", request.lines().next());
            let _ = socket.write_all(b"HTTP/1.1 404 Not Found\r\n\r\n").await;
            socket.close();
            continue;
        }

        // pass all request headers so the upgrade request can be validated
        let headers_iter = request
            .lines()
            .skip(1)
            .filter_map(|line| line.split_once(':'))
            .map(|(name, value)| (name.trim(), value.trim().as_bytes()));
        let web_socket_context = match embedded_websocket::read_http_header(headers_iter) {
            Ok(Some(ctx)) => ctx,
            other => {
                warn!("Invalid websocket upgrade request: {:?}", other.map(|_| ()));
                let _ = socket.write_all(b"HTTP/1.1 400 Bad Request\r\n\r\n").await;
                socket.close();
                continue;
            }
        };

        let mut ws_server = embedded_websocket::WebSocketServer::new_server();
        let mut response_buf = [0; 256];
        let len = ws_server.server_accept(&web_socket_context.sec_websocket_key, None, &mut response_buf).unwrap();

        if socket.write_all(&response_buf[..len]).await.is_err() {
            socket.close();
            continue;
        }

        info!("WebSocket session opened");

        while let Ok(_) = UART_TO_WS.try_receive() {}

        let mut socket_rx_buf = [0; 2048];
        let mut frame_buf = [0; 2048];
        let mut rx_pos = 0;

        loop {
            match embassy_futures::select::select(
                socket.read(&mut socket_rx_buf[rx_pos..]),
                UART_TO_WS.receive()
            ).await {
                embassy_futures::select::Either::First(Ok(0)) => {
                    break;
                }
                embassy_futures::select::Either::First(Ok(n)) => {
                    rx_pos += n;
                    let mut start_idx = 0;
                    while start_idx < rx_pos {
                        if rx_pos - start_idx < 2 {
                            break;
                        }
                        let buf = &socket_rx_buf[start_idx..rx_pos];
                        let opcode = buf[0] & 0x0F;
                        let masked = buf[1] & 0x80 != 0;
                        let mut payload_len = (buf[1] & 0x7F) as usize;
                        let mut offset = 2;
                        
                        if payload_len == 126 {
                            if buf.len() < 4 { break; }
                            payload_len = ((buf[2] as usize) << 8) | (buf[3] as usize);
                            offset = 4;
                        } else if payload_len == 127 {
                            break;
                        }
                        
                        let mut mask_key = [0_u8; 4];
                        if masked {
                            if buf.len() < offset + 4 { break; }
                            mask_key.copy_from_slice(&buf[offset..offset+4]);
                            offset += 4;
                        }
                        
                        if buf.len() < offset + payload_len { break; }
                        
                        if opcode == 0x01 || opcode == 0x02 { 
                            if payload_len <= MAX_RECEIVE_LEN {
                                for i in 0..payload_len {
                                    frame_buf[i] = buf[offset + i] ^ mask_key[i % 4];
                                }
                                
                                debug!("received {} bytes on ws -> forwarding to UART", payload_len);
                                
                                // fits, payload_len <= MAX_RECEIVE_LEN
                                if let Ok(to_send) = WsFrame::from_slice(&frame_buf[..payload_len]) {
                                    WS_TO_UART.send(to_send).await;
                                }
                            } else {
                                let _ = socket.write_all(&[0x88, 0x02, 0x03, 0xE8]).await; 
                                socket.flush().await.ok();
                                socket.close();
                                return;
                            }
                        } else if opcode == 0x08 { 
                            break;
                        }
                        
                        start_idx += offset + payload_len;
                    }
                    
                    if start_idx > 0 && start_idx < rx_pos {
                        socket_rx_buf.copy_within(start_idx..rx_pos, 0);
                        rx_pos -= start_idx;
                    } else if start_idx == rx_pos {
                        rx_pos = 0;
                    } else if rx_pos == socket_rx_buf.len() {
                        rx_pos = 0; 
                    }
                }
                embassy_futures::select::Either::First(Err(_)) => {
                    break;
                }
                embassy_futures::select::Either::Second(data) => {
                    let len = data.len();
                    let mut header = [0_u8; 4];
                    let header_len;
                    
                    header[0] = 0x82; 
                    if len < 126 {
                        header[1] = len as u8;
                        header_len = 2;
                    } else {
                        header[1] = 126;
                        header[2] = (len >> 8) as u8;
                        header[3] = len as u8;
                        header_len = 4;
                    }
                    
                    if socket.write_all(&header[..header_len]).await.is_err() {
                        break;
                    }
                    if socket.write_all(&data).await.is_err() {
                        break;
                    }
                }
            }
        }
        
        info!("WebSocket session closed");
        socket.close();
    }
}

#[embassy_executor::task]
async fn uart_task(
    uart: esp_hal::peripherals::UART1<'static>,
    rx_pin: esp_hal::peripherals::GPIO17<'static>,
    tx_pin: esp_hal::peripherals::GPIO16<'static>,
    led_conn: Output<'static>,
) {
    // wake up early so the 128 byte HW FIFO doesn't overflow at 1 Mbaud
    let uart_config = UartConfig::default()
        .with_baudrate(1_000_000)
        .with_rx(esp_hal::uart::RxConfig::default().with_fifo_full_threshold(32));

    let uart = Uart::new(uart, uart_config)
        .unwrap()
        .with_rx(rx_pin)
        .with_tx(tx_pin)
        .into_async();
    let (uart_rx, uart_tx) = uart.split();

    embassy_futures::join::join(uart_rx_loop(uart_rx, led_conn), uart_tx_loop(uart_tx)).await;
}

async fn uart_rx_loop(mut uart_rx: UartRx<'static, Async>, mut led_conn: Output<'static>) {
    let mut buf = [0_u8; 4096];
    let mut dec_buf = [0_u8; 2048];
    let mut buf_len = 0;
    // false until a frame delimiter was seen, so a partially received frame (at startup or after
    // an RX error) is dropped instead of being decoded
    let mut synced = false;

    let mut led_div_counter = 0_u16;

    loop {
        let mut read_buf = [0_u8; 256];
        match uart_rx.read_async(&mut read_buf).await {
            Ok(bytes_read) if bytes_read > 0 => {
                let available_space = buf.len() - buf_len;
                let read_len = core::cmp::min(bytes_read, available_space);
                buf[buf_len..buf_len + read_len].copy_from_slice(&read_buf[..read_len]);
                buf_len += read_len;
                
                let mut start_idx = 0;
                while let Some(pos) = buf[start_idx..buf_len].iter().position(|&b| b == 0x00) {
                    let end_idx = start_idx + pos;
                    let frame = &buf[start_idx..=end_idx];
                    if !synced {
                        synced = true;
                    } else if frame.len() > 1 {
                        if let Ok(report) = cobs::decode(frame, &mut dec_buf) {
                            debug!("decoded {}B frame, forwarding to websocket", report.frame_size());
                            match UartFrame::from_slice(&dec_buf[..report.frame_size()]) {
                                Ok(to_send) => {
                                    let _ = UART_TO_WS.try_send(to_send);
                                }
                                Err(_) => warn!("Dropping frame of {}B, larger than {}B", report.frame_size(), MAX_FRAME_LEN),
                            }
                        } else {
                            warn!("COBS decode failed for frame of size {}", frame.len());
                        }
                    }
                    start_idx = end_idx + 1;
                }
                
                if start_idx > 0 {
                    buf.copy_within(start_idx..buf_len, 0);
                    buf_len -= start_idx;
                }
                
                if buf_len == buf.len() {
                    warn!("UART buffer full, clearing");
                    buf_len = 0;
                    synced = false;
                }
            }
            Ok(_) => {}
            Err(e) => {
                warn!("UART RX error: {:?}, resyncing", e);
                buf_len = 0;
                synced = false;
            }
        }

        if led_div_counter >= 10 {
            led_conn.toggle();
            led_div_counter = 0;
        } else {
            led_div_counter += 1;
        }
    }
}

async fn uart_tx_loop(mut uart_tx: UartTx<'static, Async>) {
    let mut enc_buf = [0; 256];
    loop {
        let payload = WS_TO_UART.receive().await;
        let enc_len = cobs::encode_including_sentinels(&payload, &mut enc_buf);
        enc_buf[enc_len] = 0x00; 
        
        let _ = uart_tx.write_async(&enc_buf[..=enc_len]).await;
        let _ = uart_tx.flush_async().await;
    }
}
