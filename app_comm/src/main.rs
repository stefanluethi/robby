#![no_std]
#![no_main]

extern crate alloc;
use alloc::string::String;
use alloc::vec::Vec;
use core::mem::MaybeUninit;

use embassy_executor::Spawner;
use embassy_net::{
    tcp::TcpSocket, Config, Stack, StackResources,
};
use embassy_time::{Duration, Timer};
use embassy_sync::{
    blocking_mutex::raw::NoopRawMutex,
    channel::Channel,
};
use embedded_io_async::{Read, Write};

use esp_backtrace as _;
use esp_println::println;

use esp_hal::{
    clock::CpuClock,
    gpio::{Level, Output},
    interrupt::software::SoftwareInterruptControl,
    timer::timg::TimerGroup,
    uart::{UartRx, UartTx, config::Config as UartConfig, Uart},
    Async,
};

const WIFI_SSID: &str = "robby";
const WIFI_PASSWORD: &str = "robby1234";

const MAX_RECEIVE_LEN: usize = 128;

#[global_allocator]
static ALLOCATOR: esp_alloc::EspHeap = esp_alloc::EspHeap::empty();

fn init_heap() {
    const HEAP_SIZE: usize = 32 * 1024;
    static mut HEAP: [MaybeUninit<u8>; HEAP_SIZE] = [MaybeUninit::uninit(); HEAP_SIZE];

    unsafe {
        ALLOCATOR.init(HEAP.as_mut_ptr() as *mut u8, HEAP_SIZE);
    }
}

// Channel to send messages from UART to WS
static UART_TO_WS: Channel<NoopRawMutex, Vec<u8>, 4> = Channel::new();
// Channel to send messages from WS to UART
static WS_TO_UART: Channel<NoopRawMutex, Vec<u8>, 4> = Channel::new();

#[esp_hal::main]
async fn main(spawner: Spawner) -> ! {
    init_heap();
    esp_println::logger::init_logger_from_env();

    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    let sw_int = SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);
    let timg0 = TimerGroup::new(peripherals.TIMG0);
    
    esp_rtos::start(timg0.timer0, sw_int.software_interrupt0);

    let mut led_conn = Output::new(peripherals.GPIO15, Level::High);
    let mut rf_switch_power = Output::new(peripherals.GPIO3, Level::Low);
    let mut rf_switch_external = Output::new(peripherals.GPIO14, Level::High);

    let _ = rf_switch_power.set_low();
    let _ = rf_switch_external.set_high();

    let mut uart_config = UartConfig::default();
    uart_config.baudrate = 1_000_000;
    
    let mut uart = Uart::new(peripherals.UART1, uart_config).unwrap()
        .with_rx(peripherals.GPIO17)
        .with_tx(peripherals.GPIO16)
        .into_async();

    let (uart_rx, uart_tx) = uart.split();

    let (controller, interfaces) = esp_radio::wifi::new(
        peripherals.WIFI, 
        Default::default(),
    ).unwrap();

    let wifi = interfaces.sta;

    let net_config = Config::dhcpv4(Default::default());

    let seed = 1234; 
    static STACK: static_cell::StaticCell<Stack<esp_radio::wifi::WifiDevice>> = static_cell::StaticCell::new();
    static RESOURCES: static_cell::StaticCell<StackResources<3>> = static_cell::StaticCell::new();

    let stack = &*STACK.init(Stack::new(
        wifi,
        net_config,
        RESOURCES.init(StackResources::<3>::new()),
        seed,
    ));

    spawner.spawn(connection(controller)).ok();
    spawner.spawn(net_task(stack)).ok();

    loop {
        if stack.is_link_up() {
            break;
        }
        Timer::after(Duration::from_millis(500)).await;
    }

    println!("Waiting to get IP address...");
    loop {
        if let Some(config) = stack.config_v4() {
            println!("Got IP: {}", config.address);
            break;
        }
        Timer::after(Duration::from_millis(500)).await;
    }

    let _ = led_conn.set_low();

    spawner.spawn(ws_server_task(stack)).ok();
    spawner.spawn(uart_rx_task(uart_rx, led_conn)).ok();
    spawner.spawn(uart_tx_task(uart_tx)).ok();

    loop {
        Timer::after(Duration::from_secs(10)).await;
    }
}

#[embassy_executor::task]
async fn connection(mut controller: esp_radio::wifi::WifiController<'static>) {
    loop {
        match esp_radio::wifi::get_wifi_state() {
            esp_radio::wifi::WifiState::StaConnected => {
                controller.wait_for_event(esp_radio::wifi::WifiEvent::StaDisconnected).await;
                Timer::after(Duration::from_millis(5000)).await
            }
            _ => {}
        }
        if !matches!(controller.is_started(), Ok(true)) {
            let client_config = esp_radio::wifi::Configuration::Client(esp_radio::wifi::ClientConfiguration {
                ssid: WIFI_SSID.try_into().unwrap(),
                password: WIFI_PASSWORD.try_into().unwrap(),
                ..Default::default()
            });
            controller.set_configuration(&client_config).unwrap();
            controller.start().await.unwrap();
            println!("Wifi started!");
        }

        match controller.connect().await {
            Ok(_) => println!("Wifi connected!"),
            Err(e) => {
                println!("Failed to connect to wifi: {:?}", e);
                Timer::after(Duration::from_millis(5000)).await
            }
        }
    }
}

#[embassy_executor::task]
async fn net_task(stack: &'static Stack<esp_radio::wifi::WifiDevice<'static>>) {
    stack.run().await
}

#[embassy_executor::task]
async fn ws_server_task(stack: &'static Stack<esp_radio::wifi::WifiDevice<'static>>) {
    let mut rx_buffer = [0; 4096];
    let mut tx_buffer = [0; 4096];

    loop {
        let mut socket = TcpSocket::new(stack, &mut rx_buffer, &mut tx_buffer);
        socket.set_timeout(Some(embassy_time::Duration::from_secs(10)));

        println!("Listening on TCP:80...");
        if let Err(e) = socket.accept(80).await {
            println!("Accept error: {:?}", e);
            continue;
        }
        println!("Connection from {:?}", socket.remote_endpoint());

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
            let _ = socket.write_all(b"HTTP/1.1 404 Not Found\r\n\r\n").await;
            socket.close();
            continue;
        }
        
        let mut key = "";
        for line in request.lines() {
            let lower = line.to_lowercase();
            if lower.starts_with("sec-websocket-key:") {
                key = line[18..].trim();
            }
        }
        
        if key.is_empty() {
            let _ = socket.write_all(b"HTTP/1.1 400 Bad Request\r\n\r\n").await;
            socket.close();
            continue;
        }

        let accept_key = embedded_websocket::generate_accept_key(key);
        
        let mut response = String::new();
        response.push_str("HTTP/1.1 101 Switching Protocols\r\n");
        response.push_str("Upgrade: websocket\r\n");
        response.push_str("Connection: Upgrade\r\n");
        response.push_str("Sec-WebSocket-Accept: ");
        response.push_str(&accept_key);
        response.push_str("\r\n\r\n");

        if socket.write_all(response.as_bytes()).await.is_err() {
            socket.close();
            continue;
        }

        println!("WebSocket session opened");

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
                                
                                println!("received {} bytes on ws -> forwarding to UART", payload_len);
                                
                                let mut to_send = Vec::new();
                                to_send.extend_from_slice(&frame_buf[..payload_len]);
                                WS_TO_UART.send(to_send).await;
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
                    let mut header_len = 0;
                    
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
        
        println!("WebSocket session closed");
        socket.close();
    }
}

#[embassy_executor::task]
async fn uart_rx_task(mut uart_rx: UartRx<'static, Async>, mut led_conn: Output<'static>) {
    let mut buf = [0_u8; 4096];
    let mut dec_buf = [0_u8; 2048];
    let mut buf_len = 0;
    
    let mut led_div_counter = 0_u16;

    loop {
        let mut read_buf = [0_u8; 256];
        match uart_rx.read(&mut read_buf).await {
            Ok(bytes_read) if bytes_read > 0 => {
                let available_space = buf.len() - buf_len;
                let read_len = core::cmp::min(bytes_read, available_space);
                buf[buf_len..buf_len + read_len].copy_from_slice(&read_buf[..read_len]);
                buf_len += read_len;
                
                let mut start_idx = 0;
                while let Some(pos) = buf[start_idx..buf_len].iter().position(|&b| b == 0x00) {
                    let end_idx = start_idx + pos;
                    let frame = &buf[start_idx..=end_idx];
                    if frame.len() > 1 {
                        if let Ok(report) = cobs::decode(frame, &mut dec_buf) {
                            println!("decoded {}B frame, forwarding to websocket", report.frame_size());
                            let mut to_send = Vec::new();
                            to_send.extend_from_slice(&dec_buf[..report.frame_size()]);
                            let _ = UART_TO_WS.try_send(to_send);
                        } else {
                            println!("COBS decode failed for frame of size {}", frame.len());
                        }
                    }
                    start_idx = end_idx + 1;
                }
                
                if start_idx > 0 {
                    buf.copy_within(start_idx..buf_len, 0);
                    buf_len -= start_idx;
                }
                
                if buf_len == buf.len() {
                    println!("UART buffer full, clearing");
                    buf_len = 0;
                }
            }
            _ => {}
        }

        if led_div_counter >= 10 {
            led_conn.toggle();
            led_div_counter = 0;
        } else {
            led_div_counter += 1;
        }
    }
}

#[embassy_executor::task]
async fn uart_tx_task(mut uart_tx: UartTx<'static, Async>) {
    let mut enc_buf = [0; 256];
    loop {
        let payload = WS_TO_UART.receive().await;
        let enc_len = cobs::encode_including_sentinels(&payload, &mut enc_buf);
        enc_buf[enc_len] = 0x00; 
        
        let _ = uart_tx.write_all(&enc_buf[..=enc_len]).await;
        let _ = uart_tx.flush().await;
    }
}
