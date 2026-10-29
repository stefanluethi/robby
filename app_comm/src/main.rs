//! robby bridge: the ESP32-C6 between the STM32 (protobuf frames on the UART) and the
//! outpost on the PC (`blip-outpost protobuf`, on a WebSocket), see `proto/README.md`.
//!
//! The bridge doesn't decode the protobuf messages, so a change of the schema needs no new
//! firmware: it checks and unwraps the frames of the STM32 and forwards their payloads, and
//! wraps the payloads of the outpost into frames for the STM32.
//!
//! ```text
//! UART task (interrupt executor)        WebSocket task (thread executor)
//! read ─ deframe (COBS, CRC) ─► UPLINK ──► binary frame ─► ws://<ip>/ws/robby
//! write ◄──── frame ◄─────── DOWNLINK ◄── binary frame ◄── outpost
//!                                         text frame (JSON counters) once per second ─►
//! ```

#![no_std]
#![no_main]

extern crate alloc;
use alloc::string::String;
use core::sync::atomic::{AtomicI32, AtomicU32, Ordering::Relaxed};

use embassy_executor::Spawner;
use embassy_futures::select::{select, select3, Either, Either3};
use embassy_net::{tcp::TcpSocket, Config, Stack, StackResources};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, channel::Channel};
use embassy_time::{Duration, Instant, Timer};
use embedded_io_async::Write;

use esp_backtrace as _;
use log::{debug, info, warn};

use esp_hal::{
    clock::CpuClock,
    gpio::{Level, Output},
    interrupt::{software::SoftwareInterruptControl, Priority},
    timer::timg::TimerGroup,
    uart::{Config as UartConfig, Uart, UartRx, UartTx},
    Async,
};

use bridge_core::stats::write_units;
use bridge_core::{frame, Counter, Counters, Deframer, DownlinkFrame, FrameError, Report, MAX_PAYLOAD};

use static_cell::StaticCell;

esp_bootloader_esp_idf::esp_app_desc!();

const WIFI_SSID: &str = env!("WIFI_SSID");
const WIFI_PASSWORD: &str = env!("WIFI_PASSWORD");

/// Largest message from the outpost: a downlink payload (an `Invoke` takes < 100 bytes)
const MAX_REQUEST_LEN: usize = 1024;
/// Uplink payloads between the UART and the WebSocket: ~0.3 s at full UART load
const UPLINK_QUEUE_LEN: usize = 32;
const REPORT_PERIOD_US: u64 = 1_000_000;

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

// Everything of the bridge is static, nothing on the heap: the radio needs all of it (a queue
// of frames on the heap exhausted it, see f570e27).
type UplinkPayload = heapless::Vec<u8, MAX_PAYLOAD>;
/// Checked uplink payloads, from the UART task to the WebSocket task
static UPLINK: Channel<CriticalSectionRawMutex, UplinkPayload, UPLINK_QUEUE_LEN> = Channel::new();
/// Downlink frames, from the WebSocket task to the UART task
static DOWNLINK: Channel<CriticalSectionRawMutex, DownlinkFrame, 8> = Channel::new();

static COUNTERS: Counters = Counters::new();
/// µs the CPU spent waiting for an interrupt, wraps
static IDLE_US: AtomicU32 = AtomicU32::new(0);
const RSSI_UNKNOWN: i32 = i32::MIN;
static RSSI: AtomicI32 = AtomicI32::new(RSSI_UNKNOWN);

fn now_us() -> u64 {
    Instant::now().as_micros()
}

/// Waits for an interrupt with interrupts masked, so the time until it arrives can be counted
/// before its handler runs (WFI wakes up on a pending interrupt also while they are masked).
/// The scheduler doesn't keep this context, it starts over on each idle period.
extern "C" fn idle_hook() -> ! {
    loop {
        critical_section::with(|_| {
            let start = Instant::now();
            esp_hal::interrupt::wait_for_interrupt();
            IDLE_US.fetch_add(start.elapsed().as_micros() as u32, Relaxed);
        });
    }
}

#[esp_rtos::main]
async fn main(spawner: Spawner) -> ! {
    init_heap();
    esp_println::logger::init_logger_from_env();

    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    let sw_int = SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);
    let timg0 = TimerGroup::new(peripherals.TIMG0);

    esp_rtos::start_with_idle_hook(timg0.timer0, sw_int.software_interrupt0, idle_hook);

    let led_conn = Output::new(peripherals.GPIO15, Level::High, esp_hal::gpio::OutputConfig::default());
    let mut rf_switch_power = Output::new(peripherals.GPIO3, Level::Low, esp_hal::gpio::OutputConfig::default());
    let mut rf_switch_external = Output::new(peripherals.GPIO14, Level::High, esp_hal::gpio::OutputConfig::default());

    rf_switch_power.set_low();
    rf_switch_external.set_high();

    let (controller, interfaces) = esp_radio::wifi::new(peripherals.WIFI, Default::default()).unwrap();

    // Started after the radio init, which masks interrupts long enough to overflow the UART FIFO.
    // UART runs in a higher priority interrupt executor, so the network stack can't delay it
    // long enough to overflow the HW FIFO
    static UART_EXECUTOR: StaticCell<esp_rtos::embassy::InterruptExecutor<1>> = StaticCell::new();
    let uart_executor = UART_EXECUTOR.init(esp_rtos::embassy::InterruptExecutor::new(sw_int.software_interrupt1));
    let uart_spawner = uart_executor.start(Priority::Priority2);
    uart_spawner.spawn(uart_task(peripherals.UART1, peripherals.GPIO17, peripherals.GPIO16, led_conn).unwrap());

    let wifi = interfaces.station;

    let net_config = Config::dhcpv4(Default::default());

    let seed = 1234;
    static STACK: StaticCell<Stack<'static>> = StaticCell::new();
    static RESOURCES: StaticCell<StackResources<3>> = StaticCell::new();

    let (stack, runner) = embassy_net::new(wifi, net_config, RESOURCES.init(StackResources::<3>::new()), seed);
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
            .with_password(String::from(WIFI_PASSWORD)),
    );
    controller.set_config(&client_config).unwrap();

    loop {
        match controller.connect_async().await {
            Ok(_) => {
                info!("Wifi connected!");
                // only reconnect once the connection is actually lost, meanwhile read the RSSI
                let rssi = async {
                    loop {
                        if let Ok(rssi) = controller.rssi() {
                            RSSI.store(rssi, Relaxed);
                        }
                        Timer::after(Duration::from_secs(1)).await;
                    }
                };
                let _ = select(controller.wait_for_disconnect_async(), rssi).await;
                RSSI.store(RSSI_UNKNOWN, Relaxed);
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

/// The state of the bridge that outlives a WebSocket session.
struct Bridge {
    next_report_us: u64,
    /// IDLE_US and the time at the report before
    idle_prev: (u32, u64),
}

#[embassy_executor::task]
async fn ws_server_task(stack: &'static Stack<'static>) {
    let mut rx_buffer = [0; 2048];
    // a larger window keeps up with the round trips of WiFi
    let mut tx_buffer = [0; 16384];
    let mut bridge = Bridge { next_report_us: 0, idle_prev: (IDLE_US.load(Relaxed), now_us()) };

    loop {
        let mut socket = TcpSocket::new(*stack, &mut rx_buffer, &mut tx_buffer);
        socket.set_timeout(Some(Duration::from_secs(10)));

        info!("Listening on TCP:80...");
        // without a client the uplink frames go nowhere: the outpost wants them current
        let accepted = match select(socket.accept(80), discard_uplink()).await {
            Either::First(accepted) => accepted,
            Either::Second(never) => match never {},
        };
        if let Err(e) = accepted {
            warn!("Accept error: {:?}", e);
            continue;
        }
        info!("Connection from {:?}", socket.remote_endpoint());

        if accept_websocket(&mut socket).await.is_ok() {
            info!("WebSocket session opened");
            let _ = session(&mut socket, &mut bridge).await;
            info!("WebSocket session closed");
        }
        socket.close();
        // let the close go out before the buffers are reused
        let _ = embassy_time::with_timeout(Duration::from_secs(1), socket.flush()).await;
        socket.abort();
    }
}

/// Drops the uplink payloads while no client is connected.
async fn discard_uplink() -> core::convert::Infallible {
    loop {
        UPLINK.receive().await;
    }
}

/// Reads the HTTP request and answers the WebSocket upgrade.
async fn accept_websocket(socket: &mut TcpSocket<'_>) -> Result<(), ()> {
    let mut buf = [0; 1024];
    let mut pos = 0;
    while pos < buf.len() {
        match socket.read(&mut buf[pos..pos + 1]).await {
            Ok(0) | Err(_) => break,
            Ok(_) => {
                pos += 1;
                if pos >= 4 && &buf[pos - 4..pos] == b"\r\n\r\n" {
                    break;
                }
            }
        }
    }

    let request = core::str::from_utf8(&buf[..pos]).unwrap_or("");
    if !request.starts_with("GET /ws/robby") {
        warn!("Rejecting request: {:?}", request.lines().next());
        let _ = socket.write_all(b"HTTP/1.1 404 Not Found\r\n\r\n").await;
        return Err(());
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
            return Err(());
        }
    };

    let mut ws_server = embedded_websocket::WebSocketServer::new_server();
    let mut response_buf = [0; 256];
    let len = ws_server
        .server_accept(&web_socket_context.sec_websocket_key, None, &mut response_buf)
        .map_err(|_| ())?;
    socket.write_all(&response_buf[..len]).await.map_err(|_| ())
}

const OPCODE_TEXT: u8 = 0x1;
const OPCODE_BINARY: u8 = 0x2;
const OPCODE_CLOSE: u8 = 0x8;
const OPCODE_PING: u8 = 0x9;
const OPCODE_PONG: u8 = 0xA;

/// One client: the units of the counters, then the uplink payloads, the downlink payloads of
/// the client and the counters, until it closes or the connection fails.
async fn session(socket: &mut TcpSocket<'_>, bridge: &mut Bridge) -> Result<(), ()> {
    let mut text = heapless::String::<1024>::new();
    // fits, checked by the tests of bridge-core
    let _ = write_units(&mut text);
    send(socket, OPCODE_TEXT, text.as_bytes()).await?;
    bridge.next_report_us = now_us() + REPORT_PERIOD_US;

    let mut rx_buf = [0_u8; MAX_REQUEST_LEN + 14];
    let mut rx_len = 0;
    loop {
        match select3(
            socket.read(&mut rx_buf[rx_len..]),
            UPLINK.receive(),
            Timer::at(Instant::from_micros(bridge.next_report_us)),
        )
        .await
        {
            Either3::First(Ok(0) | Err(_)) => return Err(()),
            Either3::First(Ok(n)) => {
                rx_len += n;
                let mut start = 0;
                while let Some((opcode, payload, end)) = ws_frame(&mut rx_buf[start..rx_len]) {
                    let payload = start + payload.start..start + payload.end;
                    start += end;
                    match opcode {
                        OPCODE_BINARY => forward_downlink(&rx_buf[payload]),
                        OPCODE_PING => send(socket, OPCODE_PONG, &rx_buf[payload]).await?,
                        OPCODE_CLOSE => return Ok(()),
                        _ => {}
                    }
                }
                rx_buf.copy_within(start..rx_len, 0);
                rx_len -= start;
                if rx_len == rx_buf.len() {
                    warn!("Message larger than {} bytes, closing", MAX_REQUEST_LEN);
                    // close frame, status 1009: message too big
                    let _ = socket.write_all(&[0x88, 0x02, 0x03, 0xF1]).await;
                    return Err(());
                }
            }
            Either3::Second(payload) => send(socket, OPCODE_BINARY, &payload).await?,
            Either3::Third(()) => {}
        }

        if now_us() >= bridge.next_report_us {
            report(socket, bridge).await?;
        }
    }
}

/// Frames a downlink payload of the outpost for the STM32.
fn forward_downlink(payload: &[u8]) {
    let Some(frame): Option<DownlinkFrame> = frame(payload) else {
        warn!("Dropping a downlink message of {} bytes, more than {}", payload.len(), bridge_core::MAX_DOWNLINK_PAYLOAD);
        return COUNTERS.add(Counter::DownlinkDrops, 1);
    };
    if DOWNLINK.try_send(frame).is_err() {
        return COUNTERS.add(Counter::DownlinkDrops, 1);
    }
    COUNTERS.add(Counter::DownlinkFrames, 1);
}

/// The bridge's counters as JSON in a text frame.
async fn report(socket: &mut TcpSocket<'_>, bridge: &mut Bridge) -> Result<(), ()> {
    let now = now_us();
    let idle = IDLE_US.load(Relaxed);
    let (idle_before, at) = bridge.idle_prev;
    let cpu_idle = (now > at).then(|| (100.0 * idle.wrapping_sub(idle_before) as f32 / (now - at) as f32).min(100.0));
    bridge.idle_prev = (idle, now);
    let rssi = RSSI.load(Relaxed);
    let report = Report { values: COUNTERS.snapshot(), cpu_idle, rssi: (rssi != RSSI_UNKNOWN).then_some(rssi) };
    let mut text = heapless::String::<1024>::new();
    // fits, checked by the tests of bridge-core
    let _ = report.write_json(&mut text);
    send(socket, OPCODE_TEXT, text.as_bytes()).await?;
    // the next one a period later, without catching up after a stall
    let next = bridge.next_report_us + REPORT_PERIOD_US;
    bridge.next_report_us = if next <= now { now + REPORT_PERIOD_US } else { next };
    Ok(())
}

/// Sends one unmasked WebSocket frame of at most 64 KiB, and counts it.
async fn send(socket: &mut TcpSocket<'_>, opcode: u8, payload: &[u8]) -> Result<(), ()> {
    let len = payload.len();
    let long = [0x80 | opcode, 126, (len >> 8) as u8, len as u8];
    let short = [0x80 | opcode, len as u8];
    let header = if len < 126 { &short[..] } else { &long[..] };
    let start = Instant::now();
    socket.write_all(header).await.map_err(|_| ())?;
    socket.write_all(payload).await.map_err(|_| ())?;
    // until the bytes are in the TCP buffer: long when WiFi doesn't keep up
    COUNTERS.max(Counter::WsSendMax, start.elapsed().as_micros() as u32);
    COUNTERS.add(Counter::WsTxBytes, len as u32);
    COUNTERS.add(Counter::WsFrames, 1);
    Ok(())
}

/// The first complete WebSocket frame in `buf`, unmasked in place: its opcode, where its
/// payload is and its length. `None` until the frame is complete.
fn ws_frame(buf: &mut [u8]) -> Option<(u8, core::ops::Range<usize>, usize)> {
    let opcode = *buf.first()? & 0x0F;
    let masked = *buf.get(1)? & 0x80 != 0;
    let (len, mut offset) = match buf[1] & 0x7F {
        126 => (u16::from_be_bytes([*buf.get(2)?, *buf.get(3)?]) as usize, 4),
        127 => (u64::from_be_bytes(buf.get(2..10)?.try_into().ok()?) as usize, 10),
        len => (len as usize, 2),
    };
    let mask = if masked {
        let mask: [u8; 4] = buf.get(offset..offset + 4)?.try_into().ok()?;
        offset += 4;
        Some(mask)
    } else {
        None
    };
    let end = offset.checked_add(len)?;
    let payload = buf.get_mut(offset..end)?;
    if let Some(mask) = mask {
        for (i, b) in payload.iter_mut().enumerate() {
            *b ^= mask[i % 4];
        }
    }
    Some((opcode, offset..end, end))
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

    let uart = Uart::new(uart, uart_config).unwrap().with_rx(rx_pin).with_tx(tx_pin).into_async();
    let (uart_rx, uart_tx) = uart.split();

    embassy_futures::join::join(uart_rx_loop(uart_rx, led_conn), uart_tx_loop(uart_tx)).await;
}

/// Reads the frames of the STM32, checks them and queues their payloads for the WebSocket.
async fn uart_rx_loop(mut uart_rx: UartRx<'static, Async>, mut led_conn: Output<'static>) {
    let mut deframer = Deframer::new();
    // the STM32 may be in the middle of a frame
    deframer.resync();
    let mut led_div_counter = 0_u16;

    loop {
        let mut read_buf = [0_u8; 256];
        match uart_rx.read_async(&mut read_buf).await {
            Ok(n) => {
                COUNTERS.add(Counter::UartRxBytes, n as u32);
                for &b in &read_buf[..n] {
                    if let Some(frame) = deframer.push(b) {
                        on_uplink_frame(frame);
                    }
                }
            }
            Err(e) => {
                COUNTERS.add(Counter::UartRxErrors, 1);
                debug!("UART RX error: {:?}, resyncing", e);
                deframer.resync();
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

fn on_uplink_frame(frame: Result<&[u8], FrameError>) {
    let payload = match frame {
        Ok(payload) => payload,
        Err(FrameError::Crc) => return COUNTERS.add(Counter::CrcErrors, 1),
        Err(FrameError::Cobs | FrameError::TooLong) => return COUNTERS.add(Counter::CobsErrors, 1),
    };
    COUNTERS.add(Counter::UartFrames, 1);
    // fits: the deframer returns at most MAX_PAYLOAD bytes
    let Ok(payload) = UplinkPayload::from_slice(payload) else { return };
    if UPLINK.try_send(payload).is_err() {
        COUNTERS.add(Counter::QueueDrops, 1);
    } else {
        COUNTERS.max(Counter::QueueHighWater, UPLINK.len() as u32);
    }
}

async fn uart_tx_loop(mut uart_tx: UartTx<'static, Async>) {
    loop {
        let frame = DOWNLINK.receive().await;
        let _ = uart_tx.write_async(&frame).await;
        let _ = uart_tx.flush_async().await;
    }
}
