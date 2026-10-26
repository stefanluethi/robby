//! robby outpost: the ESP32-C6 between the STM32 (protobuf frames on the UART) and blip
//! (CBOR on a WebSocket), see `proto/README.md` and `outpost-core`.
//!
//! ```text
//! UART task (interrupt executor)        WebSocket task (thread executor)
//! read ─ deframe ─ seq ─► UPLINK ──────► translate ─ batch ─► ws://<ip>/ws/robby
//! write ◄──────────────── DOWNLINK ◄──── parse Invoke ◄─────── blip
//! ```
//!
//! With the `bench-wifi` feature there's no UART: the outpost generates `bench/synthetic`
//! samples itself at the rate of the `bench/load` action.

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
#[cfg(not(feature = "bench-wifi"))]
use log::debug;
use log::{info, warn};

use esp_hal::{
    clock::CpuClock,
    gpio::{Level, Output},
    interrupt::software::SoftwareInterruptControl,
    timer::timg::TimerGroup,
};
#[cfg(not(feature = "bench-wifi"))]
use esp_hal::{
    interrupt::Priority,
    uart::{Config as UartConfig, Uart, UartRx, UartTx},
    Async,
};

#[cfg(not(feature = "bench-wifi"))]
use outpost_core::actions::DownlinkFrame;
use outpost_core::blip::ActionOutcome;
use outpost_core::framing::MAX_PAYLOAD;
use outpost_core::{
    parse_request, Batcher, Counter, Counters, Out, PushError, Rejected, Report, Translator, UplinkCursor,
};
#[cfg(not(feature = "bench-wifi"))]
use outpost_core::{peek_seq, Deframer, FrameError, SeqEvent, SeqTracker};

use static_cell::StaticCell;

esp_bootloader_esp_idf::esp_app_desc!();

const WIFI_SSID: &str = env!("WIFI_SSID");
const WIFI_PASSWORD: &str = env!("WIFI_PASSWORD");

/// Largest WebSocket frame to blip: a CBOR sequence of ~60 IMU samples
const BATCH_LIMIT: usize = 4096;
/// A batch goes out at the latest this long after its first message
const FLUSH_AFTER_US: u64 = 20_000;
/// Largest request from blip (an `Invoke` takes < 100 bytes)
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

// Everything of the outpost is static, nothing on the heap: the radio needs all of it (a queue
// of frames on the heap exhausted it, see f570e27).
type UplinkPayload = heapless::Vec<u8, MAX_PAYLOAD>;
/// Checked uplink payloads, from the UART task to the WebSocket task
static UPLINK: Channel<CriticalSectionRawMutex, UplinkPayload, UPLINK_QUEUE_LEN> = Channel::new();
/// Downlink frames, from the WebSocket task to the UART task
#[cfg(not(feature = "bench-wifi"))]
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

    #[cfg_attr(feature = "bench-wifi", allow(unused_variables))]
    let led_conn = Output::new(peripherals.GPIO15, Level::High, esp_hal::gpio::OutputConfig::default());
    let mut rf_switch_power = Output::new(peripherals.GPIO3, Level::Low, esp_hal::gpio::OutputConfig::default());
    let mut rf_switch_external = Output::new(peripherals.GPIO14, Level::High, esp_hal::gpio::OutputConfig::default());

    rf_switch_power.set_low();
    rf_switch_external.set_high();

    let (controller, interfaces) = esp_radio::wifi::new(peripherals.WIFI, Default::default()).unwrap();

    // Started after the radio init, which masks interrupts long enough to overflow the UART FIFO.
    // UART runs in a higher priority interrupt executor, so the network stack can't delay it
    // long enough to overflow the HW FIFO
    #[cfg(not(feature = "bench-wifi"))]
    {
        static UART_EXECUTOR: StaticCell<esp_rtos::embassy::InterruptExecutor<1>> = StaticCell::new();
        let uart_executor = UART_EXECUTOR.init(esp_rtos::embassy::InterruptExecutor::new(sw_int.software_interrupt1));
        let uart_spawner = uart_executor.start(Priority::Priority2);
        uart_spawner.spawn(uart_task(peripherals.UART1, peripherals.GPIO17, peripherals.GPIO16, led_conn).unwrap());
    }
    #[cfg(feature = "bench-wifi")]
    info!("bench-wifi: no UART, bench/load sets the rate of the synthetic samples");

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

type WsBatcher = Batcher<BATCH_LIMIT>;

/// The state of the outpost that outlives a WebSocket session.
struct Outpost {
    translator: Translator,
    batcher: WsBatcher,
    #[cfg(feature = "bench-wifi")]
    synthetic: outpost_core::SyntheticGen,
    next_report_us: u64,
    /// IDLE_US and the time at the report before
    idle_prev: (u32, u64),
}

#[embassy_executor::task]
async fn ws_server_task(stack: &'static Stack<'static>) {
    let mut rx_buffer = [0; 2048];
    // the traces stream at ~55 kB/s, a larger window keeps up with the round trips of WiFi
    let mut tx_buffer = [0; 16384];
    let mut outpost = Outpost {
        translator: Translator::new(),
        batcher: WsBatcher::new(BATCH_LIMIT, FLUSH_AFTER_US),
        #[cfg(feature = "bench-wifi")]
        synthetic: outpost_core::SyntheticGen::new(),
        next_report_us: 0,
        idle_prev: (IDLE_US.load(Relaxed), now_us()),
    };

    loop {
        let mut socket = TcpSocket::new(*stack, &mut rx_buffer, &mut tx_buffer);
        socket.set_timeout(Some(Duration::from_secs(10)));

        info!("Listening on TCP:80...");
        // without a client the frames are still translated, so the outpost keeps track of the
        // tasks, the link rates and the time of the STM32
        let accepted = match select(socket.accept(80), drain(&mut outpost.translator)).await {
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
            let _ = session(&mut socket, &mut outpost).await;
            info!("WebSocket session closed");
        }
        socket.close();
        // let the close go out before the buffers are reused
        let _ = embassy_time::with_timeout(Duration::from_secs(1), socket.flush()).await;
        socket.abort();
    }
}

/// Translates the uplink frames without a client, the messages go nowhere.
async fn drain(translator: &mut Translator) -> core::convert::Infallible {
    loop {
        let payload = UPLINK.receive().await;
        let mut cursor = UplinkCursor::new(&payload);
        loop {
            match cursor.next(translator, now_us()) {
                Ok(Some(_)) => {}
                Ok(None) => break,
                Err(_) => {
                    COUNTERS.add(Counter::DecodeErrors, 1);
                    break;
                }
            }
        }
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

/// One client: the hello, then uplink messages, answers to its requests and the counters,
/// until it closes or the connection fails.
async fn session(socket: &mut TcpSocket<'_>, outpost: &mut Outpost) -> Result<(), ()> {
    outpost.translator.on_connect();
    outpost.batcher.clear();
    for i in 0..outpost.translator.hello_len() {
        push(socket, &mut outpost.batcher, &outpost.translator.hello(i)).await?;
    }
    send(socket, &mut outpost.batcher).await?;
    #[cfg(feature = "bench-wifi")]
    outpost.synthetic.restart(now_us());
    outpost.next_report_us = now_us() + REPORT_PERIOD_US;

    let mut rx_buf = [0_u8; MAX_REQUEST_LEN];
    let mut rx_len = 0;
    loop {
        let wake = [
            Some(outpost.next_report_us),
            outpost.batcher.deadline(),
            outpost.translator.next_timeout(),
            #[cfg(feature = "bench-wifi")]
            outpost.synthetic.next_due_us(),
        ]
        .into_iter()
        .flatten()
        .min()
        .unwrap_or(u64::MAX);

        match select3(
            socket.read(&mut rx_buf[rx_len..]),
            UPLINK.receive(),
            Timer::at(Instant::from_micros(wake)),
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
                        // binary or text
                        0x1 | 0x2 => handle_request(socket, outpost, &rx_buf[payload]).await?,
                        // close
                        0x8 => return Ok(()),
                        _ => {}
                    }
                }
                rx_buf.copy_within(start..rx_len, 0);
                rx_len -= start;
                if rx_len == rx_buf.len() {
                    warn!("Request larger than {} bytes, closing", MAX_REQUEST_LEN);
                    // close frame, status 1009: message too big
                    let _ = socket.write_all(&[0x88, 0x02, 0x03, 0xF1]).await;
                    return Err(());
                }
            }
            Either3::Second(payload) => {
                let mut cursor = UplinkCursor::new(&payload);
                loop {
                    match cursor.next(&mut outpost.translator, now_us()) {
                        Ok(Some(out)) => push(socket, &mut outpost.batcher, &out).await?,
                        Ok(None) => break,
                        Err(_) => {
                            COUNTERS.add(Counter::DecodeErrors, 1);
                            break;
                        }
                    }
                }
            }
            Either3::Third(()) => {}
        }

        while let Some(out) = outpost.translator.expired(now_us()) {
            push(socket, &mut outpost.batcher, &out).await?;
        }
        if now_us() >= outpost.next_report_us {
            report(socket, outpost).await?;
        }
        #[cfg(feature = "bench-wifi")]
        generate(socket, outpost).await?;
        if outpost.batcher.is_due(now_us()) {
            send(socket, &mut outpost.batcher).await?;
        }
    }
}

/// Answers a request of blip: passes an `Invoke` on to the STM32 or answers it right away.
async fn handle_request(socket: &mut TcpSocket<'_>, outpost: &mut Outpost, request: &[u8]) -> Result<(), ()> {
    let mut reason = heapless::String::<64>::new();
    let id = match parse_request(request) {
        Ok(invocation) => {
            #[cfg(feature = "bench-wifi")]
            let result = match invocation.action {
                outpost_core::Action::BenchLoad { rate, .. } => {
                    info!("bench/load: {} samples/s", rate);
                    outpost.synthetic.set_rate(rate, now_us());
                    Ok(())
                }
                _ => Err("no STM32 in the bench-wifi firmware"),
            };
            #[cfg(not(feature = "bench-wifi"))]
            let result = if DOWNLINK.is_full() {
                Err("the UART to the STM32 is busy")
            } else {
                outpost.translator.forward(&invocation, now_us()).map(|frame| {
                    // not full, checked above, and only this task sends
                    let _ = DOWNLINK.try_send(frame);
                })
            };
            match result {
                // the STM32 answers
                #[cfg(not(feature = "bench-wifi"))]
                Ok(()) => return Ok(()),
                #[cfg(feature = "bench-wifi")]
                Ok(()) => {
                    let out = Out::ActionResult { id: invocation.id, result: ActionOutcome::Ok };
                    push(socket, &mut outpost.batcher, &out).await?;
                    return send(socket, &mut outpost.batcher).await;
                }
                Err(e) => {
                    let _ = reason.push_str(e);
                    invocation.id
                }
            }
        }
        Err(Rejected::Answer { id, reason: r }) => {
            reason = r;
            id
        }
        Err(Rejected::Unreadable) => {
            warn!("Ignoring a request that isn't an Invoke ({} bytes)", request.len());
            return Ok(());
        }
    };
    let out = Out::ActionResult { id, result: ActionOutcome::Err(&reason) };
    push(socket, &mut outpost.batcher, &out).await?;
    send(socket, &mut outpost.batcher).await
}

/// The outpost's counters as `outpost/*` values.
async fn report(socket: &mut TcpSocket<'_>, outpost: &mut Outpost) -> Result<(), ()> {
    let now = now_us();
    let idle = IDLE_US.load(Relaxed);
    let (idle_before, at) = outpost.idle_prev;
    let cpu_idle = (now > at).then(|| (100.0 * idle.wrapping_sub(idle_before) as f32 / (now - at) as f32).min(100.0));
    outpost.idle_prev = (idle, now);
    let rssi = RSSI.load(Relaxed);
    let report = Report {
        timestamp: outpost.translator.stm32_time_ns(now),
        values: COUNTERS.snapshot(),
        cpu_idle,
        rssi: (rssi != RSSI_UNKNOWN).then_some(rssi),
    };
    for i in 0..Report::LEN {
        push(socket, &mut outpost.batcher, &report.message(i)).await?;
    }
    // the next one a period later, without catching up after a stall
    let next = outpost.next_report_us + REPORT_PERIOD_US;
    outpost.next_report_us = if next <= now { now + REPORT_PERIOD_US } else { next };
    Ok(())
}

/// `bench-wifi`: the synthetic samples due now, a few at a time so requests are still read.
#[cfg(feature = "bench-wifi")]
async fn generate(socket: &mut TcpSocket<'_>, outpost: &mut Outpost) -> Result<(), ()> {
    // samples that are more than 100 ms behind are skipped and counted
    let (due, skipped) = outpost.synthetic.due(now_us(), 100_000);
    COUNTERS.add(Counter::BenchDrops, skipped);
    for _ in 0..due.min(256) {
        let out = outpost.synthetic.next_sample();
        push(socket, &mut outpost.batcher, &out).await?;
    }
    Ok(())
}

/// Adds a message to the batch, sends the batch first if it's full.
async fn push(socket: &mut TcpSocket<'_>, batcher: &mut WsBatcher, out: &Out<'_>) -> Result<(), ()> {
    match batcher.push(out, now_us()) {
        Ok(()) => Ok(()),
        Err(PushError::Full) => {
            send(socket, batcher).await?;
            if batcher.push(out, now_us()).is_err() {
                warn!("Dropping a message larger than a batch");
            }
            Ok(())
        }
        Err(PushError::TooLarge) => {
            warn!("Dropping a message larger than a batch");
            Ok(())
        }
    }
}

/// Sends the batch as one binary WebSocket frame and counts it.
async fn send(socket: &mut TcpSocket<'_>, batcher: &mut WsBatcher) -> Result<(), ()> {
    if batcher.is_empty() {
        return Ok(());
    }
    let frame = batcher.frame();
    let len = frame.len();
    let header = [0x82, 126, (len >> 8) as u8, len as u8];
    let header = if len < 126 { &[0x82, len as u8][..] } else { &header[..] };
    let start = Instant::now();
    socket.write_all(header).await.map_err(|_| ())?;
    socket.write_all(frame).await.map_err(|_| ())?;
    // until the bytes are in the TCP buffer: long when WiFi doesn't keep up
    COUNTERS.max(Counter::WsSendMax, start.elapsed().as_micros() as u32);
    COUNTERS.add(Counter::WsTxBytes, len as u32);
    COUNTERS.add(Counter::WsFrames, 1);
    COUNTERS.add(Counter::WsMessages, batcher.messages());
    batcher.clear();
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

#[cfg(not(feature = "bench-wifi"))]
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

/// Reads the frames of the STM32, checks them and queues their payloads. Only the cheap part
/// happens here, in the interrupt executor: the translation runs in the WebSocket task.
#[cfg(not(feature = "bench-wifi"))]
async fn uart_rx_loop(mut uart_rx: UartRx<'static, Async>, mut led_conn: Output<'static>) {
    let mut deframer = Deframer::new();
    // the STM32 may be in the middle of a frame
    deframer.resync();
    let mut seq = SeqTracker::new();
    let mut led_div_counter = 0_u16;

    loop {
        let mut read_buf = [0_u8; 256];
        match uart_rx.read_async(&mut read_buf).await {
            Ok(n) => {
                COUNTERS.add(Counter::UartRxBytes, n as u32);
                for &b in &read_buf[..n] {
                    if let Some(frame) = deframer.push(b) {
                        on_uplink_frame(frame, &mut seq);
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

#[cfg(not(feature = "bench-wifi"))]
fn on_uplink_frame(frame: Result<&[u8], FrameError>, seq: &mut SeqTracker) {
    let payload = match frame {
        Ok(payload) => payload,
        Err(FrameError::Crc) => return COUNTERS.add(Counter::CrcErrors, 1),
        Err(FrameError::Cobs | FrameError::TooLong) => return COUNTERS.add(Counter::CobsErrors, 1),
    };
    let Ok(s) = peek_seq(payload) else {
        return COUNTERS.add(Counter::DecodeErrors, 1);
    };
    match seq.check(s) {
        SeqEvent::InOrder => {}
        SeqEvent::Gap(n) => COUNTERS.add(Counter::SeqGaps, n),
        SeqEvent::Reset => COUNTERS.add(Counter::Stm32Resets, 1),
    }
    COUNTERS.add(Counter::UartFrames, 1);
    // fits: the deframer returns at most MAX_PAYLOAD bytes
    let Ok(payload) = UplinkPayload::from_slice(payload) else { return };
    if UPLINK.try_send(payload).is_err() {
        COUNTERS.add(Counter::QueueDrops, 1);
    } else {
        COUNTERS.max(Counter::QueueHighWater, UPLINK.len() as u32);
    }
}

#[cfg(not(feature = "bench-wifi"))]
async fn uart_tx_loop(mut uart_tx: UartTx<'static, Async>) {
    loop {
        let frame = DOWNLINK.receive().await;
        let _ = uart_tx.write_async(&frame).await;
        let _ = uart_tx.flush_async().await;
    }
}
