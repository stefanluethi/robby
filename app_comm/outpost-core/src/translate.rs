//! The records of the link as blip messages, see "Records and their blip messages" in
//! `proto/README.md`.

use heapless::{String, Vec};

use crate::actions::{self, Invocation, Pending};
use crate::blip::{ActionOutcome, Component, Out, Path, Val};
use crate::link::{Component as LinkComponent, Record, Record_::Kind};
use crate::uplink::samples;

/// g per LSB of the LSM6DSO at ±2 g
pub const ACC_SCALE: f32 = 0.000061;

/// Most task names the outpost keeps for their metadata.
pub const MAX_TASKS: usize = 24;

/// How long the outpost waits for the `ActionResult` of the STM32.
pub const ACTION_TIMEOUT_US: u64 = 1_000_000;

pub const IMU_ACC: Path<'static> = Path::fixed("imu/acc");
pub const BENCH_SYNTHETIC: Path<'static> = Path::fixed("bench/synthetic");
const TOF_DISTANCE: Path<'static> = Path::fixed("tof/distance");
const BATTERY: [Path<'static>; 5] = [
    Path::fixed("battery/soc"),
    Path::fixed("battery/voltage"),
    Path::fixed("battery/cell1"),
    Path::fixed("battery/cell2"),
    Path::fixed("battery/current"),
];
const LINK_TOTALS: [Path<'static>; 9] = [
    Path::fixed("link/tx_bytes"),
    Path::fixed("link/tx_frames"),
    Path::fixed("link/tx_frames_dropped"),
    Path::fixed("link/records_dropped"),
    Path::fixed("link/rx_frames"),
    Path::fixed("link/rx_errors"),
    Path::fixed("link/rx_seq_gaps"),
    Path::fixed("link/baud"),
    Path::fixed("link/tx_write_max"),
];
const LINK_RATES: [Path<'static>; 4] = [
    Path::fixed("link/tx_rate"),
    Path::fixed("link/tx_frame_rate"),
    Path::fixed("link/tx_load"),
    Path::fixed("link/tx_write_time"),
];
const TASK_UNITS: [(&str, &str); 3] = [(".runtime", "%"), (".stack_usage", "%"), (".stack_size", "B")];

/// The units sent to each client, besides those of the tasks and of `outpost/*`.
const UNITS: [(&str, &str); 19] = [
    ("imu/acc.x", "g"),
    ("imu/acc.y", "g"),
    ("imu/acc.z", "g"),
    ("bench/synthetic.x", "g"),
    ("bench/synthetic.y", "g"),
    ("bench/synthetic.z", "g"),
    ("tof/distance", "mm"),
    ("battery/soc", "%"),
    ("battery/voltage", "V"),
    ("battery/cell1", "V"),
    ("battery/cell2", "V"),
    ("battery/current", "mA"),
    ("link/tx_bytes", "B"),
    ("link/baud", "Bd"),
    ("link/tx_write_max", "µs"),
    ("link/tx_rate", "B/s"),
    ("link/tx_frame_rate", "1/s"),
    ("link/tx_load", "%"),
    ("link/tx_write_time", "%"),
];

/// What [`Translator::prepare`] found out about a record, so that [`message`] only reads.
#[derive(Debug, Default, Clone, Copy)]
pub struct Prepared {
    /// The number of blip messages of the record
    pub(crate) count: u16,
    /// The task is new: its metadata goes first.
    new_task: bool,
    rates: Option<[f32; 4]>,
}

#[derive(Debug, Clone, Copy)]
struct LinkSample {
    t_ns: u64,
    tx_bytes: u32,
    tx_frames: u32,
    tx_write_us: u32,
}

/// The state of the translation: what it needs to remember between records.
pub struct Translator {
    tasks: Vec<String<15>, MAX_TASKS>,
    link_prev: Option<LinkSample>,
    /// The newest STM32 time and the ESP32 time it arrived at
    last_time: Option<(u64, u64)>,
    pending: Pending,
    downlink_seq: u32,
}

impl Default for Translator {
    fn default() -> Self {
        Self::new()
    }
}

impl Translator {
    pub const fn new() -> Self {
        Self { tasks: Vec::new(), link_prev: None, last_time: None, pending: Pending::new(), downlink_seq: 0 }
    }

    /// A client connected: invocations of the one before aren't answered any more.
    pub fn on_connect(&mut self) {
        self.pending.clear();
    }

    /// The outpost's estimate of the STM32 uptime: the newest time received plus the ESP32
    /// time since; the ESP32 uptime before the first record.
    pub fn stm32_time_ns(&self, now_us: u64) -> u64 {
        match self.last_time {
            Some((t_ns, at_us)) => t_ns.wrapping_add(now_us.saturating_sub(at_us) * 1000),
            None => now_us * 1000,
        }
    }

    /// Remembers what the record changes and counts its messages.
    pub(crate) fn prepare(&mut self, record: &Record, now_us: u64) -> Prepared {
        let mut prepared = Prepared::default();
        let Some(kind) = &record.kind else {
            // a record kind of a newer STM32 firmware
            return prepared;
        };
        let time = match kind {
            Kind::Imu(b) => Some(last_sample_time(b.t_first_ns, b.period_ns, samples(kind))),
            Kind::Synthetic(b) => Some(last_sample_time(b.t_first_ns, b.period_ns, samples(kind))),
            Kind::Distance(d) => Some(d.t_ns),
            Kind::Battery(b) => Some(b.t_ns),
            Kind::Task(t) => Some(t.t_ns),
            Kind::Log(l) => Some(l.t_ns),
            Kind::LinkStats(s) => Some(s.t_ns),
            Kind::ActionResult(_) => None,
        };
        if let Some(t) = time {
            self.last_time = Some((t, now_us));
        }
        prepared.count = match kind {
            Kind::Imu(_) | Kind::Synthetic(_) => samples(kind).min(u16::MAX as usize) as u16,
            Kind::Distance(_) | Kind::Log(_) => 1,
            Kind::Battery(_) => BATTERY.len() as u16,
            Kind::Task(t) => {
                prepared.new_task = self.remember_task(&t.name);
                if prepared.new_task {
                    TASK_UNITS.len() as u16 + 1
                } else {
                    1
                }
            }
            // only to the client that invoked it, and only once
            Kind::ActionResult(a) => u16::from(self.pending.take(a.id)),
            Kind::LinkStats(s) => {
                let now = LinkSample { t_ns: s.t_ns, tx_bytes: s.tx_bytes, tx_frames: s.tx_frames, tx_write_us: s.tx_write_us };
                prepared.rates = self.link_prev.filter(|prev| now.t_ns > prev.t_ns).map(|prev| {
                    let dt_s = (now.t_ns - prev.t_ns) as f32 * 1e-9;
                    let tx_rate = now.tx_bytes.wrapping_sub(prev.tx_bytes) as f32 / dt_s;
                    let frame_rate = now.tx_frames.wrapping_sub(prev.tx_frames) as f32 / dt_s;
                    let load = if s.baud > 0 { 100.0 * tx_rate / (s.baud as f32 / 10.0) } else { 0.0 };
                    let write_time = 100.0 * now.tx_write_us.wrapping_sub(prev.tx_write_us) as f32 * 1e-6 / dt_s;
                    [tx_rate, frame_rate, load, write_time]
                });
                self.link_prev = Some(now);
                (LINK_TOTALS.len() + if prepared.rates.is_some() { LINK_RATES.len() } else { 0 }) as u16
            }
        };
        prepared
    }

    fn remember_task(&mut self, name: &String<15>) -> bool {
        if self.tasks.contains(name) {
            return false;
        }
        self.tasks.push(name.clone()).is_ok()
    }

    /// The number of messages each new client gets first.
    pub fn hello_len(&self) -> usize {
        UNITS.len() + crate::stats::COUNTERS.len() + crate::stats::GAUGES.len() + TASK_UNITS.len() * self.tasks.len() + 1
    }

    /// Message `i` of [`Self::hello_len`] for each new client: the units, then the actions.
    /// Units of counters without one go out empty, so a client sees all `outpost/*` paths.
    pub fn hello(&self, i: usize) -> Out<'_> {
        let fixed = UNITS.len();
        let counters = crate::stats::COUNTERS.len() + crate::stats::GAUGES.len();
        if i < fixed {
            let (path, unit) = UNITS[i];
            return Out::Metadata { path: Path::fixed(path), unit };
        }
        let i = i - fixed;
        if i < counters {
            let (path, unit) = crate::stats::unit(i);
            return Out::Metadata { path: Path::fixed(path), unit };
        }
        let i = i - counters;
        if let Some(name) = self.tasks.get(i / TASK_UNITS.len()) {
            let (suffix, unit) = TASK_UNITS[i % TASK_UNITS.len()];
            return Out::Metadata { path: Path { prefix: "tasks/", name, suffix }, unit };
        }
        Out::Actions
    }

    /// Turns a checked invocation into a `Downlink` frame for the STM32 and waits for its
    /// answer. Errs with the reason for blip if too many are waiting.
    pub fn forward(&mut self, invocation: &Invocation, now_us: u64) -> Result<actions::DownlinkFrame, &'static str> {
        if !self.pending.add(invocation.id, now_us + ACTION_TIMEOUT_US) {
            return Err("too many actions waiting for the STM32");
        }
        let seq = self.downlink_seq;
        self.downlink_seq = self.downlink_seq.wrapping_add(1);
        Ok(actions::downlink_frame(seq, invocation))
    }

    /// The answer to an invocation the STM32 didn't answer in time, one at a time.
    pub fn expired(&mut self, now_us: u64) -> Option<Out<'static>> {
        let id = self.pending.expired(now_us)?;
        Some(Out::ActionResult { id, result: ActionOutcome::Err("no answer from the STM32") })
    }

    /// When the next invocation times out.
    pub fn next_timeout(&self) -> Option<u64> {
        self.pending.next_deadline()
    }
}

fn last_sample_time(t_first_ns: u64, period_ns: u32, n: usize) -> u64 {
    t_first_ns.wrapping_add(u64::from(period_ns) * n.saturating_sub(1) as u64)
}

pub(crate) fn xyz(x: i16, y: i16, z: i16) -> Val<'static> {
    Val::Xyz([f32::from(x) * ACC_SCALE, f32::from(y) * ACC_SCALE, f32::from(z) * ACC_SCALE])
}

/// Message `i` of the record (`i < prepared.count`).
pub(crate) fn message<'r>(record: &'r Record, prepared: &Prepared, i: u16) -> Out<'r> {
    let i_us = usize::from(i);
    match &record.kind {
        Some(Kind::Imu(b)) => Out::Value {
            timestamp: b.t_first_ns.wrapping_add(u64::from(b.period_ns) * u64::from(i)),
            path: IMU_ACC,
            value: xyz(b.x[i_us], b.y[i_us], b.z[i_us]),
        },
        Some(Kind::Synthetic(b)) => Out::Value {
            timestamp: b.t_first_ns.wrapping_add(u64::from(b.period_ns) * u64::from(i)),
            path: BENCH_SYNTHETIC,
            value: xyz(b.x[i_us], b.y[i_us], b.z[i_us]),
        },
        Some(Kind::Distance(d)) => Out::Value { timestamp: d.t_ns, path: TOF_DISTANCE, value: Val::Distance(&d.mm) },
        Some(Kind::Battery(b)) => {
            let value = match i {
                0 => Val::Uint(b.soc_pct.into()),
                1 => Val::Float(b.voltage_mv as f32 / 1000.0),
                2 => Val::Float(b.cell1_mv as f32 / 1000.0),
                3 => Val::Float(b.cell2_mv as f32 / 1000.0),
                _ => Val::Int(b.current_ma.into()),
            };
            Out::Value { timestamp: b.t_ns, path: BATTERY[i_us.min(BATTERY.len() - 1)], value }
        }
        Some(Kind::Task(t)) => {
            if prepared.new_task && i_us < TASK_UNITS.len() {
                let (suffix, unit) = TASK_UNITS[i_us];
                return Out::Metadata { path: Path { prefix: "tasks/", name: &t.name, suffix }, unit };
            }
            let stack_usage = if t.stack_size_bytes == 0 {
                0.0
            } else {
                100.0 * (1.0 - t.stack_free_min_bytes as f32 / t.stack_size_bytes as f32)
            };
            Out::Value {
                timestamp: t.t_ns,
                path: Path { prefix: "tasks/", name: &t.name, suffix: "" },
                value: Val::Task {
                    priority: t.priority,
                    runtime: t.runtime_bp as f32 / 100.0,
                    stack_usage,
                    stack_size: t.stack_size_bytes,
                },
            }
        }
        Some(Kind::Log(l)) => Out::Log {
            timestamp: l.t_ns,
            // blip has no FATAL
            level: l.level.0.clamp(0, 4) as u8,
            component: component(l.component_id),
            message: &l.message,
        },
        Some(Kind::ActionResult(a)) => Out::ActionResult {
            id: a.id,
            result: if a.error.is_empty() { ActionOutcome::Ok } else { ActionOutcome::Err(&a.error) },
        },
        Some(Kind::LinkStats(s)) => {
            if let (Some(rates), Some(r)) = (prepared.rates, i_us.checked_sub(LINK_TOTALS.len())) {
                return Out::Value { timestamp: s.t_ns, path: LINK_RATES[r.min(3)], value: Val::Float(rates[r.min(3)]) };
            }
            let total = [
                s.tx_bytes,
                s.tx_frames,
                s.tx_frames_dropped,
                s.records_dropped,
                s.rx_frames,
                s.rx_errors,
                s.rx_seq_gaps,
                s.baud,
                s.tx_write_max_us,
            ];
            let i = i_us.min(LINK_TOTALS.len() - 1);
            Out::Value { timestamp: s.t_ns, path: LINK_TOTALS[i], value: Val::Uint(total[i].into()) }
        }
        // count is 0 for these
        None => Out::Metadata { path: Path::fixed(""), unit: "" },
    }
}

fn component(id: LinkComponent) -> Component {
    match id {
        LinkComponent::Cmd => Component::Name("cmd"),
        LinkComponent::Space => Component::Name("space"),
        LinkComponent::Imu => Component::Name("imu"),
        LinkComponent(n) => Component::Number(n),
    }
}
