use std::io;
use std::io::Write;
use std::time::Duration;
use cobs::CobsDecoder;
use serialport::ClearBuffer;

use actix_web::{
    App, Error, HttpRequest, HttpResponse, HttpServer,
    rt::{self, time::sleep},
    web,
};
use futures_util::StreamExt;
use actix_ws::{AggregatedMessage, Session};
use log::debug;

async fn robby(req: HttpRequest, stream: web::Payload) -> Result<HttpResponse, Error> {
    let (res, mut session, stream) = actix_ws::handle(&req, stream)?;

    let port_result = serialport::new("/dev/ttyACM0", 1_000_000)
        .timeout(Duration::from_millis(10))
        .open();

    let port = match port_result {
        Ok(port) => port,
        Err(e) => {
            log::error!("Failed to open port: {}", e);
            return Ok(res);
        }
    };
    port.clear(ClearBuffer::All).unwrap();
    let mut serial_writer = port.try_clone().unwrap();

    let mut stream = stream
        .aggregate_continuations()
        // aggregate continuation frames up to 1MiB
        .max_continuation_size(2_usize.pow(20));

    // spawn task to periodically send task manager updates
    let task_manager_sender = session.clone();
    rt::spawn(async move {
        poll_serial(task_manager_sender, port).await;
    });

    // start task to handle incoming messages
    rt::spawn(async move {
        while let Some(msg) = stream.next().await {
            match msg {
                Ok(AggregatedMessage::Text(_text)) => {

                }

                Ok(AggregatedMessage::Binary(bin)) => {
                    debug!("writing {}B to serial", bin.len());
                    serial_writer.write(&bin).unwrap();
                    serial_writer.flush().unwrap();
                }

                Ok(AggregatedMessage::Ping(msg)) => {
                    // respond to PING frame with PONG frame
                    if session.pong(&msg).await.is_err() {
                        break;
                    }
                }

                _ => {}
            };
        }
    });

    // respond immediately with response connected to WS session
    Ok(res)
}


async fn poll_serial(mut session: Session, mut port: Box<dyn serialport::SerialPort>) {
    let mut serial_buf: Vec<u8> = vec![0; 4_096];
    let mut dest = [0; 4_096];
    let mut decoder = CobsDecoder::new(&mut dest);

    loop {
        match port.read(serial_buf.as_mut_slice()) {
            Ok(t) => {
                let mut buf_index = 0;
                while buf_index < t {
                    match decoder.push(&serial_buf[buf_index..t]) {
                        Ok(None) => {break;},
                        Ok(Some(report)) => {
                            log::debug!("decoded frame size: {}, {}/{}", report.frame_size(), report.parsed_size(), t);
                            let data = Vec::from(&decoder.dest()[..report.frame_size()]);
                            if session.binary(data).await.is_err() {
                                return;
                            };
                            buf_index += report.parsed_size();
                        },
                        Err(e) => {
                            log::warn!("cobs decoder error: {e}");
                            break;
                        },
                    };
                }
            }
            Err(ref e) if e.kind() == io::ErrorKind::TimedOut => (),
            Err(e) => eprintln!("{:?}", e),
        }

        sleep(Duration::from_millis(10)).await;
    }
}

#[actix_web::main]
async fn main() -> std::io::Result<()> {
    simple_logger::init().unwrap();
    log::set_max_level(log::LevelFilter::Debug);

    HttpServer::new(|| App::new().route("/ws/robby", web::get().to(robby)))
        .bind(("127.0.0.1", 3000))?
        .run()
        .await
}

