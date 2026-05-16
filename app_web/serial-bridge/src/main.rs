use std::io;
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


async fn robby(req: HttpRequest, stream: web::Payload) -> Result<HttpResponse, Error> {
    let (res, mut session, stream) = actix_ws::handle(&req, stream)?;

    let mut stream = stream
        .aggregate_continuations()
        // aggregate continuation frames up to 1MiB
        .max_continuation_size(2_usize.pow(20));

    // spawn task to periodically send task manager updates
    let task_manager_sender = session.clone();
    rt::spawn(async move {
        poll_serial(task_manager_sender).await;
    });

    // start task to handle incoming messages
    rt::spawn(async move {
        while let Some(msg) = stream.next().await {
            match msg {
                Ok(AggregatedMessage::Text(text)) => {

                }

                Ok(AggregatedMessage::Binary(bin)) => {

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


async fn poll_serial(mut session: Session) {
    let mut port = serialport::new("/dev/ttyACM0", 1_000_000)
        .timeout(Duration::from_millis(10))
        .open()
        .expect("Failed to open port");
    port.clear(ClearBuffer::All).unwrap();

    let mut serial_buf: Vec<u8> = vec![0; 2_048];
    let mut dest = [0; 2_048];
    let mut decoder = CobsDecoder::new(&mut dest);

    loop {
        match port.read(serial_buf.as_mut_slice()) {
            Ok(t) => {
                if let Ok(result) = decoder.push(&serial_buf[..t]) {
                    if let Some(report) = result {
                        log::debug!("decoded frame size: {}", report.frame_size());
                        let data = Vec::from(&decoder.dest()[..report.frame_size()]);
                        if session.binary(data).await.is_err() {
                            return;
                        };
                    }
                } else {
                    log::warn!("cobs decoder error");
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

