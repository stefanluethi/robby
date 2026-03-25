use std::time::Duration;

use actix_web::{
    App, Error, HttpRequest, HttpResponse, HttpServer,
    rt::{self, time::sleep},
    web,
};

use actix_ws::AggregatedMessage;
use futures_util::StreamExt;
use rand::RngExt;
use log;

async fn robby(req: HttpRequest, stream: web::Payload) -> Result<HttpResponse, Error> {
    let (res, mut session, stream) = actix_ws::handle(&req, stream)?;

    let mut stream = stream
        .aggregate_continuations()
        // aggregate continuation frames up to 1MiB
        .max_continuation_size(2_usize.pow(20));

    // spawn task to periodically send task manager updates
    let mut task_manager_sender = session.clone();
    rt::spawn(async move {
        let mut rng = rand::rng();

        loop {
            let thread_table = proto::ThreadTable {
                threads: vec![proto::ThreadInfo {
                    stack_usage: 72_u8,
                    runtime: 11.2_f32 + rng.random_range(0.0..10.0),
                    name: "blinky".to_owned(),
                }],
            };
            let message = proto::Message {
                payload: proto::Payload::ThreadTable(thread_table),
            };

            let message_raw = serde_cbor_2::to_vec(&message).unwrap();
            if task_manager_sender.binary(message_raw).await.is_err() {
                break;
            }
            sleep(Duration::from_millis(500)).await;
        }
    });

    // stream measurement value
    let mut measurement_sender = session.clone();
    rt::spawn(async move {
        let mut rng = rand::rng();

        loop {
            let frame = proto::StreamFrameRaw {
                id: 42,
                values: (0..16).map(|_| rng.random_range(0..u32::MAX)).collect(),
            };
            let message = proto::Message {
                payload: proto::Payload::StreamFrame(frame),
            };

            let message_raw = serde_cbor_2::to_vec(&message).unwrap();
            // log::info!("measurement {}", hex_string::HexString::from_bytes(&message_raw).as_string());
            if measurement_sender.binary(message_raw).await.is_err() {
                break;
            }

            // ------
            let mut distances_mm: [[i16;8];24] = [[0; 8]; 24];
            for column in distances_mm.iter_mut() {
                for cell in column.iter_mut() {
                    *cell = rng.random_range(0..4000_i16);
                }
            };
            let distance_map = proto::DistanceMap {
                distances_mm,
            };
            let message_distance = proto::Message {
                payload: proto::Payload::DistanceMap(distance_map),
            };
            let distance_raw = serde_cbor_2::to_vec(&message_distance).unwrap();
            log::info!("distance {}", hex_string::HexString::from_bytes(&distance_raw).as_string());
            if measurement_sender.binary(distance_raw).await.is_err() {
                break;
            }

            sleep(Duration::from_millis(20)).await;
        }
    });
    // start task to handle incoming messages
    rt::spawn(async move {
        while let Some(msg) = stream.next().await {
            match msg {
                Ok(AggregatedMessage::Text(text)) => {
                    // echo text message
                    if session.text(text).await.is_err() {
                        break;
                    }
                }

                Ok(AggregatedMessage::Binary(bin)) => {
                    // echo binary message
                    if session.binary(bin).await.is_err() {
                        break;
                    }
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

#[actix_web::main]
async fn main() -> std::io::Result<()> {
    simple_logger::init().unwrap();
    log::set_max_level(log::LevelFilter::Info);

    HttpServer::new(|| App::new().route("/robby", web::get().to(robby)))
        .bind(("127.0.0.1", 3000))?
        .run()
        .await
}
