use std::time::Duration;

use actix_web::{
    App, Error, HttpRequest, HttpResponse, HttpServer,
    rt::{self, time::sleep},
    web,
};

use actix_ws::AggregatedMessage;
use futures_util::StreamExt;
use proto::{self, AdcMessage};
use rand::RngExt;

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
            let task_manager = proto::ThreadTable {
                threads: vec![proto::ThreadInfo {
                    stack_usage: 72_u8,
                    runtime: 11.2_f32 + rng.random_range(0.0..10.0),
                    name: "blinky".to_owned(),
                }],
            };

            let message = serde_cbor_2::to_vec(&task_manager).unwrap();
            if task_manager_sender.binary(message).await.is_err() {
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
            let message = AdcMessage {
                samples: (0..16).map(|_| rng.random_range(0..u16::MAX)).collect(),
            };

            let message = serde_cbor_2::to_vec(&message).unwrap();
            if measurement_sender.binary(message).await.is_err() {
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
    HttpServer::new(|| App::new().route("/robby", web::get().to(robby)))
        .bind(("127.0.0.1", 3000))?
        .run()
        .await
}
