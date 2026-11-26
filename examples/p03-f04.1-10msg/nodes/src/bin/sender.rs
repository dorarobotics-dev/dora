//! Sender node: emits exactly `SEND_TOTAL` (default 10) sample messages.
//!
//! One message per `tick` input (the built-in `dora/timer/millis/100` timer is
//! wired up in `dataflow.yml`). Each message carries a monotonically increasing
//! sequence number `1..=SEND_TOTAL` as a `u64` Arrow array, so the receiving
//! node can verify both the count and the sequence integrity.
//!
//! The count that gets reported is the number of `send_output` calls that
//! actually happened, not a hard-coded expectation.

use dora_node_api::{DoraNode, Event, IntoArrow, dora_core::config::DataId};

fn main() -> eyre::Result<()> {
    let (mut node, mut events) = DoraNode::init_from_env()?;

    let total: u64 = std::env::var("SEND_TOTAL")
        .ok()
        .and_then(|v| v.parse().ok())
        .unwrap_or(10);
    let output = DataId::from("message".to_owned());

    let mut sent: u64 = 0;
    while let Some(event) = events.recv() {
        match event {
            Event::Input { id, metadata, .. } => {
                if id.as_str() != "tick" {
                    println!("[IGNORED] input={id}");
                    continue;
                }
                if sent >= total {
                    // All messages are out; ignore further ticks until the run
                    // is stopped.
                    continue;
                }

                sent += 1;
                node.send_output(output.clone(), metadata.parameters, sent.into_arrow())?;
                println!("[SENT] seq={sent}");
                if sent == total {
                    println!("[SENT_TOTAL={sent}]");
                }
            }
            Event::Stop(_) => break,
            other => println!("[IGNORED] event={other:?}"),
        }
    }

    println!("[SENDER_DONE] sent={sent}");
    Ok(())
}
