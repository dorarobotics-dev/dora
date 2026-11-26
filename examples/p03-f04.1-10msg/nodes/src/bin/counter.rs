//! Counter node: receives, counts and verifies the sample messages.
//!
//! Counting happens only on the receive side, from real `INPUT` events — the
//! node never assumes how many messages were sent. It prints:
//!
//! ```text
//! [RECV] #<n> seq=<seq>             one line per received message
//! [RECV_TOTAL=<n>] sequence_ok=..   once EXPECT_TOTAL messages arrived, with a
//!                                   1..=EXPECT_TOTAL sequence-integrity check
//! [COUNTER_DONE] received=<n> ..    on STOP (graceful shutdown)
//! ```
//!
//! `run.sh` extracts `[SENT_TOTAL=..]` / `[RECV_TOTAL=..]` from the captured
//! stdout and fails the run unless both read 10 and `sequence_ok=true`.

use dora_node_api::{DoraNode, Event};

fn main() -> eyre::Result<()> {
    let (_node, mut events) = DoraNode::init_from_env()?;

    let expect: u64 = std::env::var("EXPECT_TOTAL")
        .ok()
        .and_then(|v| v.parse().ok())
        .unwrap_or(10);

    let mut received: u64 = 0;
    let mut seqs: Vec<u64> = Vec::new();

    while let Some(event) = events.recv() {
        match event {
            Event::Input { id, data, .. } => {
                if id.as_str() != "message" {
                    println!("[IGNORED] input={id}");
                    continue;
                }
                let seq = u64::try_from(&data).map_err(|e| eyre::eyre!("bad payload: {e}"))?;
                received += 1;
                seqs.push(seq);
                println!("[RECV] #{received} seq={seq}");

                if received == expect {
                    let sequence_ok = is_exact_sequence(&seqs, expect);
                    println!("[RECV_TOTAL={received}] sequence_ok={sequence_ok}");
                }
            }
            Event::Stop(_) => break,
            _ => {}
        }
    }

    let sequence_ok = is_exact_sequence(&seqs, expect);
    println!("[COUNTER_DONE] received={received} expected={expect} sequence_ok={sequence_ok}");
    Ok(())
}

fn is_exact_sequence(seqs: &[u64], expect: u64) -> bool {
    seqs.len() == expect as usize && seqs.iter().copied().eq(1..=expect)
}
