#!/usr/bin/env bash
set -euo pipefail
here=$(cd "$(dirname "$0")" && pwd)
root=$(cd "$here/../.." && pwd)
cargo build -p dora-node-api-c --release --manifest-path "$root/Cargo.toml"
mkdir -p "$here/build"
cc -std=c11 -Wall -Wextra "$here/node.c" -o "$here/build/c_node" \
    -L "${CARGO_TARGET_DIR:-$root/target}/release" -ldora_node_api_c -lm -lpthread -ldl -lrt -lz
