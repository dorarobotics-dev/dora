"""Assert every header key a C node sent arrives with its type."""
from dora import Node

seen = 0
for event in Node():
    if event["type"] != "INPUT":
        continue
    m = event["metadata"]
    assert m["frame_id"] == "rslidar", m
    assert isinstance(m["header_seq"], int) and m["header_seq"] == seen, m
    assert m["stamp_ns"] == 1000 + seen, m
    assert m["source"] == "c-header" and m["validity"] == "ok", m
    seen += 1
    if seen == 10:
        print("C_HEADER_OK")
        break
