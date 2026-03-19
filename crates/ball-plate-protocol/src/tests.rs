use core::ffi::c_void;

use crate::{
    crc8, packet::*, CmdHandler, CmdId, CommandDispatcher, CommandReceiver, SerialFramer,
};

// ── Helper: build a valid raw packet ───────────────────────

fn build_packet_raw(
    buf: &mut [u8; MAX_PACKET_SIZE],
    cmd: CmdId,
    seq: u8,
    payload: Option<&[u8]>,
) -> usize {
    let payload = payload.unwrap_or(&[]);

    buf[OFF_HEADER] = PACKET_HEADER;
    buf[OFF_VER] = PROTOCOL_VER;
    buf[OFF_CMD] = cmd.0;
    buf[OFF_SEQ] = seq;
    buf[OFF_LEN] = payload.len() as u8;

    if !payload.is_empty() {
        buf[OFF_PAYLOAD..OFF_PAYLOAD + payload.len()].copy_from_slice(payload);
    }

    buf[HEADER_SIZE + payload.len()] = crc8(&buf[..HEADER_SIZE + payload.len()]);
    HEADER_SIZE + payload.len() + 1
}

// ── Mock CommandReceiver ──────────────────────────────────

struct MockReceiver {
    queue: [Entry; Self::MAX_QUEUE],
    read_idx: usize,
    write_idx: usize,
}

#[derive(Clone, Copy)]
struct Entry {
    data: [u8; MAX_PACKET_SIZE],
    len: usize,
}

impl MockReceiver {
    const MAX_QUEUE: usize = 8;

    const fn new() -> Self {
        const EMPTY: Entry = Entry {
            data: [0u8; MAX_PACKET_SIZE],
            len: 0,
        };
        Self {
            queue: [EMPTY; Self::MAX_QUEUE],
            read_idx: 0,
            write_idx: 0,
        }
    }

    fn push(&mut self, data: &[u8], len: usize) {
        if self.write_idx < Self::MAX_QUEUE {
            let mut e = Entry {
                data: [0u8; MAX_PACKET_SIZE],
                len,
            };
            e.data[..len].copy_from_slice(&data[..len]);
            self.queue[self.write_idx] = e;
            self.write_idx += 1;
        }
    }
}

impl CommandReceiver for MockReceiver {
    fn receive(&mut self, buf: &mut [u8; MAX_PACKET_SIZE]) -> Option<usize> {
        if self.read_idx >= self.write_idx {
            return None;
        }

        let pkt = self.queue[self.read_idx];
        self.read_idx += 1;

        buf[..pkt.len].copy_from_slice(&pkt.data[..pkt.len]);
        Some(pkt.len)
    }
}

// ── CRC-8 tests ───────────────────────────────────────────

#[test]
fn test_crc8_empty() {
    let crc = crc8(&[]);
    assert_eq!(0x00, crc);
}

#[test]
fn test_crc8_known_value_deterministic() {
    let data = [0xA5, 0x01, 0x01, 0x00, 0x01, 0x01];
    let c = crc8(&data);
    assert_eq!(crc8(&data), c);
}

#[test]
fn test_crc8_single_bit_change() {
    let data1 = [0xA5, 0x01, 0x01, 0x00, 0x01, 0x01];
    let data2 = [0xA5, 0x01, 0x01, 0x00, 0x01, 0x00];
    assert_ne!(crc8(&data1), crc8(&data2));
}

// ── parse_packet tests ──────────────────────────────────────

#[test]
fn test_parse_valid_packet() {
    let mut buf = [0u8; MAX_PACKET_SIZE];
    let payload = [0x01u8];
    let len = build_packet_raw(&mut buf, CmdId::REMOTE_STOP, 42, Some(&payload));

    let pkt = parse_packet(&buf[..len]).expect("should parse");
    assert_eq!(CmdId::REMOTE_STOP, pkt.cmd);
    assert_eq!(42, pkt.seq);
    assert_eq!(1, pkt.payload.len());
    assert_eq!(0x01, pkt.payload[0]);
}

#[test]
fn test_parse_zero_payload() {
    let mut buf = [0u8; MAX_PACKET_SIZE];
    let len = build_packet_raw(&mut buf, CmdId::REMOTE_RESUME, 0, None);

    let pkt = parse_packet(&buf[..len]).expect("should parse");
    assert_eq!(0, pkt.payload.len());
}

#[test]
fn test_parse_target_update_payload() {
    let mut buf = [0u8; MAX_PACKET_SIZE];
    let x: f32 = 120.5;
    let y: f32 = 80.25;
    let mut payload = [0u8; 8];
    payload[..4].copy_from_slice(&x.to_le_bytes());
    payload[4..].copy_from_slice(&y.to_le_bytes());

    let len = build_packet_raw(&mut buf, CmdId::TARGET_UPDATE, 1, Some(&payload));

    let pkt = parse_packet(&buf[..len]).expect("should parse");
    assert_eq!(8, pkt.payload.len());
    assert!((read_f32_le(&pkt.payload[..4]) - x).abs() < 0.001);
    assert!((read_f32_le(&pkt.payload[4..8]) - y).abs() < 0.001);
}

#[test]
fn test_parse_wrong_header() {
    let mut buf = [0u8; MAX_PACKET_SIZE];
    let len = build_packet_raw(&mut buf, CmdId::REMOTE_STOP, 0, None);
    buf[OFF_HEADER] = 0xFF;

    assert!(parse_packet(&buf[..len]).is_none());
}

#[test]
fn test_parse_wrong_version() {
    let mut buf = [0u8; MAX_PACKET_SIZE];
    let len = build_packet_raw(&mut buf, CmdId::REMOTE_STOP, 0, None);
    buf[OFF_VER] = 0xFF;

    assert!(parse_packet(&buf[..len]).is_none());
}

#[test]
fn test_parse_bad_crc() {
    let mut buf = [0u8; MAX_PACKET_SIZE];
    let payload = [0x01u8];
    let len = build_packet_raw(&mut buf, CmdId::REMOTE_STOP, 0, Some(&payload));
    buf[len - 1] ^= 0xFF;

    assert!(parse_packet(&buf[..len]).is_none());
}

#[test]
fn test_parse_len_exceeds_max() {
    let mut buf = [0u8; MAX_PACKET_SIZE];
    let len = build_packet_raw(&mut buf, CmdId::REMOTE_STOP, 0, None);
    buf[OFF_LEN] = (MAX_PAYLOAD as u8) + 1;

    assert!(parse_packet(&buf[..len]).is_none());
}

#[test]
fn test_parse_rawlen_too_short() {
    let mut buf = [0u8; MAX_PACKET_SIZE];
    let payload = [0x01u8];
    let len = build_packet_raw(&mut buf, CmdId::REMOTE_STOP, 0, Some(&payload));

    assert!(parse_packet(&buf[..len - 1]).is_none());
}

#[test]
fn test_parse_minimum_rawlen() {
    let buf = [0u8; 5];
    assert!(parse_packet(&buf).is_none());
}

// ── CommandDispatcher tests ────────────────────────────────

#[derive(Default)]
struct Capture {
    last_payload: [u8; MAX_PAYLOAD],
    last_payload_len: usize,
    call_count: i32,
    last_ctx: *mut c_void,
}

impl Capture {
    fn reset(&mut self) {
        *self = Self::default();
    }
}

fn capture_handler(ctx: *mut c_void, payload: *const u8, len: usize) {
    unsafe {
        let cap = &mut *(ctx as *mut Capture);
        cap.last_ctx = ctx;
        cap.last_payload_len = len;
        cap.last_payload = [0; MAX_PAYLOAD];
        if len > 0 && !payload.is_null() {
            let src = core::slice::from_raw_parts(payload, len);
            cap.last_payload[..len].copy_from_slice(src);
        }
        cap.call_count += 1;
    }
}

#[test]
fn test_dispatcher_registers_and_dispatches() {
    let mut cap = Capture::default();
    cap.reset();

    let mut d = CommandDispatcher::new();
    let mut rx = MockReceiver::new();

    let cap_ptr: *mut c_void = (&mut cap as *mut Capture).cast();
    d.on(CmdId::REMOTE_STOP, capture_handler as CmdHandler, cap_ptr);

    let mut buf = [0u8; MAX_PACKET_SIZE];
    let payload = [0x01u8];
    let len = build_packet_raw(&mut buf, CmdId::REMOTE_STOP, 1, Some(&payload));
    rx.push(&buf, len);

    d.poll(&mut rx);

    assert_eq!(1, cap.call_count);
    assert_eq!(cap_ptr, cap.last_ctx);
    assert_eq!(1, cap.last_payload_len);
    assert_eq!(0x01, cap.last_payload[0]);
}

#[test]
fn test_dispatcher_unknown_cmd_dropped() {
    let mut cap = Capture::default();
    cap.reset();

    let mut d = CommandDispatcher::new();
    let mut rx = MockReceiver::new();

    let cap_ptr: *mut c_void = (&mut cap as *mut Capture).cast();
    d.on(CmdId::REMOTE_STOP, capture_handler as CmdHandler, cap_ptr);

    let mut buf = [0u8; MAX_PACKET_SIZE];
    let payload = [0u8; 8];
    let len = build_packet_raw(&mut buf, CmdId::TARGET_UPDATE, 1, Some(&payload));
    rx.push(&buf, len);

    d.poll(&mut rx);

    assert_eq!(0, cap.call_count);
}

#[test]
fn test_dispatcher_seq_dedup() {
    let mut cap = Capture::default();
    cap.reset();

    let mut d = CommandDispatcher::new();
    let mut rx = MockReceiver::new();

    let cap_ptr: *mut c_void = (&mut cap as *mut Capture).cast();
    d.on(CmdId::REMOTE_STOP, capture_handler as CmdHandler, cap_ptr);

    let mut buf = [0u8; MAX_PACKET_SIZE];
    let payload = [0x01u8];
    let len = build_packet_raw(&mut buf, CmdId::REMOTE_STOP, 5, Some(&payload));
    rx.push(&buf, len);
    rx.push(&buf, len);

    d.poll(&mut rx);

    assert_eq!(1, cap.call_count);
}

#[test]
fn test_dispatcher_first_packet_seq_zero() {
    let mut cap = Capture::default();
    cap.reset();

    let mut d = CommandDispatcher::new();
    let mut rx = MockReceiver::new();

    let cap_ptr: *mut c_void = (&mut cap as *mut Capture).cast();
    d.on(CmdId::REMOTE_STOP, capture_handler as CmdHandler, cap_ptr);

    let mut buf = [0u8; MAX_PACKET_SIZE];
    let payload = [0x01u8];
    let len = build_packet_raw(&mut buf, CmdId::REMOTE_STOP, 0, Some(&payload));
    rx.push(&buf, len);

    d.poll(&mut rx);

    assert_eq!(1, cap.call_count);
}

#[test]
fn test_dispatcher_different_seq_accepted() {
    let mut cap = Capture::default();
    cap.reset();

    let mut d = CommandDispatcher::new();
    let mut rx = MockReceiver::new();

    let cap_ptr: *mut c_void = (&mut cap as *mut Capture).cast();
    d.on(CmdId::REMOTE_STOP, capture_handler as CmdHandler, cap_ptr);

    let mut buf = [0u8; MAX_PACKET_SIZE];
    let payload = [0x01u8];

    let len1 = build_packet_raw(&mut buf, CmdId::REMOTE_STOP, 1, Some(&payload));
    rx.push(&buf, len1);

    let len2 = build_packet_raw(&mut buf, CmdId::REMOTE_STOP, 2, Some(&payload));
    rx.push(&buf, len2);

    d.poll(&mut rx);

    assert_eq!(2, cap.call_count);
}

#[test]
fn test_dispatcher_max_drain_per_poll() {
    let mut cap = Capture::default();
    cap.reset();

    let mut d = CommandDispatcher::new();
    let mut rx = MockReceiver::new();

    let cap_ptr: *mut c_void = (&mut cap as *mut Capture).cast();
    d.on(CmdId::REMOTE_STOP, capture_handler as CmdHandler, cap_ptr);

    let mut buf = [0u8; MAX_PACKET_SIZE];
    let payload = [0x01u8];

    for seq in 1u8..=5 {
        let len = build_packet_raw(&mut buf, CmdId::REMOTE_STOP, seq, Some(&payload));
        rx.push(&buf, len);
    }

    d.poll(&mut rx);
    assert_eq!(3, cap.call_count);

    cap.reset();
    d.poll(&mut rx);
    assert_eq!(2, cap.call_count);
}

#[test]
fn test_dispatcher_overwrite_handler() {
    struct Counts {
        first: i32,
        second: i32,
    }

    fn first(ctx: *mut c_void, _payload: *const u8, _len: usize) {
        unsafe {
            let c = &mut *(ctx as *mut Counts);
            c.first += 1;
        }
    }

    fn second(ctx: *mut c_void, _payload: *const u8, _len: usize) {
        unsafe {
            let c = &mut *(ctx as *mut Counts);
            c.second += 1;
        }
    }

    let mut counts = Counts { first: 0, second: 0 };

    let mut d = CommandDispatcher::new();
    let mut rx = MockReceiver::new();

    let ctx_ptr: *mut c_void = (&mut counts as *mut Counts).cast();
    d.on(CmdId::REMOTE_STOP, first as CmdHandler, ctx_ptr);
    d.on(CmdId::REMOTE_STOP, second as CmdHandler, ctx_ptr);

    let mut buf = [0u8; MAX_PACKET_SIZE];
    let payload = [0x01u8];
    let len = build_packet_raw(&mut buf, CmdId::REMOTE_STOP, 1, Some(&payload));
    rx.push(&buf, len);

    d.poll(&mut rx);

    assert_eq!(0, counts.first);
    assert_eq!(1, counts.second);
}

#[test]
fn test_dispatcher_invalid_packet_skipped() {
    let mut cap = Capture::default();
    cap.reset();

    let mut d = CommandDispatcher::new();
    let mut rx = MockReceiver::new();

    let cap_ptr: *mut c_void = (&mut cap as *mut Capture).cast();
    d.on(CmdId::REMOTE_STOP, capture_handler as CmdHandler, cap_ptr);

    let bad = [0xFF, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x00];
    rx.push(&bad, bad.len());

    let mut buf = [0u8; MAX_PACKET_SIZE];
    let payload = [0x01u8];
    let len = build_packet_raw(&mut buf, CmdId::REMOTE_STOP, 1, Some(&payload));
    rx.push(&buf, len);

    d.poll(&mut rx);

    assert_eq!(1, cap.call_count);
}

#[test]
fn test_dispatcher_no_data_noop() {
    let mut cap = Capture::default();
    cap.reset();

    let mut d = CommandDispatcher::new();
    let mut rx = MockReceiver::new();

    let cap_ptr: *mut c_void = (&mut cap as *mut Capture).cast();
    d.on(CmdId::REMOTE_STOP, capture_handler as CmdHandler, cap_ptr);
    d.poll(&mut rx);

    assert_eq!(0, cap.call_count);
}

// ── read_f32_le tests ───────────────────────────────────────

#[test]
fn test_read_float_alignment() {
    let mut buf = [0u8; 9];
    let val: f32 = 3.14;
    buf[1..5].copy_from_slice(&val.to_le_bytes());

    assert!((read_f32_le(&buf[1..]) - 3.14).abs() < 0.001);
}

#[test]
fn test_read_float_zero() {
    let buf = [0u8; 4];
    assert_eq!(0.0, read_f32_le(&buf));
}

#[test]
fn test_read_float_negative() {
    let val: f32 = -42.5;
    let buf = val.to_le_bytes();
    assert_eq!(-42.5, read_f32_le(&buf));
}

// ── SerialFramer tests (additional) ─────────────────────────

#[test]
fn test_framer_valid_frame() {
    let mut framer = SerialFramer::new();

    let mut buf = [0u8; MAX_PACKET_SIZE];
    let payload = [0x01u8];
    let len = build_packet_raw(&mut buf, CmdId::REMOTE_STOP, 7, Some(&payload));

    let mut framed_buf = [0u8; MAX_PACKET_SIZE];
    let mut got_len = None;
    for &b in &buf[..len] {
        if let Some(out_len) = framer.feed(b, &mut framed_buf) {
            got_len = Some(out_len);
        }
    }

    let out_len = got_len.expect("should frame");
    let pkt = parse_packet(&framed_buf[..out_len]).expect("framed packet should parse");
    assert_eq!(CmdId::REMOTE_STOP, pkt.cmd);
    assert_eq!(7, pkt.seq);
    assert_eq!(&[0x01], pkt.payload);
}

#[test]
fn test_framer_garbage_then_valid() {
    let mut framer = SerialFramer::new();

    let garbage = [0x00u8, 0x11, 0x22, 0x33];
    let mut framed_buf = [0u8; MAX_PACKET_SIZE];
    for &b in &garbage {
        assert!(framer.feed(b, &mut framed_buf).is_none());
    }

    let mut buf = [0u8; MAX_PACKET_SIZE];
    let len = build_packet_raw(&mut buf, CmdId::REMOTE_RESUME, 1, None);

    let mut got_len = None;
    for &b in &buf[..len] {
        if let Some(out_len) = framer.feed(b, &mut framed_buf) {
            got_len = Some(out_len);
        }
    }

    let out_len = got_len.expect("should frame");
    let pkt = parse_packet(&framed_buf[..out_len]).expect("framed packet should parse");
    assert_eq!(CmdId::REMOTE_RESUME, pkt.cmd);
}
