//! End-to-end test of the Phase 1 stack with **no hardware**: a daemon backed by
//! an in-memory `MockTransport`, a real Unix socket, and a real `RemoteDevice`
//! client. Proves the whole IPC path — framing, dispatch, type conversions, and
//! error mapping — works coherently.

use std::os::unix::net::UnixListener;
use std::sync::{Arc, Mutex};
use std::thread;

use aes67_client::RemoteDevice;
use aes67_config::{ControlApi, CsrMap, Device, Transport};
use aes67_transport::MockTransport;

fn test_map() -> CsrMap {
    CsrMap::from_csv(
        "\
csr_register,aes67_csr_scratch,0x90010060,1,rw
csr_register,aes67_csr_status,0x9001000c,1,ro
csr_register,aes67_csr_mac_addr_lo,0x9001001c,1,rw
csr_register,aes67_csr_mac_addr_hi,0x90010020,1,rw
csr_register,aes67_csr_ip_addr,0x90010024,1,rw
csr_register,aes67_csr_reset,0x90010094,1,rw
",
    )
    .unwrap()
}

#[test]
fn daemon_client_round_trip() {
    let sock = std::env::temp_dir().join(format!("aes67d-e2e-{}.sock", std::process::id()));
    let _ = std::fs::remove_file(&sock);
    let listener = UnixListener::bind(&sock).unwrap();

    let transport: Box<dyn Transport + Send> = Box::new(MockTransport::new());
    let device = Arc::new(Mutex::new(Device::new(transport, test_map())));
    // No persistence or discovery in this test (no config file / network).
    let server = aes67_daemon::Server::new(device, None, None);

    // Serve connections in the background (detached; the harness reaps it).
    let srv = Arc::clone(&server);
    thread::spawn(move || {
        for conn in listener.incoming() {
            match conn {
                Ok(s) => {
                    let _ = aes67_daemon::serve_connection(s, Arc::clone(&srv));
                }
                Err(_) => break,
            }
        }
    });

    let mut client = RemoteDevice::connect(&sock).expect("connect + handshake");

    // Register write/read round trip.
    client.write_register("aes67_csr_scratch", 0xdead_beef).unwrap();
    assert_eq!(client.read_register("aes67_csr_scratch").unwrap(), 0xdead_beef);

    // MAC round trip (exercises the lo/hi split end-to-end over IPC).
    let mac = [0x02, 0x00, 0x00, 0x12, 0x34, 0x56];
    client.set_mac(mac).unwrap();
    assert_eq!(client.get_mac().unwrap(), mac);

    // IP round trip (string on the wire, Ipv4Addr at the edges).
    let ip = "192.168.1.42".parse().unwrap();
    client.set_ip(ip).unwrap();
    assert_eq!(client.get_ip().unwrap(), ip);

    // Register listing comes back over the wire.
    let regs = client.list_registers().unwrap();
    assert!(regs.iter().any(|r| r.name == "aes67_csr_scratch"));

    // Error mapping: unknown register -> typed error.
    assert!(client.read_register("does_not_exist").is_err());
    // Read-only register write is rejected.
    assert!(client.write_register("aes67_csr_status", 1).is_err());

    let _ = std::fs::remove_file(&sock);
}
