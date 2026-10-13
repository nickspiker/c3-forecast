//! Wi-Fi: join the stored network as an ordinary station, take an address by DHCP (asking for the name `forecast`), answer mDNS for `forecast.local`, and serve the command protocol over TCP on port 7766.

use core::net::{Ipv4Addr, Ipv6Addr};
use core::sync::atomic::{AtomicU32, Ordering};

use edge_mdns::buf::VecBufAccess;
use edge_mdns::domain::base::Ttl;
use edge_mdns::host::Host;
use edge_mdns::io::{self, IPV4_DEFAULT_SOCKET};
use edge_mdns::HostAnswersMdnsHandler;
use edge_nal::UdpSplit;
use edge_nal_embassy::{Udp, UdpBuffers};
use embassy_executor::Spawner;
use embassy_net::tcp::TcpSocket;
use embassy_net::{DhcpConfig, Runner, Stack, StackResources};
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::signal::Signal;
use embassy_time::{Duration, Timer};
use esp_hal::peripherals::WIFI;
use esp_hal::rng::Rng;
use esp_println::println;
use esp_radio::wifi::sta::StationConfig;
use esp_radio::wifi::{AuthenticationMethodConfig, Config, Interface, WifiController};
use static_cell::StaticCell;

use crate::{Conn, Io, Shared, serve};

pub const HOSTNAME: &str = "forecast";
pub const PORT: u16 = 7766;

/// The board's IPv4 address once DHCP has given it one (0 until then).
static ADDRESS: AtomicU32 = AtomicU32::new(0);

pub fn address() -> Option<Ipv4Addr> {
    match ADDRESS.load(Ordering::Relaxed) {
        0 => None,
        a => Some(Ipv4Addr::from(a)),
    }
}

/// Bring up Wi-Fi and spawn the tasks that keep it connected and serve on it.
pub fn start(spawner: &Spawner, wifi: WIFI<'static>, ssid: &str, password: &str, board: &'static Shared) {
    let mut controller = WifiController::new(wifi, Default::default()).expect("Wi-Fi init");
    let (Ok(ssid), Ok(password)) = (ssid.try_into(), password.try_into()) else {
        println!("wifi: stored network name or password is invalid");
        return;
    };
    // WPA2 is the minimum accepted; WPA2/WPA3 networks work too.
    let station = StationConfig::default().with_ssid(ssid).with_authentication(AuthenticationMethodConfig::Wpa2Personal(password));
    controller.set_config(&Config::Station(station)).expect("Wi-Fi config");

    let mut dhcp = DhcpConfig::default();
    dhcp.hostname = HOSTNAME.try_into().ok();
    let rng = Rng::new();
    let seed = (rng.random() as u64) << 32 | rng.random() as u64;
    static RESOURCES: StaticCell<StackResources<4>> = StaticCell::new();
    let (stack, runner) = embassy_net::new(Interface::station(), embassy_net::Config::dhcpv4(dhcp), RESOURCES.init(StackResources::new()), seed);

    spawner.spawn(connection(controller).expect("wifi task"));
    spawner.spawn(net_runner(runner).expect("net task"));
    spawner.spawn(mdns(stack).expect("mdns task"));
    spawner.spawn(tcp_server(stack, board).expect("tcp task"));
}

/// Keep the station connected: retry every few seconds after a failure or a drop.
#[embassy_executor::task]
async fn connection(mut controller: WifiController<'static>) {
    loop {
        match controller.connect_async().await {
            Ok(_) => {
                println!("wifi: connected");
                let _ = controller.wait_for_disconnect_async().await;
                println!("wifi: disconnected");
                ADDRESS.store(0, Ordering::Relaxed);
            }
            Err(e) => {
                // Just the reason: the whole error's Debug output costs ~2 KB of code.
                match e {
                    esp_radio::wifi::ConnectionError::Failed(info) => println!("wifi: connect failed ({:?}); retrying", info.reason),
                    _ => println!("wifi: connect failed; retrying"),
                }
                Timer::after(Duration::from_secs(5)).await;
            }
        }
    }
}

#[embassy_executor::task]
async fn net_runner(mut runner: Runner<'static, Interface>) {
    runner.run().await
}

/// Answer mDNS for `forecast.local`, restarting with the new address whenever DHCP hands out a different one.
#[embassy_executor::task]
async fn mdns(stack: Stack<'static>) {
    loop {
        stack.wait_config_up().await;
        let Some(config) = stack.config_v4() else { continue };
        let ip = config.address.address();
        ADDRESS.store(u32::from(ip), Ordering::Relaxed);
        println!("wifi: {ip} ({HOSTNAME}.local), port {PORT}");

        let buffers = UdpBuffers::<1, 1500, 1500, 2>::new();
        let udp = Udp::new(stack, &buffers);
        let Ok(mut socket) = io::bind(&udp, IPV4_DEFAULT_SOCKET, Some(Ipv4Addr::UNSPECIFIED), None).await else {
            Timer::after(Duration::from_secs(5)).await;
            continue;
        };
        let (recv, send) = socket.split();
        let host = Host { hostname: HOSTNAME, ipv4: ip, ipv6: Ipv6Addr::UNSPECIFIED, ttl: Ttl::from_secs(60) };
        let (recv_buf, send_buf) = (VecBufAccess::<NoopRawMutex, 1500>::new(), VecBufAccess::<NoopRawMutex, 1500>::new());
        let signal = Signal::<NoopRawMutex, ()>::new();
        let responder = io::Mdns::new(Some(Ipv4Addr::UNSPECIFIED), None, recv, send, &recv_buf, &send_buf, Rng::new(), &signal);
        // Runs until the socket fails (the link went down); then wait for an address again.
        let changed = async {
            loop {
                Timer::after(Duration::from_secs(10)).await;
                if stack.config_v4().map(|c| c.address.address()) != Some(ip) {
                    return;
                }
            }
        };
        let _ = embassy_futures::select::select(responder.run(HostAnswersMdnsHandler::new(&host)), changed).await;
    }
}

/// One client at a time on PORT; each connection runs the command protocol, challenge first.
#[embassy_executor::task]
async fn tcp_server(stack: Stack<'static>, board: &'static Shared) {
    let mut rx = [0u8; 8192];
    let mut tx = [0u8; 1024];
    loop {
        let mut socket = TcpSocket::new(stack, &mut rx, &mut tx);
        // A client that goes quiet for a minute is gone.
        socket.set_timeout(Some(Duration::from_secs(60)));
        if socket.accept(PORT).await.is_err() {
            continue;
        }
        let peer = socket.remote_endpoint();
        println!("tcp: client {peer:?}");
        let mut conn = Conn::new(Io::Tcp(&mut socket));
        let _ = serve(&mut conn, board, false).await;
        drop(conn);
        socket.close();
        let _ = socket.flush().await;
        socket.abort();
        println!("tcp: client gone");
    }
}
