//! The forecast ribbon for the ESP32-C3 1.47" LCD: renders it and pushes it to the board over Wi-Fi.
//!
//! The ribbon: eight days running down the 172×320 panel, the last 24 h above a "now" line and 7 days ahead below it, rolling upward one pixel row per period (~37.6 min). Temperature (green), wind (red, log 1–64 mph) and relative humidity (blue) are solid fills from the left, added together; temperature (°F) and humidity (%) share one 0–100 axis, labelled along the top in cyan (their colours together). Day names in the left margin below the now line, at local midnights (daylight-saving changes included). Above it, the margin reads top down: the board's Wi-Fi signal as one dozenal digit (red Zil at −80 dBm or worse, through yellow, to green Stelor at −40 dBm or better), drawn by the board itself from cells sent with `g`, then humidity, temperature and wind now in their fills' colours.
//!
//! Everything is composited in linear light, fills, anti-aliased edges and text alike, then encoded to RGB565 through the panel's measured per-channel response (measurements/i1pro-2026-10-12/response.json, γ ≈ 2.78). The gridlines (every 10 on the 0–100 axis) and the now line are then applied to the codes themselves: (3v + !v) >> 2 and v ^ max.
//!
//! Usage:
//!   forecast [--preview]   render and push once (--preview: write /tmp/forecast-preview/panel.png instead)
//!   forecast run           push at every period boundary, refetching the forecast hourly

use std::io::{BufRead, BufReader, Write};
use std::net::TcpStream;
use std::path::{Path, PathBuf};
use std::time::{Duration, SystemTime, UNIX_EPOCH};

use ab_glyph::{Font, FontRef, PxScale, ScaleFont, point};
use chrono::{Local, TimeZone};

const LAT: f64 = 47.5126;
const LON: f64 = -122.5054;
const W: usize = 172;
const H: usize = 320;
const PAST_H: f64 = 24.0;
const FUTURE_H: f64 = 7.0 * 24.0;
const MARGIN: usize = 30;
const RULER: usize = 14;
const TEMP_F: (f64, f64) = (0.0, 100.0);
/// Wind on a log scale: 1 to 64 mph, six equal doublings; calm is empty.
const WIND_LOG2_MPH: (f64, f64) = (0.0, 6.0);
const KEY: &str = "/mnt/Harbor/Code/keys/forecast.key";
const HOST: &str = "forecast.local:7766";
const PX_PER_H: f64 = (H - RULER) as f64 / (PAST_H + FUTURE_H);
/// Each period moves the ribbon up exactly one pixel row.
const PERIOD_S: f64 = 3600.0 / PX_PER_H;
const TOP: [u16; 3] = [31, 63, 31];
/// The signal cell at the top-left, as the board has it: the margin's width, one reading row high.
const CELL_W: usize = MARGIN;
const CELL_H: usize = 13;

fn repo() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("../..")
}

/// Hourly forecast, unix seconds and values (NaN where missing).
struct Forecast {
    t: Vec<f64>,
    temp: Vec<f64>,
    wind: Vec<f64>,
    humidity: Vec<f64>,
}

fn fetch() -> Result<Forecast, String> {
    let url = format!(
        "https://api.open-meteo.com/v1/forecast?latitude={LAT}&longitude={LON}&hourly=temperature_2m,wind_speed_10m,relative_humidity_2m&temperature_unit=fahrenheit&wind_speed_unit=mph&timeformat=unixtime&timezone=GMT&past_days=1&forecast_days=8"
    );
    let body = ureq::get(&url).call().map_err(|e| e.to_string())?.body_mut().read_to_string().map_err(|e| e.to_string())?;
    let j: serde_json::Value = serde_json::from_str(&body).map_err(|e| e.to_string())?;
    let col = |k: &str| -> Result<Vec<f64>, String> { Ok(j["hourly"][k].as_array().ok_or(format!("no {k}"))?.iter().map(|v| v.as_f64().unwrap_or(f64::NAN)).collect()) };
    Ok(Forecast { t: col("time")?, temp: col("temperature_2m")?, wind: col("wind_speed_10m")?, humidity: col("relative_humidity_2m")? })
}

/// Linear interpolation of `ys` at `x` over increasing `xs`, held at the ends.
fn interp(xs: &[f64], ys: &[f64], x: f64) -> f64 {
    let i = xs.partition_point(|&v| v <= x).clamp(1, xs.len() - 1);
    let f = ((x - xs[i - 1]) / (xs[i] - xs[i - 1])).clamp(0.0, 1.0);
    ys[i - 1] + (ys[i] - ys[i - 1]) * f
}

/// Linear light (fraction of full drive) → RGB565 levels, nearest in light, through the panel's measured response at each 5- or 6-bit level's bit-replicated code.
struct Encoder {
    levels: [Vec<f64>; 3],
}

impl Encoder {
    fn load() -> Encoder {
        let j: serde_json::Value = serde_json::from_str(&std::fs::read_to_string(repo().join("measurements/i1pro-2026-10-12/response.json")).expect("response.json")).expect("response JSON");
        let k = |c: usize, code: u32| j["k"][c][code as usize].as_f64().unwrap();
        let levels = [(0, 5), (1, 6), (2, 5)].map(|(c, bits)| (0u32..1 << bits).map(|v| k(c, (v << (8 - bits)) | (v >> (2 * bits - 8)))).collect());
        Encoder { levels }
    }

    fn encode(&self, c: usize, light: f64) -> u16 {
        let lv = &self.levels[c];
        let i = lv.partition_point(|&v| v < light).clamp(1, lv.len() - 1);
        if light - lv[i - 1] < lv[i] - light { (i - 1) as u16 } else { i as u16 }
    }
}

/// Text in Oxanium, as coverage composited in linear light over `light`. `anchor_x`: 0 left, 0.5 centre, 1 right; vertically centred on `y`.
fn text(light: &mut [[f64; 3]], font: &FontRef, s: &str, (x, y): (f64, f64), size: f32, anchor_x: f64, colour: [f64; 3]) {
    // `size` is the em in pixels (as PIL and CSS mean it); ab_glyph scales by ascent-to-descent height instead.
    let scale = PxScale::from(size * font.height_unscaled() / font.units_per_em().unwrap_or(1000.0));
    let sf = font.as_scaled(scale);
    let width: f32 = s.chars().map(|ch| sf.h_advance(font.glyph_id(ch))).sum();
    let baseline = y as f32 + (sf.ascent() + sf.descent()) / 2.0;
    let mut pen = x as f32 - width * anchor_x as f32;
    for ch in s.chars() {
        let id = font.glyph_id(ch);
        if let Some(outline) = font.outline_glyph(id.with_scale_and_position(scale, point(pen, baseline))) {
            let b = outline.px_bounds();
            outline.draw(|gx, gy, a| {
                let (px, py) = (b.min.x as i32 + gx as i32, b.min.y as i32 + gy as i32);
                if (0..W as i32).contains(&px) && (0..H as i32).contains(&py) {
                    let p = &mut light[py as usize * W + px as usize];
                    let a = a.clamp(0.0, 1.0) as f64;
                    for c in 0..3 {
                        p[c] = p[c] * (1.0 - a) + colour[c] * a;
                    }
                }
            });
        }
        pen += sf.h_advance(id);
    }
}

/// The signal-strength digit for `rssi` (dBm), as the boards work it out: Zil at −80 or worse (or not connected) up to Stelor at −40 or better, evenly spaced in dB.
fn signal_digit(rssi: Option<i32>) -> u8 {
    rssi.map_or(0, |r| (((r + 80).clamp(0, 40) * 11 + 20) / 40) as u8)
}

/// The signal digit's colour, linear light: red at Zil, through yellow (red and green both full) at the middle, to green at Stelor.
fn signal_colour(digit: u8) -> [f64; 3] {
    let d = digit as f64 / 11.0;
    [(2.0 * (1.0 - d)).min(1.0), (2.0 * d).min(1.0), 0.0]
}

/// The readings' rows in the margin above the now line: row `i`'s right-hand end and centre line. Row 0 is the signal cell.
fn reading_at(i: usize) -> (f64, f64) {
    (MARGIN as f64 - 2.0, CELL_H as f64 * (i as f64 + 0.5))
}

/// The cells the board draws over the frame's top-left corner for its own signal (its `g`): each dozenal digit Zil..Stelor in its colour on black, CELL_W × CELL_H, in digit order, rendered like the rest of the ribbon.
fn signal_cells(enc: &Encoder, font: &FontRef) -> Vec<[u16; 3]> {
    let mut out = Vec::with_capacity(12 * CELL_W * CELL_H);
    for digit in 0..12u8 {
        let mut light = vec![[0f64; 3]; W * H];
        text(&mut light, font, &char::from_u32(0x10 + digit as u32).unwrap().to_string(), reading_at(0), 11.0, 1.0, signal_colour(digit));
        for y in 0..CELL_H {
            out.extend(light[y * W..y * W + CELL_W].iter().map(|p| [0, 1, 2].map(|c| enc.encode(c, p[c].clamp(0.0, 1.0)))));
        }
    }
    out
}

/// The ribbon as RGB565 levels (red 0–31, green 0–63, blue 0–31), for `now` (unix seconds) already snapped to a period boundary.
fn render(f: &Forecast, now: f64, enc: &Encoder, font: &FontRef) -> Vec<[u16; 3]> {
    let lane = W - MARGIN;
    let mut light = vec![[0f64; 3]; W * H];
    // Area coverage: each pixel's share of the region left of the edge, sampled at SUB points down its row, so an edge sweeping sideways is smoothed between rows as well as across them.
    const SUB: usize = 16;
    for row in 0..H - RULER {
        for sub in 0..SUB {
            let t = now - PAST_H * 3600.0 + (row as f64 + (sub as f64 + 0.5) / SUB as f64) / PX_PER_H * 3600.0;
            let frac = [
                interp(&f.t, &f.wind, t).max(1e-3).log2() / WIND_LOG2_MPH.1,
                (interp(&f.t, &f.temp, t) - TEMP_F.0) / (TEMP_F.1 - TEMP_F.0),
                interp(&f.t, &f.humidity, t) / 100.0,
            ];
            for (c, v) in frac.iter().enumerate() {
                let edge = if v.is_nan() { 0.0 } else { v.clamp(0.0, 1.0) * lane as f64 };
                for x in 0..lane {
                    light[(RULER + row) * W + MARGIN + x][c] += (edge - x as f64).clamp(0.0, 1.0) / SUB as f64;
                }
            }
        }
    }
    // Cyan, green and blue together: the axis belongs to temperature and humidity.
    let cyan = [0.0, 0.8, 1.0];
    for v in (20..100).step_by(20) {
        text(&mut light, font, &v.to_string(), (MARGIN as f64 + v as f64 / 100.0 * lane as f64, RULER as f64 - 8.0), 11.0, 0.5, cyan);
    }
    // Local midnights from the day before the window opens: a tick at each, the weekday centred on its day.
    let y_of = |t: f64| RULER as f64 + ((t - now) / 3600.0 + PAST_H) * PX_PER_H;
    let mut day = Local.timestamp_opt((now - PAST_H * 3600.0) as i64, 0).unwrap().date_naive();
    let midnight = |d: chrono::NaiveDate| Local.from_local_datetime(&d.and_hms_opt(0, 0, 0).unwrap()).earliest().unwrap().timestamp() as f64;
    // Day ticks and names only below the now line: the margin above it holds the readings.
    let top = y_of(now) + 1.0;
    let mut ticks = Vec::new();
    while y_of(midnight(day)) < H as f64 {
        let (y0, y1) = (y_of(midnight(day)), y_of(midnight(day.succ_opt().unwrap())));
        if y0 > y_of(now) {
            ticks.push(y0.round() as usize);
        }
        if y1.min(H as f64) - y0.max(top) > 12.0 {
            text(&mut light, font, &day.format("%a").to_string(), (MARGIN as f64 - 7.0, (y0.max(top) + y1.min(H as f64)) / 2.0), 11.0, 1.0, [0.55, 0.55, 0.6]);
        }
        day = day.succ_opt().unwrap();
    }
    // The margin above the now line, top down: a row left black for the board's signal cell (`signal_cells`), then humidity, temperature and wind now, each in its fill's colour.
    let now_at = |ys: &[f64]| interp(&f.t, ys, now);
    let readings = [
        (format!("{:.0}%", now_at(&f.humidity)), [0.0, 0.0, 1.0]),
        (format!("{:.0}°", now_at(&f.temp)), [0.0, 1.0, 0.0]),
        (format!("{:.0}", now_at(&f.wind)), [1.0, 0.0, 0.0]),
    ];
    for (i, (s, colour)) in readings.iter().enumerate() {
        text(&mut light, font, s, reading_at(i + 1), 11.0, 1.0, *colour);
    }

    let mut code: Vec<[u16; 3]> = light.iter().map(|p| [0, 1, 2].map(|c| enc.encode(c, p[c].clamp(0.0, 1.0)))).collect();
    // Ruler ticks every 10 (longer at the labelled 20s) in cyan and the day ticks in grey: solid, on the codes.
    let grid_x = |v: usize| MARGIN + (v as f64 / 100.0 * lane as f64 - 0.5).round() as usize;
    for v in (10..100).step_by(10) {
        for y in RULER - if v % 20 == 0 { 4 } else { 2 }..RULER {
            code[y * W + grid_x(v)] = [0, 50, 31];
        }
    }
    for &y in ticks.iter().filter(|&&y| y < H) {
        for x in MARGIN - 6..MARGIN - 1 {
            code[y * W + x] = [18, 38, 21];
        }
    }
    // Gridlines every 10 on the 0–100 axis: (3v + !v) >> 2, a quarter of the way to the inverse, on the codes.
    for v in (10..100).step_by(10) {
        let x = grid_x(v);
        for y in RULER..H {
            let p = &mut code[y * W + x];
            for c in 0..3 {
                p[c] = (3 * p[c] + (TOP[c] - p[c])) >> 2;
            }
        }
    }
    // Now: the row XORed outright.
    let yn = RULER + (PAST_H * PX_PER_H).round() as usize;
    for p in &mut code[yn * W..(yn + 1) * W] {
        for c in 0..3 {
            p[c] ^= TOP[c];
        }
    }
    code
}

fn to_bytes(code: &[[u16; 3]]) -> Vec<u8> {
    code.iter().flat_map(|p| (p[0] << 11 | p[1] << 5 | p[2]).to_be_bytes()).collect()
}

/// What the codes look like, approximately, on an ordinary screen (each level's 8-bit replicated code), at 3×.
fn preview(code: &[[u16; 3]], path: &str) {
    const Z: usize = 3;
    let mut rgb = vec![0u8; W * Z * H * Z * 3];
    for y in 0..H * Z {
        for x in 0..W * Z {
            let p = code[(y / Z) * W + x / Z];
            let v = [(p[0] << 3 | p[0] >> 2) as u8, (p[1] << 2 | p[1] >> 4) as u8, (p[2] << 3 | p[2] >> 2) as u8];
            rgb[(y * W * Z + x) * 3..][..3].copy_from_slice(&v);
        }
    }
    let mut e = png::Encoder::new(std::io::BufWriter::new(std::fs::File::create(path).unwrap()), (W * Z) as u32, (H * Z) as u32);
    e.set_color(png::ColorType::Rgb);
    e.set_depth(png::BitDepth::Eight);
    e.write_header().unwrap().write_image_data(&rgb).unwrap();
}

/// A connection to the board, past its challenge.
struct Board {
    reader: BufReader<TcpStream>,
    w: TcpStream,
}

impl Board {
    /// Connect and answer the challenge with the shared key.
    fn connect() -> Result<Board, String> {
        let key: [u8; 32] = hex(std::fs::read_to_string(KEY).map_err(|e| e.to_string())?.trim()).ok_or("bad key file")?;
        let stream = TcpStream::connect(HOST).map_err(|e| format!("connect: {e}"))?;
        stream.set_read_timeout(Some(Duration::from_secs(30))).ok();
        let mut board = Board { reader: BufReader::new(stream.try_clone().map_err(|e| e.to_string())?), w: stream };
        let line = board.line()?;
        let nonce: [u8; 32] = hex(line.split_whitespace().nth(1).ok_or("no challenge")?).ok_or("bad challenge")?;
        writeln!(board.w, "auth {}", blake3::keyed_hash(&key, &nonce).to_hex()).map_err(|e| e.to_string())?;
        if !board.line()?.starts_with("ok") {
            return Err("auth refused".into());
        }
        Ok(board)
    }

    fn line(&mut self) -> Result<String, String> {
        let mut line = String::new();
        self.reader.read_line(&mut line).map_err(|e| e.to_string())?;
        Ok(line)
    }

    /// The board's `info`: its Wi-Fi signal (dBm; None when it isn't connected) and the first 16 hex digits of its signal cells' BLAKE3 (`-` for none).
    fn info(&mut self) -> Result<(Option<i32>, String), String> {
        writeln!(self.w, "info").map_err(|e| e.to_string())?;
        let line = self.line()?;
        let field = |k: &str| line.split_whitespace().skip_while(|&w| w != k).nth(1).map(str::to_string);
        Ok((field("rssi").and_then(|v| v.parse().ok()).filter(|&r| r != 0), field("cells").unwrap_or_default()))
    }

    /// Send `op hash` and `data`; the board checks the BLAKE3 before taking it.
    fn send(&mut self, op: &str, data: &[u8]) -> Result<(), String> {
        self.w.write_all(format!("{op} {}\n", blake3::hash(data).to_hex()).as_bytes()).and_then(|_| self.w.write_all(data)).map_err(|e| e.to_string())?;
        let line = self.line()?;
        if line.starts_with("ok") { Ok(()) } else { Err(line.trim().to_string()) }
    }

    /// Send the signal cells, unless the board holds these already. Whether they were sent.
    fn cells(&mut self, data: &[u8]) -> Result<bool, String> {
        if self.info()?.1 == blake3::hash(data).to_hex()[..16] {
            return Ok(false);
        }
        self.send("g", data).map(|_| true)
    }
}

fn hex(s: &str) -> Option<[u8; 32]> {
    let b = s.as_bytes();
    if b.len() != 64 {
        return None;
    }
    let mut out = [0u8; 32];
    for (i, pair) in b.chunks_exact(2).enumerate() {
        out[i] = u8::from_str_radix(std::str::from_utf8(pair).ok()?, 16).ok()?;
    }
    Some(out)
}

fn unix() -> f64 {
    SystemTime::now().duration_since(UNIX_EPOCH).unwrap().as_secs_f64()
}

/// The start of the current period.
fn snapped() -> f64 {
    (unix() / PERIOD_S).floor() * PERIOD_S
}

fn stamp() -> String {
    Local::now().format("%Y-%m-%d %H:%M").to_string()
}

fn main() {
    let arg = std::env::args().nth(1);
    let enc = Encoder::load();
    let ttf = std::fs::read(repo().join("assets/font/Oxanium-Regular+glyphs.ttf")).expect("font");
    let font = FontRef::try_from_slice(&ttf).expect("parse font");
    let cells = signal_cells(&enc, &font);
    if arg.as_deref() == Some("run") {
        let mut forecast: Option<(Forecast, f64)> = None;
        loop {
            let attempt = (|| -> Result<(), String> {
                if forecast.as_ref().is_none_or(|(_, at)| unix() - at > 3600.0) {
                    forecast = Some((fetch()?, unix()));
                }
                let mut board = Board::connect()?;
                if board.cells(&to_bytes(&cells))? {
                    println!("{} signal cells sent", stamp());
                }
                board.send("d", &to_bytes(&render(&forecast.as_ref().unwrap().0, snapped(), &enc, &font)))
            })();
            match attempt {
                Ok(()) => println!("{} pushed", stamp()),
                Err(e) => {
                    println!("{} failed ({e}); retrying in a minute", stamp());
                    std::thread::sleep(Duration::from_secs(60));
                    continue;
                }
            }
            // To just past the next period boundary.
            std::thread::sleep(Duration::from_secs_f64(PERIOD_S - unix() % PERIOD_S + 2.0));
        }
    }
    let mut code = render(&fetch().expect("forecast"), snapped(), &enc, &font);
    if arg.as_deref() == Some("--preview") {
        // With the cell the board would draw over it now, if it can be asked.
        let rssi = Board::connect().and_then(|mut b| b.info()).map_or(None, |(r, _)| r);
        println!("signal: {}", rssi.map_or("unknown".to_string(), |r| format!("{r} dBm")));
        let digit = signal_digit(rssi) as usize;
        for y in 0..CELL_H {
            code[y * W..y * W + CELL_W].copy_from_slice(&cells[(digit * CELL_H + y) * CELL_W..][..CELL_W]);
        }
        std::fs::create_dir_all("/tmp/forecast-preview").unwrap();
        preview(&code, "/tmp/forecast-preview/panel.png");
        println!("/tmp/forecast-preview/panel.png");
    } else {
        let mut board = Board::connect().expect("connect");
        if board.cells(&to_bytes(&cells)).expect("signal cells") {
            println!("signal cells sent");
        }
        board.send("d", &to_bytes(&code)).expect("push");
        println!("pushed");
    }
}
