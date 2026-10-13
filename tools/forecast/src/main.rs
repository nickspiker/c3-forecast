//! The forecast ribbon for the ESP32-C3 1.47" LCD: renders it and pushes it to the board over Wi-Fi.
//!
//! The ribbon: eight days running down the 172×320 panel, the last 24 h above a "now" line and 7 days ahead below it, rolling upward one pixel row per period (~37.6 min). Temperature (green), wind (red, log 1–64 mph) and relative humidity (blue) are solid fills from the left, added together; temperature (°F) and humidity (%) share one 0–100 axis, labelled in yellow along the top. Day names in the left margin, at local midnights (daylight-saving changes included).
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
    let yellow = [1.0, 0.8, 0.0];
    for v in (20..100).step_by(20) {
        text(&mut light, font, &v.to_string(), (MARGIN as f64 + v as f64 / 100.0 * lane as f64, RULER as f64 - 8.0), 11.0, 0.5, yellow);
    }
    // Local midnights from the day before the window opens: a tick at each, the weekday centred on its day.
    let y_of = |t: f64| RULER as f64 + ((t - now) / 3600.0 + PAST_H) * PX_PER_H;
    let mut day = Local.timestamp_opt((now - PAST_H * 3600.0) as i64, 0).unwrap().date_naive();
    let midnight = |d: chrono::NaiveDate| Local.from_local_datetime(&d.and_hms_opt(0, 0, 0).unwrap()).earliest().unwrap().timestamp() as f64;
    let mut ticks = Vec::new();
    while y_of(midnight(day)) < H as f64 {
        let (y0, y1) = (y_of(midnight(day)), y_of(midnight(day.succ_opt().unwrap())));
        if y0 >= RULER as f64 {
            ticks.push(y0.round() as usize);
        }
        if y1.min(H as f64) - y0.max(RULER as f64) > 12.0 {
            text(&mut light, font, &day.format("%a").to_string(), (MARGIN as f64 - 7.0, (y0.max(RULER as f64) + y1.min(H as f64)) / 2.0), 11.0, 1.0, [0.55, 0.55, 0.6]);
        }
        day = day.succ_opt().unwrap();
    }

    let mut code: Vec<[u16; 3]> = light.iter().map(|p| [0, 1, 2].map(|c| enc.encode(c, p[c].clamp(0.0, 1.0)))).collect();
    // Ruler ticks every 10 (longer at the labelled 20s) in yellow and the day ticks in grey: solid, on the codes.
    let grid_x = |v: usize| MARGIN + (v as f64 / 100.0 * lane as f64 - 0.5).round() as usize;
    for v in (10..100).step_by(10) {
        for y in RULER - if v % 20 == 0 { 4 } else { 2 }..RULER {
            code[y * W + grid_x(v)] = [31, 50, 0];
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

/// Authenticate with the shared key and send one frame; the board checks its BLAKE3 before drawing it.
fn push(data: &[u8]) -> Result<(), String> {
    let key: [u8; 32] = hex(std::fs::read_to_string(KEY).map_err(|e| e.to_string())?.trim()).ok_or("bad key file")?;
    let stream = TcpStream::connect(HOST).map_err(|e| format!("connect: {e}"))?;
    stream.set_read_timeout(Some(Duration::from_secs(30))).ok();
    let mut reader = BufReader::new(stream.try_clone().map_err(|e| e.to_string())?);
    let mut w = stream;
    let mut line = String::new();
    reader.read_line(&mut line).map_err(|e| e.to_string())?;
    let nonce: [u8; 32] = hex(line.split_whitespace().nth(1).ok_or("no challenge")?).ok_or("bad challenge")?;
    writeln!(w, "auth {}", blake3::keyed_hash(&key, &nonce).to_hex()).map_err(|e| e.to_string())?;
    line.clear();
    reader.read_line(&mut line).map_err(|e| e.to_string())?;
    if !line.starts_with("ok") {
        return Err("auth refused".into());
    }
    w.write_all(format!("d {}\n", blake3::hash(data).to_hex()).as_bytes()).and_then(|_| w.write_all(data)).map_err(|e| e.to_string())?;
    line.clear();
    reader.read_line(&mut line).map_err(|e| e.to_string())?;
    if line.starts_with("ok") { Ok(()) } else { Err(line.trim().to_string()) }
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
    if arg.as_deref() == Some("run") {
        let mut forecast: Option<(Forecast, f64)> = None;
        loop {
            let attempt = (|| -> Result<(), String> {
                if forecast.as_ref().is_none_or(|(_, at)| unix() - at > 3600.0) {
                    forecast = Some((fetch()?, unix()));
                }
                push(&to_bytes(&render(&forecast.as_ref().unwrap().0, snapped(), &enc, &font)))
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
    let code = render(&fetch().expect("forecast"), snapped(), &enc, &font);
    if arg.as_deref() == Some("--preview") {
        std::fs::create_dir_all("/tmp/forecast-preview").unwrap();
        preview(&code, "/tmp/forecast-preview/panel.png");
        println!("/tmp/forecast-preview/panel.png");
    } else {
        push(&to_bytes(&code)).expect("push");
        println!("pushed");
    }
}
