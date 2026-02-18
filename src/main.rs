use clap::{Parser, Subcommand};
use configparser::ini::Ini;
use hound::WavReader;
use rustfft::{FftPlanner, num_complex::Complex};
use std::collections::{HashMap, HashSet};
use std::fs;
use std::ops::Add;
use std::path::PathBuf;
use svg::node::element::{Line, Rectangle, Text as SvgText};
use svg::Document;
use roxmltree::{Document as XmlDoc, ParsingOptions};

#[derive(Parser, Debug)]
#[command(author, version, about = "Kora Tab Generator - Pro Edition")]
struct Args {
    #[command(subcommand)]
    command: Commands,
}

#[derive(Subcommand, Debug)]
enum Commands {
    /// Converts a MusicXML file into a Kora tab.
    Draw {
        /// Path to the input MusicXML file.
        source: PathBuf,
        /// Path for the output file.
        output: PathBuf,
        /// Name of the tuning to use from tuning.ini.
        #[arg(short, long)]
        tuning: String,
        /// Output format.
        #[arg(long, default_value = "svg")]
        format: String,
    },
    /// Transcribes a WAV audio file into a Kora tab.
    Transcribe {
        /// Path to the input WAV file.
        source: PathBuf,
        /// Path for the output file.
        output: PathBuf,
        /// Name of the tuning to use from tuning.ini.
        #[arg(short, long)]
        tuning: String,
        /// Output format.
        #[arg(long, default_value = "svg")]
        format: String,
    },
}

#[derive(Debug, Copy, Clone, PartialEq, Eq, Hash)]
struct Pitch(i32);   // MIDI note number * 100

impl Add for Pitch {
    type Output = Pitch;
    fn add(self, other: Pitch) -> Pitch {
        Pitch(self.0 + other.0)
    }
}

#[derive(Debug, Clone)]
struct Note {
    pitch: Pitch,
    position: f64, // General-purpose position, time in seconds for audio, or beat count for XML
}

struct KoraTuning {
    name: String,
    left: Vec<Pitch>,
    right: Vec<Pitch>,
}

/// returns midi number * 100 + microtone adjustments in cents
/// for example "A4+30" gives 1200 * 5 + 900 + 30 = 6930
fn note_to_pitch(note: &str) -> Option<Pitch> {
    let note = note.trim();
    if note.is_empty() { return None; }

    let mut main_part = note;
    let mut cents: i32 = 0;

    if let Some(plus_idx) = note.rfind('+') {
        if let Some(val_str) = note.get(plus_idx + 1..) {
            if !val_str.is_empty() && val_str.chars().all(char::is_numeric) {
                main_part = &note[..plus_idx];
                cents = val_str.parse().unwrap_or(0);
            }
        }
    } else if let Some(minus_idx) = note.rfind('-') {
        if let Some(val_str) = note.get(minus_idx + 1..) {
            if !val_str.is_empty() && val_str.chars().all(char::is_numeric) {
                main_part = &note[..minus_idx];
                cents = -val_str.parse().unwrap_or(0);
            }
        }
    }

    let (name, octave_str) = if note.len() >= 2 && (&main_part[1..2] == "b" || &main_part[1..2] == "#") {
        (&main_part[0..2], &main_part[2..])
    } else {
        (&main_part[0..1], &main_part[1..])
    };
    let base: Option<i32> = match name {
        "C" => Some(0), 
        "C#" | "Db" => Some(1), 
        "D" => Some(2), 
        "D#" | "Eb" => Some(3), 
        "E" => Some(4),
        "F" => Some(5), 
        "F#" | "Gb" => Some(6), 
        "G" => Some(7), 
        "G#" | "Ab" => Some(8), 
        "A" => Some(9), 
        "A#" | "Bb" => Some(10), 
        "B" => Some(11),
        _ => None,
    };
    let octave: i32 = octave_str.parse().unwrap_or(4);
    
    base.map(|base| Pitch((octave + 1) * 1200 + 100 * base + cents))
}

fn draw_kora_system(mut doc: Document, y_off: f64, m_num: i32, margin: f64, width: f64) -> Document {
    let m_label = SvgText::new(format!("#{}", m_num))
        .set("x", margin - 45.0).set("y", y_off + 10.0)
        .set("font-family", "Arial").set("font-size", 11).set("fill", "#AAAAAA");
    doc = doc.add(m_label);

    for i in 0..11 {
        let y = y_off + ((10 - i) as f64 * 25.0); 
        doc = doc.add(Line::new()
            .set("x1", margin).set("y1", y)
            .set("x2", width - margin).set("y2", y)
            .set("stroke", "#E8E8E8").set("stroke-width", 1));
    }
    doc = doc.add(Line::new().set("x1", margin).set("y1", y_off).set("x2", margin).set("y2", y_off + 250.0).set("stroke", "#444").set("stroke-width", 2));
    doc = doc.add(Line::new().set("x1", width - margin).set("y1", y_off).set("x2", width - margin).set("y2", y_off + 250.0).set("stroke", "#444").set("stroke-width", 2));
    doc
}

fn draw_svg(
    output_path: &PathBuf,
    song_title: &str,
    tuning: &KoraTuning,
    notes: &[Note],
) -> Result<(), Box<dyn std::error::Error>> {
    let page_width = 750.0;
    let margin = 60.0;
    let system_height = 340.0; 
    let note_spacing = 40.0;
    let top_margin = 100.0;

    // Simulate layout to determine required height
    let mut num_lines = 1;
    if !notes.is_empty() {
        let mut x_sim = 0.0;
        let mut last_pos_sim = -1.0;
        let available_width = page_width - margin * 2.0;
        for note in notes {
            if note.position > last_pos_sim {
                x_sim += note_spacing;
                last_pos_sim = note.position;
            }
            if x_sim > available_width {
                x_sim = note_spacing;
                num_lines += 1;
            }
        }
    }
    
    let total_height = top_margin + (num_lines as f64 * system_height);
    let mut svg_doc = Document::new().set("viewBox", (0, 0, page_width, total_height));
    
    // Header
    svg_doc = svg_doc.add(SvgText::new(song_title)
        .set("x", margin).set("y", 35).set("font-family", "Arial").set("font-size", 22).set("font-weight", "bold"));
    svg_doc = svg_doc.add(SvgText::new(format!("Tuning: {}", tuning.name))
        .set("x", margin).set("y", 55).set("font-family", "Arial").set("font-size", 12).set("fill", "#666666"));

    if notes.is_empty() {
        svg::save(output_path, &svg_doc)?;
        return Ok(());
    }

    let mut current_x = 0.0;
    let mut current_y_offset = top_margin; 
    let mut measure_count = 1;
    let mut last_position = -1.0;

    svg_doc = draw_kora_system(svg_doc, current_y_offset, measure_count, margin, page_width);

    for note in notes {
        // If this note is at a new time position, advance the X cursor.
        if note.position > last_position {
            current_x += note_spacing;
            last_position = note.position;

            // A simple way to wrap lines.
            if current_x > (page_width - margin * 2.0) {
                current_x = note_spacing;
                measure_count += 1; 
                current_y_offset += system_height;
                svg_doc = draw_kora_system(svg_doc, current_y_offset, measure_count, margin, page_width);
            }
            
            // Add the vertical guide line for readability, only for new time positions.
            svg_doc = svg_doc.add(Line::new()
                .set("x1", margin + current_x)
                .set("y1", current_y_offset)
                .set("x2", margin + current_x)
                .set("y2", current_y_offset + 250.0)
                .set("stroke", "#90EE90") // lightgreen
                .set("stroke-width", 1));
        }
        
        if let Some((idx, is_right)) = tuning.left.iter().position(|&p| p == note.pitch).map(|i| (i, false))
            .or_else(|| tuning.right.iter().position(|&p| p == note.pitch).map(|i| (i, true))) {
            
            let color = if is_right { "#FF0000" } else { "#0000FF" }; // Red (Right), Blue (Left)
            let label = format!("{}", idx + 1);
            let y_pos = current_y_offset + ((10 - idx) as f64 * 25.0) + 5.0;

            let text_width = if label.len() > 1 { 16.0 } else { 8.0 };
            let rect_width = text_width + 8.0;
            let rect_height = 16.0;

            let rect_x = if is_right { margin + current_x } else { margin + current_x - rect_width };
            let rect_y = y_pos - 12.0;
            let x_pos = rect_x + (rect_width / 2.0);

            let rect = Rectangle::new()
                .set("x", rect_x).set("y", rect_y)
                .set("width", rect_width).set("height", rect_height)
                .set("rx", 4.0).set("ry", 4.0).set("fill", "white")
                .set("stroke", color).set("stroke-width", 1.0);
            svg_doc = svg_doc.add(rect);

            svg_doc = svg_doc.add(SvgText::new(label)
                .set("x", x_pos).set("y", y_pos)
                .set("font-family", "Arial, sans-serif").set("font-size", 13)
                .set("font-weight", "bold").set("fill", color)
                .set("text-anchor", "middle"));
        }
    }

    svg::save(output_path, &svg_doc)?;
    Ok(())
}

fn draw_ascii(
    song_title: &str,
    tuning: &KoraTuning,
    notes: &[Note],
) -> Result<String, Box<dyn std::error::Error>> {
    const MAX_WIDTH: usize = 120;
    const POS_WIDTH: usize = 6; // provides some spacing

    let mut output = format!("Title: {}\nTuning: {}\n\n", song_title, tuning.name);
    if notes.is_empty() {
        return Ok(output);
    }

    let mut notes_by_pos: HashMap<i64, Vec<Note>> = HashMap::new();
    for note in notes {
        notes_by_pos.entry((note.position * 1000.0) as i64).or_default().push(note.clone());
    }
    let mut sorted_positions: Vec<_> = notes_by_pos.keys().cloned().collect();
    sorted_positions.sort();
    
    let positions_per_system = MAX_WIDTH / POS_WIDTH;
    let position_chunks: Vec<_> = sorted_positions.chunks(positions_per_system).collect();

    for chunk in position_chunks {
        let num_positions = chunk.len();
        let system_width = num_positions * POS_WIDTH;
        
        // 1. Create canvas with background and vertical lines
        let mut canvas: Vec<Vec<char>> = vec![vec!['-'; system_width]; 11];
        for row in 0..11 {
            for pos_idx in 0..num_positions {
                // Center the bar in each position
                canvas[row][pos_idx * POS_WIDTH + (POS_WIDTH / 2)] = '|';
            }
        }

        // 2. Overwrite with notes
        for (pos_idx, pos_key) in chunk.iter().enumerate() {
            let chord = notes_by_pos.get(pos_key).unwrap();
            
            for note in chord {
                if let Some((idx, is_right)) = tuning.left.iter().position(|&p| p == note.pitch).map(|i| (i, false))
                    .or_else(|| tuning.right.iter().position(|&p| p == note.pitch).map(|i| (i, true))) {
                    
                    let line_idx = 10 - idx;
                    let label = (idx + 1).to_string();
                    let label_chars: Vec<char> = label.chars().collect();
                    
                    let col_offset = pos_idx * POS_WIDTH;
                    let bar_col_in_pos = POS_WIDTH / 2;
                    let bar_col_abs = col_offset + bar_col_in_pos;

                    if is_right {
                        // Place label to the right of the bar, e.g., "...|12.."
                        let start_col = bar_col_abs + 1;
                        for (i, c) in label_chars.iter().enumerate() {
                            if start_col + i < col_offset + POS_WIDTH {
                                canvas[line_idx][start_col + i] = *c;
                            }
                        }
                    } else {
                        // Place label to the left of the bar, e.g., "..12|..."
                        let start_col = bar_col_abs - label_chars.len();
                        for (i, c) in label_chars.iter().enumerate() {
                            if start_col + i < bar_col_abs {
                                canvas[line_idx][start_col + i] = *c;
                            }
                        }
                    }
                }
            }
        }

        // Add the system to the output.
        for line_chars in canvas {
            output.push_str(&line_chars.iter().collect::<String>());
            output.push('\n');
        }
        output.push('\n');
        output.push('\n');
    }

    Ok(output)
}


fn parse_musicxml(source: &PathBuf) -> Result<(String, Vec<Note>), Box<dyn std::error::Error>> {
    let xml_data = fs::read_to_string(source)?;
    let opt = ParsingOptions { allow_dtd: true, ..Default::default() };
    let doc_xml = XmlDoc::parse_with_options(&xml_data, opt)?;
    
    let song_title = doc_xml.descendants()
        .find(|n| n.has_tag_name("work-title"))
        .and_then(|n| n.text())
        .unwrap_or("Untitled Kora Piece").to_string();

    let mut notes = Vec::new();
    let mut current_pos = 0.0;
    let note_spacing = 1.0; // Arbitrary position increment

    for measure in doc_xml.descendants().filter(|n| n.has_tag_name("measure")) {
        for note_node in measure.children().filter(|n| n.has_tag_name("note")) {
            let is_chord = note_node.children().any(|n| n.has_tag_name("chord"));
            if !is_chord {
                current_pos += note_spacing;
            }

            if let Some(pitch_node) = note_node.children().find(|n| n.has_tag_name("pitch")) {
                let step = pitch_node.children().find(|n| n.has_tag_name("step")).unwrap().text().unwrap();
                let octave = pitch_node.children().find(|n| n.has_tag_name("octave")).unwrap().text().unwrap();
                let alter = pitch_node.children().find(|n| n.has_tag_name("alter")).map(|n| n.text().unwrap()).unwrap_or("0");
                if let Some(base_pitch) = note_to_pitch(&format!("{}{}", step, octave)) {
                    let alter_cents = alter.parse::<f32>().map(|a| (a * 100.0) as i32).unwrap_or(0);
                    let pitch = base_pitch + Pitch(alter_cents);
                    notes.push(Note { pitch, position: current_pos });
                }
            }
        }
    }
    Ok((song_title, notes))
}

fn freq_to_pitch(freq: f32) -> Pitch {
    Pitch(6900 + (1200.0 * (freq / 440.0).log2()).round() as i32)
}

fn pitch_to_freq(p: Pitch) -> f32 {
    440.0 * 2.0f32.powf((p.0 - 6900) as f32 / 1200.0)
}

fn find_closest_note_in_tuning(pitch: Pitch, tuning: &KoraTuning) -> Option<Pitch> {
    let all_notes = tuning.left.iter().chain(tuning.right.iter());

    // Find the note in the tuning with the minimum distance to the detected midi note.
    if let Some(closest_note) = all_notes.min_by_key(|&note| (pitch.0 - note.0).abs()) {
        // Only snap if the distance is within 200 cents threshold.
        if (pitch.0 - closest_note.0).abs() <= 200 {
            return Some(*closest_note);
        }
    }

    // If no note is within the 200 cent threshold, return None.
    None
}

fn transcribe_audio(source: &PathBuf, tuning: &KoraTuning) -> Result<(String, Vec<Note>), Box<dyn std::error::Error>> {
    let mut reader = WavReader::open(source)?;
    let spec = reader.spec();
    let sample_rate = spec.sample_rate as f32;

    let mut all_pitches = tuning.left.clone();
    all_pitches.extend_from_slice(&tuning.right);
    all_pitches.sort_by_key(|p| p.0);
    all_pitches.dedup();

    let min_freq_diff = all_pitches.windows(2)
        .map(|w| pitch_to_freq(w[1]) - pitch_to_freq(w[0]))        
        .fold(f32::INFINITY, f32::min);

    let window_size = (if min_freq_diff.is_finite() && min_freq_diff > 0.0 {
        (sample_rate / min_freq_diff).ceil() as usize
    } else {
        4096
    }).next_power_of_two().max(1024).min(16384);

    let hop_size = (sample_rate / 64.0).round() as usize;

    const NOTE_STABILITY_THRESHOLD: usize = 3;
    const MAX_NOTES: usize = 4;
    const ONSET_FACTOR: f32 = 2.0;
    const ADAPTIVE_THRESHOLD_FACTOR: f32 = 10.0;

    let samples: Vec<f32> = reader.samples::<i16>()
        .map(|s| s.unwrap() as f32 / i16::MAX as f32)
        .collect();

    let mut planner = FftPlanner::new();
    let fft = planner.plan_fft_forward(window_size);
    let mut buffer = vec![Complex::new(0.0, 0.0); window_size];
    
    let hann_window: Vec<f32> = (0..window_size)
        .map(|i| 0.5 * (1.0 - (2.0 * std::f32::consts::PI * i as f32 / (window_size - 1) as f32).cos()))
        .collect();

    let mut notes: Vec<Note> = Vec::new();
    //let mut last_committed_set = HashSet::new();
    //let mut last_note_set = HashSet::<Pitch>::new();
    //let mut stable_set_counter = 0;
    let mut magnitude_history: std::collections::VecDeque<Vec<f32>> = std::collections::VecDeque::with_capacity(NOTE_STABILITY_THRESHOLD);

    for (window_idx, window) in samples.windows(window_size).step_by(hop_size).enumerate() {
        for i in 0..window_size {
            buffer[i] = Complex::new(window[i] * hann_window[i], 0.0);
        }
        fft.process(&mut buffer);

        let magnitudes: Vec<f32> = buffer.iter().map(|c| c.norm_sqr()).collect();
        
        let mut sorted_magnitudes = magnitudes.iter().cloned().collect::<Vec<_>>();
        if sorted_magnitudes.is_empty() { continue; }
        sorted_magnitudes.sort_by(|a,b| a.partial_cmp(b).unwrap());
        let median_magnitude = sorted_magnitudes[sorted_magnitudes.len() / 2];
        let power_threshold = median_magnitude * ADAPTIVE_THRESHOLD_FACTOR;

        let mut peaks = Vec::new();//freq index in fft, magnitude

        for i in 1..(magnitudes.len() / 2 - 1) {
            //detect a local maximum in the freq graph
            if magnitudes[i] > power_threshold && magnitudes[i] > magnitudes[i-1] && magnitudes[i] > magnitudes[i+1] {
                //detect if the magnitude just jumped relative to the previous average
                let is_onset = if magnitude_history.len() == NOTE_STABILITY_THRESHOLD {
                    let avg_prev_mag = magnitude_history.iter().map(|m| m[i]).sum::<f32>() / NOTE_STABILITY_THRESHOLD as f32;
                    magnitudes[i] > avg_prev_mag * ONSET_FACTOR
                } else { true };

                if is_onset {
                    peaks.push((i, magnitudes[i]));
                }
            }
        }
        
        if magnitude_history.len() == NOTE_STABILITY_THRESHOLD {
            magnitude_history.pop_front();
        }
        magnitude_history.push_back(magnitudes);

        //look for the loudest few new peaks
        peaks.sort_by(|a, b| b.1.partial_cmp(&a.1).unwrap());
        
        let current_note_set: HashSet<Pitch> = peaks.iter().take(MAX_NOTES)
            .map(|(idx, _power)| {
                let freq = *idx as f32 * sample_rate / window_size as f32;
                freq_to_pitch(freq)
            })
            .filter_map(|pitch| find_closest_note_in_tuning(pitch, tuning))
            .collect();
        
        // if current_note_set == last_note_set {
        //     stable_set_counter += 1;
        // } else {
        //     last_note_set = current_note_set;
        //     stable_set_counter = 1;
        // }

        // if stable_set_counter == NOTE_STABILITY_THRESHOLD && !last_note_set.is_empty() && last_note_set != last_committed_set {
        //     let position = (window_idx * hop_size) as f64 / sample_rate as f64;
        //     println!("Detected Chord: {:?}, Time: {:.2}s", last_note_set, position);
        //     for &pitch in &last_note_set {
        //         notes.push(Note { pitch, position });
        //     }
        //     last_committed_set = last_note_set.clone();
        // }    

            let position = (window_idx * hop_size) as f64 / sample_rate as f64;
            for &pitch in &current_note_set {
                notes.push(Note { pitch, position });
            }    
    }
    
    let song_title = source.file_stem().unwrap().to_str().unwrap().to_string();
    Ok((song_title, notes))
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args = Args::parse();
    
    let (source, output, tuning_name, format, is_audio) = match args.command {
        Commands::Draw { source, output, tuning, format } => (source, output, tuning, format, false),
        Commands::Transcribe { source, output, tuning, format } => (source, output, tuning, format, true),
    };

    let mut config = Ini::new();
    config.load("tuning.ini").map_err(|e| format!("INI Load Error: {}", e))?;

    let tuning = KoraTuning {
        name: tuning_name.clone(),
        left: config.get(&tuning_name, "left").ok_or("Left tuning missing")?.split(',').filter_map(note_to_pitch).collect(),
        right: config.get(&tuning_name, "right").ok_or("Right tuning missing")?.split(',').filter_map(note_to_pitch).collect(),
    };

    let (song_title, notes) = if is_audio {
        transcribe_audio(&source, &tuning)?
    } else {
        parse_musicxml(&source)?
    };
    
    match format.as_str() {
        "ascii" => {
            let ascii_output = draw_ascii(&song_title, &tuning, &notes)?;
            fs::write(&output, ascii_output)?;
        },
        "svg" => {
            draw_svg(&output, &song_title, &tuning, &notes)?;
        },
        _ => return Err(format!("Unsupported format: {}", format).into()),
    }
    
    println!("Success! Tab for '{}' generated at '{}'.", song_title, output.display());
    Ok(())
}