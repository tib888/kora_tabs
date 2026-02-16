use clap::{Parser, Subcommand};
use configparser::ini::Ini;
use hound::{WavReader, WavSpec};
use rustfft::{FftPlanner, num_complex::Complex};
use std::collections::HashSet;
use std::fs;
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
    /// Converts a MusicXML file into a Kora tab SVG.
    Draw {
        /// Path to the input MusicXML file.
        source: PathBuf,
        /// Path for the output SVG file.
        output: PathBuf,
        /// Name of the tuning to use from tuning.ini.
        #[arg(short, long)]
        tuning: String,
    },
    /// Transcribes a WAV audio file into a Kora tab SVG.
    Transcribe {
        /// Path to the input WAV file.
        source: PathBuf,
        /// Path for the output SVG file.
        output: PathBuf,
        /// Name of the tuning to use from tuning.ini.
        #[arg(short, long)]
        tuning: String,
    },
}

#[derive(Debug, Clone)]
struct Note {
    midi: u8,
    position: f64, // General-purpose position, time in seconds for audio, or beat count for XML
}

struct KoraTuning {
    name: String,
    left: Vec<u8>,
    right: Vec<u8>,
}

fn note_to_midi(note: &str) -> u8 {
    let note = note.trim();
    if note.is_empty() { return 0; }
    let (name, octave_str) = if note.len() >= 2 && (&note[1..2] == "b" || &note[1..2] == "#") {
        (&note[0..2], &note[2..])
    } else {
        (&note[0..1], &note[1..])
    };
    let base = match name {
        "C" => 0, "C#" | "Db" => 1, "D" => 2, "D#" | "Eb" => 3, "E" => 4,
        "F" => 5, "F#" | "Gb" => 6, "G" => 7, "G#" | "Ab" => 8, "A" => 9, "A#" | "Bb" => 10, "B" => 11,
        _ => 0,
    };
    let octave: i8 = octave_str.parse().unwrap_or(4);
    ((octave + 1) * 12 + base) as u8
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

fn draw_tab(
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
        
        if let Some((idx, is_right)) = tuning.left.iter().position(|&p| p == note.midi).map(|i| (i, false))
            .or_else(|| tuning.right.iter().position(|&p| p == note.midi).map(|i| (i, true))) {
            
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
                let midi = (note_to_midi(&format!("{}{}", step, octave)) as i16 + alter.parse::<i16>().unwrap()) as u8;
                notes.push(Note { midi, position: current_pos });
            }
        }
    }
    Ok((song_title, notes))
}

fn freq_to_midi(freq: f32) -> f32 {
    69.0 + 12.0 * (freq / 440.0).log2()
}

fn find_closest_note_in_tuning(midi: u8, tuning: &KoraTuning) -> Option<u8> {
    let all_notes = tuning.left.iter().chain(tuning.right.iter());

    // Find the note in the tuning with the minimum distance to the detected midi note.
    if let Some(closest_note) = all_notes.min_by_key(|&note_midi| (midi as i16 - *note_midi as i16).abs()) {
        // Calculate the distance in semitones (MIDI numbers are semitones).
        let distance = (midi as i16 - *closest_note as i16).abs();

        // 200 cents is 2 semitones. Only snap if the distance is within this threshold.
        if distance <= 2 {
            return Some(*closest_note);
        }
    }

    // If no note is within the 200 cent threshold, return None.
    None
}

fn transcribe_audio(source: &PathBuf, tuning: &KoraTuning) -> Result<(String, Vec<Note>), Box<dyn std::error::Error>> {
    const WINDOW_SIZE: usize = 4096;
    const HOP_SIZE: usize = 1024;
    const POWER_THRESHOLD: f32 = 10.0;
    const NOTE_STABILITY_THRESHOLD: usize = 3;
    const MAX_NOTES: usize = 4;

    let mut reader = WavReader::open(source)?;
    let spec = reader.spec();
    let sample_rate = spec.sample_rate as f32;

    let samples: Vec<f32> = reader.samples::<i16>()
        .map(|s| s.unwrap() as f32 / i16::MAX as f32)
        .collect();

    let mut planner = FftPlanner::new();
    let fft = planner.plan_fft_forward(WINDOW_SIZE);
    let mut buffer = vec![Complex::new(0.0, 0.0); WINDOW_SIZE];
    
    let hann_window: Vec<f32> = (0..WINDOW_SIZE)
        .map(|i| 0.5 * (1.0 - (2.0 * std::f32::consts::PI * i as f32 / (WINDOW_SIZE - 1) as f32).cos()))
        .collect();

    let mut notes: Vec<Note> = Vec::new();
    let mut last_committed_set = HashSet::new();
    let mut last_note_set = HashSet::new();
    let mut stable_set_counter = 0;

    for (window_idx, window) in samples.windows(WINDOW_SIZE).step_by(HOP_SIZE).enumerate() {
        for i in 0..WINDOW_SIZE {
            buffer[i] = Complex::new(window[i] * hann_window[i], 0.0);
        }
        fft.process(&mut buffer);

        let magnitudes: Vec<f32> = buffer.iter().map(|c| c.norm_sqr()).collect();
        let mut peaks = Vec::new();

        for i in 1..(magnitudes.len() / 2 - 1) {
            if magnitudes[i] > POWER_THRESHOLD && magnitudes[i] > magnitudes[i-1] && magnitudes[i] > magnitudes[i+1] {
                peaks.push((i, magnitudes[i]));
            }
        }

        peaks.sort_by(|a, b| b.1.partial_cmp(&a.1).unwrap());
        
        let current_note_set: HashSet<u8> = peaks.iter().take(MAX_NOTES)
            .map(|(idx, _power)| {
                let freq = *idx as f32 * sample_rate / WINDOW_SIZE as f32;
                freq_to_midi(freq).round() as u8
            })
            .filter_map(|midi| find_closest_note_in_tuning(midi, tuning))
            .collect();
        
        if current_note_set == last_note_set {
            stable_set_counter += 1;
        } else {
            last_note_set = current_note_set;
            stable_set_counter = 1;
        }

        if stable_set_counter == NOTE_STABILITY_THRESHOLD && !last_note_set.is_empty() && last_note_set != last_committed_set {
            let position = (window_idx * HOP_SIZE) as f64 / sample_rate as f64;
            println!("Detected Chord: {:?}, Time: {:.2}s", last_note_set, position);
            for &midi in &last_note_set {
                notes.push(Note { midi, position });
            }
            last_committed_set = last_note_set.clone();
        }
    }
    
    let song_title = source.file_stem().unwrap().to_str().unwrap().to_string();
    Ok((song_title, notes))
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args = Args::parse();
    
    let (source, output, tuning_name, is_audio) = match args.command {
        Commands::Draw { source, output, tuning } => (source, output, tuning, false),
        Commands::Transcribe { source, output, tuning } => (source, output, tuning, true),
    };

    let mut config = Ini::new();
    config.load("tuning.ini").map_err(|e| format!("INI Load Error: {}", e))?;

    let tuning = KoraTuning {
        name: tuning_name.clone(),
        left: config.get(&tuning_name, "left").ok_or("Left tuning missing")?.split(',').map(note_to_midi).collect(),
        right: config.get(&tuning_name, "right").ok_or("Right tuning missing")?.split(',').map(note_to_midi).collect(),
    };

    let (song_title, notes) = if is_audio {
        transcribe_audio(&source, &tuning)?
    } else {
        parse_musicxml(&source)?
    };
    
    draw_tab(&output, &song_title, &tuning, &notes)?;
    
    println!("Success! Tab for '{}' generated at '{}'.", song_title, output.display());
    Ok(())
}