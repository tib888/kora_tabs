use clap::{Parser, Subcommand};
use configparser::ini::Ini;
use hound::WavReader;
use rustfft::{FftPlanner, num_complex::Complex};
use std::collections::HashMap;
use std::fs;
use std::ops::Add;
use std::path::PathBuf;
use svg::node::element::{Line, Rectangle, Text as SvgText};
use svg::Document;
use roxmltree::{Document as XmlDoc, ParsingOptions};
use image::{Rgb, RgbImage};

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
        #[arg(long)]
        format: Option<String>,
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
        #[arg(long)]
        format: Option<String>,
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

#[derive(Debug, Copy, Clone)]
struct Note {
    pitch: Pitch,
    position: f64, // Position in quarter notes
}

#[derive(Debug, Clone, PartialEq, Eq, Hash)]
struct StringInfo
{
    pitch: Pitch,
    right: bool,
    index: u8,
    name: String,
}

#[derive(Debug, Clone)]
struct KoraTuning {
    name: String,
    notes: Vec<StringInfo>
}

/// returns midi number * 100 + microtone adjustments in cents
/// for example "A4+30" gives 1200 * 5 + 900 + 30 = 6930
fn note_to_pitch(note: &str) -> Option<(Pitch, String)> {
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
    
    base.map(|base| (Pitch((octave + 1) * 1200 + 100 * base + cents), name.to_string()))
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
    // doc = doc.add(Line::new().set("x1", margin).set("y1", y_off).set("x2", margin).set("y2", y_off + 250.0).set("stroke", "#444").set("stroke-width", 2));
    // doc = doc.add(Line::new().set("x1", width - margin).set("y1", y_off).set("x2", width - margin).set("y2", y_off + 250.0).set("stroke", "#444").set("stroke-width", 2));
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
    let note_spacing = 50.0;
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
                .set("y2", current_y_offset + 11.0 * 25.0)
                .set("stroke", "#90EE90") // lightgreen
                .set("stroke-width", 1));
        }
        
        if let Some(string_info) = tuning.notes.iter().find(|&p| p.pitch == note.pitch) {
            let color = if string_info.right { "#FF0000" } else { "#0000FF" }; // Red (Right), Blue (Left)
            
            let max = if string_info.right { 11 } else { 10 };
            let line_idx = (max - string_info.index) as usize;
            let label = (11 - line_idx).to_string() + string_info.name.as_str();
            let y_pos = current_y_offset + (line_idx as f64 * 25.0) + 5.0;

            let text_width = if label.len() > 1 { 16.0 } else { 8.0 };
            let rect_width = text_width + 8.0;
            let rect_height = 16.0;

            let rect_x = if string_info.right { margin + current_x } else { margin + current_x - rect_width };
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
        canvas.push(vec![' '; system_width]);// Extra line for possible 22th string
        for row in 0..12 {
            for pos_idx in 0..num_positions {
                // Center the bar in each position
                canvas[row][pos_idx * POS_WIDTH + (POS_WIDTH / 2)] = '|';
            }
        }

        // 2. Overwrite with notes
        for (pos_idx, pos_key) in chunk.iter().enumerate() {
            let chord = notes_by_pos.get(pos_key).unwrap();
            
            for note in chord {
                if let Some(string_info) = tuning.notes.iter().find(|&p| p.pitch == note.pitch) {
                    let max = if string_info.right { 11 } else { 10 };
                    let line_idx = (max - string_info.index) as usize;
                    let label = (11 - line_idx).to_string() + string_info.name.as_str();
                    let label_chars: Vec<char> = label.chars().collect();
                    
                    let col_offset = pos_idx * POS_WIDTH;
                    let bar_col_in_pos = POS_WIDTH / 2;
                    let bar_col_abs = col_offset + bar_col_in_pos;

                    if string_info.right {
                        // Place label to the right of the bar, e.g., "...|10.."
                        let start_col = bar_col_abs + 1;
                        for (i, c) in label_chars.iter().enumerate() {
                            if start_col + i < col_offset + POS_WIDTH {
                                canvas[line_idx][start_col + i] = *c;
                            }
                        }
                    } else {
                        // Place label to the left of the bar, e.g., "..11|..."
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


fn draw_musicxml(song_title: &str, notes: &[Note]) -> Result<String, Box<dyn std::error::Error>> {
    let mut xml = String::new();
    xml.push_str("<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n");
    xml.push_str("<!DOCTYPE score-partwise PUBLIC \"-//Recordare//DTD MusicXML 4.0 Partwise//EN\" \"http://www.musicxml.org/dtds/partwise.dtd\">\n");
    xml.push_str("<score-partwise version=\"4.0\">\n");

    xml.push_str(&format!("  <work>\n    <work-title>{}</work-title>\n  </work>\n", song_title));

    xml.push_str("  <part-list>\n");
    xml.push_str("    <score-part id=\"P1\">\n      <part-name>Kora</part-name>\n    </score-part>\n");
    xml.push_str("  </part-list>\n");

    xml.push_str("  <part id=\"P1\">\n");

    let divisions = 24;
    let _tempo = 120.0;//bpm

    let mut notes_by_pos: std::collections::BTreeMap<i64, Vec<&Note>> = std::collections::BTreeMap::new();
    for note in notes {
        let pos_in_divisions = (note.position * divisions as f64).round() as i64;
        notes_by_pos.entry(pos_in_divisions).or_default().push(note);
    }
    
    let measure_divisions = 4 * divisions;
    let mut measure_number = 1;
    xml.push_str(&format!("    <measure number=\"{}\">\n", measure_number));
    xml.push_str("      <attributes>\n");
    xml.push_str(&format!("        <divisions>{}</divisions>\n", divisions));
    xml.push_str("        <key><fifths>0</fifths></key>\n");
    xml.push_str("        <time><beats>4</beats><beat-type>4</beat-type></time>\n");
    xml.push_str("        <clef><sign>G</sign><line>2</line></clef>\n");
    xml.push_str("      </attributes>\n");

    let mut position_in_score: i64 = 0;

    for (event_pos, chord_notes) in notes_by_pos {
        let rest_duration = event_pos - position_in_score;
        if rest_duration > 0 {
            let mut remaining_rest = rest_duration;
            while remaining_rest > 0 {
                let current_measure_pos = position_in_score % measure_divisions;
                let space_left_in_measure = measure_divisions - current_measure_pos;
                let rest_to_write = remaining_rest.min(space_left_in_measure);
                
                if rest_to_write > 0 {
                    xml.push_str(&format!("      <note>\n        <rest/>\n        <duration>{}</duration>\n      </note>\n", rest_to_write));
                }
                
                position_in_score += rest_to_write;
                remaining_rest -= rest_to_write;

                if position_in_score % measure_divisions == 0 && remaining_rest > 0 {
                    xml.push_str("    </measure>\n");
                    measure_number += 1;
                    xml.push_str(&format!("    <measure number=\"{}\">\n", measure_number));
                }
            }
        }
        
        let note_duration = 12; // 8th note
        let mut first_note = true;
        for note in chord_notes {
            let (step, octave, alter) = pitch_to_xml_parts(note.pitch);
            xml.push_str("      <note>\n");
            if !first_note {
                xml.push_str("        <chord/>\n");
            }
            xml.push_str("        <pitch>\n");
            xml.push_str(&format!("          <step>{}</step>\n", step));
            if alter != 0.0 {
                xml.push_str(&format!("          <alter>{}</alter>\n", alter));
            }
            xml.push_str(&format!("          <octave>{}</octave>\n", octave));
            xml.push_str("        </pitch>\n");
            xml.push_str(&format!("        <duration>{}</duration>\n", note_duration));
            xml.push_str("      </note>\n");
            first_note = false;
        }

        position_in_score += note_duration;
        if position_in_score % measure_divisions == 0 {
             xml.push_str("    </measure>\n");
             measure_number += 1;
             xml.push_str(&format!("    <measure number=\"{}\">\n", measure_number));
        }
    }

    let current_measure_pos = position_in_score % measure_divisions;
    if current_measure_pos != 0 {
        let rest_to_fill = measure_divisions - current_measure_pos;
        xml.push_str(&format!("      <note>\n        <rest/>\n        <duration>{}</duration>\n      </note>\n", rest_to_fill));
        xml.push_str("    </measure>\n");
    } else {
        if !xml.ends_with("</measure>\n") {
             xml.push_str("    </measure>\n");
        }
    }

    xml.push_str("  </part>\n");
    xml.push_str("</score-partwise>\n");

    Ok(xml)
}

fn pitch_to_xml_parts(p: Pitch) -> (String, i32, f32) {
    let total_cents = p.0;
    let note_num = (total_cents as f32 / 100.0).round() as i32;
    let alter_cents = total_cents - note_num * 100;

    let octave = note_num / 12 - 1;
    let note_name = match note_num % 12 {
        0 => "C", 1 => "C", 2 => "D", 3 => "D", 4 => "E", 5 => "F",
        6 => "F", 7 => "G", 8 => "G", 9 => "A", 10 => "A", 11 => "B",
        _ => unreachable!(),
    };
    let base_alter = match note_num % 12 {
        1 | 3 | 6 | 8 | 10 => 1.0,
        _ => 0.0,
    };

    let final_alter = base_alter + (alter_cents as f32 / 100.0);
    (note_name.to_string(), octave, final_alter)
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
    let mut current_pos_in_divisions = 0.0;
    let mut divisions = 1.0;

    if let Some(part) = doc_xml.descendants().find(|n| n.has_tag_name("part")) {
        for measure in part.children().filter(|n| n.has_tag_name("measure")) {
            for element in measure.children().filter(|n| n.is_element()) {
                match element.tag_name().name() {
                    "attributes" => {
                        if let Some(divs) = element.descendants().find(|d| d.has_tag_name("divisions")) {
                            divisions = divs.text().unwrap().parse().unwrap_or(divisions);
                        }
                    },
                    "note" => {
                        let position_in_quarters = current_pos_in_divisions / divisions;
                        if let Some(pitch_node) = element.children().find(|n| n.has_tag_name("pitch")) {
                            let step = pitch_node.children().find(|n| n.has_tag_name("step")).unwrap().text().unwrap();
                            let octave = pitch_node.children().find(|n| n.has_tag_name("octave")).unwrap().text().unwrap();
                            let alter = pitch_node.children().find(|n| n.has_tag_name("alter")).map(|n| n.text().unwrap()).unwrap_or("0");
                            if let Some((base_pitch, _name)) = note_to_pitch(&format!("{}{}", step, octave)) {
                                let alter_cents = alter.parse::<f32>().map(|a| (a * 100.0) as i32).unwrap_or(0);
                                let pitch = base_pitch + Pitch(alter_cents);
                                notes.push(Note { pitch, position: position_in_quarters });
                            }
                        }

                        if !element.children().any(|c| c.has_tag_name("chord")) {
                            if let Some(duration) = element.descendants().find(|d| d.has_tag_name("duration")) {
                                current_pos_in_divisions += duration.text().unwrap().parse().unwrap_or(0.0);
                            }
                        }
                    },
                    "backup" => {
                        if let Some(duration) = element.descendants().find(|d| d.has_tag_name("duration")) {
                            current_pos_in_divisions -= duration.text().unwrap().parse().unwrap_or(0.0);
                        }
                    },
                    "forward" => {
                        if let Some(duration) = element.descendants().find(|d| d.has_tag_name("duration")) {
                            current_pos_in_divisions += duration.text().unwrap().parse().unwrap_or(0.0);
                        }
                    },
                    _ => (),
                }
            }
        }
    }
    
    if notes.is_empty() { // Fallback for simple/malformed XML
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
                    if let Some((base_pitch, _name)) = note_to_pitch(&format!("{}{}", step, octave)) {
                        let alter_cents = alter.parse::<f32>().map(|a| (a * 100.0) as i32).unwrap_or(0);
                        let pitch = base_pitch + Pitch(alter_cents);
                        notes.push(Note { pitch, position: current_pos });
                    }
                }
            }
        }
    }

    Ok((song_title, notes))
}

fn freq_to_pitch(freq: f32) -> Pitch {
    Pitch(6900 + (1200.0 * (freq / 440.0).log2()).round() as i32)
}

fn pitch_to_freq(p: &Pitch) -> f32 {
    440.0 * 2.0f32.powf((p.0 - 6900) as f32 / 1200.0)
}

fn find_closest_note_in_tuning(pitch: Pitch, tuning: &KoraTuning) -> Option<&StringInfo> {    
    // Find the note in the tuning with the minimum distance to the detected midi note.
    if let Some(closest_note) = tuning.notes.iter().min_by_key(|&note| (pitch.0 - note.pitch.0).abs()) {
        // Only snap if the distance is within 200 cents threshold.
        if (pitch.0 - closest_note.pitch.0).abs() <= 200 {
            return Some(closest_note);
        }
    }

    // If no note is within the 200 cent threshold, return None.
    None
}

fn transcribe_audio(source: &PathBuf, tuning: &KoraTuning) -> Result<(String, Vec<Note>), Box<dyn std::error::Error>> {
    //this image used for debug, contains the log-log spectrogram with the detected notes
    const IMG_WIDTH: u32 = 800;
    const MAX_IMG_HEIGHT: u32 = 8000;
    let mut img = RgbImage::new(IMG_WIDTH, MAX_IMG_HEIGHT);
    let mut final_height = 0;

    let mut reader = WavReader::open(source)?;
    let spec = reader.spec();
    let sample_rate = spec.sample_rate as f32;

    let mut all_pitches: Vec<Pitch> = tuning.notes.iter().map(|note| note.pitch).collect();
    all_pitches.sort_by_key(|p| p.0);
    all_pitches.dedup();

    let min_freq_diff = all_pitches.windows(2)
        .map(|w| pitch_to_freq(&w[1]) - pitch_to_freq(&w[0]))        
        .fold(f32::INFINITY, f32::min);

    let window_size = (if min_freq_diff.is_finite() && min_freq_diff > 0.0 {
        (sample_rate / min_freq_diff).ceil() as usize
    } else {
        4096
    }).next_power_of_two().max(1024).min(16384);

    let hop_size = (sample_rate / 48.0).round() as usize;

    const HISTORY_LENGTH: usize = 5;
    const PEAK_HISTORY_LENGTH: usize = 3;
    //const MAX_NOTES: usize = 10;
    const ONSET_FACTOR: f32 = 4.0;
    const ADAPTIVE_THRESHOLD_FACTOR: f32 = 1.0;

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
    let mut magnitude_history: std::collections::VecDeque<Vec<f32>> = std::collections::VecDeque::with_capacity(HISTORY_LENGTH);
    let mut peak_history: std::collections::VecDeque<Vec<Pitch>> = std::collections::VecDeque::with_capacity(PEAK_HISTORY_LENGTH);

    for (window_idx, window) in samples.windows(window_size).step_by(hop_size).enumerate() {
        for i in 0..window_size {
            buffer[i] = Complex::new(window[i] * hann_window[i], 0.0);
        }
        fft.process(&mut buffer);

        let magnitudes: Vec<f32> = buffer.iter().map(|c| c.norm_sqr()).collect();
       
        let num_bins = window_size / 2;
        let log_num_bins = (num_bins as f32).log10();

        if window_idx < MAX_IMG_HEIGHT as usize {
            final_height = window_idx as u32 + 1;
        
            // let max_log_mag = magnitudes.iter()
            //     .map(|&m| if m > 0.0 { m.log10() } else { 0.0 })
            //     .fold(0.0, f32::max)
            //     .max(1.0);

            let max_log_mag = 6.0;

            for x in 0..IMG_WIDTH {
                let log_i = (x as f32 / (IMG_WIDTH - 1) as f32) * log_num_bins;
                let i = 10.0f32.powf(log_i).round() as usize;

                if i < num_bins {
                    let log_mag = if magnitudes[i] > 0.0 { magnitudes[i].log10() } else { 0.0 };
                    let intensity = ((log_mag / max_log_mag) * 255.0).min(255.0) as u8;
                    img.put_pixel(x, window_idx as u32, Rgb([0, 0, intensity]));
                }
            }
        }
        
        // let mut sorted_magnitudes = magnitudes.iter().cloned().collect::<Vec<_>>();
        // if sorted_magnitudes.is_empty() { continue; }
        // sorted_magnitudes.sort_by(|a,b| a.partial_cmp(b).unwrap());
        // let median_magnitude = sorted_magnitudes[sorted_magnitudes.len() / 2];
        // let power_threshold = (median_magnitude * ADAPTIVE_THRESHOLD_FACTOR);//.max(10.0);
        let power_threshold = 0.01f32;

        let mut peaks = Vec::new();//freq index in fft, magnitude

        let max_freq_index = magnitudes.len() / 2;//remember Nyquist's law
        for i in 1..(max_freq_index - 1) {
            if magnitudes[i] > power_threshold && magnitudes[i] > magnitudes[i - 1] && magnitudes[i] > magnitudes[i + 1] {
                let is_onset = if magnitude_history.len() == HISTORY_LENGTH {
                    let avg_prev_mag = magnitude_history.iter().map(|m| m[i]).sum::<f32>() / HISTORY_LENGTH as f32;
                    magnitudes[i] > avg_prev_mag * ONSET_FACTOR
                } else { true };

                if is_onset {
                    peaks.push((i, magnitudes[i]));
                }
            }
        }
        
        if magnitude_history.len() >= HISTORY_LENGTH {
            magnitude_history.pop_front();
        }
        magnitude_history.push_back(magnitudes);

        peaks.sort_by(|a, b| b.1.partial_cmp(&a.1).unwrap());

        if window_idx < MAX_IMG_HEIGHT as usize {

            //mark the detected peaks with white/gray instead of only blue pixels
            for (peak_idx, _magnitude) in &peaks {
                if *peak_idx > 0 && *peak_idx < num_bins {
                     let log_peak_idx = (*peak_idx as f32).log10();
                     let x = ((log_peak_idx / log_num_bins) * (IMG_WIDTH - 1) as f32).round() as u32;
                     if x < IMG_WIDTH {
                        let color = img.get_pixel(x, window_idx as u32);
                        img.put_pixel(x, window_idx as u32, Rgb([color.0[2], color.0[2], color.0[2]]));
                     }
                }
            }

            if window_idx % 10 == 0 {
                //mark the exact freq of possible notes in the tuning with red dots
                for note in tuning.notes.iter() {
                    let freq= pitch_to_freq(&note.pitch);
                    let peak_idx = (freq * max_freq_index as f32 / sample_rate) as usize;                    
                    let log_peak_idx = (peak_idx as f32).log10();
                    let x = ((log_peak_idx / log_num_bins) * (IMG_WIDTH - 1) as f32).round() as u32;
                    if x < IMG_WIDTH {
                        img.put_pixel(x, window_idx as u32, Rgb([200, 0, 0]));
                    }
                }
            }
        }

        let current_peaks: Vec<Pitch> = peaks.iter().filter_map(|(idx, magnitude)| {
            if magnitude * 4.0 < peaks[0].1 {
                return None;
            }
            let freq = *idx as f32 * sample_rate / max_freq_index as f32;
            Some(freq_to_pitch(freq))
        }).collect();
                
        // calculate the position in time:
        let position = (window_idx * hop_size) as f64 / sample_rate as f64;
        let snapped_position = (position * 48.0).round() / 48.0; // Round to nearest 1/24 beat collect the notes picked together
        let quarter_duration_secs = 0.5; // at 120bpm
        let position_in_quarters = snapped_position / quarter_duration_secs;

        for &peak in &current_peaks {            
            if peak_history.len() >= PEAK_HISTORY_LENGTH {
                let stable_peak = peak_history.iter().filter(|peak_set| peak_set.contains(&peak)).count() >= PEAK_HISTORY_LENGTH - 1;
                let new_peak = !peak_history.front().map_or(false, |last_set| last_set.contains(&peak));
                if stable_peak && new_peak {
                    if let Some(string_info) = find_closest_note_in_tuning(peak, tuning) {
                        notes.push(Note { pitch: string_info.pitch, position:position_in_quarters });

                        if window_idx < MAX_IMG_HEIGHT as usize {
                            let freq= pitch_to_freq(&string_info.pitch);
                            let peak_idx = (freq * max_freq_index as f32 / sample_rate) as usize;                    
                            let log_peak_idx = (peak_idx as f32).log10();
                            let x = ((log_peak_idx / log_num_bins) * (IMG_WIDTH - 1) as f32).round() as u32;
                            if x < IMG_WIDTH {
                                img.put_pixel(x, window_idx as u32, Rgb([0, 255, 0]));
                                img.put_pixel(x-1, window_idx as u32, Rgb([0, 255, 0]));//repeat to be more visible
                                img.put_pixel(x+1, window_idx as u32, Rgb([0, 255, 0]));//repeat to be more visible
                            }
                        }
                    }
                }
            }
        }   

        if peak_history.len() >= PEAK_HISTORY_LENGTH {
            peak_history.pop_front();
        }
        peak_history.push_back(current_peaks);         
    }
    
    if final_height > 0 {
        let cropped_img = image::imageops::crop_imm(&img, 0, 0, IMG_WIDTH, final_height).to_image();
        cropped_img.save("dump.bmp")?;
    }

    let song_title = source.file_stem().unwrap().to_str().unwrap().to_string();
    Ok((song_title, notes))
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args = Args::parse();
    
    let (source, output, tuning_name, format_option, is_audio) = match args.command {
        Commands::Draw { source, output, tuning, format } => (source, output, tuning, format, false),
        Commands::Transcribe { source, output, tuning, format } => (source, output, tuning, format, true),
    };

    let format = match format_option {
        Some(f) => f,
        None => {
            output.extension()
                .and_then(std::ffi::OsStr::to_str)
                .map(|s| s.to_lowercase())
                .unwrap_or_default() // empty string if no extension
        }
    };

    let mut config = Ini::new();
    config.load("tuning.ini").map_err(|e| format!("INI Load Error: {}", e))?;

    let mut notes: Vec<StringInfo> = config.get(&tuning_name, "left").ok_or("Left tuning missing or tuning name is misspelled")?.split(',').enumerate()
        .map(|(idx, note)| {
            let (pitch, name) = note_to_pitch(note).unwrap();
            StringInfo { pitch, right: false, index: idx as u8, name }
        }).chain(
        config.get(&tuning_name, "right").ok_or("Right tuning missing")?.split(',').enumerate()
        .map(|(idx, note)| {
            let (pitch, name) = note_to_pitch(note).unwrap();
            StringInfo { pitch, right: true, index: idx as u8, name }
        })).collect();
    notes.sort_by_key(|note| note.pitch.0);

    let tuning = KoraTuning {
        name: tuning_name.clone(),
        notes
    };

    let (song_title, notes) = if is_audio {
        transcribe_audio(&source, &tuning)?
    } else {
        parse_musicxml(&source)?
    };
    
    match format.as_str() {
        "ascii" | "txt" => {
            let ascii_output = draw_ascii(&song_title, &tuning, &notes)?;
            fs::write(&output, ascii_output)?;
        },
        "svg" | "" => { // Default to svg
            draw_svg(&output, &song_title, &tuning, &notes)?;
        },
        "musicxml" | "xml" => {
            let musicxml_output = draw_musicxml(&song_title, &notes)?;
            fs::write(&output, musicxml_output)?;
        },
        _ => return Err(format!("Unsupported format or unknown extension: '{}'", format).into()),
    }
    
    println!("Success! Tab for '{}' generated at '{}'.", song_title, output.display());
    Ok(())
}