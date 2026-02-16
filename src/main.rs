use clap::Parser;
use configparser::ini::Ini;
use std::fs;
use std::path::PathBuf;
use svg::node::element::{Line, Rectangle, Text as SvgText};
use svg::Document;
use roxmltree::{Document as XmlDoc, ParsingOptions};

#[derive(Parser, Debug)]
#[command(author, version, about = "Kora Tab Generator - Pro Edition")]
struct Args {
    source: PathBuf,
    output: PathBuf,
    #[arg(short, long)]
    tuning: String,
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

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args = Args::parse();
    let mut config = Ini::new();
    config.load("tuning.ini").map_err(|e| format!("INI Load Error: {}", e))?;

    let tuning = KoraTuning {
        name: args.tuning.clone(),
        left: config.get(&args.tuning, "left").ok_or("Left tuning missing")?.split(',').map(note_to_midi).collect(),
        right: config.get(&args.tuning, "right").ok_or("Right tuning missing")?.split(',').map(note_to_midi).collect(),
    };

    let xml_data = fs::read_to_string(&args.source)?;
    let opt = ParsingOptions { allow_dtd: true, ..Default::default() };
    let doc_xml = XmlDoc::parse_with_options(&xml_data, opt)?;
    
    // Get Song Title from MusicXML
    let song_title = doc_xml.descendants()
        .find(|n| n.has_tag_name("work-title"))
        .and_then(|n| n.text())
        .unwrap_or("Untitled Kora Piece");

    let page_width = 750.0;
    let margin = 60.0;
    let system_height = 340.0; 
    let note_spacing = 40.0;
    
    let mut svg_doc = Document::new().set("viewBox", (0, 0, page_width, 1150));
    
    // Header
    svg_doc = svg_doc.add(SvgText::new(song_title)
        .set("x", margin).set("y", 35).set("font-family", "Arial").set("font-size", 22).set("font-weight", "bold"));
    svg_doc = svg_doc.add(SvgText::new(format!("Tuning: {}", tuning.name))
        .set("x", margin).set("y", 55).set("font-family", "Arial").set("font-size", 12).set("fill", "#666666"));

    let mut current_x = 0.0;
    let mut current_y_offset = 100.0; 
    let mut measure_count = 1;
    let mut measures_in_current_line = 0;

    svg_doc = draw_kora_system(svg_doc, current_y_offset, measure_count, margin, page_width);

    for measure in doc_xml.descendants().filter(|n| n.has_tag_name("measure")) {
        if measures_in_current_line >= 4 {
            measures_in_current_line = 0;
            current_x = 0.0;
            current_y_offset += system_height;
            svg_doc = draw_kora_system(svg_doc, current_y_offset, measure_count, margin, page_width);
        }

        // Measure separator
        svg_doc = svg_doc.add(Line::new()
            .set("x1", margin + current_x).set("y1", current_y_offset)
            .set("x2", margin + current_x).set("y2", current_y_offset + 250.0)
            .set("stroke", "#CCCCCC").set("stroke-width", 1.0));

        for note in measure.children().filter(|n| n.has_tag_name("note")) {
            let is_chord = note.children().any(|n| n.has_tag_name("chord"));
            if !is_chord {
                current_x += note_spacing;
                // Vertical guide line
                svg_doc = svg_doc.add(Line::new()
                    .set("x1", margin + current_x).set("y1", current_y_offset)
                    .set("x2", margin + current_x).set("y2", current_y_offset + 250.0)
                    .set("stroke", "#00FF00").set("stroke-width", 1));
            }

            if let Some(pitch_node) = note.children().find(|n| n.has_tag_name("pitch")) {
                let step = pitch_node.children().find(|n| n.has_tag_name("step")).unwrap().text().unwrap();
                let octave = pitch_node.children().find(|n| n.has_tag_name("octave")).unwrap().text().unwrap();
                let alter = pitch_node.children().find(|n| n.has_tag_name("alter")).map(|n| n.text().unwrap()).unwrap_or("0");
                let midi = (note_to_midi(&format!("{}{}", step, octave)) as i16 + alter.parse::<i16>().unwrap()) as u8;
                
                if let Some((idx, is_right)) = tuning.left.iter().position(|&p| p == midi).map(|i| (i, false))
                    .or_else(|| tuning.right.iter().position(|&p| p == midi).map(|i| (i, true))) {
                    
                    let color = if is_right { "#FF0000" } else { "#0000FF" }; // Red (Right), Blue (Left)
                    let label = format!("{}", idx + 1);
                    
                    let y_pos = current_y_offset + ((10 - idx) as f64 * 25.0) + 5.0;

                    // Estimate text width to calculate the size of the bounding box
                    let text_width = if label.len() > 1 { 16.0 } else { 8.0 };
                    let rect_width = text_width + 8.0;
                    let rect_height = 16.0;

                    // Position the rectangle so its edge is on the vertical line, preventing overlap.
                    let rect_x = if is_right {
                        margin + current_x
                    } else {
                        margin + current_x - rect_width
                    };
                    let rect_y = y_pos - 12.0; // Center vertically around text baseline.

                    // The text's x position is the center of the rectangle.
                    let x_pos = rect_x + (rect_width / 2.0);

                    // Draw the rounded rectangle.
                    let rect = Rectangle::new()
                        .set("x", rect_x)
                        .set("y", rect_y)
                        .set("width", rect_width)
                        .set("height", rect_height)
                        .set("rx", 4.0)
                        .set("ry", 4.0)
                        .set("fill", "white") // White background to mask staff lines.
                        .set("stroke", color)
                        .set("stroke-width", 1.0);
                    svg_doc = svg_doc.add(rect);

                    // Draw the text label inside the rectangle.
                    svg_doc = svg_doc.add(SvgText::new(label)
                        .set("x", x_pos)
                        .set("y", y_pos)
                        .set("font-family", "Arial, sans-serif")
                        .set("font-size", 13)
                        .set("font-weight", "bold")
                        .set("fill", color)
                        .set("text-anchor", "middle")); // Center the number in the box.
                }
            }
        }
        current_x += 10.0;
        measure_count += 1;
        measures_in_current_line += 1;
    }

    svg::save(&args.output, &svg_doc)?;
    println!("Success! Pro Tab for '{}' generated.", song_title);
    Ok(())
}