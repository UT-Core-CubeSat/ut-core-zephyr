use plotters::prelude::*;
use std::collections::HashMap;
use std::path::Path;
use crate::parameters::Parameters;

// Define a struct for plot variables
pub struct PlotVariable {
    pub color: RGBColor,
    pub unit: String,
    pub index: usize,
    pub value: f64,
    pub disabled: bool,
}

pub struct Plotter {
    time: Vec<f64>,
    index: usize,
    data: HashMap<String, Vec<f64>>,
    plots: HashMap<String, PlotVariable>,
    chart_ctx: DrawingArea<BitMapBackend, plotters::coord::Shift>,
}

impl Plotter {
    pub fn new(plots: HashMap<String, PlotVariable>, parameters: &Parameters) -> Self {
        // Initialize time array
        let time_len = (parameters.length / parameters.sample).ceil() as usize;
        let time = vec![f64::NAN; time_len];
        
        // Initialize data storage
        let mut data = HashMap::new();
        for name in plots.keys() {
            data.insert(name.clone(), vec![f64::NAN; time_len]);
        }
        
        // Create drawing area
        let output_file = "plots.png";
        let root_area = BitMapBackend::new(output_file, (1200, 800)).into_drawing_area();
        
        // Clear the drawing area
        root_area.fill(&WHITE).unwrap();
        
        // Create Plotter instance
        Plotter {
            time,
            index: 0,
            data,
            plots,
            chart_ctx: root_area,
        }
    }
    
    pub fn update(&mut self, plots: &HashMap<String, PlotVariable>, parameters: &Parameters) {
        // Update time
        self.time[self.index] = parameters.time;
        
        // Update data
        for (name, variable) in plots {
            if !variable.disabled {
                if let Some(data_vec) = self.data.get_mut(name) {
                    data_vec[self.index] = variable.value;
                }
            }
        }
        
        // Only redraw occasionally to improve performance
        if self.index % 10 == 0 {
            self.redraw_plots();
        }
        
        // Increment index
        self.index += 1;
    }
    
    fn redraw_plots(&self) {
        // Clear previous plots
        self.chart_ctx.fill(&WHITE).unwrap();
        
        // Create a grid for subplots
        let areas = self.chart_ctx.split_evenly((3, 3));
        
        // Plot each variable
        for (name, variable) in &self.plots {
            if variable.disabled {
                continue;
            }
            
            let area_idx = variable.index - 1; // Convert to 0-based index
            if area_idx >= areas.len() {
                continue;
            }
            
            // Get data
            let data = self.data.get(name).unwrap();
            
            // Filter out NaN values
            let plot_data: Vec<(f64, f64)> = self.time[0..self.index]
                .iter()
                .zip(data[0..self.index].iter())
                .filter(|(t, y)| !t.is_nan() && !y.is_nan())
                .map(|(t, y)| (*t, *y))
                .collect();
            
            if plot_data.is_empty() {
                continue;
            }
            
            // Find min and max for y axis
            let y_min = plot_data.iter().map(|(_, y)| *y).fold(f64::INFINITY, f64::min);
            let y_max = plot_data.iter().map(|(_, y)| *y).fold(f64::NEG_INFINITY, f64::max);
            let y_range = y_max - y_min;
            
            // Create the chart
            let mut chart = ChartBuilder::on(&areas[area_idx])
                ..caption(name, ("sans-serif", 20))
                .margin(5)
                .x_label_area_size(30)
                .y_label_area_size(30)
                .build_cartesian_2d(
                    0.0..self.time[self.index.saturating_sub(1)], 
                    (y_min - 0.1 * y_range)..(y_max + 0.1 * y_range)
                )
                .unwrap();
            
            // Enable mesh
            chart.configure_mesh().draw().unwrap();
            
            // Set Y-axis label with unit
            let ylabel = if variable.unit.is_empty() {
                name.clone()
            } else {
                format!("{}({})", name, variable.unit)
            };
            
            chart.configure_mesh().y_desc(ylabel).draw().unwrap();
            
            // Convert RGB color
            let color = variable.color;
            
            // Draw the line
            chart
                .draw_series(LineSeries::new(plot_data, color))
                .unwrap()
                .label(name)
                .legend(|(x, y)| PathElement::new(vec![(x, y), (x + 20, y)], color));
            
            // Draw the legend
            chart
                .configure_series_labels()
                .background_style(WHITE.mix(0.8))
                .border_style(BLACK)
                .draw()
                .unwrap();
        }
        
        // Save the plot
        println!("Updated plots");
    }
}

// Helper function to create RGB colors
pub fn rgb(r: u8, g: u8, b: u8) -> RGBColor {
    RGBColor(r, g, b)
}