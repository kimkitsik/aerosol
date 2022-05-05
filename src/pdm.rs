pub struct Pdm {
    period: u32,
    shift: u32,

    next_update_time: u64,

    target_value: f32,
    current_output: bool,
    output_accumulator: f32,
}

impl Pdm {
    pub fn new(period: u32, shift: u32) -> Self {
        Self {
            period,
            shift,
            next_update_time: 0,
            target_value: 0.0,
            current_output: false,
            output_accumulator: 0.0,
        }
    }

    pub fn set_target(&mut self, value: f32) {
        if value < 0.0 {
            self.target_value = 0.0
        } else if value > 1.0 {
            self.target_value = 1.0
        } else {
            self.target_value = value;
        }
    }

    pub fn poll(&mut self, time: u64) -> Option<bool> {
        if time < self.next_update_time {
            None
        } else {
            self.next_update_time = (((time.saturating_sub(self.shift as u64)) / self.period as u64 + 1) * self.period as u64).wrapping_add(self.shift as u64);

            self.output_accumulator += self.target_value;
            if self.current_output {
                self.output_accumulator -= 1.0;
            }

            self.current_output = self.output_accumulator > 0.0;
            Some(self.current_output)
        }
    }
}