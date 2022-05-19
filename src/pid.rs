pub struct PID {
    prev_input: f32,
    pub kp: f32,
    pub kd: f32,
    pub pid_p: f32,
    pub pid_d: f32,
    pub target_temp: f32,
    output: f32,
}

impl PID {
    pub fn new() -> Self {
        PID {
            prev_input: 0.0,
            kp: 0.0,
            kd: 0.0,
            pid_p: 0.0,
            pid_d: 0.0,
            target_temp: 0.0,
            output: 0.0,
        }
    }

    pub fn set_input(&mut self, input: f32) -> f32 {

        //PID abil temperatuuri kontrollimine
        let pid_error = self.target_temp - input; //arvutame vea sihttemp ja päris temp vahel

        self.pid_p = self.kp * pid_error;
        self.pid_d = self.kd * (input - self.prev_input);

        self.prev_input = input;
        self.output = self.pid_p + self.pid_d;

        if self.output < 0.0 {
            self.output = 0.0
        }
        if self.output > 1.0 {
            self.output = 1.0
        }

        self.prev_input = pid_error;

        self.output
    }
}