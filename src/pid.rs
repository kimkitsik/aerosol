pub struct PID {
    prev_error: f64,
    pub kp: f64,
    pub kd: f64,
    pub target_temp: f64,
}

impl PID {
    pub fn new() -> Self {
        PID {
            prev_error: 0.0,
            kp: 0.0,
            kd: 0.0,
            target_temp: 0.0,
        }
    }

    pub fn set_input(&mut self, temp_read: f32) -> f64 {
        let mut pid_error = 0.0;
        let mut pid_value = 0.0;

        //PID constants
        //let (mut kp, mut kd) = (0.06, 0.1);
        let (mut pid_p, mut pid_d) = (0.0, 0.0);

        //PID abil temperatuuri kontrollimine
        pid_error = self.target_temp - temp_read as f64; //arvutame vea sihttemp ja päris temp vahel
        pid_p = (self.kp * pid_error) as f64;
        pid_d = self.kd * (pid_error - self.prev_error);
        pid_value = pid_p + pid_d;

        if pid_value < 0.0 {
            pid_value = 0.0
        }
        if pid_value > 1.0 {
            pid_value = 1.0
        }

        self.prev_error = pid_error;

        pid_value
    }
}