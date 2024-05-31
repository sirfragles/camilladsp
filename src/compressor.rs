use crate::audiodevice::AudioChunk;
use crate::config;
use crate::filters::Processor;
use crate::limiter::Limiter;
use crate::PrcFmt;
use crate::Res;

#[derive(Clone, Debug)]
pub struct Compressor {
    pub name: String,
    pub channels: usize,
    pub monitor_channels: Vec<usize>,
    pub process_channels: Vec<usize>,
    pub attack: PrcFmt,
    pub release: PrcFmt,
    pub threshold: PrcFmt,
    pub factor: PrcFmt,
    pub makeup_gain: PrcFmt,
    pub limiter: Option<Limiter>,
    pub samplerate: usize,
    pub scratch: Vec<PrcFmt>,
    pub prev_loudness: PrcFmt,
}

impl Compressor {
    /// Creates a Compressor from a config struct
    pub fn from_config(
        name: &str,
        config: config::CompressorParameters,
        samplerate: usize,
        chunksize: usize,
    ) -> Self {
        let name = name.to_string();
        let channels = config.channels;
        let srate = samplerate as PrcFmt;
        let monitor_channels = config.monitor_channels().or_else(|| (0..channels).collect());
        let process_channels = config.process_channels().or_else(|| (0..channels).collect());

        let attack = (-1.0 / srate / config.attack).exp();
        let release = (-1.0 / srate / config.release).exp();
        let clip_limit = config.clip_limit.map(|lim| (10.0 as PrcFmt).powf(lim / 20.0));

        let scratch = vec![0.0; chunksize];
        let limiter = config.clip_limit.map(|limit| {
            let limitconf = config::LimiterParameters {
                clip_limit: limit,
                soft_clip: config.soft_clip,
            };
            Limiter::from_config("Limiter", limitconf)
        });

        debug!(
            "Creating compressor '{}', channels: {}, monitor_channels: {:?}, process_channels: {:?}, attack: {}, release: {}, threshold: {}, factor: {}, makeup_gain: {}, soft_clip: {}, clip_limit: {:?}",
            name, channels, process_channels, monitor_channels, attack, release, config.threshold, config.factor, config.makeup_gain(), config.soft_clip(), clip_limit
        );

        Compressor {
            name,
            channels,
            monitor_channels,
            process_channels,
            attack,
            release,
            threshold: config.threshold,
            factor: config.factor,
            makeup_gain: config.makeup_gain(),
            limiter,
            samplerate,
            scratch,
            prev_loudness: -100.0,
        }
    }

    /// Sum all channels that are included in loudness monitoring, store result in self.scratch
    fn sum_monitor_channels(&mut self, input: &AudioChunk) {
        let first_channel = self.monitor_channels[0];
        self.scratch.copy_from_slice(&input.waveforms[first_channel]);
        for &ch in &self.monitor_channels[1..] {
            self.scratch.iter_mut().zip(&input.waveforms[ch]).for_each(|(acc, &val)| *acc += val);
        }
    }

    /// Estimate loudness, store result in self.scratch
    fn estimate_loudness(&mut self) {
        for val in &mut self.scratch {
            *val = 20.0 * (val.abs() + 1.0e-9).log10();
            *val = if *val >= self.prev_loudness {
                self.attack * self.prev_loudness + (1.0 - self.attack) * *val
            } else {
                self.release * self.prev_loudness + (1.0 - self.release) * *val
            };
            self.prev_loudness = *val;
        }
    }

    /// Calculate linear gain, store result in self.scratch
    fn calculate_linear_gain(&mut self) {
        for val in &mut self.scratch {
            *val = if *val > self.threshold {
                -(*val - self.threshold) * (self.factor - 1.0) / self.factor
            } else {
                0.0
            };
            *val += self.makeup_gain;
            *val = (10.0 as PrcFmt).powf(*val / 20.0);
        }
    }

    fn apply_gain(&self, input: &mut [PrcFmt]) {
        input.iter_mut().zip(&self.scratch).for_each(|(val, &gain)| *val *= gain);
    }

    fn apply_limiter(&self, input: &mut [PrcFmt]) {
        if let Some(limiter) = &self.limiter {
            limiter.apply_clip(input);
        }
    }
}

impl Processor for Compressor {
    fn name(&self) -> &str {
        &self.name
    }

    /// Apply a Compressor to an AudioChunk, modifying it in-place.
    fn process_chunk(&mut self, input: &mut AudioChunk) -> Res<()> {
        self.sum_monitor_channels(input);
        self.estimate_loudness();
        self.calculate_linear_gain();
        for &ch in &self.process_channels {
            self.apply_gain(&mut input.waveforms[ch]);
            self.apply_limiter(&mut input.waveforms[ch]);
        }
        Ok(())
    }

    fn update_parameters(&mut self, config: config::Processor) {
        if let config::Processor::Compressor { parameters: config, .. } = config {
            let channels = config.channels;
            let srate = self.samplerate as PrcFmt;

            self.monitor_channels = config.monitor_channels().or_else(|| (0..channels).collect());
            self.process_channels = config.process_channels().or_else(|| (0..channels).collect());

            self.attack = (-1.0 / srate / config.attack).exp();
            self.release = (-1.0 / srate / config.release).exp();
            let clip_limit = config.clip_limit.map(|lim| (10.0 as PrcFmt).powf(lim / 20.0));

            self.limiter = config.clip_limit.map(|limit| {
                let limitconf = config::LimiterParameters {
                    clip_limit: limit,
                    soft_clip: config.soft_clip,
                };
                Limiter::from_config("Limiter", limitconf)
            });

            self.threshold = config.threshold;
            self.factor = config.factor;
            self.makeup_gain = config.makeup_gain();

            debug!(
                "Updated compressor '{}', monitor_channels: {:?}, process_channels: {:?}, attack: {}, release: {}, threshold: {}, factor: {}, makeup_gain: {}, soft_clip: {}, clip_limit: {:?}",
                self.name, self.monitor_channels, self.process_channels, self.attack, self.release, self.threshold, self.factor, self.makeup_gain, config.soft_clip(), clip_limit
            );
        } else {
            panic!("Invalid config change!");
        }
    }
}

/// Validate the compressor config, to give a helpful message instead of a panic.
pub fn validate_compressor(config: &config::CompressorParameters) -> Res<()> {
    if config.attack <= 0.0 {
        return Err(config::ConfigError::new("Attack value must be larger than zero.").into());
    }
    if config.release <= 0.0 {
        return Err(config::ConfigError::new("Release value must be larger than zero.").into());
    }
    let channels = config.channels;
    for &ch in &config.monitor_channels() {
        if ch >= channels {
            return Err(config::ConfigError::new(&format!(
                "Invalid monitor channel: {}, max is: {}.",
                ch, channels - 1
            )).into());
        }
    }
    for &ch in &config.process_channels() {
        if ch >= channels {
            return Err(config::ConfigError::new(&format!(
                "Invalid channel to process: {}, max is: {}.",
                ch, channels - 1
            )).into());
        }
    }
    Ok(())
}
