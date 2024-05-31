extern crate alsa;
extern crate nix;

use alsa::pcm::{Frames, HwParams, SwParams};
use std::fmt::Debug;
use std::thread;
use std::time::Duration;

use crate::config;
use crate::Res;

pub trait DeviceBufferManager {
    fn data(&self) -> &DeviceBufferData;
    fn data_mut(&mut self) -> &mut DeviceBufferData;

    fn apply_start_threshold(&mut self, swp: &SwParams) -> Res<()>;

    fn calculate_buffer_size(&self, min_period: Frames) -> Frames {
        let data = self.data();
        let min_size = 4 * min_period;
        let frames_needed = ((3 * data.chunksize) as f32 / data.resampling_ratio).ceil() as Frames;

        // Ensure buffer size is at least 4 times the minimum period size and a power of two
        let buffer_size = frames_needed.max(min_size);
        buffer_size.next_power_of_two()
    }

    fn calculate_buffer_size_alt(&self, min_period: Frames) -> Frames {
        let data = self.data();
        let min_size = 4 * min_period;
        let frames_needed = ((3 * data.chunksize) as f32 / data.resampling_ratio).ceil() as Frames;

        // Ensure buffer size is at least 4 times the minimum period size and 3 times a power of two
        let buffer_size = frames_needed.max(min_size);
        3 * ((buffer_size / 3).next_power_of_two())
    }

    fn apply_buffer_size(&mut self, hwp: &HwParams) -> Res<()> {
        let min_period = hwp.get_period_size_min().unwrap_or(0);
        let buffer_frames = self.calculate_buffer_size(min_period);
        let alt_buffer_frames = self.calculate_buffer_size_alt(min_period);
        let data = self.data_mut();

        debug!("Setting buffer size to {} frames", buffer_frames);
        data.bufsize = match hwp.set_buffer_size_near(buffer_frames) {
            Ok(frames) => frames,
            Err(_) => {
                debug!(
                    "Device did not accept a buffer size of {} frames, trying {}",
                    buffer_frames, alt_buffer_frames
                );
                hwp.set_buffer_size_near(alt_buffer_frames)?
            }
        };

        debug!("Device is using a buffer size of {} frames", data.bufsize);
        Ok(())
    }

    fn apply_period_size(&mut self, hwp: &HwParams) -> Res<()> {
        let data = self.data_mut();
        let period_frames = data.bufsize / 8;

        debug!("Setting period size to {} frames", period_frames);
        data.period = match hwp.set_period_size_near(period_frames, alsa::ValueOr::Nearest) {
            Ok(frames) => frames,
            Err(_) => {
                let alt_period_frames = 3 * (period_frames / 2).next_power_of_two();
                debug!(
                    "Device did not accept a period size of {} frames, trying {}",
                    period_frames, alt_period_frames
                );
                hwp.set_period_size_near(alt_period_frames, alsa::ValueOr::Nearest)?
            }
        };

        Ok(())
    }

    fn apply_avail_min(&mut self, swp: &SwParams) -> Res<()> {
        let data = self.data_mut();

        if data.io_size < data.period {
            warn!(
                "Trying to set avail_min to {}, must be larger than or equal to period of {}",
                data.io_size, data.period
            );
        } else if data.io_size > data.bufsize {
            let msg = format!(
                "Trying to set avail_min to {}, must be smaller than or equal to device buffer size of {}",
                data.io_size, data.bufsize
            );
            error!("{}", msg);
            return Err(config::ConfigError::new(&msg).into());
        }

        data.avail_min = data.io_size;
        swp.set_avail_min(data.avail_min)?;
        Ok(())
    }

    fn update_io_size(&mut self, swp: &SwParams, io_size: Frames) -> Res<()> {
        let data = self.data_mut();
        data.io_size = io_size;
        self.apply_avail_min(swp)?;
        self.apply_start_threshold(swp)?;
        Ok(())
    }

    fn frames_to_stall(&self) -> Frames {
        let data = self.data();
        data.bufsize - data.avail_min + 1
    }
}

#[derive(Debug)]
pub struct DeviceBufferData {
    bufsize: Frames,
    period: Frames,
    threshold: Frames,
    avail_min: Frames,
    io_size: Frames,
    chunksize: Frames,
    resampling_ratio: f32,
}

impl DeviceBufferData {
    pub fn buffersize(&self) -> Frames {
        self.bufsize
    }
}

#[derive(Debug)]
pub struct CaptureBufferManager {
    data: DeviceBufferData,
}

impl CaptureBufferManager {
    pub fn new(chunksize: Frames, resampling_ratio: f32) -> Self {
        let init_io_size = (chunksize as f32 / resampling_ratio).ceil() as Frames;
        CaptureBufferManager {
            data: DeviceBufferData {
                bufsize: 0,
                period: 0,
                threshold: 0,
                avail_min: 0,
                io_size: init_io_size,
                resampling_ratio,
                chunksize,
            },
        }
    }
}

impl DeviceBufferManager for CaptureBufferManager {
    fn data(&self) -> &DeviceBufferData {
        &self.data
    }

    fn data_mut(&mut self) -> &mut DeviceBufferData {
        &mut self.data
    }

    fn apply_start_threshold(&mut self, swp: &SwParams) -> Res<()> {
        let threshold = 0;
        swp.set_start_threshold(threshold)?;
        self.data.threshold = threshold;
        Ok(())
    }
}

#[derive(Debug)]
pub struct PlaybackBufferManager {
    data: DeviceBufferData,
    target_level: Frames,
}

impl PlaybackBufferManager {
    pub fn new(chunksize: Frames, target_level: Frames) -> Self {
        PlaybackBufferManager {
            data: DeviceBufferData {
                bufsize: 0,
                period: 0,
                threshold: 0,
                avail_min: 0,
                io_size: chunksize,
                resampling_ratio: 1.0,
                chunksize,
            },
            target_level,
        }
    }

    pub fn sleep_for_target_delay(&self, millis_per_frame: f32) {
        let sleep_millis = (self.target_level as f32 * millis_per_frame).round() as u64;
        trace!(
            "Sleeping for {} frames = {} ms",
            self.target_level, sleep_millis
        );
        thread::sleep(Duration::from_millis(sleep_millis));
    }
}

impl DeviceBufferManager for PlaybackBufferManager {
    fn data(&self) -> &DeviceBufferData {
        &self.data
    }

    fn data_mut(&mut self) -> &mut DeviceBufferData {
        &mut self.data
    }

    fn apply_start_threshold(&mut self, swp: &SwParams) -> Res<()> {
        let threshold = 1;
        swp.set_start_threshold(threshold)?;
        self.data.threshold = threshold;
        Ok(())
    }
}
