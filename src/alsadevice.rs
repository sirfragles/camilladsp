extern crate alsa;
extern crate nix;

use alsa::ctl::{ElemId, ElemIface, ElemType, ElemValue, HCtl};
use alsa::pcm::{Access, Format, Frames, HwParams, PCM};
use alsa::{Direction, ValueOr};
use alsa_sys;
use parking_lot::{Mutex, RwLock, RwLockUpgradableReadGuard};
use std::ffi::CString;
use std::sync::mpsc;
use std::sync::mpsc::Receiver;
use std::sync::{Arc, Barrier};
use std::thread;
use std::time::Instant;

use crate::alsadevice_buffermanager::{CaptureBufferManager, DeviceBufferManager, PlaybackBufferManager};
use crate::alsadevice_utils::{adjust_speed, list_device_names, state_desc};
use crate::config::{self, SampleFormat};
use crate::conversions::{buffer_to_chunk_rawbytes, chunk_to_buffer_rawbytes};
use crate::countertimer;
use crate::CommandMessage;
use crate::PrcFmt;
use crate::ProcessingState;
use crate::Res;
use crate::StatusMessage;
use crate::{CaptureStatus, PlaybackStatus};

lazy_static! {
    static ref ALSA_MUTEX: Mutex<()> = Mutex::new(());
}

pub struct AlsaPlaybackDevice {
    pub devname: String,
    pub samplerate: usize,
    pub chunksize: usize,
    pub channels: usize,
    pub sample_format: SampleFormat,
    pub target_level: usize,
    pub adjust_period: f32,
    pub enable_rate_adjust: bool,
}

pub struct AlsaCaptureDevice {
    pub devname: String,
    pub samplerate: usize,
    pub capture_samplerate: usize,
    pub resampler_config: Option<config::Resampler>,
    pub chunksize: usize,
    pub channels: usize,
    pub sample_format: SampleFormat,
    pub silence_threshold: PrcFmt,
    pub silence_timeout: PrcFmt,
    pub stop_on_rate_change: bool,
    pub rate_measure_interval: f32,
}

struct CaptureChannels {
    audio: mpsc::SyncSender<AudioMessage>,
    status: crossbeam_channel::Sender<StatusMessage>,
    command: mpsc::Receiver<CommandMessage>,
}

struct PlaybackChannels {
    audio: mpsc::Receiver<AudioMessage>,
    status: crossbeam_channel::Sender<StatusMessage>,
}

struct CaptureParams {
    channels: usize,
    sample_format: SampleFormat,
    silence_timeout: PrcFmt,
    silence_threshold: PrcFmt,
    chunksize: usize,
    store_bytes_per_sample: usize,
    bytes_per_frame: usize,
    samplerate: usize,
    capture_samplerate: usize,
    async_src: bool,
    capture_status: Arc<RwLock<CaptureStatus>>,
    stop_on_rate_change: bool,
    rate_measure_interval: f32,
}

struct PlaybackParams {
    channels: usize,
    target_level: usize,
    adjust_period: f32,
    adjust_enabled: bool,
    sample_format: SampleFormat,
    playback_status: Arc<RwLock<PlaybackStatus>>,
    bytes_per_frame: usize,
    samplerate: usize,
    chunksize: usize,
}

enum CaptureResult {
    Normal,
    Stalled,
}

#[derive(Debug)]
enum PlaybackResult {
    Normal,
    Stalled,
}

/// Play a buffer.
fn play_buffer(
    mut buffer: &[u8],
    pcmdevice: &alsa::PCM,
    io: &alsa::pcm::IO<u8>,
    millis_per_frame: f32,
    bytes_per_frame: usize,
    buf_manager: &mut PlaybackBufferManager,
) -> Res<PlaybackResult> {
    let playback_state = pcmdevice.state_raw();
    if playback_state < 0 {
        let nixerr = alsa::nix::errno::from_i32(-playback_state);
        error!(
            "PB: Alsa snd_pcm_state() of playback device returned an unexpected error: {}",
            nixerr
        );
        return Err(Box::new(nixerr));
    } else if playback_state == alsa_sys::SND_PCM_STATE_XRUN as i32 {
        warn!("PB: Prepare playback after buffer underrun");
        pcmdevice.prepare()?;
        buf_manager.sleep_for_target_delay(millis_per_frame);
    } else if playback_state == alsa_sys::SND_PCM_STATE_PREPARED as i32 {
        info!("PB: Starting playback from Prepared state");
        buf_manager.sleep_for_target_delay(millis_per_frame);
    } else if playback_state != alsa_sys::SND_PCM_STATE_RUNNING as i32 {
        warn!(
            "PB: device is in an unexpected state: {}",
            state_desc(playback_state as u32)
        );
    }

    let frames_to_write = buffer.len() / bytes_per_frame;
    let mut retry_count: usize = 0;
    loop {
        retry_count += 1;
        if retry_count >= 100 {
            warn!("PB: giving up after {} write attempts", retry_count);
            return Err(DeviceError::new("Aborting playback after too many write attempts").into());
        }
        let timeout_millis = (2.0 * millis_per_frame * frames_to_write as f32) as u32;
        match pcmdevice.wait(Some(timeout_millis)) {
            Ok(true) => (),
            Ok(false) => return Ok(PlaybackResult::Stalled),
            Err(err) => {
                warn!("PB: device failed while waiting for available buffer space, error: {}", err);
                return Err(Box::new(err));
            }
        }

        match io.writei(buffer) {
            Ok(frames_written) => {
                let cur_frames_to_write = buffer.len() / bytes_per_frame;
                if frames_written == cur_frames_to_write {
                    break;
                } else {
                    buffer = &buffer[frames_written * bytes_per_frame..];
                }
            }
            Err(err) => {
                if err.nix_error() == alsa::nix::errno::Errno::EAGAIN {
                    trace!("PB: encountered EAGAIN error on write, trying again");
                } else {
                    warn!("PB: write error, trying to recover. Error: {}", err);
                    pcmdevice.prepare()?;
                    buf_manager.sleep_for_target_delay(millis_per_frame);
                    io.writei(buffer)?;
                    break;
                }
            }
        }
    }
    Ok(PlaybackResult::Normal)
}

/// Capture a buffer.
fn capture_buffer(
    mut buffer: &mut [u8],
    pcmdevice: &alsa::PCM,
    io: &alsa::pcm::IO<u8>,
    samplerate: usize,
    frames_to_read: usize,
    bytes_per_frame: usize,
) -> Res<CaptureResult> {
    let capture_state = pcmdevice.state_raw();
    if capture_state == alsa_sys::SND_PCM_STATE_XRUN as i32 {
        warn!("Prepare capture device");
        pcmdevice.prepare()?;
    } else if capture_state < 0 {
        let nixerr = alsa::nix::errno::from_i32(-capture_state);
        error!("Alsa snd_pcm_state() of capture device returned an unexpected error: {}", capture_state);
        return Err(Box::new(nixerr));
    } else if capture_state != alsa_sys::SND_PCM_STATE_RUNNING as i32 {
        debug!("Starting capture from state: {}", state_desc(capture_state as u32));
        pcmdevice.start()?;
    }

    let millis_per_chunk = 1000 * frames_to_read / samplerate;
    loop {
        let timeout_millis = (4 * millis_per_chunk).max(10) as u32;
        match pcmdevice.wait(Some(timeout_millis)) {
            Ok(true) => (),
            Ok(false) => return Ok(CaptureResult::Stalled),
            Err(err) => {
                warn!("Capture device failed while waiting for available frames, error: {}", err);
                return Err(Box::new(err));
            }
        }
        match io.readi(buffer) {
            Ok(frames_read) => {
                let frames_req = buffer.len() / bytes_per_frame;
                if frames_read == frames_req {
                    return Ok(CaptureResult::Normal);
                } else {
                    buffer = &mut buffer[frames_read * bytes_per_frame..];
                }
            }
            Err(err) => match err.nix_error() {
                alsa::nix::errno::Errno::EIO | alsa::nix::errno::Errno::EPIPE => {
                    warn!("Capture failed, error: {}", err);
                    return Err(Box::new(err));
                }
                _ => {
                    warn!("Capture failed, error: {}", err);
                    return Err(Box::new(err));
                }
            },
        }
    }
}

/// Open an Alsa PCM device
fn open_pcm(
    devname: String,
    samplerate: u32,
    channels: u32,
    sample_format: &SampleFormat,
    buf_manager: &mut dyn DeviceBufferManager,
    capture: bool,
) -> Res<alsa::PCM> {
    let direction = if capture { "Capture" } else { "Playback" };
    debug!("Available {} devices: {:?}", direction, list_device_names(capture));

    let _lock = ALSA_MUTEX.lock();
    let pcmdev = alsa::PCM::new(&devname, if capture { Direction::Capture } else { Direction::Playback }, true)?;

    let hwp = HwParams::any(&pcmdev)?;
    hwp.set_channels(channels)?;
    hwp.set_rate(samplerate, ValueOr::Nearest)?;
    hwp.set_format(match sample_format {
        SampleFormat::S16LE => Format::s16(),
        SampleFormat::S24LE => Format::s24(),
        SampleFormat::S24LE3 => Format::s24_3(),
        SampleFormat::S32LE => Format::s32(),
        SampleFormat::FLOAT32LE => Format::float(),
        SampleFormat::FLOAT64LE => Format::float64(),
    })?;
    hwp.set_access(Access::RWInterleaved)?;
    buf_manager.apply_buffer_size(&hwp)?;
    buf_manager.apply_period_size(&hwp)?;

    pcmdev.hw_params(&hwp)?;

    let swp = pcmdev.sw_params_current()?;
    buf_manager.apply_start_threshold(&swp)?;
    buf_manager.apply_avail_min(&swp)?;

    pcmdev.sw_params(&swp)?;
    debug!("{} device \"{}\" successfully opened", direction, devname);
    Ok(pcmdev)
}

fn playback_loop_bytes(
    channels: PlaybackChannels,
    pcmdevice: &alsa::PCM,
    params: PlaybackParams,
    buf_manager: &mut PlaybackBufferManager,
) {
    let mut timer = countertimer::Stopwatch::new();
    let mut chunk_stats = ChunkStats::new(params.channels);
    let mut buffer_avg = countertimer::Averager::new();
    let millis_per_frame: f32 = 1000.0 / params.samplerate as f32;
    let mut device_stalled = false;

    let io = pcmdevice.io_bytes();
    let mut buffer = vec![0u8; params.chunksize * params.bytes_per_frame];

    let mut element_uac2_gadget = setup_elements(pcmdevice, params.channels);
    let mut capture_speed: f64 = 1.0;
    let mut prev_delay_diff: Option<f64> = None;

    loop {
        if let Some(eos) = device_stalled.then(|| drain_check_eos(&channels.audio)).flatten() {
            channels.audio.send(eos).unwrap_or(());
            break;
        }

        match channels.audio.recv() {
            Ok(AudioMessage::Audio(chunk)) => {
                let delay_at_chunk_recvd = if !device_stalled && pcmdevice.state_raw() == alsa_sys::SND_PCM_STATE_RUNNING as i32 {
                    pcmdevice.status().ok().map(|status| status.get_delay())
                } else {
                    None
                };

                chunk_to_buffer_rawbytes(&chunk, &mut buffer, &params.sample_format);
                if let Err(err) = play_buffer(&buffer, pcmdevice, &io, millis_per_frame, params.bytes_per_frame, buf_manager) {
                    channels.status.send(StatusMessage::PlaybackError(err.to_string())).unwrap_or(());
                }

                device_stalled = false;
                chunk.update_stats(&mut chunk_stats);
                let mut playback_status = params.playback_status.write();
                playback_status.clipped_samples += conversion_result.1;
                playback_status.signal_rms.add_record_squared(chunk_stats.rms_linear());
                playback_status.signal_peak.add_record(chunk_stats.peak_linear());

                if let Some(delay) = delay_at_chunk_recvd {
                    if delay != 0 {
                        buffer_avg.add_value(delay as f64);
                    }
                }

                if timer.larger_than_millis((1000.0 * params.adjust_period) as u64) && buffer_avg.average().is_some() {
                    adjust_playback(params, buf_manager, &element_uac2_gadget, &mut capture_speed, &mut prev_delay_diff, buffer_avg.average().unwrap());
                    timer.restart();
                    buffer_avg.restart();
                }
            }
            Ok(AudioMessage::Pause) => (),
            Ok(AudioMessage::EndOfStream) => {
                channels.status.send(StatusMessage::PlaybackDone).unwrap_or(());
                break;
            }
            Err(err) => {
                error!("PB: Message channel error: {}", err);
                channels.status.send(StatusMessage::PlaybackError(err.to_string())).unwrap_or(());
                break;
            }
        }
    }
}

fn setup_elements(pcmdevice: &alsa::PCM, channels: usize) -> Option<Elem> {
    let pcminfo = pcmdevice.info().unwrap();
    let card = pcminfo.get_card();
    let device = pcminfo.get_device();
    let subdevice = pcminfo.get_subdevice();

    if card < 0 {
        return None;
    }

    let h = HCtl::new(&format!("hw:{}", card), false).unwrap();
    h.load().unwrap();

    let mut elid_uac2_gadget = ElemId::new(ElemIface::PCM);
    elid_uac2_gadget.set_device(device);
    elid_uac2_gadget.set_subdevice(subdevice);
    elid_uac2_gadget.set_name(&CString::new("Playback Pitch 1000000").unwrap());

    h.find_elem(&elid_uac2_gadget)
}

fn drain_check_eos(audio: &Receiver<AudioMessage>) -> Option<AudioMessage> {
    let mut eos: Option<AudioMessage> = None;
    while let Some(msg) = audio.try_iter().next() {
        if let AudioMessage::EndOfStream = msg {
            eos = Some(msg);
        }
    }
    eos
}

fn adjust_playback(
    params: PlaybackParams,
    buf_manager: &mut PlaybackBufferManager,
    element_uac2_gadget: &Option<Elem>,
    capture_speed: &mut f64,
    prev_delay_diff: &mut Option<f64>,
    avg_delay: f64,
) {
    if let Some(avg_delay) = avg_delay {
        let (new_capture_speed, new_delay_diff) = adjust_speed(
            avg_delay,
            params.target_level,
            *prev_delay_diff,
            *capture_speed,
        );
        if prev_delay_diff.is_some() {
            *capture_speed = new_capture_speed;
            if let Some(elem_uac2_gadget) = element_uac2_gadget {
                let mut elval = ElemValue::new(ElemType::Integer).unwrap();
                elval.set_integer(0, (1_000_000.0 / *capture_speed) as i32).unwrap();
                elem_uac2_gadget.write(&elval).unwrap();
            }
        }
        *prev_delay_diff = Some(new_delay_diff);
    }
    let mut playback_status = params.playback_status.write();
    playback_status.buffer_level = avg_delay as usize;
    debug!(
        "PB: buffer level: {:.1}, signal rms: {:?}",
        avg_delay,
        playback_status.signal_rms.last()
    );
}

fn capture_loop_bytes(
    channels: CaptureChannels,
    pcmdevice: &alsa::PCM,
    params: CaptureParams,
    mut resampler: Option<Box<dyn VecResampler<PrcFmt>>>,
    buf_manager: &mut CaptureBufferManager,
) {
    let io = pcmdevice.io_bytes();
    let mut element_uac2_gadget = setup_elements(pcmdevice, params.channels);

    let buffer_frames = buf_manager.data().buffersize() as usize;
    let mut buffer = vec![0u8; buffer_frames * params.bytes_per_frame];

    let mut capture_bytes = params.chunksize * params.channels * params.store_bytes_per_sample;
    let mut capture_frames = params.chunksize as Frames;
    let mut averager = countertimer::TimeAverage::new();
    let mut watcher_averager = countertimer::TimeAverage::new();
    let mut valuewatcher = countertimer::ValueWatcher::new(
        params.capture_samplerate as f32,
        RATE_CHANGE_THRESHOLD_VALUE,
        RATE_CHANGE_THRESHOLD_COUNT,
    );
    let rate_measure_interval_ms = (1000.0 * params.rate_measure_interval) as u64;
    let mut rate_adjust = 0.0;
    let mut silence_counter = countertimer::SilenceCounter::new(
        params.silence_threshold,
        params.silence_timeout,
        params.capture_samplerate,
        params.chunksize,
    );
    let mut state = ProcessingState::Running;
    let mut value_range = 0.0;
    let mut device_stalled = false;
    let mut chunk_stats = ChunkStats::new(params.channels);
    let mut channel_mask = vec![true; params.channels];

    loop {
        match channels.command.try_recv() {
            Ok(CommandMessage::Exit) => {
                channels.audio.send(AudioMessage::EndOfStream).unwrap_or(());
                channels.status.send(StatusMessage::CaptureDone).unwrap_or(());
                break;
            }
            Ok(CommandMessage::SetSpeed { speed }) => {
                rate_adjust = speed;
                if let Some(elem_loopback) = &element_loopback {
                    elem_loopback.set_integer(0, (100_000.0 / speed) as i32).unwrap();
                } else if let Some(elem_uac2_gadget) = &element_uac2_gadget {
                    elem_uac2_gadget.set_integer(0, (speed * 1_000_000.0) as i32).unwrap();
                } else if let Some(resampl) = &mut resampler {
                    if params.async_src {
                        if resampl.set_resample_ratio_relative(speed, true).is_err() {
                            debug!("Failed to set resampling speed to {}", speed);
                        }
                    } else {
                        warn!("Requested rate adjust of synchronous resampler. Ignoring request.");
                    }
                }
            }
            Err(mpsc::TryRecvError::Empty) => (),
            Err(mpsc::TryRecvError::Disconnected) => {
                error!("Command channel was closed");
                break;
            }
        };

        let (new_capture_bytes, new_capture_frames) = nbr_capture_bytes_and_frames(
            capture_bytes,
            capture_frames,
            &resampler,
            &params,
            &mut buffer,
        );
        if new_capture_bytes != capture_bytes {
            capture_bytes = new_capture_bytes;
            capture_frames = new_capture_frames;
            update_avail_min(pcmdevice, new_capture_frames, buf_manager).unwrap_or(());
        }

        if let Err(err) = capture_buffer(
            &mut buffer[0..capture_bytes],
            pcmdevice,
            &io,
            params.capture_samplerate,
            capture_frames as usize,
            params.bytes_per_frame,
        ) {
            channels.status.send(StatusMessage::CaptureError(err.to_string())).unwrap_or(());
            channels.audio.send(AudioMessage::EndOfStream).unwrap_or(());
            return;
        };

        process_capture_data(
            channels,
            buf_manager,
            &params,
            &mut resampler,
            &mut averager,
            &mut watcher_averager,
            &mut valuewatcher,
            rate_measure_interval_ms,
            rate_adjust,
            &mut silence_counter,
            &mut state,
            &mut value_range,
            &mut chunk_stats,
            &mut channel_mask,
            &mut buffer,
            capture_bytes,
        );
    }

    params.capture_status.write().state = ProcessingState::Inactive;
}

fn process_capture_data(
    channels: CaptureChannels,
    buf_manager: &mut CaptureBufferManager,
    params: &CaptureParams,
    resampler: &mut Option<Box<dyn VecResampler<PrcFmt>>>,
    averager: &mut countertimer::TimeAverage,
    watcher_averager: &mut countertimer::TimeAverage,
    valuewatcher: &mut countertimer::ValueWatcher,
    rate_measure_interval_ms: u64,
    rate_adjust: f64,
    silence_counter: &mut countertimer::SilenceCounter,
    state: &mut ProcessingState,
    value_range: &mut f64,
    chunk_stats: &mut ChunkStats,
    channel_mask: &mut Vec<bool>,
    buffer: &mut Vec<u8>,
    capture_bytes: usize,
) {
    let capture_res = capture_buffer(
        &mut buffer[0..capture_bytes],
        pcmdevice,
        &io,
        params.capture_samplerate,
        capture_frames as usize,
        params.bytes_per_frame,
    );

    match capture_res {
        Ok(CaptureResult::Normal) => {
            averager.add_value(capture_bytes);
            let capture_status = params.capture_status.upgradable_read();
            if averager.larger_than_millis(capture_status.update_interval as u64) {
                let bytes_per_sec = averager.average();
                averager.restart();
                let measured_rate_f = bytes_per_sec
                    / (params.channels * params.store_bytes_per_sample) as f64;
                let mut capture_status = RwLockUpgradableReadGuard::upgrade(capture_status);
                capture_status.measured_samplerate = measured_rate_f as usize;
                capture_status.signal_range = *value_range as f32;
                capture_status.rate_adjust = rate_adjust as f32;
                capture_status.state = *state;
            }

            watcher_averager.add_value(capture_bytes);
            if watcher_averager.larger_than_millis(rate_measure_interval_ms) {
                let bytes_per_sec = watcher_averager.average();
                watcher_averager.restart();
                let measured_rate_f = bytes_per_sec / (params.channels * params.store_bytes_per_sample) as f64;
                if valuewatcher.check_value(measured_rate_f as f32) {
                    if params.stop_on_rate_change {
                        channels.audio.send(AudioMessage::EndOfStream).unwrap_or(());
                        channels.status.send(StatusMessage::CaptureFormatChange(measured_rate_f as usize)).unwrap_or(());
                        return;
                    }
                }
            }
        }
        Ok(CaptureResult::Stalled) => {
            device_stalled = true;
            pcmdevice.drop().unwrap_or_else(|err| warn!("Capture error {:?}", err));
            pcmdevice.prepare().unwrap_or_else(|err| warn!("Capture error {:?}", err));
            params.capture_status.write().state = ProcessingState::Stalled;
        }
        Err(err) => {
            channels.status.send(StatusMessage::CaptureError(err.to_string())).unwrap_or(());
            channels.audio.send(AudioMessage::EndOfStream).unwrap_or(());
            return;
        }
    };

    let mut chunk = buffer_to_chunk_rawbytes(
        &buffer[0..capture_bytes],
        params.channels,
        &params.sample_format,
        capture_bytes,
        &params.capture_status.read().used_channels,
    );
    chunk.update_stats(chunk_stats);
    let mut capture_status = params.capture_status.write();
    capture_status.signal_rms.add_record_squared(chunk_stats.rms_linear());
    capture_status.signal_peak.add_record(chunk_stats.peak_linear());
    *value_range = chunk.maxval - chunk.minval;

    *state = silence_counter.update(*value_range);
    if *state == ProcessingState::Running {
        if let Some(resampl) = &mut resampler {
            chunk.update_channel_mask(channel_mask);
            let new_waves = resampl.process(&chunk.waveforms, Some(channel_mask)).unwrap();
            let mut chunk_frames = new_waves.iter().map(|w| w.len()).max().unwrap_or(params.chunksize);
            chunk.frames = chunk_frames;
            chunk.valid_frames = chunk.frames;
            chunk.waveforms = new_waves;
        }
        channels.audio.send(AudioMessage::Audio(chunk)).unwrap_or(());
    } else if *state == ProcessingState::Paused || *state == ProcessingState::Stalled {
        channels.audio.send(AudioMessage::Pause).unwrap_or(());
    }
}

fn nbr_capture_bytes_and_frames(
    capture_bytes: usize,
    capture_frames: Frames,
    resampler: &Option<Box<dyn VecResampler<PrcFmt>>>,
    params: &CaptureParams,
    buf: &mut Vec<u8>,
) -> (usize, Frames) {
    let (capture_bytes_new, capture_frames_new) = if let Some(resampl) = &resampler {
        let frames = resampl.input_frames_next();
        (
            frames * params.channels * params.store_bytes_per_sample,
            frames as Frames,
        )
    } else {
        (capture_bytes, capture_frames)
    };
    if capture_bytes_new > buf.len() {
        buf.extend(vec![0u8; capture_bytes_new - buf.len()]);
    }
    (capture_bytes_new, capture_frames_new)
}

fn update_avail_min(
    pcmdevice: &PCM,
    frames: Frames,
    buf_manager: &mut dyn DeviceBufferManager,
) -> Res<()> {
    let swp = pcmdevice.sw_params_current()?;
    buf_manager.update_io_size(&swp, frames)?;
    pcmdevice.sw_params(&swp)?;
    Ok(())
}

/// Start a playback thread listening for AudioMessages via a channel.
impl PlaybackDevice for AlsaPlaybackDevice {
    fn start(
        &mut self,
        channel: mpsc::Receiver<AudioMessage>,
        barrier: Arc<Barrier>,
        status_channel: crossbeam_channel::Sender<StatusMessage>,
        playback_status: Arc<RwLock<PlaybackStatus>>,
    ) -> Res<Box<thread::JoinHandle<()>>> {
        let devname = self.devname.clone();
        let target_level = self.target_level.max(self.chunksize);
        let adjust_period = self.adjust_period;
        let adjust_enabled = self.enable_rate_adjust;
        let samplerate = self.samplerate;
        let chunksize = self.chunksize;
        let channels = self.channels;
        let bytes_per_sample = self.sample_format.bytes_per_sample();
        let sample_format = self.sample_format;
        let mut buf_manager = PlaybackBufferManager::new(chunksize as Frames, target_level as Frames);

        let handle = thread::Builder::new()
            .name("AlsaPlayback".to_string())
            .spawn(move || {
                match open_pcm(devname, samplerate as u32, channels as u32, &sample_format, &mut buf_manager, false) {
                    Ok(pcmdevice) => {
                        status_channel.send(StatusMessage::PlaybackReady).unwrap_or(());

                        barrier.wait();
                        let pb_params = PlaybackParams {
                            channels,
                            target_level,
                            adjust_period,
                            adjust_enabled,
                            sample_format,
                            playback_status,
                            bytes_per_frame: channels * bytes_per_sample,
                            samplerate,
                            chunksize,
                        };
                        let pb_channels = PlaybackChannels {
                            audio: channel,
                            status: status_channel,
                        };
                        playback_loop_bytes(pb_channels, &pcmdevice, pb_params, &mut buf_manager);
                    }
                    Err(err) => {
                        status_channel.send(StatusMessage::PlaybackError(err.to_string())).unwrap_or(());
                        barrier.wait();
                    }
                }
            })?;
        Ok(Box::new(handle))
    }
}

/// Start a capture thread providing AudioMessages via a channel.
impl CaptureDevice for AlsaCaptureDevice {
    fn start(
        &mut self,
        channel: mpsc::SyncSender<AudioMessage>,
        barrier: Arc<Barrier>,
        status_channel: crossbeam_channel::Sender<StatusMessage>,
        command_channel: mpsc::Receiver<CommandMessage>,
        capture_status: Arc<RwLock<CaptureStatus>>,
    ) -> Res<Box<thread::JoinHandle<()>>> {
        let devname = self.devname.clone();
        let samplerate = self.samplerate;
        let capture_samplerate = self.capture_samplerate;
        let chunksize = self.chunksize;
        let channels = self.channels;
        let store_bytes_per_sample = self.sample_format.bytes_per_sample();
        let silence_timeout = self.silence_timeout;
        let silence_threshold = self.silence_threshold;
        let sample_format = self.sample_format;
        let resampler_config = self.resampler_config;
        let async_src = resampler_is_async(&resampler_config);
        let stop_on_rate_change = self.stop_on_rate_change;
        let rate_measure_interval = self.rate_measure_interval;

        let mut buf_manager = CaptureBufferManager::new(
            chunksize as Frames,
            samplerate as f32 / capture_samplerate as f32,
        );

        let handle = thread::Builder::new()
            .name("AlsaCapture".to_string())
            .spawn(move || {
                let resampler = new_resampler(
                    &resampler_config,
                    channels,
                    samplerate,
                    capture_samplerate,
                    chunksize,
                );
                match open_pcm(devname, capture_samplerate as u32, channels as u32, &sample_format, &mut buf_manager, true) {
                    Ok(pcmdevice) => {
                        status_channel.send(StatusMessage::CaptureReady).unwrap_or(());

                        barrier.wait();
                        let cap_params = CaptureParams {
                            channels,
                            sample_format,
                            silence_timeout,
                            silence_threshold,
                            chunksize,
                            store_bytes_per_sample,
                            bytes_per_frame: channels * store_bytes_per_sample,
                            samplerate,
                            capture_samplerate,
                            async_src,
                            capture_status,
                            stop_on_rate_change,
                            rate_measure_interval,
                        };
                        let cap_channels = CaptureChannels {
                            audio: channel,
                            status: status_channel,
                            command: command_channel,
                        };
                        capture_loop_bytes(cap_channels, &pcmdevice, cap_params, resampler, &mut buf_manager);
                    }
                    Err(err) => {
                        status_channel.send(StatusMessage::CaptureError(err.to_string())).unwrap_or(());
                        barrier.wait();
                    }
                }
            })?;
        Ok(Box::new(handle))
    }
}
