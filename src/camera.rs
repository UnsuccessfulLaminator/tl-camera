use libloading::Library;
use std::ffi::{CStr, CString, OsStr};
use std::ffi::{c_int, c_uint, c_char, c_void, c_longlong, c_double};
use std::fmt::{Display, Formatter};



#[allow(non_camel_case_types)]
type handle_t = *const c_void;

type OpenSdkFn = unsafe extern fn() -> c_int;
type GetLastErrorFn = unsafe extern fn() -> *const c_char;
type DiscoverAvailableCamerasFn = unsafe extern fn(*mut c_char, c_int) -> c_int;
type OpenCameraFn = unsafe extern fn(*const c_char, *mut handle_t) -> c_int;
type CloseCameraFn = unsafe extern fn(handle_t) -> c_int;
type SetExposureTimeFn = unsafe extern fn(handle_t, c_longlong) -> c_int;
type GetGainRangeFn = unsafe extern fn(handle_t, *mut c_int, *mut c_int) -> c_int;
type ConvertGainToDbFn = unsafe extern fn(handle_t, c_int, *mut c_double) -> c_int;
type ConvertDbToGainFn = unsafe extern fn(handle_t, c_double, *mut c_int) -> c_int;
type GetGainFn = unsafe extern fn(handle_t, *mut c_int) -> c_int;
type SetGainFn = unsafe extern fn(handle_t, c_int) -> c_int;
type SetFramesPerTriggerFn = unsafe extern fn(handle_t, c_uint) -> c_int;
type SetImagePollTimeoutFn = unsafe extern fn(handle_t, c_int) -> c_int;
type ArmFn = unsafe extern fn(handle_t, c_int) -> c_int;
type DisarmFn = unsafe extern fn(handle_t) -> c_int;
type IssueSoftwareTriggerFn = unsafe extern fn(handle_t) -> c_int;

type GetPendingFrameOrNullFn = unsafe extern fn(
    handle_t, *mut *const u16, *mut c_int, *mut *const c_char, *mut c_int
) -> c_int;

type GetRoiFn = unsafe extern fn(
    handle_t, *mut c_int, *mut c_int, *mut c_int, *mut c_int
) -> c_int;

macro_rules! tlc_call {
    ($lib:expr, $name:expr, $sig:ty; $($arg:expr),*) => {
        unsafe {
            if let Ok(f) = $lib.get::<$sig>($name.as_bytes()) {
                if f($($arg),*) == 0 { Ok(()) }
                else { Err(get_last_error($lib)) }
            }
            else { panic!("couldn't get symbol {}", $name) }
        }
    }
}

unsafe fn get_last_error(lib: &Library) -> TlcError {
    if let Ok(f) = lib.get::<GetLastErrorFn>(b"tl_camera_get_last_error") {
        let error_ptr = f();
        let error = CStr::from_ptr(error_ptr)
            .to_string_lossy()
            .into_owned();

        TlcError(error)
    }
    else { panic!("couldn't get error status") }
}



#[derive(Debug)]
pub struct TlcError(pub String);

impl Display for TlcError {
    fn fmt(&self, f: &mut Formatter<'_>) -> Result<(), std::fmt::Error> {
        write!(f, "{}", &self.0)
    }
}

impl std::error::Error for TlcError {}



type TlcResult<T> = Result<T, TlcError>;



pub struct Tlc {
    lib: Library
}

impl Tlc {
    pub fn from_path<P: AsRef<OsStr>>(path: P) -> Result<Self, TlcError> {
        let lib = unsafe { Library::new(path) }.map_err(|_| {
            TlcError("couldn't load library".to_string())
        })?;

        tlc_call!(&lib, "tl_camera_open_sdk", OpenSdkFn;)?;

        Ok(Self { lib })
    }

    pub fn discover_cameras(&self) -> TlcResult<Vec<String>> {
        let mut buf = vec![0u8; 1024];

        tlc_call!(
            &self.lib, "tl_camera_discover_available_cameras",
            DiscoverAvailableCamerasFn;
            buf.as_mut_ptr() as *mut c_char, buf.len() as c_int
        )?;

        let camera_ids = CStr::from_bytes_until_nul(&buf)
            .expect("library returned unterminated string")
            .to_str()
            .expect("library returned invalid UTF-8 string");

        Ok(camera_ids.split(' ')
            .filter(|id| id.len() > 0)
            .map(|id| id.to_string())
            .collect())
    }

    pub fn open_camera(&self, id: &str) -> TlcResult<TlcCamera> {
        let mut handle = std::ptr::null();

        tlc_call!(
            &self.lib, "tl_camera_open_camera", OpenCameraFn;
            CString::new(id).unwrap().as_ptr(), &mut handle
        )?;

        Ok(TlcCamera {
            lib: &self.lib,
            handle
        })
    }

    pub fn open_first_camera(&self) -> TlcResult<Option<TlcCamera>> {
        self.discover_cameras()?
            .first()
            .map(|id| self.open_camera(&id))
            .transpose()
    }
}



#[derive(Copy, Clone, Debug, PartialEq)]
pub enum Frames {
    Unlimited,
    Limited(usize)
}



#[derive(Copy, Clone, Debug, PartialEq)]
pub struct Roi {
    pub x0: usize,
    pub y0: usize,
    pub x1: usize,
    pub y1: usize
}

impl Roi {
    pub fn area(&self) -> usize {
        self.width()*self.height()
    }

    pub fn width(&self) -> usize {
        self.x0.abs_diff(self.x1)
    }

    pub fn height(&self) -> usize {
        self.y0.abs_diff(self.y1)
    }

    pub fn dim(&self) -> (usize, usize) {
        (self.width(), self.height())
    }
}



pub struct Frame<D> {
    pub data: D,
    pub roi: Roi,
    pub index: usize,
    metadata: Vec<c_char>
}



pub struct TlcCamera<'a> {
    lib: &'a Library,
    handle: handle_t
}

impl<'a> Drop for TlcCamera<'a> {
    fn drop(&mut self) {
        tlc_call!(
            &self.lib, "tl_camera_close_camera", CloseCameraFn;
            self.handle
        ).expect("couldn't close camera");
    }
}

impl<'a> TlcCamera<'a> {
    pub fn set_exposure_us(&self, t: usize) -> TlcResult<()> {
        tlc_call!(
            &self.lib, "tl_camera_set_exposure_time", SetExposureTimeFn;
            self.handle, t as c_longlong
        )
    }

    pub fn gain_index_to_db(&self, idx: u64) -> TlcResult<f64> {
        let mut db = 0.;

        tlc_call!(
            &self.lib, "tl_camera_convert_gain_to_decibels", ConvertGainToDbFn;
            self.handle, idx as c_int, &mut db
        )?;

        Ok(db as f64)
    }

    pub fn db_to_gain_index(&self, db: f64) -> TlcResult<u64> {
        let mut idx = 0;

        tlc_call!(
            &self.lib, "tl_camera_convert_decibels_to_gain", ConvertDbToGainFn;
            self.handle, db as c_double, &mut idx
        )?;

        Ok(idx as u64)
    }

    pub fn gain_range(&self) -> TlcResult<(u64, u64)> {
        let (mut min, mut max) = (0, 0);

        tlc_call!(
            &self.lib, "tl_camera_get_gain_range", GetGainRangeFn;
            self.handle, &mut min, &mut max
        )?;

        Ok((min as u64, max as u64))
    }
    
    pub fn gain_range_db(&self) -> TlcResult<(f64, f64)> {
        let (min, max) = self.gain_range()?;
        let min_db = self.gain_index_to_db(min)?;
        let max_db = self.gain_index_to_db(max)?;

        Ok((min_db, max_db))
    }

    pub fn gain(&self) -> TlcResult<u64> {
        let mut gain = 0;

        tlc_call!(
            &self.lib, "tl_camera_get_gain", GetGainFn;
            self.handle, &mut gain
        )?;

        Ok(gain as u64)
    }

    pub fn gain_db(&self) -> TlcResult<f64> {
        self.gain_index_to_db(self.gain()?)
    }

    pub fn set_gain(&self, gain: u64) -> TlcResult<()> {
        tlc_call!(
            &self.lib, "tl_camera_set_gain", SetGainFn;
            self.handle, gain as c_int
        )
    }

    pub fn set_gain_db(&self, gain: f64) -> TlcResult<()> {
        self.set_gain(self.db_to_gain_index(gain)?)
    }

    pub fn set_frames_per_trigger(&self, frames: Frames) -> TlcResult<()> {
        let n = match frames {
            Frames::Unlimited => 0,
            Frames::Limited(n) => n
        };
            
        tlc_call!(
            &self.lib, "tl_camera_set_frames_per_trigger_zero_for_unlimited",
            SetFramesPerTriggerFn;
            self.handle, n as c_uint
        )
    }

    pub fn set_frame_timeout_ms(&self, t: usize) -> TlcResult<()> {
        tlc_call!(
            &self.lib, "tl_camera_set_image_poll_timeout",
            SetImagePollTimeoutFn;
            self.handle, t as c_int
        )
    }

    pub fn arm(&self, buffer_frames: usize) -> TlcResult<()> {
        tlc_call!(
            &self.lib, "tl_camera_arm", ArmFn;
            self.handle, buffer_frames as c_int
        )
    }

    pub fn disarm(&self) -> TlcResult<()> {
        tlc_call!(&self.lib, "tl_camera_disarm", DisarmFn; self.handle)
    }

    pub fn trigger(&self) -> TlcResult<()> {
        tlc_call!(
            &self.lib, "tl_camera_issue_software_trigger",
            IssueSoftwareTriggerFn;
            self.handle
        )
    }

    pub fn roi(&self) -> TlcResult<Roi> {
        let (mut x0, mut y0, mut x1, mut y1) = (0, 0, 0, 0);

        tlc_call!(
            &self.lib, "tl_camera_get_roi", GetRoiFn;
            self.handle, &mut x0, &mut y0, &mut x1, &mut y1
        )?;

        Ok(Roi {
            x0: x0 as usize,
            y0: y0 as usize,
            x1: x1 as usize,
            y1: y1 as usize
        })
    }

    pub fn map_pending_frame<F, R>(&self, mut f: F) -> TlcResult<Option<R>>
    where F: FnMut(Frame<&[u16]>) -> R {
        let mut img_ptr = std::ptr::null();
        let mut frame_idx = 0;
        let mut metadata_ptr = std::ptr::null();
        let mut metadata_len = 0;

        tlc_call!(
            &self.lib, "tl_camera_get_pending_frame_or_null",
            GetPendingFrameOrNullFn;
            self.handle,
            &mut img_ptr, &mut frame_idx, &mut metadata_ptr, &mut metadata_len
        )?;

        if img_ptr.is_null() { return Ok(None); }

        let roi = self.roi()?;
        let img_len = roi.area();
        let img = unsafe {
            std::slice::from_raw_parts(img_ptr, img_len)
        };

        let metadata = unsafe {
            std::slice::from_raw_parts(metadata_ptr, metadata_len as usize)
        };

        let frame = Frame {
            data: img,
            roi: roi,
            index: frame_idx as usize,
            metadata: metadata.to_vec()
        };

        Ok(Some(f(frame)))
    }

    pub fn pending_frame(&self) -> TlcResult<Option<Frame<Vec<u16>>>> {
        self.map_pending_frame(|f| {
            Frame {
                data: f.data.to_vec(),
                roi: f.roi,
                index: f.index,
                metadata: f.metadata
            }
        })
    }
}
