use libloading::Library;
use std::ffi::{CStr, CString, OsStr};
use std::ffi::{c_int, c_uint, c_char, c_void, c_longlong, c_double};
use std::fmt::{Display, Formatter};
use std::ops::{Index, IndexMut};



#[allow(non_camel_case_types)]
type handle_t = *const c_void;

#[allow(dead_code)]
type FrameCallbackPtr = unsafe extern "C" fn(
    handle_t, *mut u16, c_int, *mut c_char, c_int, *mut c_void
);

type OpenSdkFn = unsafe extern fn() -> c_int;
type CloseSdkFn = unsafe extern fn() -> c_int;
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
type GetBinxFn = unsafe extern fn(handle_t, *mut c_int) -> c_int;
type GetBinyFn = unsafe extern fn(handle_t, *mut c_int) -> c_int;
type GetBinxRangeFn = unsafe extern fn(handle_t, *mut c_int, *mut c_int) -> c_int;
type GetBinyRangeFn = unsafe extern fn(handle_t, *mut c_int, *mut c_int) -> c_int;
type SetBinxFn = unsafe extern fn(handle_t, c_int) -> c_int;
type SetBinyFn = unsafe extern fn(handle_t, c_int) -> c_int;
type GetBitDepthFn = unsafe extern fn(handle_t, *mut c_int) -> c_int;

#[allow(dead_code)]
type SetFrameAvailableCallbackFn = unsafe extern fn(
    handle_t, Option<FrameCallbackPtr>, *mut c_void
) -> c_int;

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

impl Drop for Tlc {
    fn drop(&mut self) {
        tlc_call!(&self.lib, "tl_camera_close_sdk", CloseSdkFn;)
            .expect("couldn't close tl_camera SDK");
    }
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

    pub fn open_camera(&self, id: &str) -> TlcResult<Camera> {
        let mut handle = std::ptr::null();

        tlc_call!(
            &self.lib, "tl_camera_open_camera", OpenCameraFn;
            CString::new(id).unwrap().as_ptr(), &mut handle
        )?;

        let cam = Camera::new(&self.lib, handle, id)?;

        Ok(cam)
    }

    pub fn open_first_camera(&self) -> TlcResult<Option<Camera>> {
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



#[derive(Copy, Clone, Debug, PartialEq, Default)]
pub struct Roi {
    pub x0: usize,
    pub y0: usize,
    pub x1: usize,
    pub y1: usize,
    pub bin_x: usize,
    pub bin_y: usize
}

impl Roi {
    pub fn real_area(&self) -> usize {
        self.real_width()*self.real_height()
    }

    pub fn image_area(&self) -> usize {
        self.image_width()*self.image_height()
    }

    pub fn real_width(&self) -> usize {
        self.x0.abs_diff(self.x1)
    }

    pub fn image_width(&self) -> usize {
        self.real_width()/self.bin_x
    }

    pub fn real_height(&self) -> usize {
        self.y0.abs_diff(self.y1)
    }

    pub fn image_height(&self) -> usize {
        self.real_height()/self.bin_y
    }

    pub fn real_dim(&self) -> (usize, usize) {
        (self.real_width(), self.real_height())
    }

    pub fn image_dim(&self) -> (usize, usize) {
        (self.image_width(), self.image_height())
    }
}



pub struct Frame<D: AsRef<[u16]>> {
    pub data: D,
    pub roi: Roi,
    pub index: usize,
    metadata: Vec<c_char>
}

pub type OFrame = Frame<Vec<u16>>;
pub type BFrame<'a> = Frame<&'a [u16]>;

impl<D: AsRef<[u16]>> Index<[usize; 2]> for Frame<D> {
    type Output = u16;

    fn index(&self, idx: [usize; 2]) -> &u16 {
        let (w, h) = self.roi.image_dim();

        if idx[0] >= w { panic!("x index out of range in frame"); }
        if idx[1] >= h { panic!("y index out of range in frame"); }

        &self.data.as_ref()[w*idx[1]+idx[0]]
    }
}

impl<D: AsRef<[u16]> + AsMut<[u16]>> IndexMut<[usize; 2]> for Frame<D> {
    fn index_mut(&mut self, idx: [usize; 2]) -> &mut u16 {
        let (w, h) = self.roi.image_dim();

        if idx[0] >= w { panic!("x index out of range in frame"); }
        if idx[1] >= h { panic!("y index out of range in frame"); }

        &mut self.data.as_mut()[w*idx[1]+idx[0]]
    }
}

impl<D: AsRef<[u16]>> Frame<D> {
    pub fn width(&self) -> usize {
        self.roi.image_width()
    }

    pub fn height(&self) -> usize {
        self.roi.image_height()
    }

    pub fn dim(&self) -> (usize, usize) {
        (self.width(), self.height())
    }
}



pub struct Camera<'a> {
    lib: &'a Library,
    handle: handle_t,
    id: String,
    roi: Roi
}

impl<'a> Drop for Camera<'a> {
    fn drop(&mut self) {
        self.disarm().expect("couldn't disarm camera");

        tlc_call!(
            &self.lib, "tl_camera_close_camera", CloseCameraFn;
            self.handle
        ).expect("couldn't close camera");
    }
}

impl<'a> Camera<'a> {
    fn new(lib: &'a Library, handle: handle_t, id: &str) -> TlcResult<Self> {
        let mut cam = Self {
            lib, handle,
            id: id.to_string(),
            roi: Roi::default()
        };

        cam.set_frame_timeout_ms(10)?;
        cam.update_roi()?;

        Ok(cam)
    }

    // Update the cached ROI by reading from the camera.
    // This is only called when the camera is initialised, and any subsequent
    // operations involving the ROI can set/get the cached version, and send it
    // to the camera when necessary.
    fn update_roi(&mut self) -> TlcResult<()> {
        let (bin_x, bin_y) = self.bin_size()?;
        let (mut x0, mut y0, mut x1, mut y1) = (0, 0, 0, 0);

        tlc_call!(
            &self.lib, "tl_camera_get_roi", GetRoiFn;
            self.handle, &mut x0, &mut y0, &mut x1, &mut y1
        )?;

        self.roi.x0 = x0 as usize;
        self.roi.y0 = y0 as usize;
        self.roi.x1 = x1 as usize+1;
        self.roi.y1 = y1 as usize+1;
        self.roi.bin_x = bin_x;
        self.roi.bin_y = bin_y;

        Ok(())
    }

    pub fn roi(&self) -> Roi {
        self.roi
    }

    pub fn id(&self) -> &str {
        &self.id
    }

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

    pub fn set_frame_timeout_ms(&self, t: u64) -> TlcResult<()> {
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

    pub fn map_pending_frame<F, R>(&self, mut f: F) -> TlcResult<Option<R>>
    where F: FnMut(BFrame) -> R {
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

        let img_len = self.roi.image_area();
        let img = unsafe {
            std::slice::from_raw_parts(img_ptr, img_len)
        };

        let metadata = unsafe {
            std::slice::from_raw_parts(metadata_ptr, metadata_len as usize)
        };

        let frame = BFrame {
            data: img,
            roi: self.roi,
            index: frame_idx as usize,
            metadata: metadata.to_vec()
        };

        Ok(Some(f(frame)))
    }

    pub fn pending_frame(&self) -> TlcResult<Option<OFrame>> {
        self.map_pending_frame(|f| {
            OFrame {
                data: f.data.to_vec(),
                roi: f.roi,
                index: f.index,
                metadata: f.metadata
            }
        })
    }

    pub fn map_await_frame<F, R>(&self, timeout_ms: u64, mut f: F)
    -> TlcResult<Option<R>> where F: FnMut(BFrame) -> R {
        let inst = std::time::Instant::now();
        let mut result = None;

        while (inst.elapsed().as_millis() as u64) < timeout_ms {
            result = self.map_pending_frame(&mut f)?;

            if result.is_some() { break; }
        }

        Ok(result)
    }

    pub fn await_frame(&self, timeout_ms: u64) -> TlcResult<Option<OFrame>> {
        self.map_await_frame(timeout_ms, |f| {
            OFrame {
                data: f.data.to_vec(),
                roi: f.roi,
                index: f.index,
                metadata: f.metadata
            }
        })
    }

    pub fn snapshot(&self, timeout_ms: u64) -> TlcResult<Option<OFrame>> {
        self.set_frames_per_trigger(Frames::Limited(1))?;
        self.arm(2)?;
        self.trigger()?;

        let frame = self.await_frame(timeout_ms)?;

        self.disarm()?;

        Ok(frame)
    }

    #[allow(dead_code)]
    fn set_frame_callback_raw(
        &self, f: Option<FrameCallbackPtr>, ctx: *mut c_void
    ) -> TlcResult<()> {
        tlc_call!(
            &self.lib, "tl_camera_set_frame_available_callback",
            SetFrameAvailableCallbackFn;
            self.handle, f, ctx
        )
    }

    pub fn bin_size(&self) -> TlcResult<(usize, usize)> {
        let (mut w, mut h) = (0, 0);

        tlc_call!(
            &self.lib, "tl_camera_get_binx", GetBinxFn; self.handle, &mut w
        )?;

        tlc_call!(
            &self.lib, "tl_camera_get_biny", GetBinyFn; self.handle, &mut h
        )?;

        Ok((w as usize, h as usize))
    }

    pub fn set_bin_size(&mut self, w: usize, h: usize) -> TlcResult<()> {
        tlc_call!(
            &self.lib, "tl_camera_set_binx", SetBinxFn; self.handle, w as c_int
        )?;

        tlc_call!(
            &self.lib, "tl_camera_set_biny", SetBinyFn; self.handle, h as c_int
        )?;

        self.update_roi()
    }

    pub fn bin_range(&self) -> TlcResult<(usize, usize, usize, usize)> {
        let (mut w_min, mut w_max) = (0, 0);
        let (mut h_min, mut h_max) = (0, 0);

        tlc_call!(
            &self.lib, "tl_camera_get_binx_range", GetBinxRangeFn;
            self.handle, &mut w_min, &mut w_max
        )?;
        
        tlc_call!(
            &self.lib, "tl_camera_get_biny_range", GetBinyRangeFn;
            self.handle, &mut h_min, &mut h_max
        )?;

        Ok((w_min as usize, w_max as usize, h_min as usize, h_max as usize))
    }

    pub fn bit_depth(&self) -> TlcResult<usize> {
        let mut depth = 0;

        tlc_call!(
            &self.lib, "tl_camera_get_bit_depth", GetBitDepthFn;
            self.handle, &mut depth
        )?;

        Ok(depth as usize)
    }
}
