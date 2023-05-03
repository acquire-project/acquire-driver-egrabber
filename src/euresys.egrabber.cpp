/// @file Driver wrapping the Euresys EGrabber API.
/// Written to target the ViewWorks VP-151MX-M6H00
#include "device/props/camera.h"
#include "device/kit/camera.h"
#include "device/kit/driver.h"
#include "platform.h"
#include "logger.h"

#include <EGrabber.h>

#include <stdexcept>
#include <unordered_map>
#include <mutex>
#include <cmath>
#include <cstring>

constexpr size_t NBUFFERS = 16;

#define countof(e) (sizeof(e) / sizeof(*(e)))

#define LOG(...) aq_logger(0, __FILE__, __LINE__, __FUNCTION__, __VA_ARGS__)
#define LOGE(...) aq_logger(1, __FILE__, __LINE__, __FUNCTION__, __VA_ARGS__)
#define EXPECT(e, ...)                                                         \
    do {                                                                       \
        if (!(e)) {                                                            \
            LOGE(__VA_ARGS__);                                                 \
            throw std::runtime_error("Expression was false: " #e);             \
        }                                                                      \
    } while (0)
#define CHECK(e) EXPECT(e, "Expression evaluated as false:\n\t%s", #e)

// #define echo(str) echo_(str, __LINE__)
#define echo(str) str

namespace {

std::string
echo_(const std::string& val, int line)
{
    LOG("ECHO STRING %s (line %d)", val.c_str(), line);
    return val;
}

namespace ES = Euresys;

struct EGCamera final : private Camera
{
    explicit EGCamera(const ES::EGrabberCameraInfo& info);
    ~EGCamera();

    void set(struct CameraProperties* properties);
    void get(struct CameraProperties* properties);
    void get_meta(struct CameraPropertyMetadata* meta) const;
    void get_shape(struct ImageShape* shape) const;
    void start();
    void stop();
    void execute_trigger() const;
    void get_frame(void* im, size_t* nbytes, struct ImageInfo* info);

  private:
    mutable ES::EGrabber<> grabber_;
    struct CameraProperties last_known_settings_;
    struct CameraPropertyMetadata last_known_capabilities_;
    uint64_t frame_id_;
    mutable std::mutex lock_;

    // Maps GenICam PixelFormat names to SampleType.
    const std::unordered_map<std::string, SampleType> px_type_table_;
    const std::unordered_map<SampleType, std::string> px_type_inv_table_;

    // Maps GenICam TriggerActivation names to TriggerEdge
    const std::unordered_map<std::string, TriggerEdge> trig_edge_table_;
    const std::unordered_map<TriggerEdge, std::string> trig_edge_inv_table_;

    enum TrigSrc
    {
        Trig_Line0 = 0,
        Trig_Software = 1,
        Trig_Unknown
    };
    // Maps GenICam TriggerSource to TrigSrc
    const std::unordered_map<std::string, TrigSrc> trig_src_table_;

    void query_exposure_time_capabilities_(CameraPropertyMetadata* meta) const;
    void query_binning_capabilities_(CameraPropertyMetadata* meta) const;
    void query_roi_offset_capabilities_(CameraPropertyMetadata* meta) const;
    void query_roi_shape_capabilities_(CameraPropertyMetadata* meta) const;
    void query_pixel_type_capabilities_(CameraPropertyMetadata* meta) const;
    static void query_triggering_capabilities_(CameraPropertyMetadata* meta);

    float maybe_set_exposure_time_us_(float target_us, float last_value_us);
    uint8_t maybe_set_binning(uint8_t target, uint8_t last_value);
    SampleType maybe_set_px_type(SampleType target, SampleType last_known);
    CameraProperties::camera_properties_offset_s maybe_set_offset(
      CameraProperties::camera_properties_offset_s target,
      CameraProperties::camera_properties_offset_s last);
    CameraProperties::camera_properties_shape_s maybe_set_shape(
      CameraProperties::camera_properties_shape_s target,
      CameraProperties::camera_properties_shape_s last);
    void maybe_set_trigger(Trigger& target, const Trigger& last);
};

struct EGDriver final : public Driver
{
    EGDriver();

    uint32_t device_count();
    void describe(DeviceIdentifier* identifier, uint64_t i);
    void open(uint64_t device_id, struct Device** out);
    static void close(struct Device* in);

  private:
    ES::EGenTL gentl_;
};

template<typename K, typename V>
V
at_or(const std::unordered_map<K, V>& table, const K& key, V dflt)
{
    const auto it = table.find(key);
    if (it == std::end(table)) {
        return dflt;
    } else {
        return it->second;
    }
}

enum DeviceStatusCode
eecam_set(struct Camera* self_, struct CameraProperties* settings)
{
    try {
        CHECK(self_);
        ((struct EGCamera*)self_)->set(settings);
        return Device_Ok;
    } catch (const std::exception& exc) {
        LOGE("Exception: %s\n", exc.what());
    } catch (...) {
        LOGE("Exception: (unknown)");
    }
    return Device_Err;
}

enum DeviceStatusCode
eecam_get(const struct Camera* self_, struct CameraProperties* settings)
{
    try {
        CHECK(self_);
        ((struct EGCamera*)self_)->get(settings);
        return Device_Ok;
    } catch (const std::exception& exc) {
        LOGE("Exception: %s\n", exc.what());
    } catch (...) {
        LOGE("Exception: (unknown)");
    }
    return Device_Err;
}

enum DeviceStatusCode
eecam_get_meta(const struct Camera* self_, struct CameraPropertyMetadata* meta)
{
    try {
        CHECK(self_);
        ((struct EGCamera*)self_)->get_meta(meta);
        return Device_Ok;
    } catch (const std::exception& exc) {
        LOGE("Exception: %s\n", exc.what());
    } catch (...) {
        LOGE("Exception: (unknown)");
    }
    return Device_Err;
}

enum DeviceStatusCode
eecam_get_shape(const struct Camera* self_, struct ImageShape* shape)
{
    try {
        CHECK(self_);
        ((struct EGCamera*)self_)->get_shape(shape);
        return Device_Ok;
    } catch (const std::exception& exc) {
        LOGE("Exception: %s\n", exc.what());
    } catch (...) {
        LOGE("Exception: (unknown)");
    }
    return Device_Err;
}

enum DeviceStatusCode
eecam_stop(struct Camera* self_)
{
    try {
        CHECK(self_);
        ((struct EGCamera*)self_)->stop();
        return Device_Ok;
    } catch (const std::exception& exc) {
        LOGE("Exception: %s\n", exc.what());
    } catch (...) {
        LOGE("Exception: (unknown)");
    }
    return Device_Err;
}

enum DeviceStatusCode
eecam_start(struct Camera* self_)
{
    // If things didn't get shut down properly before,
    // sometimes start fails.
    for (int attempts = 0; attempts < 2; ++attempts) {
        try {
            CHECK(self_);
            ((struct EGCamera*)self_)->start();
            return Device_Ok;
        } catch (const std::exception& exc) {
            LOGE("Exception: %s\n", exc.what());
        } catch (...) {
            LOGE("Exception: (unknown)");
        }
        LOGE("Retrying camera start");
        eecam_stop(self_);
    }
    return Device_Err;
}

enum DeviceStatusCode
eecam_execute_trigger(struct Camera* self_)
{
    try {
        CHECK(self_);
        ((struct EGCamera*)self_)->execute_trigger();
        return Device_Ok;
    } catch (const std::exception& exc) {
        LOGE("Exception: %s\n", exc.what());
    } catch (...) {
        LOGE("Exception: (unknown)");
    }
    return Device_Err;
}

enum DeviceStatusCode
eecam_get_frame(struct Camera* self_,
                void* im,
                size_t* nbytes,
                struct ImageInfo* info)
{
    try {
        CHECK(self_);
        ((struct EGCamera*)self_)->get_frame(im, nbytes, info);
        return Device_Ok;
    } catch (const std::exception& exc) {
        LOGE("Exception: %s\n", exc.what());
    } catch (...) {
        LOGE("Exception: (unknown)");
    }
    return Device_Err;
}

uint32_t
eecam_device_count(struct Driver* self_)
{
    try {
        CHECK(self_);
        return ((struct EGDriver*)self_)->device_count();
    } catch (const std::exception& exc) {
        LOGE("Exception: %s\n", exc.what());
    } catch (...) {
        LOGE("Exception: (unknown)");
    }
    return 0;
}

enum DeviceStatusCode
eecam_describe(const struct Driver* self_,
               struct DeviceIdentifier* identifier,
               uint64_t i)
{
    try {
        CHECK(self_);
        ((struct EGDriver*)self_)->describe(identifier, i);
        return Device_Ok;
    } catch (const std::exception& exc) {
        LOGE("Exception: %s\n", exc.what());
    } catch (...) {
        LOGE("Exception: (unknown)");
    }
    return Device_Err;
}

enum DeviceStatusCode
eecam_open(struct Driver* self_, uint64_t device_id, struct Device** out)
{
    try {
        CHECK(self_);
        ((struct EGDriver*)self_)->open(device_id, out);
        return Device_Ok;
    } catch (const std::exception& exc) {
        LOGE("Exception: %s\n", exc.what());
    } catch (...) {
        LOGE("Exception: (unknown)");
    }
    return Device_Err;
}

enum DeviceStatusCode
eecam_close(struct Driver* self_, struct Device* in)
{
    try {
        CHECK(self_);
        ((struct EGDriver*)self_)->close(in);
        return Device_Ok;
    } catch (const std::exception& exc) {
        LOGE("Exception: %s\n", exc.what());
    } catch (...) {
        LOGE("Exception: (unknown)");
    }
    return Device_Err;
}

enum DeviceStatusCode
eecam_shutdown_(struct Driver* self_)
{
    try {
        CHECK(self_);
        delete (EGDriver*)self_;
        return Device_Ok;
    } catch (const std::exception& exc) {
        LOGE("Exception: %s\n", exc.what());
    } catch (...) {
        LOGE("Exception: (unknown)");
    }
    return Device_Err;
}

EGCamera::EGCamera(const ES::EGrabberCameraInfo& info)
  : Camera{ .set = ::eecam_set,
            .get = ::eecam_get,
            .get_meta = ::eecam_get_meta,
            .get_shape = ::eecam_get_shape,
            .start = ::eecam_start,
            .stop = ::eecam_stop,
            .execute_trigger = ::eecam_execute_trigger,
            .get_frame = ::eecam_get_frame,
  }
  , grabber_(info)
  , last_known_settings_{}
  , px_type_table_ {
        { "Mono8", SampleType_u8 },
        { "Mono10", SampleType_u10 },
        { "Mono12", SampleType_u12 },
        { "Mono14", SampleType_u14 },
        { "Mono16", SampleType_u16 },
    }
  , px_type_inv_table_ {
      { SampleType_u8 , "Mono8" },
      { SampleType_u10, "Mono10"},
      { SampleType_u12, "Mono12"},
      { SampleType_u14, "Mono14"},
      { SampleType_u16, "Mono16"},
  }
  ,trig_edge_table_{
      { "RisingEdge", TriggerEdge_Rising },
      { "FallingEdge", TriggerEdge_Falling },
      { "AnyEdge", TriggerEdge_AnyEdge },
      { "LevelHigh", TriggerEdge_LevelHigh },
      { "LevelLow", TriggerEdge_LevelLow },
  }
  ,trig_edge_inv_table_{
      { TriggerEdge_Rising, "RisingEdge"},
      { TriggerEdge_Falling, "FallingEdge"},
      { TriggerEdge_AnyEdge, "AnyEdge"},
      { TriggerEdge_LevelHigh, "LevelHigh"},
      { TriggerEdge_LevelLow, "LevelLow"},
  }
  , trig_src_table_{
      { "Line0", Trig_Line0},
      { "Software", Trig_Software},
  }
  , frame_id_(0)
{
    grabber_.stop(); // just in case
    grabber_.execute<ES::RemoteModule>("AcquisitionStop");
    grabber_.setString<ES::RemoteModule>("TriggerMode", "Off");
    get(&last_known_settings_);
    get_meta(&last_known_capabilities_);
}

EGCamera::~EGCamera()
{
    try {
        stop();
        // Stop should take care of things but we _really_ want the camera
        // to stop with triggering disables when it's closed so that it's
        // available if/when we try to restart it.
        grabber_.execute<ES::RemoteModule>("AcquisitionStop");
        grabber_.setString<ES::RemoteModule>("TriggerMode", "Off");
    } catch (...) {
        ;
    }
}

void
EGCamera::set(struct CameraProperties* properties)
{
    const std::scoped_lock lock(lock_);
    using namespace Euresys;

    last_known_settings_.exposure_time_us = maybe_set_exposure_time_us_(
      properties->exposure_time_us, last_known_settings_.exposure_time_us);

    last_known_settings_.binning =
      maybe_set_binning(properties->binning, last_known_settings_.binning);

    last_known_settings_.pixel_type = maybe_set_px_type(
      properties->pixel_type, last_known_settings_.pixel_type);

    last_known_settings_.offset =
      maybe_set_offset(properties->offset, last_known_settings_.offset);

    last_known_settings_.shape =
      maybe_set_shape(properties->shape, last_known_settings_.shape);

    maybe_set_trigger(properties->input_triggers.frame_start,
                      last_known_settings_.input_triggers.frame_start);

    grabber_.reallocBuffers(NBUFFERS);
}

template<typename T>
static T
clamp(T val, float low, float high)
{
    float fval = float(val);
    return (fval < low)    ? static_cast<T>(low)
           : (fval > high) ? static_cast<T>(high)
                           : val;
}

float
EGCamera::maybe_set_exposure_time_us_(float target_us, float last_value_us)
{
    if (fabsf(target_us - last_value_us) > 1e-9) {
        target_us = clamp(target_us,
                          last_known_capabilities_.exposure_time_us.low,
                          last_known_capabilities_.exposure_time_us.high);
        grabber_.setFloat<Euresys::RemoteModule>("ExposureTime", target_us);
        return target_us;
    }
    return last_value_us;
}

void
EGCamera::get_meta(struct CameraPropertyMetadata* meta) const
{
    const std::scoped_lock lock(lock_);
    query_exposure_time_capabilities_(meta);
    meta->line_interval_us = { .writable = false };
    meta->readout_direction = { .writable = false };
    query_binning_capabilities_(meta);
    query_roi_offset_capabilities_(meta);
    query_roi_shape_capabilities_(meta);
    query_pixel_type_capabilities_(meta);
    query_triggering_capabilities_(meta);
}

void
EGCamera::query_exposure_time_capabilities_(CameraPropertyMetadata* meta) const
{
    using namespace Euresys;

    EXPECT(grabber_.getString<RemoteModule>(
             query::info("ExposureTime", "Unit")) == "us",
           "Expected ExposureTime units to be microseconds");

    meta->exposure_time_us = {
        .writable = (bool)grabber_.getInteger<RemoteModule>(
          query::writeable("ExposureTime")),
        .low = (float)grabber_.getFloat<RemoteModule>("ExposureTimeMinReg"),
        .high = (float)grabber_.getFloat<RemoteModule>("ExposureTimeMaxReg"),
        .type = PropertyType_FloatingPrecision,
    };
}

void
EGCamera::query_binning_capabilities_(CameraPropertyMetadata* meta) const
{
    using namespace Euresys;
    meta->binning = {
        .writable = (bool)grabber_.getInteger<RemoteModule>(
          query::writeable("BinningHorizontal")),
        .low = 1.0,
        .high = 4.0,
        .type = PropertyType_FixedPrecision,
    };
    /* Note:
    Assumes BinningHorizontal and BinningVertical are the same.

    Assumes the available binning is 1,2,4.
    There's a more principled way to do this by doing something like

        grabber.getStringList<RemoteModule>(query::enumEntries("BinningHorizontal"))

    The (available) enum names are "X1", "X2", "X4" for the ViewWorks
    VP-151MX-M6H00
    */
}
void
EGCamera::query_roi_offset_capabilities_(CameraPropertyMetadata* meta) const
{
    using namespace Euresys;
    meta->offset = {
        .x = {
          .writable = (bool)grabber_.getInteger<RemoteModule>(
            query::writeable("OffsetX")),
          .low = (float)grabber_.getInteger<RemoteModule>("OffsetXMinReg"),
          .high = (float)grabber_.getInteger<RemoteModule>("OffsetXMaxReg"),
          .type = PropertyType_FixedPrecision,
        },
        .y = {
          .writable = (bool)grabber_.getInteger<RemoteModule>(
            query::writeable("OffsetY")),
          .low = (float)grabber_.getInteger<RemoteModule>("OffsetYMinReg"),
          .high = (float)grabber_.getInteger<RemoteModule>("OffsetYMaxReg"),
          .type = PropertyType_FixedPrecision,
        },
    };
}
void
EGCamera::query_roi_shape_capabilities_(CameraPropertyMetadata* meta) const
{
    using namespace Euresys;
    meta->shape = {
        .x = {
          .writable = (bool)grabber_.getInteger<RemoteModule>(
            query::writeable("Width")),
          .low = (float)grabber_.getInteger<RemoteModule>("WidthMinReg"),
          .high = (float)grabber_.getInteger<RemoteModule>("WidthMaxReg"),
          .type = PropertyType_FixedPrecision,
        },
        .y = {
          .writable = (bool)grabber_.getInteger<RemoteModule>(
            query::writeable("Height")),
          .low = (float)grabber_.getInteger<RemoteModule>("HeightMinReg"),
          .high = (float)grabber_.getInteger<RemoteModule>("HeightMaxReg"),
          .type = PropertyType_FixedPrecision,
        },
    };
}

void
EGCamera::query_pixel_type_capabilities_(CameraPropertyMetadata* meta) const
{
    meta->supported_pixel_types = 0;
    for (const auto& v : grabber_.getStringList<ES::RemoteModule>(
           ES::query::enumEntries("PixelFormat"))) {
        meta->supported_pixel_types |=
          (1ULL << at_or(px_type_table_, v, SampleType_Unknown));
    }
}

void
EGCamera::query_triggering_capabilities_(CameraPropertyMetadata* meta)
{
    // Hard-coding 1 input trigger line based on manual inspection of
    // Vieworks camera properties.
    meta->triggers = {
        .frame_start = { .input = 1, .output = 0 },
    };
    meta->digital_lines = {
        .line_count=2,
        .names = {
          "Line0",
          "Software",
        },
    };
}

void
EGCamera::get(struct CameraProperties* properties)
{
    const std::scoped_lock lock(lock_);
    using namespace Euresys;
    *properties = {
        .exposure_time_us = (float)grabber_.getFloat<RemoteModule>(echo("ExposureTime")),
        .binning = (uint8_t)grabber_.getInteger<RemoteModule>(echo("BinningHorizontal")),
        .pixel_type =
          at_or(px_type_table_,grabber_.getString<RemoteModule>(echo("PixelFormat")),SampleType_Unknown),
        .offset = {
          .x = (uint32_t)grabber_.getInteger<RemoteModule>(echo("OffsetX")),
          .y = (uint32_t)grabber_.getInteger<RemoteModule>(echo("OffsetY")),
        },
        .shape = {
          .x = (uint32_t)grabber_.getInteger<RemoteModule>(echo("Width")),
          .y = (uint32_t)grabber_.getInteger<RemoteModule>(echo("Height")),
        },
    };
    {
        // There's only one selectable trigger for the Viewworks camera:
        // "ExposureStart". So, we assume that's selected, and get the values.
        // We're also only concerned with "Line0" and "Software"

        // Setup defaults
        Trigger dflt{
            .enable = 0,
            .line = 0, // Line0 by default
            .kind = Signal_Input,
            .edge = TriggerEdge_Rising,
        };
        properties->input_triggers.frame_start = dflt;

        // Read from trigger source
        const auto src =
          at_or(trig_src_table_,
                grabber_.getString<RemoteModule>(echo("TriggerSource")),
                Trig_Unknown);
        switch (src) {
            case Trig_Line0:
            case Trig_Software:
                // The only TriggerSelector on the vieworks is ExposureStart.
                // Treat that as frame_start here.
                properties->input_triggers.frame_start = {
                    .enable = (uint8_t)grabber_.getInteger<RemoteModule>(
                      echo("TriggerMode")),
                    .line = static_cast<uint8_t>(src),
                    .kind = Signal_Input,
                    .edge = at_or(trig_edge_table_,
                                  grabber_.getString<RemoteModule>(
                                    echo("TriggerActivation")),
                                  TriggerEdge_Unknown),
                };
                break;
            default:;
        }
    }

    last_known_settings_ = *properties;
}
uint8_t
EGCamera::maybe_set_binning(uint8_t target, uint8_t last_value)
{
    // FIXME: on some cameras it seems like only horizontal or vertical are
    // writable
    //        this might be when binning is unsupported - i.e only binning=1 is
    //        available.
    if (target != last_value) {
        target = clamp(target,
                       last_known_capabilities_.binning.low,
                       last_known_capabilities_.binning.high);
        if (last_known_capabilities_.binning.writable) {
            grabber_.setInteger<ES::RemoteModule>(echo("BinningHorizontal"),
                                                  target);
            grabber_.setInteger<ES::RemoteModule>(echo("BinningVertical"),
                                                  target);
        }
        return target;
    }
    return last_value;
}

SampleType
EGCamera::maybe_set_px_type(SampleType target, SampleType last_known)
{
    CHECK(target < SampleTypeCount);
    if (target != last_known) {
        grabber_.setString<ES::RemoteModule>(echo("PixelFormat"),
                                             px_type_inv_table_.at(target));
        return target;
    }
    return last_known;
}
CameraProperties::camera_properties_offset_s
EGCamera::maybe_set_offset(CameraProperties::camera_properties_offset_s target,
                           CameraProperties::camera_properties_offset_s last)
{
    if (target.x != last.x) {

        target.x = clamp(target.x,
                         last_known_capabilities_.offset.x.low,
                         last_known_capabilities_.offset.x.high);
        grabber_.setInteger<ES::RemoteModule>(echo("OffsetX"), target.x);
        last.x = target.x;
    }
    if (target.y != last.y) {

        target.y = clamp(target.y,
                         last_known_capabilities_.offset.y.low,
                         last_known_capabilities_.offset.y.high);
        grabber_.setInteger<ES::RemoteModule>(echo("OffsetY"), target.y);
        last.y = target.y;
    }
    return last;
}

CameraProperties::camera_properties_shape_s
EGCamera::maybe_set_shape(CameraProperties::camera_properties_shape_s target,
                          CameraProperties::camera_properties_shape_s last)
{
    if (target.x != last.x) {
        target.x = clamp(target.x,
                         last_known_capabilities_.shape.x.low,
                         last_known_capabilities_.shape.x.high);
        grabber_.setInteger<ES::RemoteModule>(echo("Width"), target.x);
        last.x = target.x;
    }
    if (target.y != last.y) {
        target.y = clamp(target.y,
                         last_known_capabilities_.shape.y.low,
                         last_known_capabilities_.shape.y.high);
        grabber_.setInteger<ES::RemoteModule>(echo("Height"), target.y);
        last.y = target.y;
    }
    return last;
}

bool
is_equal(const Trigger& lhs, const Trigger& rhs)
{
    return memcmp(&lhs, &rhs, sizeof(Trigger)) == 0;
}

void
EGCamera::maybe_set_trigger(Trigger& target, const Trigger& last)
{
    // Only consider frame_start
    if (is_equal(target, last))
        return; // No change

    {
        const char* sources[] = { "Line0", "Software" };
        const char* modes[] = { "Off", "On" };
        const char* activations[] = { "RisingEdge", "FallingEdge" };

        // constraints
        // These are assumptions used in the code below.
        EXPECT(target.line < 2,
               "Trigger line must be Line0 (0) or Software (1). Got: %d",
               target.line);
        EXPECT(target.edge < countof(activations),
               "Trigger edge must be Rising (%d) or Falling (%d). Got: %d",
               TriggerEdge_Rising,
               TriggerEdge_Falling,
               target.edge);
        EXPECT(target.enable < 2,
               "Expect trigger enable to be 0 or 1. Got: %d",
               target.enable);
        target.kind = Signal_Input; // force for Vieworks

        grabber_.setString<ES::RemoteModule>(echo("TriggerSource"),
                                             sources[target.line]);
        grabber_.setString<ES::RemoteModule>(echo("TriggerMode"),
                                             modes[target.enable]);
        grabber_.setString<ES::RemoteModule>(echo("TriggerActivation"),
                                             activations[target.edge]);
    }
}

void
EGCamera::start()
{
    const std::scoped_lock lock(lock_);
    frame_id_ = 0;
    grabber_.reallocBuffers(NBUFFERS);
    grabber_.start();
}

void
EGCamera::stop()
{
    const std::scoped_lock lock(lock_);
    grabber_.stop();
    grabber_.setString<ES::RemoteModule>(echo("TriggerMode"), "Off");
    grabber_.cancelPop();
}

void
EGCamera::get_shape(struct ImageShape* shape) const
{
    const std::scoped_lock lock(lock_);
    uint32_t w = grabber_.getWidth();
    uint32_t h = grabber_.getHeight();

    *shape = {
        .dims = {
            .channels = 1,
            .width = w,
            .height = h,
            .planes = 1,
        },
        .strides = {
          .channels = 1,
          .width = 1,
          .height = w,
          .planes = w*h,
        },
        .type = at_or(px_type_table_,grabber_.getString<ES::RemoteModule>("PixelFormat"),SampleType_Unknown),
    };
}
void
EGCamera::execute_trigger() const
{
    const std::scoped_lock lock(lock_);
    grabber_.execute<ES::RemoteModule>("TriggerSoftware");
}

void
EGCamera::get_frame(void* im, size_t* nbytes, struct ImageInfo* info)
{
    // Locking: This function is basically read-only when it comes to EGCamera
    // state so it doesn't need a scoped lock.

    // Instancing the buffer blocks until the camera acquires the next
    // frame. This could block for an indeterminate amount of time, e.g. when
    // waiting on an external trigger.
    Euresys::ScopedBuffer buffer(grabber_);

    const auto timestamp_ns =
      buffer.getInfo<uint64_t>(ES::gc::BUFFER_INFO_TIMESTAMP_NS);
    const auto height = buffer.getInfo<size_t>(ES::gc::BUFFER_INFO_HEIGHT);

    auto buf_info = buffer.getInfo();
    CHECK(*nbytes >= buf_info.size);
    EXPECT(buf_info.base, "Expected non-null pointer");

    if (buf_info.deliveredHeight != height) {
        LOGE("Delivered height and height are different: %d != %d",
             (int)buf_info.deliveredHeight,
             (int)height);
    }

    std::memcpy(im, buf_info.base, buf_info.size);
    *info = {
        .shape = {
              .dims = { .channels = 1,
                        .width = (uint32_t)buf_info.width,
                        .height = (uint32_t)height,
                        .planes = 1 },
              .strides = { .channels = 1,
                           .width = 1,
                           .height = (int64_t)buf_info.width,
                           .planes = (int64_t)(buf_info.width * height),
              },
              .type = at_or(px_type_table_, buf_info.pixelFormat,
              SampleType_Unknown),
          },
          .hardware_timestamp = timestamp_ns,
          .hardware_frame_id = frame_id_++,
    };
}

//
//      EGDRIVER IMPLEMENTATION
//

EGDriver::EGDriver()
  : Driver{
      .device_count = ::eecam_device_count,
      .describe = ::eecam_describe,
      .open = ::eecam_open,
      .close = ::eecam_close,
      .shutdown = ::eecam_shutdown_,
  }
{
}

void
EGDriver::describe(DeviceIdentifier* identifier, uint64_t i)
{
    ES::EGrabberDiscovery discovery(gentl_);
    discovery.discover();

    // EGrabber api expects an int32
    // DeviceManager device_id expects a uint8
    EXPECT(i < (1 << 8), "Expected a uint8 device index. Got: %llu", i);

    Euresys::EGrabber<> grabber(discovery.cameras((int)i));

    const auto vendor_name =
      grabber.getString<Euresys::RemoteModule>("DeviceVendorName");
    const auto device_name =
      grabber.getString<Euresys::RemoteModule>("DeviceModelName");
    const auto device_sn =
      grabber.getString<Euresys::RemoteModule>("DeviceSerialNumber");

    *identifier = DeviceIdentifier{
        .device_id = (uint8_t)i,
        .kind = DeviceKind_Camera,
    };

    snprintf(identifier->name,
             sizeof(identifier->name),
             "%s %s %s",
             vendor_name.c_str(),
             device_name.c_str(),
             device_sn.c_str());
}

uint32_t
EGDriver::device_count()
{
    ES::EGrabberDiscovery discovery(gentl_);
    discovery.discover();
    return discovery.cameraCount();
}

void
EGDriver::open(uint64_t device_id, struct Device** out)
{
    CHECK(out);
    EXPECT(device_id < (1ULL << 8 * sizeof(int)) - 1,
           "Expected an int32 device id. Got: %llu",
           device_id);

    Euresys::EGrabberDiscovery discovery(gentl_);
    discovery.discover();

    *out = (Device*)new EGCamera(discovery.cameras((int)device_id));
}

void
EGDriver::close(struct Device* in)
{
    CHECK(in);
    auto camera = (EGCamera*)in;
    delete camera;
}

} // end anonymous namespace

acquire_export struct Driver*
acquire_driver_init_v0(acquire_reporter_t reporter)
{
    try {
        logger_set_reporter(reporter);
        return new EGDriver;
    } catch (const std::exception& exc) {
        LOGE("Exception: %s\n", exc.what());
    } catch (...) {
        LOGE("Exception: (unknown)");
    }
    return nullptr;
}

// TODO: (nclack) use BufferInfo in get_shape?
// TODO: (nclack) Timestamp and frame id
