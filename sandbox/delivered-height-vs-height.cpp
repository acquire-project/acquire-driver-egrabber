#include "acquire.h"
#include "device/hal/device.manager.h"
#include "platform.h"
#include "logger.h"

#include <cstdio>
#include <iostream>
#include <stdexcept>

static class IntrospectiveLogger
{
  public:
    explicit IntrospectiveLogger()
      : different_heights_(false){};

    // inspect for "[stream 0] Dropped", otherwise pass the mesage through
    void report_and_inspect(int is_error,
                            const char* file,
                            int line,
                            const char* function,
                            const char* msg)
    {
        std::string m(msg);
        if (m.find("Delivered height and height are different:")) {
            different_heights_ = true;
        }

        printf("%s%s(%d) - %s: %s\n",
               is_error ? "ERROR " : "",
               file,
               line,
               function,
               msg);
    }

    [[nodiscard]] bool different_heights() const { return different_heights_; }

  private:
    bool different_heights_;
} introspective_logger;

static void
reporter(int is_error,
         const char* file,
         int line,
         const char* function,
         const char* msg)
{
    introspective_logger.report_and_inspect(
      is_error, file, line, function, msg);
}

/// Helper for passing size static strings as function args.
/// For a function: `f(char*,size_t)` use `f(SIZED("hello"))`.
/// Expands to `f("hello",5)`.
#define SIZED(str) str, sizeof(str)

#define L (aq_logger)
#define LOG(...) L(0, __FILE__, __LINE__, __FUNCTION__, __VA_ARGS__)
#define ERR(...) L(1, __FILE__, __LINE__, __FUNCTION__, __VA_ARGS__)
#define EXPECT(e, ...)                                                         \
    do {                                                                       \
        if (!(e)) {                                                            \
            char buf[1 << 8] = { 0 };                                          \
            ERR(__VA_ARGS__);                                                  \
            snprintf(buf, sizeof(buf) - 1, __VA_ARGS__);                       \
            throw std::runtime_error(buf);                                     \
        }                                                                      \
    } while (0)
#define CHECK(e) EXPECT(e, "Expression evaluated as false: %s", #e)
#define DEVOK(e) CHECK(Device_Ok == (e))
#define OK(e) CHECK(AcquireStatus_Ok == (e))

int
main()
{
    auto runtime = acquire_init(reporter);

    try {
        CHECK(runtime);
        auto dm = acquire_device_manager(runtime);
        CHECK(dm);

        AcquireProperties props = {};
        OK(acquire_get_configuration(runtime, &props));

        DEVOK(device_manager_select(dm,
                                    DeviceKind_Camera,
                                    SIZED("VIEWORKS.*") - 1,
                                    &props.video[0].camera.identifier));
        DEVOK(device_manager_select(dm,
                                    DeviceKind_Storage,
                                    SIZED("tiff") - 1,
                                    &props.video[0].storage.identifier));

        storage_properties_init(&props.video[0].storage.settings,
                                0,
                                SIZED("vieworks.zarr"),
                                0,
                                0,
                                { 0 },
                                0);

        // avoid initing w zero shape
        props.video[0].camera.settings.shape = {
            .x = 14192,
            .y = 10640,
        };

        OK(acquire_configure(runtime, &props));

        AcquirePropertyMetadata metadata = { 0 };
        OK(acquire_get_configuration_metadata(runtime, &metadata));

        props.video[0].camera.settings.binning = 1;
        props.video[0].camera.settings.pixel_type = SampleType_u12;
        props.video[0].camera.settings.shape = {
            .x = (uint32_t)metadata.video[0].camera.shape.x.high,
            .y = (uint32_t)metadata.video[0].camera.shape.y.high,
        };
        props.video[0].camera.settings.exposure_time_us = 1e4;
        props.video[0].max_frame_count = 1000;

        OK(acquire_configure(runtime, &props));

        const auto next = [](VideoFrame* cur) -> VideoFrame* {
            return (VideoFrame*)(((uint8_t*)cur) + cur->bytes_of_frame);
        };

        const auto consumed_bytes = [](const VideoFrame* const cur,
                                       const VideoFrame* const end) -> size_t {
            return (uint8_t*)end - (uint8_t*)cur;
        };

        struct clock clock;
        static double time_limit_ms =
          (double)props.video[0].max_frame_count *
          props.video[0].camera.settings.exposure_time_us / 1000.0;
        clock_init(&clock);
        clock_shift_ms(&clock, time_limit_ms);
        OK(acquire_start(runtime));
        {
            uint64_t nframes = 0;
            VideoFrame *beg, *end, *cur;

            while (DeviceState_Running == acquire_get_state(runtime) &&
                   nframes < props.video[0].max_frame_count &&
                   !introspective_logger.different_heights()) {
                struct clock throttle = { 0 };
                clock_init(&throttle);
                EXPECT(clock_cmp_now(&clock) < 0,
                       "Timeout at %f ms",
                       clock_toc_ms(&clock) + time_limit_ms);
                OK(acquire_map_read(runtime, 0, &beg, &end));
                for (cur = beg; cur < end; cur = next(cur)) {
                    LOG("stream %d counting frame w id %d (nframes = %d)",
                        0,
                        cur->frame_id,
                        nframes);
                    CHECK(cur->shape.dims.width ==
                          props.video[0].camera.settings.shape.x);
                    CHECK(cur->shape.dims.height ==
                          props.video[0].camera.settings.shape.y);
                    ++nframes;
                }

                {
                    uint32_t n = (uint32_t)consumed_bytes(beg, end);
                    OK(acquire_unmap_read(runtime, 0, n));
                    if (n)
                        LOG("stream %d consumed bytes %d", 0, n);
                }
                clock_sleep_ms(&throttle, 100.0f);

                LOG("stream %d nframes %d. remaining time %f s",
                    0,
                    nframes,
                    -1e-3 * clock_toc_ms(&clock));
            }

            do {
                OK(acquire_map_read(runtime, 0, &beg, &end));
                for (cur = beg; cur < end; cur = next(cur)) {
                    LOG("[Flush] stream %d counting frame w id %d",
                        0,
                        cur->frame_id);
                    CHECK(cur->shape.dims.width ==
                          props.video[0].camera.settings.shape.x);
                    CHECK(cur->shape.dims.height ==
                          props.video[0].camera.settings.shape.y);
                    ++nframes;
                }

                {
                    uint32_t n = (uint32_t)consumed_bytes(beg, end);
                    OK(acquire_unmap_read(runtime, 0, n));
                    if (n)
                        LOG("[Flush] stream %d consumed bytes %d", 0, n);
                }
            } while (beg != end);

            CHECK(nframes == props.video[0].max_frame_count);
        }

        OK(acquire_abort(runtime));

        CHECK(introspective_logger.different_heights());
        OK(acquire_shutdown(runtime));
        return 0;
    } catch (const std::runtime_error& e) {
        ERR("Runtime error: %s", e.what());

    } catch (...) {
        ERR("Uncaught exception");
    }
    acquire_shutdown(runtime);
    return 1;
}
