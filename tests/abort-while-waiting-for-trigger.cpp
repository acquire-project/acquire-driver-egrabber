#include "acquire.h"
#include "device/hal/device.manager.h"
#include "device/props/components.h"
#include "logger.h"
#include "platform.h"
#include <stdexcept>

void
reporter(int is_error,
         const char* file,
         int line,
         const char* function,
         const char* msg)
{
    fprintf(is_error ? stderr : stdout,
            "%s%s(%d) - %s: %s\n",
            is_error ? "ERROR " : "",
            file,
            line,
            function,
            msg);
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
        AcquireProperties props = {};
        DEVOK(device_manager_select(dm,
                                    DeviceKind_Camera,
                                    SIZED("vieworks.*") - 1,
                                    &props.video[0].camera.identifier));
        DEVOK(device_manager_select(dm,
                                    DeviceKind_Storage,
                                    SIZED("trash") - 1,
                                    &props.video[0].storage.identifier));
        props.video[0].camera.settings.input_triggers.frame_start.enable = 1;
        props.video[0].camera.settings.input_triggers.frame_start.edge =
          TriggerEdge_Rising;
        props.video[0].max_frame_count = 10;
        OK(acquire_configure(runtime, &props));
        OK(acquire_start(runtime));
        clock_sleep_ms(0, 500);
        OK(acquire_abort(runtime));
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
