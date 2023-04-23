#include "platform.h"
#include "logger.h"

#include <EGrabber.h>
#include <iostream>

static void
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

int
info()
{
    Euresys::EGenTL gentl;
    Euresys::EGrabber<> grabber(gentl);

    try {
        const auto interface_id =
          grabber.getString<Euresys::InterfaceModule>("InterfaceID");
        const auto device_id =
          grabber.getString<Euresys::DeviceModule>("DeviceID");
        const auto w = grabber.getInteger<Euresys::RemoteModule>("Width");
        const auto h = grabber.getInteger<Euresys::RemoteModule>("Height");

        std::cout << "Interface id: " << interface_id << std::endl
                  << "Device id: " << device_id << std::endl
                  << "Width: " << w << std::endl
                  << "Height: " << h << std::endl;
        return 0;
    } catch (const std::exception& e) {
        std::cout << "Error: " << e.what() << std::endl;
    }
    return 1;
}

int
discover()
{
    try {
        Euresys::EGenTL gentl;
        Euresys::EGrabberDiscovery discovery(gentl);
        discovery.discover();

        for (int i = 0; i < discovery.cameraCount(); ++i) {
            Euresys::EGrabber<Euresys::CallbackOnDemand> grabber(
              discovery.cameras(i));

            const auto interface_id =
              grabber.getString<Euresys::InterfaceModule>("InterfaceID");
            const auto device_id =
              grabber.getString<Euresys::DeviceModule>("DeviceID");
            const auto w = grabber.getInteger<Euresys::RemoteModule>("Width");
            const auto h = grabber.getInteger<Euresys::RemoteModule>("Height");

            const auto device_name =
              grabber.getString<Euresys::RemoteModule>("DeviceModelName");
            const auto device_sn =
              grabber.getString<Euresys::RemoteModule>("DeviceSerialNumber");

            std::cout << "CAMERA INDEX: " << i << std::endl
                      << "Interface id: " << interface_id << std::endl
                      << "Device id: " << device_id << std::endl
                      << "Device name: " << device_name << std::endl
                      << "Device SN: " << device_sn << std::endl
                      << "Width: " << w << std::endl
                      << "Height: " << h << std::endl;
        }

        return 0;
    } catch (const std::exception& e) {
        std::cout << "Error: " << e.what() << std::endl;
    }
    return 1;
}

int
acquire()
{
    try {
        Euresys::EGenTL gentl;
        Euresys::EGrabberDiscovery discovery(gentl);
        discovery.discover();

        if (discovery.cameraCount() == 0) {
            std::cout << "No camera detected" << std::endl;
            return 0;
        }

        Euresys::EGrabber<> grabber = discovery.cameras(0);

        grabber.setString<Euresys::RemoteModule>("PixelFormat", "Mono12");
        grabber.setFloat<Euresys::RemoteModule>("ExposureTime", 10.0);
        grabber.setInteger<Euresys::RemoteModule>("BinningVertical", 1);

        grabber.setInteger<Euresys::StreamModule>("BufferPartCount", 1);
        uint64_t width = grabber.getInteger<Euresys::StreamModule>("Width");
        uint64_t height = grabber.getInteger<Euresys::StreamModule>("Height");

        auto bpp = grabber.getInteger<Euresys::StreamModule>("PixelSize");
        std::cout << grabber.getPixelFormat() << " bpp: " << bpp << " bits"
                  << std::endl;

        const int images_per_buffer = 1;
        const int buffer_count = 4;
        std::cout << "Allocate "
                  << images_per_buffer * buffer_count * width * height * 1e-6 *
                       (bpp / 8)
                  << " MB" << std::endl;
        grabber.setInteger<Euresys::StreamModule>("BufferPartCount",
                                                  images_per_buffer);

        const size_t payload_size = grabber.getPayloadSize();
        std::cout << "payload size: " << (payload_size * 1e-6) << " MB"
                  << std::endl;

        std::cout << "HERE " << __LINE__ << std::endl;
        grabber.reallocBuffers(buffer_count);
        std::cout << "HERE " << __LINE__ << std::endl;
        grabber.start();
        std::cout << "STARTED " << __LINE__ << std::endl;

        struct clock clock = { 0 };
        clock_init(&clock);
        clock_shift_ms(&clock, 10000);
        std::cout << "Clock start at: " << clock_toc_ms(&clock) << std::endl;
        while (clock_cmp_now(&clock) < 0) { // grab for 10 seconds
            Euresys::ScopedBuffer buffer(grabber);
            uint8_t* bufferPtr =
              buffer.getInfo<uint8_t*>(Euresys::gc::BUFFER_INFO_BASE);
            size_t imageSize =
              buffer.getInfo<size_t>(Euresys::ge::BUFFER_INFO_CUSTOM_PART_SIZE);
            auto info = buffer.getInfo();

            // process available images
            size_t delivered = buffer.getInfo<size_t>(
              Euresys::ge::BUFFER_INFO_CUSTOM_NUM_DELIVERED_PARTS);
            size_t processed = 0;
            while (processed < delivered) {
                uint8_t* imagePtr = bufferPtr + processed * imageSize;
                //                processImage(imagePtr, imageSize, width,
                //                height);
                ++processed;
            }
            if (true) {
                uint64_t fr = grabber.getInteger<Euresys::StreamModule>(
                  "StatisticsFrameRate");
                uint64_t dr = grabber.getInteger<Euresys::StreamModule>(
                  "StatisticsDataRate");
                std::cout << dr << " MB/s, " << fr << " fps (" << delivered
                          << " delivered)" << std::endl;
            }
        }

        return 0;
    } catch (const std::exception& e) {
        std::cout << "Error: " << e.what() << std::endl;
    }
    return 1;
}

static void
showElements(const std::string& moduleName,
             const std::string& attr,
             const std::vector<std::string>& vect)
{
    std::cout << moduleName << " " << attr << ": " << std::endl;
    typedef std::vector<std::string>::const_iterator string_iterator;
    for (string_iterator it = vect.begin(); it != vect.end(); it++) {
        std::cout << "  " << *it << std::endl;
    }
}

int
query_features()
{
    Euresys::EGenTL genTL;
    Euresys::EGrabber<> grabber(genTL);
    showElements(
      "RemoteModule",
      "features",
      grabber.getStringList<Euresys::RemoteModule>(Euresys::query::features()));
    std::cout << "ExposureTimeMaxReg: "
              << grabber.getFloat<Euresys::RemoteModule>("ExposureTimeMaxReg")
              << std::endl;
    std::cout << "ExposureTimeMinReg: "
              << grabber.getFloat<Euresys::RemoteModule>("ExposureTimeMinReg")
              << std::endl;
    std::cout << "Exposure/Unit: "
              << grabber.getString<Euresys::RemoteModule>(
                   Euresys::query::info("ExposureTime", "Unit"))
              << std::endl;
    std::cout << "Exposure/pMin: "
              << grabber.getString<Euresys::RemoteModule>(
                   Euresys::query::info("ExposureTime", "pMin"))
              << std::endl;
    std::cout << "Exposure ?Writable: "
              << grabber.getInteger<Euresys::RemoteModule>(
                   Euresys::query::writeable("ExposureTime"))
              << std::endl;
    std::cout << "LineSelectorListReg ?Writable: "
              << grabber.getInteger<Euresys::RemoteModule>(
                   Euresys::query::writeable("LineSelectorListReg"))
              << std::endl;
    std::cout << "writable query (LineSelectorListReg): "
              << std::string(Euresys::query::writeable("LineSelectorListReg"))
              << std::endl;
    std::cout << "BinningHorizontal: "
              << grabber.getString<Euresys::RemoteModule>("BinningHorizontal")
              << "\t"
              << grabber.getInteger<Euresys::RemoteModule>("BinningHorizontal")
              << std::endl;
    std::cout << "enum entries query: "
              << std::string(Euresys::query::enumEntries("BinningHorizontal"))
              << std::endl;

    showElements("RemoteModule",
                 "BinningHorizontal",
                 grabber.getStringList<Euresys::RemoteModule>(
                   Euresys::query::enumEntries("BinningHorizontal")));

    showElements("RemoteModule",
                 "PixelFormat",
                 grabber.getStringList<Euresys::RemoteModule>(
                   Euresys::query::enumEntries("PixelFormat")));
    return 1;
}

int
set_triggers(Euresys::EGrabber<>& grabber,
             uint8_t last[2],
             uint8_t target[2],
             uint8_t is_edge_falling)
{
    uint8_t old_state = 0, new_state = 0;
    for (uint8_t i = 0; i < 2; ++i) {
        old_state |= (last[i] << i);
        new_state |= (target[i] << i);
    }
    if (old_state == 3) {
        std::cout << "Expected at most one enabled line in previous state."
                  << std::endl;
        return 0;
    }

    if (new_state == 3) {
        // Both lines look enabled, but we just switched one of them on.
        // Switch the other one off.
        new_state = old_state ^ 3;
    }

    if (new_state == 3) {
        std::cout << "Expected at most one enabled line in new state."
                  << std::endl;
        return 0;
    }

    // Compute the selected line.
    // The selected line is the enabled line, or if no lines are
    // enabled, The line that changed.
    uint8_t selected_line = 0;
    if (new_state) {
        selected_line = new_state - 1; // new_state is either 1 or 2
    } else {
        uint8_t changed = new_state ^ old_state;
        if (changed > 1)
            selected_line = 1;
        // otherwise selected_line remains 0
    }

    const char* sources[] = { "Line0", "Software" };
    const char* modes[] = { "Off", "On" };
    const char* activations[] = { "RisingEdge", "FallingEdge" };
    std::cout << "selected line: " << sources[selected_line] << std::endl;
    grabber.setString<Euresys::RemoteModule>("TriggerSource",
                                             sources[selected_line]);
    grabber.setString<Euresys::RemoteModule>(
      "TriggerMode", modes[(new_state >> selected_line) & 1]);
    grabber.setString<Euresys::RemoteModule>("TriggerActivation",
                                             activations[is_edge_falling]);
    std::cout << "Triggers have been set" << std::endl;
    return 1;
}

int
fiddle_with_triggers()
{
    Euresys::EGenTL genTL;
    Euresys::EGrabber<> grabber(genTL);

    uint8_t last[2] = { 1, 0 };
    uint8_t target[2] = { 0, 0 };
    set_triggers(grabber, last, target, 0);

    return 1;
}

int
main()
{
    logger_set_reporter(reporter);
    // info();
    discover();
    // acquire();
    //    query_features();
    // fiddle_with_triggers();
    return 0;
}
