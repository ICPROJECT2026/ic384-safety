// Intel OpenNI bridge. Keep the legacy driver in this process, separate from
// native Apple Silicon Python. Stdout is a framed binary stream, not text.
#include <OniCAPI.h>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <csignal>

static volatile std::sig_atomic_t running = 1;
static void stop(int) { running = 0; }
static int error(const char* stage) {
    std::fprintf(stderr, "%s: %s\n", stage, oniGetExtendedError());
    return 1;
}

struct Header {
    char magic[4];
    uint32_t width, height, index;
    uint64_t sensor_us, host_ns;
};
static_assert(sizeof(Header) == 32, "unexpected protocol layout");

int main(int argc, char** argv) {
    if (argc == 2 && std::strcmp(argv[1], "--help") == 0) {
        std::puts("Astra Pro x86_64 depth bridge; stdout: ADP1 + header + uint16 millimetres");
        return 0;
    }
    std::signal(SIGTERM, stop);
    std::signal(SIGINT, stop);
    auto version = oniGetVersion();
    if (oniInitialize(version.major*1000 + version.minor)) return error("initialize");
    OniDeviceHandle device = nullptr;
    OniStreamHandle stream = nullptr;
    if (oniDeviceOpen(nullptr, &device)) {
        int code = error("open camera (check USB connection and close other depth viewers)");
        oniShutdown();
        return code;
    }
    int result = 1;
    do {
        if (oniDeviceCreateStream(device, ONI_SENSOR_DEPTH, &stream)) { error("create stream"); break; }
        OniVideoMode mode = {ONI_PIXEL_FORMAT_DEPTH_1_MM, 640, 480, 30};
        if (oniStreamSetProperty(stream, ONI_STREAM_PROPERTY_VIDEO_MODE, &mode, sizeof(mode))) {
            error("set 640x480 millimetre mode"); break;
        }
        OniBool mirrored = FALSE;
        int size = sizeof(mirrored);
        if (oniStreamSetProperty(stream, ONI_STREAM_PROPERTY_MIRRORING, &mirrored, size)
                || oniStreamGetProperty(stream, ONI_STREAM_PROPERTY_MIRRORING, &mirrored, &size)
                || mirrored) {
            error("disable depth mirroring"); break;
        }
        if (oniStreamStart(stream)) { error("start stream"); break; }
        result = 0;
        while (running) {
            int index = 0;
            int status = oniWaitForAnyStream(&stream, 1, &index, 500);
            if (status == ONI_STATUS_TIME_OUT) continue;
            if (status) { result = error("wait frame"); break; }
            OniFrame* frame = nullptr;
            if (oniStreamReadFrame(stream, &frame)) { result = error("read frame"); break; }
            if (frame->videoMode.pixelFormat != ONI_PIXEL_FORMAT_DEPTH_1_MM
                    || frame->width <= 0 || frame->height <= 0
                    || frame->width > 4096 || frame->height > 4096
                    || frame->stride < frame->width*2 || !frame->data
                    || frame->dataSize < (frame->height-1)*frame->stride + frame->width*2) {
                std::fprintf(stderr, "Invalid depth frame; refusing output\n");
                oniFrameRelease(frame); result = 1; break;
            }
            timespec now{};
            clock_gettime(CLOCK_MONOTONIC, &now);
            Header header = {{'A','D','P','1'}, uint32_t(frame->width), uint32_t(frame->height),
                             uint32_t(frame->frameIndex), frame->timestamp,
                             uint64_t(now.tv_sec)*1000000000ULL + uint64_t(now.tv_nsec)};
            bool written = std::fwrite(&header, sizeof(header), 1, stdout) == 1;
            for (int y = 0; written && y < frame->height; ++y) {
                const auto* row = static_cast<const char*>(frame->data) + y*frame->stride;
                written = std::fwrite(row, 2, frame->width, stdout) == size_t(frame->width);
            }
            oniFrameRelease(frame);
            if (!written || std::fflush(stdout)) { result = 1; break; }
        }
    } while (false);
    if (stream) { oniStreamStop(stream); oniStreamDestroy(stream); }
    oniDeviceClose(device);
    oniShutdown();
    return result;
}
