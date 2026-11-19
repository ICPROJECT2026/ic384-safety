"""Mac depth-driver diagnostic. Does not publish robot motion permission."""
import argparse
from array import array
from dataclasses import dataclass
import json
import math
from pathlib import Path
import queue
import struct
import subprocess
import sys
import threading
import time

HERE = Path(__file__).resolve().parent
SYSTEM = HERE.parent
DEFAULT_KIT = SYSTEM.parent / 'drivers' / 'AstraPro_Mac_Depth_Kit_v1.0'
HEADER = struct.Struct('<4sIIIQQ')


@dataclass
class DepthFrame:
    width: int
    height: int
    index: int
    sensor_us: int
    host_ns: int
    millimetres: array


def read_exact(stream, count):
    data = bytearray()
    while len(data) < count:
        block = stream.read(count-len(data))
        if not block:
            raise EOFError('depth stream closed or frame truncated')
        data.extend(block)
    return bytes(data)


def read_frame(stream):
    magic, width, height, index, sensor_us, host_ns = HEADER.unpack(read_exact(stream, HEADER.size))
    if magic != b'ADP1' or not 0 < width <= 4096 or not 0 < height <= 4096 or width*height > 4096*2160:
        raise ValueError('invalid depth frame header')
    values = array('H')
    values.frombytes(read_exact(stream, width*height*2))
    if sys.byteorder != 'little':
        values.byteswap()
    return DepthFrame(width, height, index, sensor_us, host_ns, values)


def summarize(frame, now_ns=None, timeout=.3):
    now_ns = time.monotonic_ns() if now_ns is None else now_ns
    age = (now_ns-frame.host_ns)/1e9
    valid = [z for z in frame.millimetres if 600 <= z <= 8000]
    healthy = 0 <= age <= timeout and len(valid) >= max(9, len(frame.millimetres)*.5)
    nearest = min(valid)/1000 if valid else None
    if not healthy:
        zone = 'UNKNOWN'
    elif nearest <= 1.0:
        zone = 'DEPTH_WITHIN_STOP_DISTANCE'
    elif nearest >= 1.5:
        zone = 'DEPTH_BEYOND_RELEASE_DISTANCE'
    else:
        zone = 'DEPTH_BETWEEN_THRESHOLDS'
    return {'diagnostic_only': True, 'motion_allowed': False,
            'frame': frame.index, 'width': frame.width, 'height': frame.height,
            'age_s': round(age, 4), 'depth_valid': healthy,
            'valid_fraction': round(len(valid)/len(frame.millimetres), 4),
            'nearest_depth_m': nearest, 'depth_zone': zone,
            'person_distance_verified': False}


def build_reader(kit):
    if sys.platform != 'darwin':
        raise RuntimeError('this reader requires macOS; use the ROS2 version on Jetson')
    driver = kit/'Driver'
    required = [kit/'Source/Include/OniCAPI.h', driver/'libOpenNI2.dylib',
                driver/'OpenNI2/Drivers/liborbbec.dylib',
                driver/'OpenNI2/Drivers/libusb-1.0.0.dylib']
    for path in required:
        if not path.is_file():
            raise FileNotFoundError('missing camera kit file: '+str(path))
    binary = HERE/'bin/astra_depth_reader'
    binary.parent.mkdir(exist_ok=True)
    subprocess.run(['xcrun', 'clang++', '-arch', 'x86_64', '-mmacosx-version-min=15.0',
                    '-std=c++17', str(HERE/'depth_reader.cpp'),
                    '-I'+str(kit/'Source/Include'), '-L'+str(driver), '-lOpenNI2',
                    '-Wl,-rpath,'+str(driver), '-o', str(binary)], check=True)
    return binary


def main():
    parser = argparse.ArgumentParser(description='Astra Pro Mac depth check; no ROS2/YOLO needed')
    parser.add_argument('--kit', type=Path, default=DEFAULT_KIT)
    parser.add_argument('--build-only', action='store_true')
    parser.add_argument('--frames', type=int, default=10)
    parser.add_argument('--timeout', type=float, default=10.)
    args = parser.parse_args()
    if args.frames <= 0 or not math.isfinite(args.timeout) or args.timeout <= 0:
        parser.error('frames and timeout must be positive')
    child = None
    try:
        kit = args.kit.expanduser().resolve()
        binary = build_reader(kit)
        if args.build_only:
            subprocess.run([str(binary), '--help'], check=True)
            print('Built and launched Intel bridge; no camera was opened.')
            return 0
        child = subprocess.Popen([str(binary)], stdout=subprocess.PIPE, cwd=str(kit/'Driver'),
                                 stdin=subprocess.DEVNULL)
        mailbox = queue.Queue(maxsize=1)

        def receive():
            try:
                while True:
                    value = read_frame(child.stdout)
                    if mailbox.full():
                        try:
                            mailbox.get_nowait()
                        except queue.Empty:
                            pass
                    mailbox.put_nowait(value)
            except Exception as exc:
                # Preserve an explicit failure after the last frame.
                if mailbox.full():
                    try:
                        mailbox.get_nowait()
                    except queue.Empty:
                        pass
                mailbox.put_nowait(exc)

        threading.Thread(target=receive, daemon=True).start()
        deadline = time.monotonic()+args.timeout
        previous = None
        for _ in range(args.frames):
            remaining = deadline-time.monotonic()
            if remaining <= 0:
                raise TimeoutError('no fresh depth frames within diagnostic deadline')
            item = mailbox.get(timeout=remaining)
            if isinstance(item, Exception):
                raise item
            if previous is not None and (item.index <= previous.index or item.sensor_us <= previous.sensor_us):
                raise ValueError('depth frame index or source timestamp did not advance')
            report = summarize(item)
            print(json.dumps(report, allow_nan=False), flush=True)
            if not report['depth_valid']:
                raise ValueError('stale frame or insufficient valid depth; see diagnostic output')
            previous = item
        return 0
    except (Exception, KeyboardInterrupt) as exc:
        reason = str(exc) or type(exc).__name__
        print(json.dumps({'diagnostic_only': True, 'motion_allowed': False, 'error': reason}), file=sys.stderr)
        return 1
    finally:
        if child is not None:
            if child.poll() is None:
                child.terminate()
                try:
                    child.wait(timeout=2)
                except subprocess.TimeoutExpired:
                    child.kill()
                    child.wait()
            child.stdout.close()


if __name__ == '__main__':
    raise SystemExit(main())
