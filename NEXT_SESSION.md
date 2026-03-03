# Next Session — FPS Investigation

## Current Status

| Thing | Status |
|-------|--------|
| WebRTC video (all 4 agents) | ✅ Working |
| GPU rendering (NVIDIA EGL / T4) | ✅ Confirmed — gz sim using 35% GPU, 208 MiB VRAM |
| Dashboard (overview, agent pages) | ✅ Working |
| aiortc track-reuse bug | ✅ Fixed |
| Overview canvas height | ✅ Fixed |

## The Problem

**Goal: 30+ fps in the browser. Currently getting ~17fps.**

### What we tried and measured

| Config | Achieved fps | Notes |
|--------|-------------|-------|
| 480×360 @ 5fps (t3.medium, software) | 5 fps | Old baseline |
| 480×360 @ 30fps (g4dn, software Mesa) | ~27 fps | Just changing update_rate |
| 480×360 @ 60fps (g4dn, NVIDIA EGL GPU) | ~27 fps | GPU confirmed, same ceiling |
| 640×360 @ 45fps (g4dn, NVIDIA EGL GPU) | ~17 fps | **REGRESSION** — unexpected |
| 640×480 @ 60fps (g4dn, NVIDIA EGL GPU) | ~13 fps | Drones only |

### Key observation

Going 480×360 → 640×360 (78% more pixels) dropped fps from 27 to 17.
GPU utilization didn't change (still ~35%). This means the bottleneck is NOT the GPU.

The spikes (std dev 50-67ms, max 0.5-0.9s) on 640×360 but clean output on 480×360
strongly suggest the bottleneck is in the **ROS pipeline** — specifically
`image_transport republish` (raw → compressed JPEG) or the `ros_gz_bridge`.

## What to Investigate Next

### Step 1 — Isolate where the bottleneck is

Run these in parallel (separate SSH sessions or screen):

```bash
# Terminal 1 — measure raw camera from Gazebo
source /opt/ros/jazzy/setup.bash
ros2 topic hz /robot1/camera/image_raw

# Terminal 2 — measure after image_transport (compressed)
source /opt/ros/jazzy/setup.bash
ros2 topic hz /robot1/camera/compressed

# Terminal 3 — CPU usage breakdown
top -b -n 5 | grep -E 'image_transport|parameter_bridge|gz'
```

If `image_raw` is fast (close to 45fps) but `compressed` is slow (~17fps),
the bottleneck is `image_transport republish` JPEG encoding.

If both are slow (~17fps), the bottleneck is `gz_bridge` or Gazebo publish rate.

### Step 2 — If image_transport is the bottleneck

Options (in order of impact):
1. Lower JPEG quality: `-p jpeg_quality:=75` (currently 95) — reduces encode time
2. Use `theora` or `ffmpeg` transport instead of `compressed`
3. Skip image_transport entirely — aiortc reads `/image_raw` directly (it already does)

**Note**: The aiortc agent subscribes to `/robot1/camera/image_raw` directly —
it does NOT use the compressed topic. The compressed topic is only used by
the rosbridge → VideoFeed.jsx path which is now UNUSED (VideoFeed uses WebRTC).
So image_transport may be irrelevant to the actual WebRTC fps!

### Step 3 — Measure what the browser actually sees

Check the fps HUD in the browser (bottom-left of video panel).
The `fps` counter in VideoFeed uses `requestVideoFrameCallback` on the WebRTC stream.
This is the ground truth — not `ros2 topic hz`.

If browser shows 27fps while `ros2 topic hz /robot1/camera/image_raw` shows 17fps,
it means the aiortc queue is buffering and smoothing — the actual bottleneck
may be in the ROS→aiortc push_frame path, not in WebRTC encoding.

## Current EC2 Config

### SDF (/opt/gazebo/c2_arena.sdf)
- All 4 cameras: **640×360, update_rate=45, R8G8B8**
- robot1/robot2 FOV: 1.047 rad (60°), near=0.05, far=50
- drone1/drone2 FOV: 0.9 rad (51°), near=0.1, far=80

### start_sim.sh (/opt/gazebo/start_sim.sh)
- NVIDIA EGL: `__EGL_VENDOR_LIBRARY_FILENAMES=/usr/share/glvnd/egl_vendor.d/10_nvidia.json`
- Mesa vars REMOVED
- image_transport: jpeg_quality=95 (potentially reduce)

### aiortc (/opt/webrtc/aiortc_agent.py)
- Fresh RosVideoTrack per PC (track-reuse bug fixed)
- Subscribes to `/robot1/camera/image_raw` (raw, not compressed)
- Software encoding via Python `av` library (libvpx/libx264)
- nvh264enc NOT installed — GPU encoding not yet set up

## Start Everything

```bash
ssh -i ViDeG.pem ubuntu@18.236.30.71 "/opt/gazebo/start_sim.sh"
# Wait ~40s then:
ssh -i ViDeG.pem ubuntu@18.236.30.71 "/opt/webrtc/start_webrtc.sh"
```

## The Remaining Goal

Target: **30fps in the browser WebRTC feed**.

Most likely path to get there:
1. Check if aiortc is actually getting 27fps (raw topic) vs 17fps — this matters
2. If aiortc is fine, the browser fps should already be ~27fps (check browser HUD)
3. If truly bottlenecked, investigate gz_bridge publish rate vs Gazebo render rate
4. Optionally: install gstreamer-plugins-nvenc for GPU H264 encoding (reduce CPU encode load)
