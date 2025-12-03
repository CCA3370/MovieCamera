# MovieCamera

A cinematic camera plugin for X-Plane 12 that provides automatic smooth camera movements with transitions between cockpit and external views. Features continuous, slow camera drift for a professional film-like experience.

## Features

### Menu Controls
Access the plugin from `Plugins > MovieCamera`:
- **Auto**: Toggle automatic mode (activates based on flight conditions)
- **Start**: Manually start camera control
- **Stop**: Stop camera control
- **Settings**: Open the settings window (ImGui-based UI)

### Camera Modes

1. **Off Mode**: Plugin is inactive, normal X-Plane camera control
2. **Manual Mode**: Camera control started manually via the Start button
3. **Auto Mode**: Automatically activates when:
   - Aircraft is on ground and stationary, OR
   - Aircraft is above the configured altitude AND mouse has been idle for the configured delay

### Intelligent Dynamic Camera System

The plugin **automatically reads aircraft dimensions** from X-Plane's datarefs (which are loaded from the .acf file) and **intelligently calculates camera positions** based on the aircraft's actual size:

- **Wingspan-based scaling**: External camera distances are calculated relative to the aircraft's wingspan
- **Fuselage length awareness**: Front/rear camera shots are positioned based on actual fuselage length
- **Height-adaptive positions**: Vertical camera placements adjust to aircraft height
- **Pilot eye position**: Cockpit views are scaled based on the actual cockpit size

This means camera shots work correctly for any aircraft - from small GA planes to large airliners - without manual configuration.

### Cinematic Camera System

The plugin provides smooth transitions between various camera angles with **continuous multi-axis drift motion** using sinusoidal easing for a cinematic feel:

**Cockpit Views (11 angles):**
- Center Panel (slow zoom with subtle drift)
- Left/Right Panel (gentle pan with zoom)
- Overhead Panel (slow tilt)
- PFD/ND View (close-up with zoom breathing)
- Pilot/Copilot View (subtle look-around)
- Left/Right Window (slow pan)
- Pedestal View

**External Views (16 angles):**
- Front Hero Shot (positioned offset to avoid clipping)
- Rear Chase (elevated and offset for better framing)
- Left/Right Flyby (dramatic side sweep)
- High Orbit (circling view from above)
- Low Angle Front (dramatic upward shot)
- Quarter views (front/rear, left/right - all offset from center)
- Wing/Engine close-ups (positioned outside aircraft)
- Tail View
- Belly View (looking up from below)

### Smooth Camera Movement
- **Sinusoidal Drift Motion**: Camera drift uses smooth sine-wave based easing instead of linear motion, creating a floating, organic feel
- **Multi-frequency oscillation**: Different camera axes use slightly different frequencies for natural-looking movement
- **Cockpit Zoom Breathing**: In cockpit views, the focal length slowly changes to simulate aperture/depth-of-field effects
- **Smooth Transitions**: Ease-in-out cubic transitions between shots

### Custom Camera Paths

Create your own camera movement paths with keyframe-based animation:

**Features:**
- Define multiple keyframes with position, rotation, zoom, focal length, and aperture
- Smooth interpolation between keyframes using ease-in-out curves
- Support for looping paths
- Save and load custom paths (stored in `camera_paths.cfg`)
- Preview paths in real-time while editing
- **3D Trajectory Visualization** - See your camera path drawn in 3D space

**Path Editor:**
1. Open Settings window from `Plugins > MovieCamera > Settings`
2. Click "New Path" to create a new custom path
3. **Capture Camera Positions**:
   - Position X-Plane's camera where you want a keyframe
   - Click "Capture Position" to add the current camera location as a keyframe
   - Repeat to build your complete camera path
   - Use "Update Selected" to modify an existing keyframe with the current camera position
4. Enable "Show 3D Trajectory" to visualize:
   - **Green line**: The camera path trajectory
   - **Yellow dots**: Keyframe positions
   - **Red dot**: Currently selected keyframe
5. Fine-tune keyframe parameters:
   - **Time**: When this keyframe occurs (seconds)
   - **Position (X, Y, Z)**: Camera offset from aircraft
   - **Rotation (Pitch, Heading, Roll)**: Camera orientation
   - **Zoom**: Camera zoom level
   - **Focal Length**: Lens focal length (mm)
   - **Aperture**: f-stop value for DOF simulation
6. Click "Preview" to see the path in action
7. Click "Save Path" to save for future use

### Intelligent Shot Switching
- Each shot lasts 6-15 seconds (configurable)
- Smooth ease-in-out transitions between shots
- At least 3 consecutive shots of the same type (cockpit or external) before switching to the other type

### Mouse Pause Feature
When mouse movement is detected:
- Camera control is paused
- View returns to default
- After the configured delay without mouse movement, camera control resumes

## Settings

Configure via `Plugins > MovieCamera > Settings`:
- **Delay (seconds)**: Time to wait after mouse stops moving before activating/resuming camera (default: 60)
- **Auto Alt (ft)**: Altitude threshold above which Auto mode can activate (default: 18000)
- **Shot Duration Min/Max (s)**: Range for random shot duration (default: 6-15 seconds)
- **Custom Camera Paths**: Create, edit, and manage custom camera movement paths

## Building

### Prerequisites
- CMake 3.16 or later
- C++17 compatible compiler
- X-Plane SDK 4.2.0 (included in XPSDK420.zip)
- OpenGL development headers (libgl1-mesa-dev on Linux)

### Linux
```bash
# Extract SDK
unzip XPSDK420.zip

# Build
mkdir build && cd build
cmake ..
make
```

### Windows (MSVC)
```bash
# Extract SDK
# Build using Visual Studio or CMake

mkdir build && cd build
cmake ..
cmake --build . --config Release
```

### macOS
```bash
# Extract SDK
unzip XPSDK420.zip

# Build
mkdir build && cd build
cmake ..
make
```

## Dependencies

- X-Plane SDK 4.2.0 (XPSDK420.zip in the repository)
- Dear ImGui (included in `imgui/` directory, based on ImgWindow integration from BetterPushbackMod)

## Installation

1. Build the plugin to get `MovieCamera.xpl`
2. Copy `MovieCamera.xpl` to `X-Plane 12/Resources/plugins/MovieCamera/64/`
3. Restart X-Plane

## Credits

- ImgWindow integration code based on [BetterPushbackMod](https://github.com/olivierbutler/BetterPusbackMod) by Christopher Collins

## License

See LICENSE file for details.