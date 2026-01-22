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
- **Instant View Switching**: Camera switches between viewpoints instantly for a snappy, responsive feel

### Intelligent Shot Switching
- Each shot lasts 6-15 seconds (configurable)
- Smooth ease-in-out transitions between shots
- At least 3 consecutive shots of the same type (cockpit or external) before switching to the other type

### Mouse Pause Feature
When mouse movement is detected:
- Camera control is paused
- View returns to default
- After the configured delay without mouse movement, camera control resumes

### Aircraft Attitude Compensation
External camera shots now account for the aircraft's full attitude (heading, pitch, and roll), keeping camera positions relative to the aircraft's orientation during climbs, dives, and turns.

### Cinematic Effects (X-Plane 12+)

The plugin now supports advanced cinematic camera effects using X-Plane's native datarefs:

**FOV/Focal Length Control:**
- Simulate different camera lenses by controlling field of view
- Preset buttons for common focal lengths: 24mm, 35mm, 50mm, 85mm, 135mm
- Smooth FOV transitions between shots
- Configurable transition speed

**Handheld Camera Effect:**
- Adds realistic camera shake for external views
- Adjustable intensity from subtle to dramatic
- Perfect for chase cam and action shots

**G-Force Camera Effect:**
- Internal view camera responds to aircraft G-forces
- Creates immersive cockpit experience during maneuvers

## Settings

Configure via `Plugins > MovieCamera > Settings`:
- **Delay (seconds)**: Time to wait after mouse stops moving before activating/resuming camera (default: 60)
- **Auto Alt (ft)**: Altitude threshold above which Auto mode can activate (default: 18000)
- **Shot Duration Min/Max (s)**: Range for random shot duration (default: 6-15 seconds)
- **Cinematic Effects**:
  - **Enable FOV Effect**: Toggle focal length simulation
  - **Base FOV**: Default field of view angle
  - **Transition Speed**: How fast FOV changes between shots
  - **Enable Handheld Effect**: Toggle camera shake
  - **Shake Intensity**: Amount of camera shake
  - **Enable G-Force Effect**: Toggle G-force camera movement

Settings are automatically saved to `settings.cfg` when the plugin is disabled and loaded when enabled.

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