# MovieCamera

A cinematic camera plugin for X-Plane 12 that provides automatic smooth camera movements with transitions between cockpit and external views.

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

### Cinematic Camera System

The plugin provides smooth transitions between various camera angles:

**Cockpit Views (9 angles):**
- Center Panel
- Left Panel (throttle/navigation)
- Right Panel (radios/autopilot)
- Overhead Panel
- PFD View (closeup)
- ND/MFD View (closeup)
- Front Window
- Left Window
- Right Window

**External Views (12 angles):**
- Front View
- Rear View
- Left/Right Side Views
- Top View
- Bottom Front View
- Quarter views (front/rear, left/right)
- Wing Views (left/right)

### Intelligent Shot Switching
- Each shot lasts 3-5 seconds (configurable)
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
- **Shot Duration Min/Max (s)**: Range for random shot duration (default: 3-5 seconds)

## Building

### Prerequisites
- CMake 3.16 or later
- C++17 compatible compiler
- X-Plane SDK 4.2.0 (included in XPSDK420.zip)

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