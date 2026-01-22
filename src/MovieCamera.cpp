/**
 * MovieCamera Plugin for X-Plane 12
 * 
 * A cinematic camera plugin that provides automatic camera movements
 * with smooth transitions between cockpit and external views.
 * 
 * Uses ImGui for the settings window (based on ImgWindow integration).
 */

// Must be defined before any includes to prevent Windows min/max macros
// from conflicting with std::min/std::max
#if defined(_WIN32) || defined(WIN32) || defined(IBM)
#define NOMINMAX
#endif

#include "XPLMDefs.h"
#include "XPLMPlugin.h"
#include "XPLMProcessing.h"
#include "XPLMDataAccess.h"
#include "XPLMMenus.h"
#include "XPLMDisplay.h"
#include "XPLMCamera.h"
#include "XPLMGraphics.h"
#include "XPLMUtilities.h"

#include "ImgWindow.h"
#include "imgui.h"

// OpenGL for trajectory drawing
#if IBM
#include <windows.h>
#include <GL/gl.h>
#elif APL
#include <OpenGL/gl.h>
#else
#include <GL/gl.h>
#endif

#include <cstdlib>
#include <cstring>
#include <cmath>
#include <ctime>
#include <cstdio>
#include <string>
#include <vector>
#include <algorithm>
#include <memory>

// Plugin Info
#define PLUGIN_NAME        "MovieCamera"
#define PLUGIN_SIG         "com.moviecamera.xplane"
#define PLUGIN_DESCRIPTION "Cinematic camera plugin with automatic smooth camera movements"

// Mathematical constants
constexpr float PI = 3.14159265359f;
constexpr float TWO_PI = 2.0f * PI;

// FOV and focal length constants
// Based on 35mm full-frame equivalent (36mm sensor width)
constexpr float SENSOR_WIDTH_MM = 36.0f;           // 35mm full-frame sensor width
constexpr float DEFAULT_FOV_DEG = 60.0f;           // Default X-Plane horizontal FOV
constexpr float MIN_FOV_DEG = 20.0f;               // Minimum FOV (telephoto, ~90mm equivalent)
constexpr float MAX_FOV_DEG = 120.0f;              // Maximum FOV (wide angle, ~15mm equivalent)
constexpr float FOV_TRANSITION_SPEED = 15.0f;      // FOV change speed (degrees per second)

// Plugin State
enum class PluginMode {
    Off,      // Plugin functionality is off
    Manual,   // Manual control (Start pressed)
    Auto      // Automatic mode based on conditions
};

enum class CameraType {
    Cockpit,
    External
};

struct CameraShot {
    CameraType type;
    float x, y, z;           // Position offset from aircraft
    float pitch, heading, roll;
    float zoom;
    float duration;          // How long this shot lasts (3-5 seconds)
    std::string name;
    
    // Camera drift parameters (how the camera moves during the shot)
    float driftX, driftY, driftZ;          // Position drift per second
    float driftPitch, driftHeading, driftRoll;  // Rotation drift per second
    float driftZoom;                        // Zoom drift per second (for cockpit)
};

// Aircraft dimension constants
constexpr float STANDARD_WINGSPAN = 35.0f;         // Standard wingspan for scaling (meters, similar to B737/A320)
constexpr float STANDARD_FUSELAGE_LENGTH = 40.0f;  // Standard fuselage length (meters)
constexpr float STANDARD_HEIGHT = 12.0f;           // Standard aircraft height (meters)
constexpr float MIN_WINGSPAN = 5.0f;               // Minimum valid wingspan (meters)
constexpr float MAX_WINGSPAN = 100.0f;             // Maximum valid wingspan (meters)
constexpr float MIN_FUSELAGE_LENGTH = 5.0f;        // Minimum valid fuselage length (meters)
constexpr float MAX_FUSELAGE_LENGTH = 100.0f;      // Maximum valid fuselage length (meters)
constexpr float MIN_HEIGHT = 2.0f;                 // Minimum valid aircraft height (meters)
constexpr float MAX_HEIGHT = 30.0f;                // Maximum valid aircraft height (meters)

// Estimation multipliers for deriving aircraft dimensions from limited data
// CG range is typically 20-30% of total fuselage length
constexpr float CG_TO_FUSELAGE_MULTIPLIER = 4.0f;
// Pilot is typically located 30-40% from nose, so multiply Z position to estimate length
constexpr float PILOT_Z_TO_FUSELAGE_MULTIPLIER = 2.5f;
// Pilot eye is typically at 60-80% of aircraft height
constexpr float PILOT_Y_TO_HEIGHT_MULTIPLIER = 1.5f;
// Ground clearance estimation (meters)
constexpr float ESTIMATED_GROUND_CLEARANCE = 2.0f;
// Minimum valid CG range to use for estimation (meters)
constexpr float MIN_VALID_CG_RANGE = 0.5f;

// Camera safety constants
constexpr float MIN_CAMERA_HEIGHT_ABOVE_GROUND = 2.0f;   // Minimum camera height above ground (meters)
constexpr float MIN_CAMERA_DISTANCE_FROM_AIRCRAFT = 5.0f; // Minimum distance to ensure aircraft is visible (meters)
constexpr float ZOOM_SCALE_FACTOR = 0.7f;                 // Base zoom factor scaled by aircraft size
constexpr float EASE_IN_DURATION_RATIO = 0.3f;            // Ratio of shot duration for ease-in phase (30%)
constexpr float CLOSE_DISTANCE_SCALE = 0.8f;              // Scale factor for close-up camera distances

/**
 * Aircraft dimensions structure
 * Stores dimensions read from X-Plane datarefs to calculate dynamic camera positions
 */
struct AircraftDimensions {
    float wingspan;          // Wing span in meters
    float fuselageLength;    // Approximate fuselage length in meters
    float height;            // Aircraft height in meters (e.g., from ground to top of tail)
    float pilotEyeX;         // Pilot eye X position (lateral offset from centerline)
    float pilotEyeY;         // Pilot eye Y position (vertical offset from CG)
    float pilotEyeZ;         // Pilot eye Z position (longitudinal offset from CG)
    
    // Default values for a medium-sized aircraft (similar to B737/A320)
    void setDefaults() {
        wingspan = STANDARD_WINGSPAN;
        fuselageLength = STANDARD_FUSELAGE_LENGTH;
        height = STANDARD_HEIGHT;
        pilotEyeX = -0.5f;
        pilotEyeY = 2.5f;
        pilotEyeZ = -15.0f;
    }
    
    // Get a scale factor relative to a "standard" medium aircraft
    float getScaleFactor() const {
        return wingspan / STANDARD_WINGSPAN;
    }
};

// Global aircraft dimensions
static AircraftDimensions g_aircraftDims;

// Plugin Global State
static PluginMode g_pluginMode = PluginMode::Off;
static bool g_functionActive = false;    // Is the camera control actually running
static bool g_functionPaused = false;    // Temporarily paused due to mouse movement

// Settings
static float g_delaySeconds = 60.0f;      // Delay before auto-activation (seconds)
static float g_autoAltFt = 18000.0f;      // Altitude threshold for auto mode (feet)
static float g_shotMinDuration = 6.0f;    // Minimum shot duration (longer for cinematic drift)
static float g_shotMaxDuration = 15.0f;   // Maximum shot duration (longer for cinematic drift)

// Mouse tracking
static int g_lastMouseX = 0;
static int g_lastMouseY = 0;
static float g_mouseIdleTime = 0.0f;      // Time since last mouse movement

// Camera control state
static float g_currentShotTime = 0.0f;
static float g_shotElapsedTime = 0.0f;    // Time elapsed in current shot (for drift calculation)
static int g_currentShotIndex = -1;
static int g_consecutiveSameTypeCount = 0;
static CameraType g_lastShotType = CameraType::Cockpit;
static CameraShot g_currentShot;           // Store current shot for drift calculation

// Smooth transition state
static float g_transitionProgress = 0.0f;
static float g_transitionDuration = 1.0f;  // 1 second transition
static XPLMCameraPosition_t g_startPos;
static XPLMCameraPosition_t g_targetPos;
static bool g_inTransition = false;

// Menu items
static XPLMMenuID g_menuId = nullptr;
static int g_menuItemAuto = -1;
static int g_menuItemStart = -1;
static int g_menuItemStop = -1;
static int g_menuItemSettings = -1;

// Datarefs
static XPLMDataRef g_drLatitude = nullptr;
static XPLMDataRef g_drLongitude = nullptr;
static XPLMDataRef g_drElevation = nullptr;
static XPLMDataRef g_drLocalX = nullptr;
static XPLMDataRef g_drLocalY = nullptr;
static XPLMDataRef g_drLocalZ = nullptr;
static XPLMDataRef g_drPitch = nullptr;
static XPLMDataRef g_drRoll = nullptr;
static XPLMDataRef g_drHeading = nullptr;
static XPLMDataRef g_drGroundSpeed = nullptr;
static XPLMDataRef g_drOnGround = nullptr;
static XPLMDataRef g_drElevationM = nullptr;  // Elevation in meters (we convert to feet for comparison)
static XPLMDataRef g_drPilotX = nullptr;
static XPLMDataRef g_drPilotY = nullptr;
static XPLMDataRef g_drPilotZ = nullptr;
static XPLMDataRef g_drViewType = nullptr;
static XPLMDataRef g_drTerrainY = nullptr;         // Terrain Y coordinate at aircraft position (AGL reference)

// Aircraft dimension datarefs (read from .acf file by X-Plane)
static XPLMDataRef g_drAcfSizeX = nullptr;         // Aircraft shadow/view size X (width/wingspan)
static XPLMDataRef g_drAcfSizeZ = nullptr;         // Aircraft shadow/view size Z (length)
static XPLMDataRef g_drAcfSemilenSEG = nullptr;    // Wing semilen per segment (array[56])
static XPLMDataRef g_drAcfSemilenJND = nullptr;    // Joined wing semilen (array[56])
static XPLMDataRef g_drAcfCgZFwd = nullptr;        // CG forward limit Z (approximates nose position)
static XPLMDataRef g_drAcfCgZAft = nullptr;        // CG aft limit Z (approximates tail position)
static XPLMDataRef g_drAcfMinY = nullptr;          // Minimum Y coordinate (bottom, e.g., gear)
static XPLMDataRef g_drAcfMaxY = nullptr;          // Maximum Y coordinate (top, e.g., tail)
static XPLMDataRef g_drAcfPeX = nullptr;           // Pilot eye X position
static XPLMDataRef g_drAcfPeY = nullptr;           // Pilot eye Y position  
static XPLMDataRef g_drAcfPeZ = nullptr;           // Pilot eye Z position

// Camera effect datarefs (writable - for cinematic effects)
static XPLMDataRef g_drFovHorizontal = nullptr;    // Horizontal field of view (degrees) - writable
static XPLMDataRef g_drFovVertical = nullptr;      // Vertical field of view (degrees) - writable
static XPLMDataRef g_drHandheldCam = nullptr;      // Handheld camera shake for external views - writable
static XPLMDataRef g_drGloadedCam = nullptr;       // G-loaded camera for internal views - writable
static XPLMDataRef g_drViewIsExternal = nullptr;   // Is view external? (readonly)
static XPLMDataRef g_drIsReplay = nullptr;         // Is in replay mode? (readonly)

// Cinematic effect settings
static bool g_enableFovEffect = true;              // Enable focal length simulation via FOV
static bool g_enableHandheldEffect = false;        // Enable handheld camera shake (off by default, user preference)
static bool g_enableGForceEffect = false;          // Enable G-force camera effect for internal views
static float g_baseFov = 60.0f;                    // Base horizontal FOV (degrees)
static float g_currentFov = 60.0f;                 // Current FOV being applied
static float g_targetFov = 60.0f;                  // Target FOV for smooth transitions
static float g_originalFov = 60.0f;                // Store original FOV to restore on stop
static float g_fovTransitionSpeed = 15.0f;         // FOV transition speed (degrees per second)
static float g_handheldIntensity = 0.5f;           // Handheld camera shake intensity (0-1)
static float g_originalHandheldCam = 0.0f;         // Store original handheld camera setting
static float g_originalGloadedCam = 0.0f;          // Store original G-loaded camera setting

// Flight loop callback
static XPLMFlightLoopID g_flightLoopId = nullptr;

// Predefined camera shots
static std::vector<CameraShot> g_cockpitShots;
static std::vector<CameraShot> g_externalShots;

/**
 * Settings Window using ImgWindow
 */
class SettingsWindow : public ImgWindow {
public:
    SettingsWindow();
    virtual ~SettingsWindow() = default;
    
protected:
    void buildInterface() override;
};

static std::unique_ptr<SettingsWindow> g_settingsWindow;

// Function declarations
static void ReadAircraftDimensions();
static void GenerateDynamicCameraShots();
static void UpdateMenuState();
static float FlightLoopCallback(float inElapsedSinceLastCall, float inElapsedTimeSinceLastFlightLoop, int inCounter, void* inRefcon);
static int CameraControlCallback(XPLMCameraPosition_t* outCameraPosition, int inIsLosingControl, void* inRefcon);
static void MenuHandler(void* inMenuRef, void* inItemRef);
static void StartCameraControl();
static void StopCameraControl();
static void PauseCameraControl();
static void ResumeCameraControl();
static bool CheckAutoConditions();
static CameraShot SelectNextShot();
static float Lerp(float a, float b, float t);
static float EaseInOutCubic(float t);
static float LinearDrift(float baseValue, float driftAmount, float normalizedTime);
static void SaveSettings();
static void LoadSettings();
static std::string GetPluginPath();
static float FocalLengthToFov(float focalLengthMm);
static float FovToFocalLength(float fovDeg);
static void ApplyFovEffect(float targetFocalLength, float deltaTime);
static void SaveCameraEffectState();
static void RestoreCameraEffectState();

/**
 * SettingsWindow constructor
 */
SettingsWindow::SettingsWindow() :
    ImgWindow(100, 800, 600, 100, xplm_WindowDecorationRoundRectangle, xplm_WindowLayerFloatingWindows)
{
    SetWindowTitle("MovieCamera Settings");
    SetWindowResizingLimits(450, 550, 700, 900);
}

/**
 * Build ImGui interface for settings
 */
void SettingsWindow::buildInterface() {
    ImGui::Text("MovieCamera Settings");
    ImGui::Separator();
    ImGui::Spacing();
    
    // Delay setting
    ImGui::Text("Delay (seconds):");
    ImGui::SameLine();
    ImGui::SetNextItemWidth(100);
    if (ImGui::InputFloat("##delay", &g_delaySeconds, 1.0f, 10.0f, "%.0f")) {
        if (g_delaySeconds < 1.0f) g_delaySeconds = 1.0f;
        if (g_delaySeconds > 300.0f) g_delaySeconds = 300.0f;
    }
    ImGui::SameLine();
    ImGui::TextDisabled("(?)");
    if (ImGui::IsItemHovered()) {
        ImGui::SetTooltip("Time to wait after mouse stops moving before activating camera");
    }
    
    ImGui::Spacing();
    
    // Auto Altitude setting
    ImGui::Text("Auto Alt (ft):");
    ImGui::SameLine();
    ImGui::SetNextItemWidth(100);
    if (ImGui::InputFloat("##autoalt", &g_autoAltFt, 100.0f, 1000.0f, "%.0f")) {
        if (g_autoAltFt < 0.0f) g_autoAltFt = 0.0f;
        if (g_autoAltFt > 50000.0f) g_autoAltFt = 50000.0f;
    }
    ImGui::SameLine();
    ImGui::TextDisabled("(?)");
    if (ImGui::IsItemHovered()) {
        ImGui::SetTooltip("Altitude above which Auto mode can activate (feet MSL)");
    }
    
    ImGui::Spacing();
    ImGui::Separator();
    ImGui::Text("Shot Duration Range:");
    
    ImGui::Text("Min (s):");
    ImGui::SameLine();
    ImGui::SetNextItemWidth(80);
    if (ImGui::InputFloat("##shotmin", &g_shotMinDuration, 0.5f, 1.0f, "%.1f")) {
        if (g_shotMinDuration < 1.0f) g_shotMinDuration = 1.0f;
        if (g_shotMinDuration > g_shotMaxDuration) g_shotMinDuration = g_shotMaxDuration;
    }
    
    ImGui::SameLine();
    ImGui::Text("Max (s):");
    ImGui::SameLine();
    ImGui::SetNextItemWidth(80);
    if (ImGui::InputFloat("##shotmax", &g_shotMaxDuration, 0.5f, 1.0f, "%.1f")) {
        if (g_shotMaxDuration < g_shotMinDuration) g_shotMaxDuration = g_shotMinDuration;
        if (g_shotMaxDuration > 30.0f) g_shotMaxDuration = 30.0f;
    }
    
    ImGui::Spacing();
    ImGui::Separator();
    ImGui::Text("Status:");
    
    const char* modeStr = "Off";
    if (g_pluginMode == PluginMode::Auto) modeStr = "Auto";
    else if (g_pluginMode == PluginMode::Manual) modeStr = "Manual";
    ImGui::Text("Mode: %s", modeStr);
    
    const char* stateStr = "Inactive";
    if (g_functionActive && !g_functionPaused) stateStr = "Active";
    else if (g_functionActive && g_functionPaused) stateStr = "Paused";
    ImGui::Text("State: %s", stateStr);
    
    ImGui::Text("Mouse Idle: %.1f s", g_mouseIdleTime);
    
    ImGui::Spacing();
    ImGui::Separator();
    ImGui::Spacing();
    
    // Cinematic Effects Section
    ImGui::Text("Cinematic Effects");
    ImGui::SameLine();
    ImGui::TextDisabled("(?)");
    if (ImGui::IsItemHovered()) {
        ImGui::SetTooltip("Configure camera effects for more cinematic footage.\nFOV control simulates different focal lengths.\nHandheld effect adds realistic camera shake.");
    }
    
    // FOV/Focal Length Effect
    ImGui::Checkbox("Enable FOV Effect", &g_enableFovEffect);
    if (ImGui::IsItemHovered()) {
        ImGui::SetTooltip("Enable focal length simulation via FOV control");
    }
    
    if (g_enableFovEffect) {
        ImGui::Indent();
        
        // Display current focal length equivalent
        float currentFocalLength = FovToFocalLength(g_baseFov);
        ImGui::Text("Focal Length: %.1f mm (%.1f° FOV)", currentFocalLength, g_baseFov);
        
        // FOV slider
        ImGui::SetNextItemWidth(200);
        if (ImGui::SliderFloat("Base FOV##fov", &g_baseFov, MIN_FOV_DEG, MAX_FOV_DEG, "%.1f°")) {
            // Keep within valid range
            if (g_baseFov < MIN_FOV_DEG) g_baseFov = MIN_FOV_DEG;
            if (g_baseFov > MAX_FOV_DEG) g_baseFov = MAX_FOV_DEG;
        }
        
        // Focal length presets
        ImGui::Text("Presets:");
        ImGui::SameLine();
        if (ImGui::SmallButton("24mm")) g_baseFov = FocalLengthToFov(24.0f);
        ImGui::SameLine();
        if (ImGui::SmallButton("35mm")) g_baseFov = FocalLengthToFov(35.0f);
        ImGui::SameLine();
        if (ImGui::SmallButton("50mm")) g_baseFov = FocalLengthToFov(50.0f);
        ImGui::SameLine();
        if (ImGui::SmallButton("85mm")) g_baseFov = FocalLengthToFov(85.0f);
        ImGui::SameLine();
        if (ImGui::SmallButton("135mm")) g_baseFov = FocalLengthToFov(135.0f);
        
        // Transition speed
        ImGui::SetNextItemWidth(150);
        ImGui::SliderFloat("Transition Speed##fovspeed", &g_fovTransitionSpeed, 1.0f, 30.0f, "%.1f");
        if (ImGui::IsItemHovered()) {
            ImGui::SetTooltip("Speed of FOV transitions between shots (degrees per second)");
        }
        
        ImGui::Unindent();
    }
    
    // Handheld Camera Effect
    ImGui::Checkbox("Enable Handheld Effect", &g_enableHandheldEffect);
    if (ImGui::IsItemHovered()) {
        ImGui::SetTooltip("Enable handheld camera shake effect for external views");
    }
    
    if (g_enableHandheldEffect) {
        ImGui::Indent();
        ImGui::SetNextItemWidth(150);
        ImGui::SliderFloat("Shake Intensity##shake", &g_handheldIntensity, 0.0f, 1.0f, "%.2f");
        if (ImGui::IsItemHovered()) {
            ImGui::SetTooltip("Amount of camera shake (0 = none, 1 = maximum)");
        }
        ImGui::Unindent();
    }
    
    // G-Force Camera Effect (Internal views)
    ImGui::Checkbox("Enable G-Force Effect", &g_enableGForceEffect);
    if (ImGui::IsItemHovered()) {
        ImGui::SetTooltip("Enable G-force camera movement for internal views");
    }
    
    ImGui::Spacing();
    ImGui::Separator();
    ImGui::Spacing();
    
    if (ImGui::Button("Close", ImVec2(80, 0))) {
        SetVisible(false);
    }
}

/**
 * Read aircraft dimensions from X-Plane datarefs
 * Uses multiple data sources for best accuracy:
 * 1. acf_size_x/z - Shadow and viewing distance (most reliable for overall size)
 * 2. acf_semilen_SEG/JND - Wing segment semi-lengths for accurate wingspan
 * 3. CG limits and pilot eye position - for fuselage length estimation
 */
static void ReadAircraftDimensions() {
    // Set defaults first
    g_aircraftDims.setDefaults();
    
    // Read pilot eye position (relative to aircraft CG)
    if (g_drAcfPeX) g_aircraftDims.pilotEyeX = XPLMGetDataf(g_drAcfPeX);
    if (g_drAcfPeY) g_aircraftDims.pilotEyeY = XPLMGetDataf(g_drAcfPeY);
    if (g_drAcfPeZ) g_aircraftDims.pilotEyeZ = XPLMGetDataf(g_drAcfPeZ);
    
    // ========================================
    // Method 1: Use acf_size_x for wingspan (most reliable)
    // This is the shadow/viewing distance size set in Plane Maker
    // ========================================
    if (g_drAcfSizeX) {
        float sizeX = XPLMGetDataf(g_drAcfSizeX);
        if (sizeX > 5.0f) {  // Reasonable minimum wingspan
            g_aircraftDims.wingspan = sizeX;
            char msg[128];
            snprintf(msg, sizeof(msg), "MovieCamera: Using acf_size_x for wingspan: %.1fm\n", sizeX);
            XPLMDebugString(msg);
        }
    }
    
    // ========================================
    // Method 2: Use acf_semilen for more precise wingspan calculation
    // Sum the maximum semi-lengths from each wing segment
    // ========================================
    if (g_drAcfSemilenJND) {
        float semilenData[56];
        int count = XPLMGetDatavf(g_drAcfSemilenJND, semilenData, 0, 56);
        if (count > 0) {
            float maxSemilen = 0.0f;
            for (int i = 0; i < count; i++) {
                if (semilenData[i] > maxSemilen) {
                    maxSemilen = semilenData[i];
                }
            }
            if (maxSemilen > 2.0f) {  // Minimum reasonable semi-span
                float wingspan = maxSemilen * 2.0f;  // Full span = 2 * semi-span
                // Only use if significantly different from size_x (could be more accurate)
                if (wingspan > g_aircraftDims.wingspan * 0.8f && wingspan < g_aircraftDims.wingspan * 1.5f) {
                    // Values are close, prefer the larger one
                    if (wingspan > g_aircraftDims.wingspan) {
                        g_aircraftDims.wingspan = wingspan;
                    }
                } else if (g_aircraftDims.wingspan < MIN_WINGSPAN) {
                    // size_x didn't work, use semilen
                    g_aircraftDims.wingspan = wingspan;
                }
                char msg[128];
                snprintf(msg, sizeof(msg), "MovieCamera: Wing semilen max: %.1fm, calculated wingspan: %.1fm\n", maxSemilen, wingspan);
                XPLMDebugString(msg);
            }
        }
    }
    
    // ========================================
    // Method 3: Use acf_size_z for fuselage length (most reliable)
    // ========================================
    if (g_drAcfSizeZ) {
        float sizeZ = XPLMGetDataf(g_drAcfSizeZ);
        if (sizeZ > 5.0f) {  // Reasonable minimum length
            g_aircraftDims.fuselageLength = sizeZ;
            char msg[128];
            snprintf(msg, sizeof(msg), "MovieCamera: Using acf_size_z for fuselage length: %.1fm\n", sizeZ);
            XPLMDebugString(msg);
        }
    }
    
    // Fallback: Use CG limits if size_z didn't work
    if (g_aircraftDims.fuselageLength < MIN_FUSELAGE_LENGTH + 1.0f) {
        float cgZFwd = 0.0f, cgZAft = 0.0f;
        if (g_drAcfCgZFwd) cgZFwd = XPLMGetDataf(g_drAcfCgZFwd);
        if (g_drAcfCgZAft) cgZAft = XPLMGetDataf(g_drAcfCgZAft);
        
        if (cgZFwd != 0.0f || cgZAft != 0.0f) {
            float cgRange = std::abs(cgZAft - cgZFwd);
            if (cgRange > MIN_VALID_CG_RANGE) {
                g_aircraftDims.fuselageLength = cgRange * CG_TO_FUSELAGE_MULTIPLIER;
            }
        }
    }
    
    // Alternative: Use pilot eye Z position to estimate aircraft size
    if (g_aircraftDims.pilotEyeZ != 0.0f && g_aircraftDims.fuselageLength < MIN_FUSELAGE_LENGTH + 1.0f) {
        g_aircraftDims.fuselageLength = std::abs(g_aircraftDims.pilotEyeZ) * PILOT_Z_TO_FUSELAGE_MULTIPLIER;
    }
    
    // ========================================
    // Height estimation
    // ========================================
    float minY = 0.0f;
    if (g_drAcfMinY) minY = XPLMGetDataf(g_drAcfMinY);
    
    if (minY != 0.0f && g_aircraftDims.pilotEyeY > 0.0f) {
        // Calculate height from gear to pilot eye position, plus estimated distance above pilot
        g_aircraftDims.height = g_aircraftDims.pilotEyeY - minY + ESTIMATED_GROUND_CLEARANCE;
    } else if (g_aircraftDims.pilotEyeY > 0.0f) {
        // Estimate height from pilot eye Y position plus ground clearance
        g_aircraftDims.height = g_aircraftDims.pilotEyeY * PILOT_Y_TO_HEIGHT_MULTIPLIER + ESTIMATED_GROUND_CLEARANCE;
    }
    
    // ========================================
    // Validate and constrain dimensions
    // ========================================
    if (g_aircraftDims.wingspan < MIN_WINGSPAN) g_aircraftDims.wingspan = STANDARD_WINGSPAN;
    if (g_aircraftDims.wingspan > MAX_WINGSPAN) g_aircraftDims.wingspan = MAX_WINGSPAN;
    if (g_aircraftDims.fuselageLength < MIN_FUSELAGE_LENGTH) g_aircraftDims.fuselageLength = STANDARD_FUSELAGE_LENGTH;
    if (g_aircraftDims.fuselageLength > MAX_FUSELAGE_LENGTH) g_aircraftDims.fuselageLength = MAX_FUSELAGE_LENGTH;
    if (g_aircraftDims.height < MIN_HEIGHT) g_aircraftDims.height = STANDARD_HEIGHT;
    if (g_aircraftDims.height > MAX_HEIGHT) g_aircraftDims.height = MAX_HEIGHT;
    
    char msg[256];
    snprintf(msg, sizeof(msg), "MovieCamera: Final aircraft dims - Wingspan: %.1fm, Length: %.1fm, Height: %.1fm, PilotEye: (%.1f, %.1f, %.1f)\n",
             g_aircraftDims.wingspan, g_aircraftDims.fuselageLength, g_aircraftDims.height,
             g_aircraftDims.pilotEyeX, g_aircraftDims.pilotEyeY, g_aircraftDims.pilotEyeZ);
    XPLMDebugString(msg);
}

/**
 * Calculate the minimum camera distance required to keep the aircraft visible
 * This ensures the camera is far enough to frame the aircraft properly
 */
static float CalculateMinVisibleDistance() {
    // The larger the aircraft, the farther the camera needs to be
    // Use the maximum dimension (wingspan or fuselage) as reference
    float maxDimension = std::max(g_aircraftDims.wingspan, g_aircraftDims.fuselageLength);
    // Camera should be at least 1.5x the max dimension away to ensure full visibility
    return std::max(maxDimension * 1.5f, MIN_CAMERA_DISTANCE_FROM_AIRCRAFT);
}

/**
 * Calculate intelligent zoom based on aircraft size and camera distance
 * Larger aircraft need lower zoom (wider view) to stay in frame
 * Smaller aircraft need higher zoom (closer view) to be visible
 * Farther cameras may need more zoom to keep aircraft visible
 */
static float CalculateIntelligentZoom(float baseZoom, float cameraDistance) {
    // Get the scaling factor for the aircraft
    float scale = g_aircraftDims.getScaleFactor();
    
    // For larger aircraft (scale > 1), reduce zoom to fit in frame
    // For smaller aircraft (scale < 1), increase zoom to make aircraft more visible
    float zoomAdjustment = 1.0f / std::sqrt(scale);
    
    // Adjust zoom based on camera distance - farther cameras need more zoom
    // Use wingspan as reference distance
    float distanceFactor = cameraDistance / (g_aircraftDims.wingspan * 2.0f);
    distanceFactor = std::clamp(distanceFactor, 0.7f, 1.5f);
    
    // Combine adjustments - farther distance increases zoom slightly
    float adjustedZoom = baseZoom * zoomAdjustment * ZOOM_SCALE_FACTOR * distanceFactor;
    
    // Clamp to reasonable zoom range (0.5 = wide, 2.0 = telephoto)
    return std::clamp(adjustedZoom, 0.5f, 2.0f);
}

/**
 * Generate dynamic camera shots based on aircraft dimensions
 * This calculates camera positions relative to the aircraft's actual size
 * 
 * Camera positioning principles:
 * 1. External shots are positioned based on aircraft size (wingspan, fuselage length, height)
 * 2. Camera should never clip into the aircraft model
 * 3. Each shot provides a visually distinct perspective
 * 4. Zoom levels are calculated to keep aircraft well-framed
 * 5. Drift amounts create smooth, cinematic camera movement
 */
static void GenerateDynamicCameraShots() {
    // Get scale factor based on aircraft size
    float scale = g_aircraftDims.getScaleFactor();
    float wingspan = g_aircraftDims.wingspan;
    float fuselageLen = g_aircraftDims.fuselageLength;
    float height = g_aircraftDims.height;
    
    // Clear existing shots
    g_cockpitShots.clear();
    g_externalShots.clear();
    
    // =====================================================
    // COCKPIT SHOTS - These are relative to pilot eye position
    // Scale cockpit movements based on cockpit size estimation
    // =====================================================
    float cockpitScale = std::sqrt(scale);  // Use sqrt for subtler scaling in cockpit
    
    // Center panel view - main instrument scan position
    g_cockpitShots.push_back({CameraType::Cockpit, 0.0f, 0.12f * cockpitScale, 0.35f * cockpitScale,
                              -10.0f, 0.0f, 0.0f, 1.0f, 9.0f, "Center Panel",
                              0.0f, 0.008f, 0.015f, 0.15f, 0.0f, 0.0f, 0.025f});
    
    // Left panel - throttle quadrant area
    g_cockpitShots.push_back({CameraType::Cockpit, -0.22f * cockpitScale, 0.08f * cockpitScale, 0.25f * cockpitScale,
                              -15.0f, -30.0f, 0.0f, 1.15f, 8.0f, "Left Panel",
                              0.008f, 0.0f, 0.01f, 0.12f, 0.8f, 0.0f, 0.02f});
    
    // Right panel - radio/FMS area
    g_cockpitShots.push_back({CameraType::Cockpit, 0.22f * cockpitScale, 0.08f * cockpitScale, 0.25f * cockpitScale,
                              -15.0f, 30.0f, 0.0f, 1.15f, 8.0f, "Right Panel",
                              -0.008f, 0.0f, 0.01f, 0.12f, -0.8f, 0.0f, 0.02f});
    
    // Overhead panel - looking up at switches
    g_cockpitShots.push_back({CameraType::Cockpit, 0.0f, 0.30f * cockpitScale, 0.12f * cockpitScale,
                              -50.0f, 0.0f, 0.0f, 1.05f, 7.0f, "Overhead Panel",
                              0.0f, -0.008f, 0.008f, 1.2f, 0.0f, 0.0f, 0.015f});
    
    // PFD closeup - primary flight display focus
    g_cockpitShots.push_back({CameraType::Cockpit, -0.10f * cockpitScale, 0.04f * cockpitScale, 0.40f * cockpitScale,
                              -5.0f, -10.0f, 0.0f, 1.5f, 9.0f, "PFD View",
                              0.004f, 0.004f, 0.012f, 0.08f, 0.25f, 0.0f, 0.035f});
    
    // ND/MFD view - navigation display focus
    g_cockpitShots.push_back({CameraType::Cockpit, 0.10f * cockpitScale, 0.04f * cockpitScale, 0.40f * cockpitScale,
                              -5.0f, 10.0f, 0.0f, 1.5f, 9.0f, "ND View",
                              -0.004f, 0.004f, 0.012f, 0.08f, -0.25f, 0.0f, 0.035f});
    
    // Pilot forward view - looking out windscreen
    g_cockpitShots.push_back({CameraType::Cockpit, -0.08f * cockpitScale, 0.20f * cockpitScale, -0.08f * cockpitScale,
                              5.0f, 3.0f, 0.0f, 0.85f, 11.0f, "Pilot View",
                              0.004f, 0.0f, 0.0f, 0.0f, 0.6f, 0.0f, 0.0f});
    
    // Co-pilot perspective
    g_cockpitShots.push_back({CameraType::Cockpit, 0.30f * cockpitScale, 0.18f * cockpitScale, 0.0f,
                              2.0f, -15.0f, 0.0f, 0.90f, 9.0f, "Copilot View",
                              -0.008f, 0.0f, 0.0f, 0.0f, 0.4f, 0.0f, 0.008f});
    
    // Left window view - scenic exterior
    g_cockpitShots.push_back({CameraType::Cockpit, -0.30f * cockpitScale, 0.12f * cockpitScale, 0.0f,
                              5.0f, -80.0f, 0.0f, 0.80f, 10.0f, "Left Window",
                              0.0f, 0.008f, 0.0f, -0.2f, 1.5f, 0.0f, 0.0f});
    
    // Right window view - scenic exterior
    g_cockpitShots.push_back({CameraType::Cockpit, 0.30f * cockpitScale, 0.12f * cockpitScale, 0.0f,
                              5.0f, 80.0f, 0.0f, 0.80f, 10.0f, "Right Window",
                              0.0f, 0.008f, 0.0f, -0.2f, -1.5f, 0.0f, 0.0f});
    
    // Pedestal/center console view - MCDU/throttles
    g_cockpitShots.push_back({CameraType::Cockpit, 0.0f, -0.05f * cockpitScale, 0.30f * cockpitScale,
                              -40.0f, 0.0f, 0.0f, 1.3f, 7.0f, "Pedestal View",
                              0.0f, 0.008f, 0.008f, 0.4f, 0.0f, 0.0f, 0.025f});
    
    // =====================================================
    // EXTERNAL SHOTS - Scaled based on aircraft dimensions
    // Positioning uses aircraft dimensions as reference
    // =====================================================
    
    // Calculate safe distances based on aircraft size
    float minVisibleDist = CalculateMinVisibleDistance();
    
    // Base distances proportional to aircraft dimensions
    float frontDist = std::max(fuselageLen * 1.4f, minVisibleDist);       // Front shots
    float rearDist = std::max(fuselageLen * 1.6f, minVisibleDist);        // Rear shots
    float sideDist = std::max(wingspan * 1.5f, minVisibleDist);           // Side shots
    float highDist = std::max(wingspan * 2.0f, minVisibleDist);           // High altitude shots
    float closeDist = std::max(wingspan * 0.8f, minVisibleDist * 0.8f);   // Close-up shots
    float midDist = std::max(wingspan * 1.2f, minVisibleDist);            // Mid-range shots
    
    // Drift amounts scale with aircraft size (larger aircraft = slower perceived drift)
    float driftScale = 0.7f + scale * 0.3f;
    
    // Calculate intelligent zoom for external shots
    float baseZoom = CalculateIntelligentZoom(0.80f, midDist);
    float closeZoom = CalculateIntelligentZoom(0.95f, closeDist);
    float wideZoom = CalculateIntelligentZoom(0.65f, highDist);
    float frontZoom = CalculateIntelligentZoom(0.85f, frontDist);
    
    // ---- HERO SHOTS (Dramatic main angles) ----
    
    // Front Hero - Classic nose-on shot, slightly elevated
    g_externalShots.push_back({CameraType::External,
                               wingspan * 0.12f, height * 0.8f, -frontDist,
                               8.0f, 178.0f, 0.0f, frontZoom, 11.0f, "Front Hero",
                               -0.08f * driftScale, 0.10f * driftScale, 0.20f * driftScale,
                               -0.20f, 0.25f, 0.0f, 0.008f});
    
    // Rear Chase - Following shot from behind
    g_externalShots.push_back({CameraType::External,
                               -wingspan * 0.15f, height * 1.1f, rearDist,
                               12.0f, 5.0f, 0.0f, baseZoom, 12.0f, "Rear Chase",
                               0.12f * driftScale, 0.06f * driftScale, -0.15f * driftScale,
                               -0.12f, -0.30f, 0.0f, 0.0f});
    
    // High Wide - Establishing shot from above
    g_externalShots.push_back({CameraType::External,
                               wingspan * 0.3f, highDist * 1.5f, fuselageLen * 0.5f,
                               55.0f, -20.0f, 0.0f, wideZoom, 14.0f, "High Wide",
                               -0.25f * driftScale, 0.02f * driftScale, 0.0f,
                               0.0f, 1.8f, 0.0f, 0.0f});
    
    // ---- FLYBY SHOTS (Side sweep angles) ----
    
    // Left Flyby - Dramatic side sweep
    g_externalShots.push_back({CameraType::External,
                               -sideDist, height * 0.5f, fuselageLen * 0.3f,
                               4.0f, 85.0f, 1.5f, baseZoom, 13.0f, "Left Flyby",
                               0.40f * driftScale, 0.08f * driftScale, -0.50f * driftScale,
                               0.0f, 0.8f, -0.08f, 0.0f});
    
    // Right Flyby - Dramatic side sweep
    g_externalShots.push_back({CameraType::External,
                               sideDist, height * 0.5f, fuselageLen * 0.3f,
                               4.0f, -85.0f, -1.5f, baseZoom, 13.0f, "Right Flyby",
                               -0.40f * driftScale, 0.08f * driftScale, -0.50f * driftScale,
                               0.0f, -0.8f, 0.08f, 0.0f});
    
    // ---- QUARTER ANGLE SHOTS (45-degree views) ----
    
    // Quarter Front Left - Approaching from front-left
    g_externalShots.push_back({CameraType::External,
                               -midDist * 0.9f, height * 1.0f, -frontDist * 0.85f,
                               12.0f, 140.0f, -0.5f, frontZoom * 0.95f, 11.0f, "Quarter FL",
                               0.20f * driftScale, 0.05f * driftScale, 0.25f * driftScale,
                               -0.10f, -0.60f, 0.04f, 0.0f});
    
    // Quarter Front Right - Approaching from front-right
    g_externalShots.push_back({CameraType::External,
                               midDist * 0.9f, height * 1.0f, -frontDist * 0.85f,
                               12.0f, -140.0f, 0.5f, frontZoom * 0.95f, 11.0f, "Quarter FR",
                               -0.20f * driftScale, 0.05f * driftScale, 0.25f * driftScale,
                               -0.10f, 0.60f, -0.04f, 0.0f});
    
    // Quarter Rear Left - Departure view from rear-left
    g_externalShots.push_back({CameraType::External,
                               -midDist * 0.8f, height * 1.4f, rearDist * 0.85f,
                               18.0f, 40.0f, 1.5f, baseZoom * 0.92f, 11.0f, "Quarter RL",
                               0.18f * driftScale, 0.04f * driftScale, -0.18f * driftScale,
                               -0.15f, -0.50f, -0.08f, 0.0f});
    
    // Quarter Rear Right - Departure view from rear-right
    g_externalShots.push_back({CameraType::External,
                               midDist * 0.8f, height * 1.4f, rearDist * 0.85f,
                               18.0f, -40.0f, -1.5f, baseZoom * 0.92f, 11.0f, "Quarter RR",
                               -0.18f * driftScale, 0.04f * driftScale, -0.18f * driftScale,
                               -0.15f, 0.50f, 0.08f, 0.0f});
    
    // ---- CLOSE-UP SHOTS (Detail views) ----
    
    // Wing Left Close - Wing and engine detail
    g_externalShots.push_back({CameraType::External,
                               -closeDist * 0.9f, height * 0.4f, fuselageLen * 0.15f,
                               8.0f, 65.0f, -2.0f, closeZoom, 9.0f, "Wing Left",
                               0.08f * driftScale, 0.03f * driftScale, -0.10f * driftScale,
                               0.0f, 0.50f, 0.12f, 0.0f});
    
    // Wing Right Close - Wing and engine detail
    g_externalShots.push_back({CameraType::External,
                               closeDist * 0.9f, height * 0.4f, fuselageLen * 0.15f,
                               8.0f, -65.0f, 2.0f, closeZoom, 9.0f, "Wing Right",
                               -0.08f * driftScale, 0.03f * driftScale, -0.10f * driftScale,
                               0.0f, -0.50f, -0.12f, 0.0f});
    
    // Engine Left - Engine nacelle focus
    g_externalShots.push_back({CameraType::External,
                               -wingspan * 0.35f, height * 0.2f, -fuselageLen * 0.05f,
                               6.0f, 70.0f, 0.0f, closeZoom * 1.15f, 8.0f, "Engine L",
                               0.05f * driftScale, 0.025f * driftScale, -0.08f * driftScale,
                               0.0f, 0.35f, 0.0f, 0.0f});
    
    // Engine Right - Engine nacelle focus
    g_externalShots.push_back({CameraType::External,
                               wingspan * 0.35f, height * 0.2f, -fuselageLen * 0.05f,
                               6.0f, -70.0f, 0.0f, closeZoom * 1.15f, 8.0f, "Engine R",
                               -0.05f * driftScale, 0.025f * driftScale, -0.08f * driftScale,
                               0.0f, -0.35f, 0.0f, 0.0f});
    
    // Tail View - Empennage focus
    g_externalShots.push_back({CameraType::External,
                               -wingspan * 0.2f, height * 1.3f, rearDist * 1.2f,
                               25.0f, 8.0f, 0.0f, baseZoom * 0.95f, 10.0f, "Tail View",
                               0.08f * driftScale, 0.05f * driftScale, -0.10f * driftScale,
                               -0.20f, -0.50f, 0.0f, 0.0f});
    
    // ---- SPECIALTY SHOTS (Unique angles) ----
    
    // Low Front - Dramatic low angle looking up
    g_externalShots.push_back({CameraType::External,
                               wingspan * 0.25f, height * 0.15f, -frontDist * 0.7f,
                               -18.0f, 165.0f, 2.0f, frontZoom * 1.05f, 9.0f, "Low Front",
                               -0.08f * driftScale, 0.12f * driftScale, 0.18f * driftScale,
                               0.30f, 0.40f, -0.15f, 0.0f});
    
    // Belly View - Looking up from below
    g_externalShots.push_back({CameraType::External,
                               wingspan * 0.15f, -height * 0.8f, fuselageLen * 0.1f,
                               -40.0f, -8.0f, 0.0f, baseZoom * 1.05f, 8.0f, "Belly View",
                               -0.04f * driftScale, 0.06f * driftScale, 0.0f,
                               0.25f, 0.35f, 0.0f, 0.0f});
    
    // Side Profile - Pure side view
    g_externalShots.push_back({CameraType::External,
                               -sideDist * 0.85f, height * 0.6f, 0.0f,
                               3.0f, 90.0f, 0.0f, baseZoom * 0.95f, 10.0f, "Side Profile L",
                               0.30f * driftScale, 0.04f * driftScale, 0.0f,
                               0.0f, 0.0f, 0.0f, 0.0f});
    
    // Nose Close - Cockpit window close-up
    g_externalShots.push_back({CameraType::External,
                               -wingspan * 0.08f, height * 0.5f, -fuselageLen * 0.55f,
                               5.0f, 175.0f, 0.0f, closeZoom * 1.2f, 8.0f, "Nose Close",
                               0.04f * driftScale, 0.06f * driftScale, 0.12f * driftScale,
                               -0.08f, 0.20f, 0.0f, 0.015f});
    
    char msg[128];
    snprintf(msg, sizeof(msg), "MovieCamera: Generated %zu cockpit and %zu external shots (scale: %.2f)\n",
             g_cockpitShots.size(), g_externalShots.size(), scale);
    XPLMDebugString(msg);
}

/**
 * Update menu item states based on current plugin state
 */
static void UpdateMenuState() {
    if (!g_menuId) return;
    
    // Auto mode checkbox
    XPLMCheckMenuItem(g_menuId, g_menuItemAuto, 
        g_pluginMode == PluginMode::Auto ? xplm_Menu_Checked : xplm_Menu_Unchecked);
    
    // Start/Stop availability based on current state
    if (g_pluginMode == PluginMode::Auto) {
        // In auto mode, both Start and Stop are disabled
        XPLMEnableMenuItem(g_menuId, g_menuItemStart, 0);
        XPLMEnableMenuItem(g_menuId, g_menuItemStop, 0);
    } else {
        // In manual mode, enable based on function state
        XPLMEnableMenuItem(g_menuId, g_menuItemStart, !g_functionActive ? 1 : 0);
        XPLMEnableMenuItem(g_menuId, g_menuItemStop, g_functionActive ? 1 : 0);
    }
    
    // Check marks for Start/Stop based on active state
    XPLMCheckMenuItem(g_menuId, g_menuItemStart, 
        (g_pluginMode == PluginMode::Manual && g_functionActive) ? xplm_Menu_Checked : xplm_Menu_Unchecked);
    XPLMCheckMenuItem(g_menuId, g_menuItemStop, xplm_Menu_Unchecked);
}

/**
 * Menu handler callback
 */
static void MenuHandler(void* inMenuRef, void* inItemRef) {
    (void)inMenuRef;
    intptr_t menuItem = reinterpret_cast<intptr_t>(inItemRef);
    
    switch (menuItem) {
        case 0:  // Auto
            if (g_pluginMode == PluginMode::Auto) {
                // Turn off auto mode
                g_pluginMode = PluginMode::Off;
                StopCameraControl();
            } else {
                // Turn on auto mode
                g_pluginMode = PluginMode::Auto;
                g_mouseIdleTime = 0.0f;
            }
            break;
            
        case 1:  // Start
            if (g_pluginMode != PluginMode::Auto && !g_functionActive) {
                g_pluginMode = PluginMode::Manual;
                StartCameraControl();
            }
            break;
            
        case 2:  // Stop
            if (g_functionActive) {
                g_pluginMode = PluginMode::Off;
                StopCameraControl();
            }
            break;
            
        case 3:  // Settings
            if (g_settingsWindow) {
                if (g_settingsWindow->GetVisible()) {
                    g_settingsWindow->SetVisible(false);
                } else {
                    g_settingsWindow->SetVisible(true);
                }
            }
            break;
    }
    
    UpdateMenuState();
}

/**
 * Linear interpolation
 */
static float Lerp(float a, float b, float t) {
    return a + (b - a) * t;
}

/**
 * Transform a point from aircraft-local coordinates to world coordinates
 * considering full aircraft attitude (heading, pitch, roll)
 * @param localX, localY, localZ - Position in aircraft-local coordinates
 * @param acfX, acfY, acfZ - Aircraft position in world coordinates
 * @param heading, pitch, roll - Aircraft attitude in degrees
 * @param outX, outY, outZ - Output world coordinates
 */
static void TransformToWorldCoordinates(
    float localX, float localY, float localZ,
    float acfX, float acfY, float acfZ,
    float heading, float pitch, float roll,
    float& outX, float& outY, float& outZ)
{
    // Convert angles to radians
    float h = heading * PI / 180.0f;
    float p = pitch * PI / 180.0f;
    float r = roll * PI / 180.0f;
    
    // Precompute trigonometric values
    float cosH = std::cos(h), sinH = std::sin(h);
    float cosP = std::cos(p), sinP = std::sin(p);
    float cosR = std::cos(r), sinR = std::sin(r);
    
    // Combined rotation matrix (ZYX order: heading, then pitch, then roll)
    // This matches X-Plane's coordinate system conventions
    // X-Plane uses: heading (psi) around Y, pitch (theta) around X, roll (phi) around Z
    
    // First rotate by heading (around Y axis)
    float x1 = localX * cosH - localZ * sinH;
    float y1 = localY;
    float z1 = localX * sinH + localZ * cosH;
    
    // Then rotate by pitch (around X axis) - note: X-Plane pitch positive = nose up
    float x2 = x1;
    float y2 = y1 * cosP + z1 * sinP;
    float z2 = -y1 * sinP + z1 * cosP;
    
    // Finally rotate by roll (around Z axis)
    float x3 = x2 * cosR - y2 * sinR;
    float y3 = x2 * sinR + y2 * cosR;
    float z3 = z2;
    
    // Add aircraft position
    outX = acfX + x3;
    outY = acfY + y3;
    outZ = acfZ + z3;
}

/**
 * Ease in-out cubic for smooth transitions
 */
static float EaseInOutCubic(float t) {
    return t < 0.5f ? 4.0f * t * t * t : 1.0f - std::pow(-2.0f * t + 2.0f, 3.0f) / 2.0f;
}

/**
 * Normalize angle to -180 to 180 range
 */
static float NormalizeAngle(float angle) {
    while (angle > 180.0f) angle -= 360.0f;
    while (angle < -180.0f) angle += 360.0f;
    return angle;
}

/**
 * Interpolate angles properly handling wraparound
 */
static float LerpAngle(float a, float b, float t) {
    float diff = NormalizeAngle(b - a);
    return a + diff * t;
}

/**
 * Ease-in only function for smooth start without slowdown at end
 * This creates acceleration at start but maintains constant speed until end
 */
static float EaseInCubic(float t) {
    return t * t * t;
}

/**
 * Linear drift with smooth ease-in only for consistent camera movement
 * Creates a steady, directional drift that accelerates smoothly at start
 * but maintains constant speed until end (no slowdown before cut)
 * Once a drift direction is set, it maintains that direction throughout the shot
 */
static float LinearDrift(float baseValue, float driftAmount, float normalizedTime) {
    // Use ease-in only for smooth start without slowdown at end
    // normalizedTime is 0 at start, 1 at end of shot
    // Apply ease-in only for first portion (EASE_IN_DURATION_RATIO) of the shot, then linear progression
    float smoothT;
    if (normalizedTime < EASE_IN_DURATION_RATIO) {
        // Ease-in phase: smooth acceleration using cubic easing
        float t = normalizedTime / EASE_IN_DURATION_RATIO;  // Normalize to 0-1 for ease-in portion
        smoothT = EaseInCubic(t) * EASE_IN_DURATION_RATIO;  // Scale back to 0-EASE_IN_DURATION_RATIO range
    } else {
        // Linear phase: constant speed until end (no deceleration)
        // Calculate the derivative at the end of ease-in phase to ensure continuity
        // Derivative of t^3 at t=1 is 3, so slope at transition is 3 * EASE_IN_DURATION_RATIO
        // We continue from the ease-in endpoint with this slope
        float slope = 3.0f * EASE_IN_DURATION_RATIO;  // Derivative of EaseInCubic(t) * EASE_IN_DURATION_RATIO at t=1
        float easeInEndValue = EASE_IN_DURATION_RATIO;  // Value at end of ease-in phase (t^3 at t=1 = 1, scaled)
        smoothT = easeInEndValue + slope * (normalizedTime - EASE_IN_DURATION_RATIO);
    }
    // Clamp smoothT to prevent overshooting beyond the intended drift amount
    smoothT = std::min(smoothT, 1.0f);
    return baseValue + driftAmount * smoothT;
}

/**
 * Ease in-out sine for extra smooth interpolation
 */
static float EaseInOutSine(float t) {
    return -(std::cos(PI * t) - 1.0f) / 2.0f;
}

/**
 * Ensure camera Y position is above ground level
 * Prevents camera from going underground
 * Uses X-Plane terrain datarefs for accurate ground level
 * @param cameraY The camera's Y position in world coordinates
 * @return Adjusted Y position that is above ground
 */
static float EnsureAboveGround(float cameraY) {
    // Calculate terrain height at aircraft position
    // Method: Aircraft local_y minus y_agl (height above ground) gives terrain Y
    float terrainY = 0.0f;
    
    if (g_drTerrainY && g_drLocalY) {
        float aircraftY = XPLMGetDataf(g_drLocalY);
        float agl = XPLMGetDataf(g_drTerrainY);  // y_agl is height above ground
        terrainY = aircraftY - agl;  // Calculate actual terrain Y coordinate
    } else if (g_drLocalY) {
        // Fallback: estimate terrain from aircraft position and height
        float aircraftY = XPLMGetDataf(g_drLocalY);
        terrainY = aircraftY - g_aircraftDims.height * 2.0f;  // Conservative estimate
    }
    
    // Ensure camera is at least MIN_CAMERA_HEIGHT_ABOVE_GROUND meters above terrain
    float minCameraY = terrainY + MIN_CAMERA_HEIGHT_ABOVE_GROUND;
    
    return std::max(cameraY, minCameraY);
}

/**
 * Validate and adjust camera position in world coordinates to ensure aircraft is visible
 * This checks distance from aircraft and applies corrections if needed
 * @param worldCamX, worldCamY, worldCamZ - Camera position in world coordinates
 * @param acfX, acfY, acfZ - Aircraft position in world coordinates
 * @param type - The type of camera shot
 */
static void ValidateCameraPosition(float& worldCamX, float& worldCamY, float& worldCamZ,
                                    float acfX, float acfY, float acfZ, CameraType type) {
    if (type == CameraType::Cockpit) {
        // Cockpit views don't need distance validation
        return;
    }
    
    // Calculate current distance from aircraft center in world space
    float dx = worldCamX - acfX;
    float dy = worldCamY - acfY;
    float dz = worldCamZ - acfZ;
    float distance = std::sqrt(dx * dx + dy * dy + dz * dz);
    float minDistance = CalculateMinVisibleDistance();
    
    // If too close, scale position outward to minimum distance
    if (distance < minDistance && distance > 0.001f) {
        float scaleFactor = minDistance / distance;
        worldCamX = acfX + dx * scaleFactor;
        worldCamY = acfY + dy * scaleFactor;
        worldCamZ = acfZ + dz * scaleFactor;
    }
}

/**
 * Get the plugin directory path
 */
static std::string GetPluginPath() {
    char path[512];
    XPLMGetPluginInfo(XPLMGetMyID(), nullptr, path, nullptr, nullptr);
    std::string pathStr(path);
    
    // Remove plugin filename to get directory
    size_t lastSlash = pathStr.find_last_of("/\\");
    if (lastSlash != std::string::npos) {
        pathStr = pathStr.substr(0, lastSlash + 1);
    }
    return pathStr;
}

/**
 * Convert focal length in millimeters to field of view in degrees
 * Uses the standard 35mm full-frame sensor width (36mm)
 * Formula: FOV = 2 * atan(sensorWidth / (2 * focalLength)) * 180 / PI
 */
static float FocalLengthToFov(float focalLengthMm) {
    if (focalLengthMm <= 0.0f) focalLengthMm = 50.0f;  // Default to 50mm if invalid
    float fovRad = 2.0f * std::atan(SENSOR_WIDTH_MM / (2.0f * focalLengthMm));
    return fovRad * 180.0f / PI;
}

/**
 * Convert field of view in degrees to focal length in millimeters
 * Inverse of FocalLengthToFov
 * Formula: focalLength = sensorWidth / (2 * tan(FOV/2))
 */
static float FovToFocalLength(float fovDeg) {
    if (fovDeg <= 0.0f || fovDeg >= 180.0f) fovDeg = 60.0f;  // Default to 60° if invalid
    float fovRad = fovDeg * PI / 180.0f;
    return SENSOR_WIDTH_MM / (2.0f * std::tan(fovRad / 2.0f));
}

/**
 * Apply FOV effect with smooth transition
 * Gradually adjusts the field of view towards the target value
 */
static void ApplyFovEffect(float targetFov, float deltaTime) {
    if (!g_drFovHorizontal) return;
    
    // Smooth transition to target FOV
    float diff = targetFov - g_currentFov;
    float maxChange = g_fovTransitionSpeed * deltaTime;
    
    if (std::abs(diff) <= maxChange) {
        g_currentFov = targetFov;
    } else if (diff > 0) {
        g_currentFov += maxChange;
    } else {
        g_currentFov -= maxChange;
    }
    
    // Apply to X-Plane
    XPLMSetDataf(g_drFovHorizontal, g_currentFov);
    
    // Also set vertical FOV if available (maintain aspect ratio)
    if (g_drFovVertical) {
        // Assuming 16:9 aspect ratio
        float vFov = 2.0f * std::atan(std::tan(g_currentFov * PI / 360.0f) * 9.0f / 16.0f) * 180.0f / PI;
        XPLMSetDataf(g_drFovVertical, vFov);
    }
}

/**
 * Save the current X-Plane camera effect state before taking control
 * Stores original FOV and camera effect settings for restoration
 */
static void SaveCameraEffectState() {
    // Save original FOV
    if (g_drFovHorizontal) {
        g_originalFov = XPLMGetDataf(g_drFovHorizontal);
        g_currentFov = g_baseFov;  // Start at our base FOV
    } else {
        g_originalFov = DEFAULT_FOV_DEG;
        g_currentFov = g_baseFov;
    }
    
    // Save original handheld camera setting
    if (g_drHandheldCam) {
        g_originalHandheldCam = XPLMGetDataf(g_drHandheldCam);
    }
    
    // Save original G-loaded camera setting
    if (g_drGloadedCam) {
        g_originalGloadedCam = XPLMGetDataf(g_drGloadedCam);
    }
    
    XPLMDebugString("MovieCamera: Camera effect state saved\n");
}

/**
 * Restore the original X-Plane camera effect state
 * Called when releasing camera control
 */
static void RestoreCameraEffectState() {
    // Restore original FOV
    if (g_drFovHorizontal) {
        XPLMSetDataf(g_drFovHorizontal, g_originalFov);
    }
    
    // Also restore vertical FOV
    if (g_drFovVertical) {
        float vFov = 2.0f * std::atan(std::tan(g_originalFov * PI / 360.0f) * 9.0f / 16.0f) * 180.0f / PI;
        XPLMSetDataf(g_drFovVertical, vFov);
    }
    
    // Restore handheld camera setting
    if (g_drHandheldCam) {
        XPLMSetDataf(g_drHandheldCam, g_originalHandheldCam);
    }
    
    // Restore G-loaded camera setting
    if (g_drGloadedCam) {
        XPLMSetDataf(g_drGloadedCam, g_originalGloadedCam);
    }
    
    XPLMDebugString("MovieCamera: Camera effect state restored\n");
}

/**
 * Save plugin settings to a file
 */
static void SaveSettings() {
    std::string path = GetPluginPath() + "settings.cfg";
    FILE* file = fopen(path.c_str(), "w");
    if (!file) {
        XPLMDebugString("MovieCamera: Failed to save settings\n");
        return;
    }
    
    fprintf(file, "# MovieCamera Settings\n");
    fprintf(file, "version 2\n");
    fprintf(file, "delay_seconds %.1f\n", g_delaySeconds);
    fprintf(file, "auto_alt_ft %.0f\n", g_autoAltFt);
    fprintf(file, "shot_min_duration %.1f\n", g_shotMinDuration);
    fprintf(file, "shot_max_duration %.1f\n", g_shotMaxDuration);
    
    // Cinematic effects settings
    fprintf(file, "enable_fov_effect %d\n", g_enableFovEffect ? 1 : 0);
    fprintf(file, "base_fov %.1f\n", g_baseFov);
    fprintf(file, "fov_transition_speed %.1f\n", g_fovTransitionSpeed);
    fprintf(file, "enable_handheld_effect %d\n", g_enableHandheldEffect ? 1 : 0);
    fprintf(file, "handheld_intensity %.2f\n", g_handheldIntensity);
    fprintf(file, "enable_gforce_effect %d\n", g_enableGForceEffect ? 1 : 0);
    
    fclose(file);
    XPLMDebugString("MovieCamera: Settings saved\n");
}

/**
 * Load plugin settings from a file
 */
static void LoadSettings() {
    std::string path = GetPluginPath() + "settings.cfg";
    FILE* file = fopen(path.c_str(), "r");
    if (!file) {
        // No saved settings file - use defaults
        return;
    }
    
    char line[256];
    while (fgets(line, sizeof(line), file)) {
        if (line[0] == '#') continue;
        
        float value;
        int intValue;
        if (sscanf(line, "delay_seconds %f", &value) == 1) {
            g_delaySeconds = std::clamp(value, 1.0f, 300.0f);
        } else if (sscanf(line, "auto_alt_ft %f", &value) == 1) {
            g_autoAltFt = std::clamp(value, 0.0f, 50000.0f);
        } else if (sscanf(line, "shot_min_duration %f", &value) == 1) {
            g_shotMinDuration = std::clamp(value, 1.0f, 30.0f);
        } else if (sscanf(line, "shot_max_duration %f", &value) == 1) {
            g_shotMaxDuration = std::clamp(value, 1.0f, 30.0f);
        } else if (sscanf(line, "enable_fov_effect %d", &intValue) == 1) {
            g_enableFovEffect = (intValue != 0);
        } else if (sscanf(line, "base_fov %f", &value) == 1) {
            g_baseFov = std::clamp(value, MIN_FOV_DEG, MAX_FOV_DEG);
        } else if (sscanf(line, "fov_transition_speed %f", &value) == 1) {
            g_fovTransitionSpeed = std::clamp(value, 1.0f, 30.0f);
        } else if (sscanf(line, "enable_handheld_effect %d", &intValue) == 1) {
            g_enableHandheldEffect = (intValue != 0);
        } else if (sscanf(line, "handheld_intensity %f", &value) == 1) {
            g_handheldIntensity = std::clamp(value, 0.0f, 1.0f);
        } else if (sscanf(line, "enable_gforce_effect %d", &intValue) == 1) {
            g_enableGForceEffect = (intValue != 0);
        }
    }
    
    // Ensure min <= max
    if (g_shotMinDuration > g_shotMaxDuration) {
        g_shotMinDuration = g_shotMaxDuration;
    }
    
    fclose(file);
    XPLMDebugString("MovieCamera: Settings loaded\n");
}

/**
 * Select the next camera shot
 */
static CameraShot SelectNextShot() {
    std::vector<CameraShot>* shotList = nullptr;
    bool canSwitchType = g_consecutiveSameTypeCount >= 3;
    
    // Determine which shot type to use
    CameraType nextType;
    if (canSwitchType) {
        // Can switch types - randomly decide
        nextType = (std::rand() % 2 == 0) ? CameraType::Cockpit : CameraType::External;
    } else {
        // Must continue with same type
        nextType = g_lastShotType;
    }
    
    // Select the appropriate shot list
    if (nextType == CameraType::Cockpit) {
        shotList = &g_cockpitShots;
    } else {
        shotList = &g_externalShots;
    }
    
    // Update tracking
    if (nextType != g_lastShotType) {
        g_consecutiveSameTypeCount = 1;
        g_lastShotType = nextType;
    } else {
        g_consecutiveSameTypeCount++;
    }
    
    // Select a random shot from the list (avoid same shot twice)
    if (shotList->empty()) {
        // Fallback - include drift values
        return {CameraType::Cockpit, 0, 0, 0, 0, 0, 0, 1.0f, 4.0f, "Default",
                0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    }
    
    int newIndex;
    do {
        newIndex = std::rand() % static_cast<int>(shotList->size());
    } while (newIndex == g_currentShotIndex && shotList->size() > 1);
    
    g_currentShotIndex = newIndex;
    
    // Get the shot and randomize duration
    CameraShot shot = (*shotList)[newIndex];
    shot.duration = g_shotMinDuration + (static_cast<float>(std::rand()) / RAND_MAX) * (g_shotMaxDuration - g_shotMinDuration);
    
    // Store current shot for drift calculation
    g_currentShot = shot;
    g_shotElapsedTime = 0.0f;
    
    return shot;
}

/**
 * Check if auto-activation conditions are met
 */
static bool CheckAutoConditions() {
    if (!g_drOnGround || !g_drGroundSpeed || !g_drElevationM) {
        return false;
    }
    
    int onGround = XPLMGetDatai(g_drOnGround);
    float groundSpeed = XPLMGetDataf(g_drGroundSpeed);  // m/s
    float elevationMeters = XPLMGetDataf(g_drElevationM);
    
    // Convert elevation from meters to feet for comparison with g_autoAltFt
    float altitudeFt = elevationMeters * 3.28084f;
    
    // Condition 1: On ground and stationary (speed < 1 m/s)
    if (onGround && groundSpeed < 1.0f) {
        return true;
    }
    
    // Condition 2: In the air above Auto Alt and mouse idle for Delay time
    if (!onGround && altitudeFt > g_autoAltFt && g_mouseIdleTime >= g_delaySeconds) {
        return true;
    }
    
    return false;
}

/**
 * Start camera control
 */
static void StartCameraControl() {
    if (g_functionActive) return;
    
    g_functionActive = true;
    g_functionPaused = false;
    g_currentShotTime = 0.0f;
    g_shotElapsedTime = 0.0f;
    g_currentShotIndex = -1;
    g_consecutiveSameTypeCount = 0;
    g_inTransition = false;
    
    // Start with a random shot type
    g_lastShotType = (std::rand() % 2 == 0) ? CameraType::Cockpit : CameraType::External;
    
    // Select the first shot (this also sets g_currentShot)
    CameraShot firstShot = SelectNextShot();
    
    // Read current camera position for smooth start
    XPLMReadCameraPosition(&g_startPos);
    
    // Calculate target position (starting position of the shot)
    float acfX = XPLMGetDataf(g_drLocalX);
    float acfY = XPLMGetDataf(g_drLocalY);
    float acfZ = XPLMGetDataf(g_drLocalZ);
    float acfHeading = XPLMGetDataf(g_drHeading);
    
    float rad = acfHeading * PI / 180.0f;
    float cosH = std::cos(rad);
    float sinH = std::sin(rad);
    
    // For cockpit shots, add pilot eye position as base offset
    float shotX = firstShot.x;
    float shotY = firstShot.y;
    float shotZ = firstShot.z;
    if (firstShot.type == CameraType::Cockpit) {
        shotX += g_aircraftDims.pilotEyeX;
        shotY += g_aircraftDims.pilotEyeY;
        shotZ += g_aircraftDims.pilotEyeZ;
    }
    
    g_targetPos.x = acfX + shotX * cosH - shotZ * sinH;
    g_targetPos.y = acfY + shotY;
    g_targetPos.z = acfZ + shotX * sinH + shotZ * cosH;
    g_targetPos.pitch = firstShot.pitch;
    g_targetPos.heading = acfHeading + firstShot.heading;
    g_targetPos.roll = firstShot.roll;
    g_targetPos.zoom = firstShot.zoom;
    
    // Instant camera switch - no smooth transition
    g_inTransition = false;
    g_transitionProgress = 0.0f;
    g_currentShotTime = firstShot.duration;
    
    // Save current camera effect state before taking control
    SaveCameraEffectState();
    
    // Apply initial handheld effect setting if enabled
    if (g_enableHandheldEffect && g_drHandheldCam) {
        XPLMSetDataf(g_drHandheldCam, g_handheldIntensity);
    }
    
    // Apply initial G-force effect setting
    if (g_enableGForceEffect && g_drGloadedCam) {
        XPLMSetDataf(g_drGloadedCam, 1.0f);
    } else if (g_drGloadedCam) {
        XPLMSetDataf(g_drGloadedCam, 0.0f);
    }
    
    // Take camera control
    XPLMControlCamera(xplm_ControlCameraForever, CameraControlCallback, nullptr);
    
    XPLMDebugString("MovieCamera: Camera control started\n");
}

/**
 * Stop camera control
 */
static void StopCameraControl() {
    if (!g_functionActive) return;
    
    g_functionActive = false;
    g_functionPaused = false;
    
    // Release camera control
    XPLMDontControlCamera();
    
    // Restore original camera effect state
    RestoreCameraEffectState();
    
    XPLMDebugString("MovieCamera: Camera control stopped\n");
}

/**
 * Pause camera control (on mouse movement)
 */
static void PauseCameraControl() {
    if (!g_functionActive || g_functionPaused) return;
    
    g_functionPaused = true;
    
    // Release camera to return to default view
    XPLMDontControlCamera();
    
    XPLMDebugString("MovieCamera: Camera control paused\n");
}

/**
 * Resume camera control after pause
 */
static void ResumeCameraControl() {
    if (!g_functionActive || !g_functionPaused) return;
    
    g_functionPaused = false;
    
    // Retake camera control
    XPLMControlCamera(xplm_ControlCameraForever, CameraControlCallback, nullptr);
    
    XPLMDebugString("MovieCamera: Camera control resumed\n");
}

/**
 * Camera control callback
 * Applies smooth drift motion during shots for cinematic feel
 */
static int CameraControlCallback(XPLMCameraPosition_t* outCameraPosition, int inIsLosingControl, void* inRefcon) {
    (void)inRefcon;
    
    if (inIsLosingControl || !outCameraPosition || !g_functionActive || g_functionPaused) {
        return 0;
    }
    
    // Get current aircraft position and orientation
    float acfX = XPLMGetDataf(g_drLocalX);
    float acfY = XPLMGetDataf(g_drLocalY);
    float acfZ = XPLMGetDataf(g_drLocalZ);
    float acfHeading = XPLMGetDataf(g_drHeading);
    float acfPitch = XPLMGetDataf(g_drPitch);
    float acfRoll = XPLMGetDataf(g_drRoll);
    
    if (g_inTransition) {
        // Smooth transition between shots using ease-in-out
        float t = EaseInOutCubic(g_transitionProgress);
        
        float transX = Lerp(g_startPos.x, g_targetPos.x, t);
        float transY = Lerp(g_startPos.y, g_targetPos.y, t);
        float transZ = Lerp(g_startPos.z, g_targetPos.z, t);
        
        // Ensure camera doesn't go underground during transition (for external shots)
        if (g_currentShot.type == CameraType::External) {
            transY = EnsureAboveGround(transY);
        }
        
        outCameraPosition->x = transX;
        outCameraPosition->y = transY;
        outCameraPosition->z = transZ;
        outCameraPosition->pitch = Lerp(g_startPos.pitch, g_targetPos.pitch, t);
        outCameraPosition->heading = LerpAngle(g_startPos.heading, g_targetPos.heading, t);
        outCameraPosition->roll = Lerp(g_startPos.roll, g_targetPos.roll, t);
        outCameraPosition->zoom = Lerp(g_startPos.zoom, g_targetPos.zoom, t);
    } else {
        // Apply shot with consistent linear drift - like Horizon game
        // Once drift direction is set at shot start, maintain it throughout
        
        // Calculate normalized time (0 at start, 1 at end of shot)
        float normalizedTime = g_shotElapsedTime / g_currentShot.duration;
        normalizedTime = std::clamp(normalizedTime, 0.0f, 1.0f);
        
        // Position drift with smooth ease-in only (no slowdown at end)
        float driftedX = LinearDrift(g_currentShot.x, g_currentShot.driftX * g_currentShot.duration, normalizedTime);
        float driftedY = LinearDrift(g_currentShot.y, g_currentShot.driftY * g_currentShot.duration, normalizedTime);
        float driftedZ = LinearDrift(g_currentShot.z, g_currentShot.driftZ * g_currentShot.duration, normalizedTime);
        
        // For cockpit shots, add pilot eye position as base offset
        // This ensures cockpit views are in the cockpit, not at the aircraft origin (CG)
        if (g_currentShot.type == CameraType::Cockpit) {
            driftedX += g_aircraftDims.pilotEyeX;
            driftedY += g_aircraftDims.pilotEyeY;
            driftedZ += g_aircraftDims.pilotEyeZ;
        }
        
        // Rotation drift with same consistent direction
        float driftedPitch = LinearDrift(g_currentShot.pitch, g_currentShot.driftPitch * g_currentShot.duration, normalizedTime);
        float driftedHeading = LinearDrift(g_currentShot.heading, g_currentShot.driftHeading * g_currentShot.duration, normalizedTime);
        float driftedRoll = LinearDrift(g_currentShot.roll, g_currentShot.driftRoll * g_currentShot.duration, normalizedTime);
        
        // Zoom drift with same consistent direction
        float driftedZoom = LinearDrift(g_currentShot.zoom, g_currentShot.driftZoom * g_currentShot.duration, normalizedTime);
        
        float worldCamX, worldCamY, worldCamZ;
        
        if (g_currentShot.type == CameraType::External) {
            // For external shots, use full 3D rotation to keep camera position
            // relative to aircraft attitude (pitch, roll, heading)
            TransformToWorldCoordinates(
                driftedX, driftedY, driftedZ,
                acfX, acfY, acfZ,
                acfHeading, acfPitch, acfRoll,
                worldCamX, worldCamY, worldCamZ);
            
            // Ensure camera doesn't go underground
            worldCamY = EnsureAboveGround(worldCamY);
            
            // Validate camera position to ensure aircraft is visible (in world-space)
            ValidateCameraPosition(worldCamX, worldCamY, worldCamZ, acfX, acfY, acfZ, g_currentShot.type);
        } else {
            // For cockpit shots, only use heading rotation (cockpit moves with aircraft)
            float rad = acfHeading * PI / 180.0f;
            float cosH = std::cos(rad);
            float sinH = std::sin(rad);
            worldCamX = acfX + driftedX * cosH - driftedZ * sinH;
            worldCamY = acfY + driftedY;
            worldCamZ = acfZ + driftedX * sinH + driftedZ * cosH;
        }
        
        outCameraPosition->x = worldCamX;
        outCameraPosition->y = worldCamY;
        outCameraPosition->z = worldCamZ;
        outCameraPosition->pitch = driftedPitch;
        outCameraPosition->heading = acfHeading + driftedHeading;
        outCameraPosition->roll = driftedRoll;
        outCameraPosition->zoom = driftedZoom;
    }
    
    // Apply FOV effect if enabled
    if (g_enableFovEffect && g_drFovHorizontal) {
        // Use a small delta time estimate for smooth transitions
        // (actual deltaTime would require tracking between frames)
        ApplyFovEffect(g_baseFov, 0.016f);  // ~60fps assumed
    }
    
    return 1;
}

/**
 * Flight loop callback for timing and state management
 */
static float FlightLoopCallback(float inElapsedSinceLastCall, float inElapsedTimeSinceLastFlightLoop, int inCounter, void* inRefcon) {
    (void)inElapsedTimeSinceLastFlightLoop;
    (void)inCounter;
    (void)inRefcon;
    
    // Check for mouse movement
    int mouseX, mouseY;
    XPLMGetMouseLocation(&mouseX, &mouseY);
    
    bool mouseMoved = (mouseX != g_lastMouseX || mouseY != g_lastMouseY);
    g_lastMouseX = mouseX;
    g_lastMouseY = mouseY;
    
    if (mouseMoved) {
        // Reset idle timer on mouse movement
        g_mouseIdleTime = 0.0f;
        
        // If function is active, pause it
        if (g_functionActive && !g_functionPaused) {
            PauseCameraControl();
        }
    } else {
        // Accumulate idle time
        g_mouseIdleTime += inElapsedSinceLastCall;
    }
    
    // Handle auto mode
    if (g_pluginMode == PluginMode::Auto) {
        bool conditionsMet = CheckAutoConditions();
        
        if (conditionsMet && !g_functionActive) {
            StartCameraControl();
        } else if (!conditionsMet && g_functionActive) {
            StopCameraControl();
        } else if (g_functionActive && g_functionPaused && g_mouseIdleTime >= g_delaySeconds) {
            // Resume after mouse idle
            ResumeCameraControl();
        }
    } else if (g_pluginMode == PluginMode::Manual && g_functionActive && g_functionPaused) {
        // In manual mode, resume after delay
        if (g_mouseIdleTime >= g_delaySeconds) {
            ResumeCameraControl();
        }
    }
    
    // Update camera shot timing
    if (g_functionActive && !g_functionPaused) {
        if (g_inTransition) {
            g_transitionProgress += inElapsedSinceLastCall / g_transitionDuration;
            if (g_transitionProgress >= 1.0f) {
                g_inTransition = false;
                g_transitionProgress = 0.0f;
                // Reset elapsed time when transition ends and drift begins
                g_shotElapsedTime = 0.0f;
            }
        } else {
            // Accumulate elapsed time for drift calculation
            g_shotElapsedTime += inElapsedSinceLastCall;
            
            g_currentShotTime -= inElapsedSinceLastCall;
            if (g_currentShotTime <= 0.0f) {
                // Time for next shot
                XPLMReadCameraPosition(&g_startPos);
                
                CameraShot nextShot = SelectNextShot();
                
                // Calculate target position (start position for the new shot)
                float acfX = XPLMGetDataf(g_drLocalX);
                float acfY = XPLMGetDataf(g_drLocalY);
                float acfZ = XPLMGetDataf(g_drLocalZ);
                float acfHeading = XPLMGetDataf(g_drHeading);
                
                float rad = acfHeading * PI / 180.0f;
                float cosH = std::cos(rad);
                float sinH = std::sin(rad);
                
                // For cockpit shots, add pilot eye position as base offset
                float shotX = nextShot.x;
                float shotY = nextShot.y;
                float shotZ = nextShot.z;
                if (nextShot.type == CameraType::Cockpit) {
                    shotX += g_aircraftDims.pilotEyeX;
                    shotY += g_aircraftDims.pilotEyeY;
                    shotZ += g_aircraftDims.pilotEyeZ;
                }
                
                g_targetPos.x = acfX + shotX * cosH - shotZ * sinH;
                g_targetPos.y = acfY + shotY;
                g_targetPos.z = acfZ + shotX * sinH + shotZ * cosH;
                g_targetPos.pitch = nextShot.pitch;
                g_targetPos.heading = acfHeading + nextShot.heading;
                g_targetPos.roll = nextShot.roll;
                g_targetPos.zoom = nextShot.zoom;
                
                // Instant camera switch - no smooth transition
                g_inTransition = false;
                g_transitionProgress = 0.0f;
                g_currentShotTime = nextShot.duration;
            }
        }
    }
    
    return -1.0f;  // Call every frame
}

/**
 * Plugin start
 */
PLUGIN_API int XPluginStart(char* outName, char* outSig, char* outDesc) {
    std::strcpy(outName, PLUGIN_NAME);
    std::strcpy(outSig, PLUGIN_SIG);
    std::strcpy(outDesc, PLUGIN_DESCRIPTION);
    
    XPLMDebugString("MovieCamera: Plugin starting...\n");
    
    // Find datarefs
    g_drLatitude = XPLMFindDataRef("sim/flightmodel/position/latitude");
    g_drLongitude = XPLMFindDataRef("sim/flightmodel/position/longitude");
    g_drElevation = XPLMFindDataRef("sim/flightmodel/position/elevation");
    g_drLocalX = XPLMFindDataRef("sim/flightmodel/position/local_x");
    g_drLocalY = XPLMFindDataRef("sim/flightmodel/position/local_y");
    g_drLocalZ = XPLMFindDataRef("sim/flightmodel/position/local_z");
    g_drPitch = XPLMFindDataRef("sim/flightmodel/position/theta");
    g_drRoll = XPLMFindDataRef("sim/flightmodel/position/phi");
    g_drHeading = XPLMFindDataRef("sim/flightmodel/position/psi");
    g_drGroundSpeed = XPLMFindDataRef("sim/flightmodel/position/groundspeed");
    g_drOnGround = XPLMFindDataRef("sim/flightmodel/failures/onground_any");
    g_drElevationM = XPLMFindDataRef("sim/flightmodel/position/elevation");  // Returns meters
    g_drPilotX = XPLMFindDataRef("sim/graphics/view/pilots_head_x");
    g_drPilotY = XPLMFindDataRef("sim/graphics/view/pilots_head_y");
    g_drPilotZ = XPLMFindDataRef("sim/graphics/view/pilots_head_z");
    g_drViewType = XPLMFindDataRef("sim/graphics/view/view_type");
    
    // Camera effect datarefs (X-Plane 12+)
    g_drFovHorizontal = XPLMFindDataRef("sim/graphics/view/field_of_view_deg");
    g_drFovVertical = XPLMFindDataRef("sim/graphics/view/vertical_field_of_view_deg");
    g_drHandheldCam = XPLMFindDataRef("sim/graphics/view/handheld_external_cam");
    g_drGloadedCam = XPLMFindDataRef("sim/graphics/view/gloaded_internal_cam");
    g_drViewIsExternal = XPLMFindDataRef("sim/graphics/view/view_is_external");
    
    if (!g_drFovHorizontal) {
        XPLMDebugString("MovieCamera: FOV dataref not found - FOV effects disabled\n");
    } else {
        XPLMDebugString("MovieCamera: FOV control enabled\n");
    }
    
    if (!g_drHandheldCam) {
        XPLMDebugString("MovieCamera: Handheld camera dataref not found - handheld effect disabled\n");
    }
    
    // Terrain height dataref for ground collision prevention
    g_drTerrainY = XPLMFindDataRef("sim/flightmodel/position/y_agl");
    // Fallback: if y_agl not available, calculate from local_y - elevation
    if (!g_drTerrainY) {
        // We'll use local_y minus elevation as a fallback in EnsureAboveGround()
        XPLMDebugString("MovieCamera: y_agl dataref not found, using fallback ground estimation\n");
    }
    
    // Find aircraft dimension datarefs (loaded from .acf file by X-Plane)
    // Primary sources for aircraft size - these are the shadow/viewing distance sizes
    g_drAcfSizeX = XPLMFindDataRef("sim/aircraft/view/acf_size_x");   // Shadow size X (width/wingspan)
    g_drAcfSizeZ = XPLMFindDataRef("sim/aircraft/view/acf_size_z");   // Shadow size Z (length)
    
    // Wing segment semi-lengths for precise wingspan calculation
    g_drAcfSemilenSEG = XPLMFindDataRef("sim/aircraft/parts/acf_semilen_SEG");  // Per-segment semi-length
    g_drAcfSemilenJND = XPLMFindDataRef("sim/aircraft/parts/acf_semilen_JND");  // Joined segment semi-length
    
    // CG limits provide good approximation of aircraft length (fallback)
    g_drAcfCgZFwd = XPLMFindDataRef("sim/aircraft/overflow/acf_cgZ_fwd");  // Forward CG limit
    g_drAcfCgZAft = XPLMFindDataRef("sim/aircraft/overflow/acf_cgZ_aft");  // Aft CG limit
    
    // Height estimation sources
    g_drAcfMinY = XPLMFindDataRef("sim/aircraft/parts/acf_gear_ynodef");  // Gear Y position for ground clearance
    g_drAcfMaxY = nullptr;
    
    // Pilot eye position - reliable datarefs for cockpit view positioning
    g_drAcfPeX = XPLMFindDataRef("sim/aircraft/view/acf_peX");   // Pilot eye X (lateral offset)
    g_drAcfPeY = XPLMFindDataRef("sim/aircraft/view/acf_peY");   // Pilot eye Y (height from CG)
    g_drAcfPeZ = XPLMFindDataRef("sim/aircraft/view/acf_peZ");   // Pilot eye Z (longitudinal from CG)
    
    // Log which datarefs were found
    char msg[512];
    snprintf(msg, sizeof(msg), "MovieCamera: Datarefs found - acf_size_x: %s, acf_size_z: %s, semilen_JND: %s\n",
             g_drAcfSizeX ? "yes" : "no",
             g_drAcfSizeZ ? "yes" : "no", 
             g_drAcfSemilenJND ? "yes" : "no");
    XPLMDebugString(msg);
    
    // Initialize with dynamic camera shots based on default aircraft dimensions
    // Will be regenerated when aircraft data is loaded
    GenerateDynamicCameraShots();
    
    // Create menu
    int pluginMenuIndex = XPLMAppendMenuItem(XPLMFindPluginsMenu(), PLUGIN_NAME, nullptr, 0);
    g_menuId = XPLMCreateMenu(PLUGIN_NAME, XPLMFindPluginsMenu(), pluginMenuIndex, MenuHandler, nullptr);
    
    g_menuItemAuto = XPLMAppendMenuItem(g_menuId, "Auto", reinterpret_cast<void*>(0), 0);
    g_menuItemStart = XPLMAppendMenuItem(g_menuId, "Start", reinterpret_cast<void*>(1), 0);
    g_menuItemStop = XPLMAppendMenuItem(g_menuId, "Stop", reinterpret_cast<void*>(2), 0);
    XPLMAppendMenuSeparator(g_menuId);
    g_menuItemSettings = XPLMAppendMenuItem(g_menuId, "Settings", reinterpret_cast<void*>(3), 0);
    
    UpdateMenuState();
    
    XPLMDebugString("MovieCamera: Plugin started successfully\n");
    
    return 1;
}

/**
 * Plugin stop
 */
PLUGIN_API void XPluginStop(void) {
    XPLMDebugString("MovieCamera: Plugin stopping...\n");
    
    // Clean up menu
    if (g_menuId) {
        XPLMDestroyMenu(g_menuId);
        g_menuId = nullptr;
    }
    
    XPLMDebugString("MovieCamera: Plugin stopped\n");
}

/**
 * Plugin enable
 */
PLUGIN_API int XPluginEnable(void) {
    XPLMDebugString("MovieCamera: Plugin enabling...\n");
    
    // Create flight loop
    XPLMCreateFlightLoop_t flightLoopParams;
    flightLoopParams.structSize = sizeof(XPLMCreateFlightLoop_t);
    flightLoopParams.phase = xplm_FlightLoop_Phase_AfterFlightModel;
    flightLoopParams.callbackFunc = FlightLoopCallback;
    flightLoopParams.refcon = nullptr;
    
    g_flightLoopId = XPLMCreateFlightLoop(&flightLoopParams);
    XPLMScheduleFlightLoop(g_flightLoopId, -1.0f, 1);
    
    // Create settings window
    g_settingsWindow = std::make_unique<SettingsWindow>();
    g_settingsWindow->SetVisible(false);
    
    // Get initial mouse position
    XPLMGetMouseLocation(&g_lastMouseX, &g_lastMouseY);
    
    // Initialize random seed once at plugin enable
    std::srand(static_cast<unsigned>(std::time(nullptr)));
    
    // Load user settings
    LoadSettings();
    
    // Read aircraft dimensions and generate dynamic camera shots
    ReadAircraftDimensions();
    GenerateDynamicCameraShots();
    
    XPLMDebugString("MovieCamera: Plugin enabled\n");
    
    return 1;
}

/**
 * Plugin disable
 */
PLUGIN_API void XPluginDisable(void) {
    XPLMDebugString("MovieCamera: Plugin disabling...\n");
    
    // Save user settings
    SaveSettings();
    
    // Stop camera control if active
    if (g_functionActive) {
        StopCameraControl();
    }
    
    // Destroy flight loop
    if (g_flightLoopId) {
        XPLMDestroyFlightLoop(g_flightLoopId);
        g_flightLoopId = nullptr;
    }
    
    // Destroy settings window
    g_settingsWindow.reset();
    
    XPLMDebugString("MovieCamera: Plugin disabled\n");
}

/**
 * Message receive
 */
PLUGIN_API void XPluginReceiveMessage(XPLMPluginID inFrom, int inMsg, void* inParam) {
    (void)inFrom;
    
    // Handle plane loaded message to reset state and recalculate camera shots
    // inParam == 0 means user's aircraft, other values are AI aircraft indices
    if (inMsg == XPLM_MSG_PLANE_LOADED && reinterpret_cast<intptr_t>(inParam) == 0) {
        // User's plane loaded - read new aircraft dimensions and regenerate camera shots
        g_mouseIdleTime = 0.0f;
        
        XPLMDebugString("MovieCamera: User aircraft loaded, reading dimensions...\n");
        ReadAircraftDimensions();
        GenerateDynamicCameraShots();
    }
}
