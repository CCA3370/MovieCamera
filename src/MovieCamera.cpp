/**
 * MovieCamera Plugin for X-Plane 12
 * 
 * A cinematic camera plugin that provides automatic camera movements
 * with smooth transitions between cockpit and external views.
 * 
 * Uses ImGui for the settings window (based on ImgWindow integration).
 */

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
};

// Plugin Global State
static PluginMode g_pluginMode = PluginMode::Off;
static bool g_functionActive = false;    // Is the camera control actually running
static bool g_functionPaused = false;    // Temporarily paused due to mouse movement

// Settings
static float g_delaySeconds = 60.0f;      // Delay before auto-activation (seconds)
static float g_autoAltFt = 18000.0f;      // Altitude threshold for auto mode (feet)
static float g_shotMinDuration = 3.0f;
static float g_shotMaxDuration = 5.0f;

// Mouse tracking
static int g_lastMouseX = 0;
static int g_lastMouseY = 0;
static float g_mouseIdleTime = 0.0f;      // Time since last mouse movement

// Camera control state
static float g_currentShotTime = 0.0f;
static int g_currentShotIndex = -1;
static int g_consecutiveSameTypeCount = 0;
static CameraType g_lastShotType = CameraType::Cockpit;

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
static void InitializeCameraShots();
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

/**
 * SettingsWindow constructor
 */
SettingsWindow::SettingsWindow() :
    ImgWindow(100, 500, 450, 200, xplm_WindowDecorationRoundRectangle, xplm_WindowLayerFloatingWindows)
{
    SetWindowTitle("MovieCamera Settings");
    SetWindowResizingLimits(350, 300, 500, 400);
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
    
    if (ImGui::Button("Close", ImVec2(80, 0))) {
        SetVisible(false);
    }
}

/**
 * Initialize predefined camera shots
 */
static void InitializeCameraShots() {
    // Cockpit shots - various instrument views and pilot perspective shots
    g_cockpitShots.clear();
    
    // Center panel view
    g_cockpitShots.push_back({CameraType::Cockpit, 0.0f, 0.2f, 0.3f, -5.0f, 0.0f, 0.0f, 1.2f, 4.0f, "Center Panel"});
    
    // Left panel (throttle/navigation)
    g_cockpitShots.push_back({CameraType::Cockpit, -0.3f, 0.1f, 0.2f, -10.0f, -30.0f, 0.0f, 1.5f, 4.0f, "Left Panel"});
    
    // Right panel (radios/autopilot)
    g_cockpitShots.push_back({CameraType::Cockpit, 0.3f, 0.1f, 0.2f, -10.0f, 30.0f, 0.0f, 1.5f, 4.0f, "Right Panel"});
    
    // Overhead panel view
    g_cockpitShots.push_back({CameraType::Cockpit, 0.0f, 0.4f, 0.1f, -60.0f, 0.0f, 0.0f, 1.3f, 3.5f, "Overhead Panel"});
    
    // PFD closeup
    g_cockpitShots.push_back({CameraType::Cockpit, -0.15f, 0.0f, 0.5f, 0.0f, -10.0f, 0.0f, 2.0f, 4.5f, "PFD View"});
    
    // ND/MFD view
    g_cockpitShots.push_back({CameraType::Cockpit, 0.15f, 0.0f, 0.5f, 0.0f, 10.0f, 0.0f, 2.0f, 4.5f, "ND/MFD View"});
    
    // Looking out front window
    g_cockpitShots.push_back({CameraType::Cockpit, 0.0f, 0.3f, -0.2f, 5.0f, 0.0f, 0.0f, 1.0f, 5.0f, "Front Window"});
    
    // Side window left
    g_cockpitShots.push_back({CameraType::Cockpit, -0.4f, 0.2f, -0.1f, 0.0f, -90.0f, 0.0f, 1.0f, 4.0f, "Left Window"});
    
    // Side window right  
    g_cockpitShots.push_back({CameraType::Cockpit, 0.4f, 0.2f, -0.1f, 0.0f, 90.0f, 0.0f, 1.0f, 4.0f, "Right Window"});
    
    // External shots - various external angles
    g_externalShots.clear();
    
    // Front view
    g_externalShots.push_back({CameraType::External, 0.0f, 2.0f, -30.0f, 5.0f, 180.0f, 0.0f, 1.0f, 4.0f, "Front View"});
    
    // Rear view
    g_externalShots.push_back({CameraType::External, 0.0f, 3.0f, 40.0f, 10.0f, 0.0f, 0.0f, 1.0f, 4.5f, "Rear View"});
    
    // Left side
    g_externalShots.push_back({CameraType::External, -25.0f, 0.0f, 0.0f, 0.0f, 90.0f, 0.0f, 1.0f, 4.0f, "Left Side"});
    
    // Right side
    g_externalShots.push_back({CameraType::External, 25.0f, 0.0f, 0.0f, 0.0f, -90.0f, 0.0f, 1.0f, 4.0f, "Right Side"});
    
    // Top view
    g_externalShots.push_back({CameraType::External, 0.0f, 30.0f, 5.0f, 75.0f, 0.0f, 0.0f, 1.0f, 3.5f, "Top View"});
    
    // Bottom front view
    g_externalShots.push_back({CameraType::External, 0.0f, -10.0f, -20.0f, -30.0f, 180.0f, 0.0f, 1.0f, 4.0f, "Bottom Front"});
    
    // Quarter front left
    g_externalShots.push_back({CameraType::External, -20.0f, 5.0f, -20.0f, 10.0f, 135.0f, 0.0f, 1.0f, 4.5f, "Quarter Front Left"});
    
    // Quarter front right
    g_externalShots.push_back({CameraType::External, 20.0f, 5.0f, -20.0f, 10.0f, -135.0f, 0.0f, 1.0f, 4.5f, "Quarter Front Right"});
    
    // Quarter rear left
    g_externalShots.push_back({CameraType::External, -20.0f, 8.0f, 30.0f, 15.0f, 45.0f, 0.0f, 1.0f, 4.0f, "Quarter Rear Left"});
    
    // Quarter rear right
    g_externalShots.push_back({CameraType::External, 20.0f, 8.0f, 30.0f, 15.0f, -45.0f, 0.0f, 1.0f, 4.0f, "Quarter Rear Right"});
    
    // Wing view left
    g_externalShots.push_back({CameraType::External, -15.0f, 2.0f, 5.0f, 5.0f, 70.0f, 0.0f, 1.2f, 4.0f, "Wing Left"});
    
    // Wing view right
    g_externalShots.push_back({CameraType::External, 15.0f, 2.0f, 5.0f, 5.0f, -70.0f, 0.0f, 1.2f, 4.0f, "Wing Right"});
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
        // Fallback
        return {CameraType::Cockpit, 0, 0, 0, 0, 0, 0, 1.0f, 4.0f, "Default"};
    }
    
    int newIndex;
    do {
        newIndex = std::rand() % static_cast<int>(shotList->size());
    } while (newIndex == g_currentShotIndex && shotList->size() > 1);
    
    g_currentShotIndex = newIndex;
    
    // Get the shot and randomize duration
    CameraShot shot = (*shotList)[newIndex];
    shot.duration = g_shotMinDuration + (static_cast<float>(std::rand()) / RAND_MAX) * (g_shotMaxDuration - g_shotMinDuration);
    
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
    g_currentShotIndex = -1;
    g_consecutiveSameTypeCount = 0;
    g_inTransition = false;
    
    // Initialize random seed
    std::srand(static_cast<unsigned>(std::time(nullptr)));
    
    // Start with a random shot type
    g_lastShotType = (std::rand() % 2 == 0) ? CameraType::Cockpit : CameraType::External;
    
    // Select the first shot
    CameraShot firstShot = SelectNextShot();
    
    // Read current camera position for smooth start
    XPLMReadCameraPosition(&g_startPos);
    
    // Calculate target position
    float acfX = XPLMGetDataf(g_drLocalX);
    float acfY = XPLMGetDataf(g_drLocalY);
    float acfZ = XPLMGetDataf(g_drLocalZ);
    float acfHeading = XPLMGetDataf(g_drHeading);
    
    float rad = acfHeading * 3.14159f / 180.0f;
    float cosH = std::cos(rad);
    float sinH = std::sin(rad);
    
    g_targetPos.x = acfX + firstShot.x * cosH - firstShot.z * sinH;
    g_targetPos.y = acfY + firstShot.y;
    g_targetPos.z = acfZ + firstShot.x * sinH + firstShot.z * cosH;
    g_targetPos.pitch = firstShot.pitch;
    g_targetPos.heading = acfHeading + firstShot.heading;
    g_targetPos.roll = firstShot.roll;
    g_targetPos.zoom = firstShot.zoom;
    
    g_inTransition = true;
    g_transitionProgress = 0.0f;
    g_currentShotTime = firstShot.duration;
    
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
 */
static int CameraControlCallback(XPLMCameraPosition_t* outCameraPosition, int inIsLosingControl, void* inRefcon) {
    (void)inRefcon;
    
    if (inIsLosingControl || !outCameraPosition || !g_functionActive || g_functionPaused) {
        return 0;
    }
    
    // Get current aircraft position
    float acfX = XPLMGetDataf(g_drLocalX);
    float acfY = XPLMGetDataf(g_drLocalY);
    float acfZ = XPLMGetDataf(g_drLocalZ);
    float acfHeading = XPLMGetDataf(g_drHeading);
    
    if (g_inTransition) {
        // Smooth transition between shots
        float t = EaseInOutCubic(g_transitionProgress);
        
        outCameraPosition->x = Lerp(g_startPos.x, g_targetPos.x, t);
        outCameraPosition->y = Lerp(g_startPos.y, g_targetPos.y, t);
        outCameraPosition->z = Lerp(g_startPos.z, g_targetPos.z, t);
        outCameraPosition->pitch = Lerp(g_startPos.pitch, g_targetPos.pitch, t);
        outCameraPosition->heading = LerpAngle(g_startPos.heading, g_targetPos.heading, t);
        outCameraPosition->roll = Lerp(g_startPos.roll, g_targetPos.roll, t);
        outCameraPosition->zoom = Lerp(g_startPos.zoom, g_targetPos.zoom, t);
    } else {
        // Use target position, following aircraft
        std::vector<CameraShot>* shotList = (g_lastShotType == CameraType::Cockpit) ? &g_cockpitShots : &g_externalShots;
        if (g_currentShotIndex >= 0 && g_currentShotIndex < static_cast<int>(shotList->size())) {
            CameraShot& shot = (*shotList)[g_currentShotIndex];
            
            float rad = acfHeading * 3.14159f / 180.0f;
            float cosH = std::cos(rad);
            float sinH = std::sin(rad);
            
            outCameraPosition->x = acfX + shot.x * cosH - shot.z * sinH;
            outCameraPosition->y = acfY + shot.y;
            outCameraPosition->z = acfZ + shot.x * sinH + shot.z * cosH;
            outCameraPosition->pitch = shot.pitch;
            outCameraPosition->heading = acfHeading + shot.heading;
            outCameraPosition->roll = shot.roll;
            outCameraPosition->zoom = shot.zoom;
        }
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
            }
        } else {
            g_currentShotTime -= inElapsedSinceLastCall;
            if (g_currentShotTime <= 0.0f) {
                // Time for next shot
                XPLMReadCameraPosition(&g_startPos);
                
                CameraShot nextShot = SelectNextShot();
                
                // Calculate target position
                float acfX = XPLMGetDataf(g_drLocalX);
                float acfY = XPLMGetDataf(g_drLocalY);
                float acfZ = XPLMGetDataf(g_drLocalZ);
                float acfHeading = XPLMGetDataf(g_drHeading);
                
                float rad = acfHeading * 3.14159f / 180.0f;
                float cosH = std::cos(rad);
                float sinH = std::sin(rad);
                
                g_targetPos.x = acfX + nextShot.x * cosH - nextShot.z * sinH;
                g_targetPos.y = acfY + nextShot.y;
                g_targetPos.z = acfZ + nextShot.x * sinH + nextShot.z * cosH;
                g_targetPos.pitch = nextShot.pitch;
                g_targetPos.heading = acfHeading + nextShot.heading;
                g_targetPos.roll = nextShot.roll;
                g_targetPos.zoom = nextShot.zoom;
                
                g_inTransition = true;
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
    
    // Initialize camera shots
    InitializeCameraShots();
    
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
    
    XPLMDebugString("MovieCamera: Plugin enabled\n");
    
    return 1;
}

/**
 * Plugin disable
 */
PLUGIN_API void XPluginDisable(void) {
    XPLMDebugString("MovieCamera: Plugin disabling...\n");
    
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
    
    // Handle plane loaded message to reset state
    if (inMsg == XPLM_MSG_PLANE_LOADED && inParam == nullptr) {
        // User's plane loaded
        g_mouseIdleTime = 0.0f;
    }
}
