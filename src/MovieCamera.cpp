/**
 * MovieCamera Plugin for X-Plane 12
 * 
 * A cinematic camera plugin that provides automatic camera movements
 * with smooth transitions between cockpit and external views.
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
#include "XPWidgets.h"
#include "XPStandardWidgets.h"

#include <cstdlib>
#include <cstring>
#include <cmath>
#include <ctime>
#include <cstdio>
#include <string>
#include <vector>
#include <algorithm>

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

// Settings window (using X-Plane Widgets)
static XPWidgetID g_settingsWidget = nullptr;
static XPWidgetID g_delayTextField = nullptr;
static XPWidgetID g_autoAltTextField = nullptr;
static XPWidgetID g_shotMinTextField = nullptr;
static XPWidgetID g_shotMaxTextField = nullptr;
static XPWidgetID g_statusLabel = nullptr;

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
static XPLMDataRef g_drAltitudeFt = nullptr;
static XPLMDataRef g_drPilotX = nullptr;
static XPLMDataRef g_drPilotY = nullptr;
static XPLMDataRef g_drPilotZ = nullptr;
static XPLMDataRef g_drViewType = nullptr;

// Flight loop callback
static XPLMFlightLoopID g_flightLoopId = nullptr;

// Predefined camera shots
static std::vector<CameraShot> g_cockpitShots;
static std::vector<CameraShot> g_externalShots;

// Function declarations
static void InitializeCameraShots();
static void UpdateMenuState();
static float FlightLoopCallback(float inElapsedSinceLastCall, float inElapsedTimeSinceLastFlightLoop, int inCounter, void* inRefcon);
static int CameraControlCallback(XPLMCameraPosition_t* outCameraPosition, int inIsLosingControl, void* inRefcon);
static void MenuHandler(void* inMenuRef, void* inItemRef);
static void CreateSettingsWidget();
static void DestroySettingsWidget();
static int SettingsWidgetHandler(XPWidgetMessage inMessage, XPWidgetID inWidget, intptr_t inParam1, intptr_t inParam2);
static void UpdateSettingsFromUI();
static void UpdateStatusLabel();
static void StartCameraControl();
static void StopCameraControl();
static void PauseCameraControl();
static void ResumeCameraControl();
static bool CheckAutoConditions();
static CameraShot SelectNextShot();
static float Lerp(float a, float b, float t);
static float EaseInOutCubic(float t);

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
            if (!g_settingsWidget) {
                CreateSettingsWidget();
            } else {
                if (XPIsWidgetVisible(g_settingsWidget)) {
                    XPHideWidget(g_settingsWidget);
                } else {
                    XPShowWidget(g_settingsWidget);
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
    if (!g_drOnGround || !g_drGroundSpeed || !g_drAltitudeFt) {
        return false;
    }
    
    int onGround = XPLMGetDatai(g_drOnGround);
    float groundSpeed = XPLMGetDataf(g_drGroundSpeed);  // m/s
    float altitude = XPLMGetDataf(g_drAltitudeFt);
    
    // Convert elevation (meters) to feet for comparison
    altitude = altitude * 3.28084f;
    
    // Condition 1: On ground and stationary (speed < 1 m/s)
    if (onGround && groundSpeed < 1.0f) {
        return true;
    }
    
    // Condition 2: In the air above Auto Alt and mouse idle for Delay time
    if (!onGround && altitude > g_autoAltFt && g_mouseIdleTime >= g_delaySeconds) {
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
    
    // Update status label if settings window is visible
    if (g_settingsWidget && XPIsWidgetVisible(g_settingsWidget)) {
        UpdateStatusLabel();
    }
    
    return -1.0f;  // Call every frame
}

/**
 * Update settings from UI text fields
 */
static void UpdateSettingsFromUI() {
    char buffer[64];
    
    if (g_delayTextField) {
        XPGetWidgetDescriptor(g_delayTextField, buffer, sizeof(buffer));
        float value = static_cast<float>(std::atof(buffer));
        if (value >= 1.0f && value <= 300.0f) {
            g_delaySeconds = value;
        }
    }
    
    if (g_autoAltTextField) {
        XPGetWidgetDescriptor(g_autoAltTextField, buffer, sizeof(buffer));
        float value = static_cast<float>(std::atof(buffer));
        if (value >= 0.0f && value <= 50000.0f) {
            g_autoAltFt = value;
        }
    }
    
    if (g_shotMinTextField) {
        XPGetWidgetDescriptor(g_shotMinTextField, buffer, sizeof(buffer));
        float value = static_cast<float>(std::atof(buffer));
        if (value >= 1.0f && value <= 30.0f) {
            g_shotMinDuration = value;
        }
    }
    
    if (g_shotMaxTextField) {
        XPGetWidgetDescriptor(g_shotMaxTextField, buffer, sizeof(buffer));
        float value = static_cast<float>(std::atof(buffer));
        if (value >= g_shotMinDuration && value <= 30.0f) {
            g_shotMaxDuration = value;
        }
    }
}

/**
 * Update status label text
 */
static void UpdateStatusLabel() {
    if (!g_statusLabel) return;
    
    const char* modeStr = "Off";
    if (g_pluginMode == PluginMode::Auto) modeStr = "Auto";
    else if (g_pluginMode == PluginMode::Manual) modeStr = "Manual";
    
    const char* stateStr = "Inactive";
    if (g_functionActive && !g_functionPaused) stateStr = "Active";
    else if (g_functionActive && g_functionPaused) stateStr = "Paused";
    
    char buffer[256];
    std::snprintf(buffer, sizeof(buffer), "Mode: %s | State: %s | Idle: %.1fs", modeStr, stateStr, g_mouseIdleTime);
    XPSetWidgetDescriptor(g_statusLabel, buffer);
}

/**
 * Settings widget message handler
 */
static int SettingsWidgetHandler(XPWidgetMessage inMessage, XPWidgetID inWidget, intptr_t inParam1, intptr_t inParam2) {
    (void)inParam2;  // Unused
    
    if (inMessage == xpMessage_CloseButtonPushed) {
        if (inWidget == g_settingsWidget) {
            UpdateSettingsFromUI();
            XPHideWidget(g_settingsWidget);
            return 1;
        }
    }
    
    if (inMessage == xpMsg_PushButtonPressed) {
        // Could handle Apply button here if we added one
        (void)inParam1;  // Unused for now
        return 0;
    }
    
    if (inMessage == xpMsg_TextFieldChanged) {
        // Could validate input here
        return 0;
    }
    
    return 0;
}

/**
 * Create the settings widget
 */
static void CreateSettingsWidget() {
    int screenWidth, screenHeight;
    XPLMGetScreenSize(&screenWidth, &screenHeight);
    
    int width = 350;
    int height = 280;
    int left = (screenWidth - width) / 2;
    int top = (screenHeight + height) / 2;
    int right = left + width;
    int bottom = top - height;
    
    // Create main window widget
    g_settingsWidget = XPCreateWidget(left, top, right, bottom,
        1,                           // Visible
        "MovieCamera Settings",      // Title
        1,                           // Root
        nullptr,                     // No container
        xpWidgetClass_MainWindow);
    
    // Set close button behavior
    XPSetWidgetProperty(g_settingsWidget, xpProperty_MainWindowHasCloseBoxes, 1);
    XPAddWidgetCallback(g_settingsWidget, SettingsWidgetHandler);
    
    int x = left + 20;
    int y = top - 40;
    int labelWidth = 120;
    int fieldWidth = 80;
    int rowHeight = 30;
    
    char buffer[64];
    
    // Delay setting
    XPCreateWidget(x, y, x + labelWidth, y - 20,
        1, "Delay (seconds):", 0, g_settingsWidget, xpWidgetClass_Caption);
    
    std::snprintf(buffer, sizeof(buffer), "%.0f", g_delaySeconds);
    g_delayTextField = XPCreateWidget(x + labelWidth + 10, y, x + labelWidth + 10 + fieldWidth, y - 20,
        1, buffer, 0, g_settingsWidget, xpWidgetClass_TextField);
    XPSetWidgetProperty(g_delayTextField, xpProperty_TextFieldType, xpTextEntryField);
    XPSetWidgetProperty(g_delayTextField, xpProperty_MaxCharacters, 6);
    
    y -= rowHeight;
    
    // Auto Alt setting
    XPCreateWidget(x, y, x + labelWidth, y - 20,
        1, "Auto Alt (ft):", 0, g_settingsWidget, xpWidgetClass_Caption);
    
    std::snprintf(buffer, sizeof(buffer), "%.0f", g_autoAltFt);
    g_autoAltTextField = XPCreateWidget(x + labelWidth + 10, y, x + labelWidth + 10 + fieldWidth, y - 20,
        1, buffer, 0, g_settingsWidget, xpWidgetClass_TextField);
    XPSetWidgetProperty(g_autoAltTextField, xpProperty_TextFieldType, xpTextEntryField);
    XPSetWidgetProperty(g_autoAltTextField, xpProperty_MaxCharacters, 6);
    
    y -= rowHeight + 10;
    
    // Shot duration section
    XPCreateWidget(x, y, x + 200, y - 20,
        1, "Shot Duration Range:", 0, g_settingsWidget, xpWidgetClass_Caption);
    
    y -= 25;
    
    // Min duration
    XPCreateWidget(x, y, x + 60, y - 20,
        1, "Min (s):", 0, g_settingsWidget, xpWidgetClass_Caption);
    
    std::snprintf(buffer, sizeof(buffer), "%.1f", g_shotMinDuration);
    g_shotMinTextField = XPCreateWidget(x + 65, y, x + 65 + 60, y - 20,
        1, buffer, 0, g_settingsWidget, xpWidgetClass_TextField);
    XPSetWidgetProperty(g_shotMinTextField, xpProperty_TextFieldType, xpTextEntryField);
    XPSetWidgetProperty(g_shotMinTextField, xpProperty_MaxCharacters, 5);
    
    // Max duration
    XPCreateWidget(x + 140, y, x + 200, y - 20,
        1, "Max (s):", 0, g_settingsWidget, xpWidgetClass_Caption);
    
    std::snprintf(buffer, sizeof(buffer), "%.1f", g_shotMaxDuration);
    g_shotMaxTextField = XPCreateWidget(x + 205, y, x + 205 + 60, y - 20,
        1, buffer, 0, g_settingsWidget, xpWidgetClass_TextField);
    XPSetWidgetProperty(g_shotMaxTextField, xpProperty_TextFieldType, xpTextEntryField);
    XPSetWidgetProperty(g_shotMaxTextField, xpProperty_MaxCharacters, 5);
    
    y -= rowHeight + 20;
    
    // Status section
    XPCreateWidget(x, y, x + 80, y - 20,
        1, "Status:", 0, g_settingsWidget, xpWidgetClass_Caption);
    
    y -= 25;
    
    g_statusLabel = XPCreateWidget(x, y, right - 20, y - 20,
        1, "Mode: Off | State: Inactive | Idle: 0.0s", 0, g_settingsWidget, xpWidgetClass_Caption);
    
    y -= rowHeight + 10;
    
    // Help text
    XPCreateWidget(x, y, right - 20, y - 40,
        1, "Auto: Activates when on ground/stationary or\nat altitude after delay. Mouse pauses camera.", 
        0, g_settingsWidget, xpWidgetClass_Caption);
}

/**
 * Destroy the settings widget
 */
static void DestroySettingsWidget() {
    if (g_settingsWidget) {
        XPDestroyWidget(g_settingsWidget, 1);
        g_settingsWidget = nullptr;
        g_delayTextField = nullptr;
        g_autoAltTextField = nullptr;
        g_shotMinTextField = nullptr;
        g_shotMaxTextField = nullptr;
        g_statusLabel = nullptr;
    }
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
    g_drAltitudeFt = XPLMFindDataRef("sim/flightmodel/position/elevation");
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
    
    // Destroy settings widget
    DestroySettingsWidget();
    
    XPLMDebugString("MovieCamera: Plugin disabled\n");
}

/**
 * Message receive
 */
PLUGIN_API void XPluginReceiveMessage(XPLMPluginID inFrom, int inMsg, void* inParam) {
    // Handle plane loaded message to reset state
    if (inMsg == XPLM_MSG_PLANE_LOADED && inParam == nullptr) {
        // User's plane loaded
        g_mouseIdleTime = 0.0f;
    }
}
