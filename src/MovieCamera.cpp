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

// OpenGL for trajectory drawing
#if IBM
#define NOMINMAX  // Prevent Windows min/max macros from conflicting with std::min/std::max
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

// Keyframe for custom camera paths
struct CameraKeyframe {
    float time;              // Time in seconds from path start
    float x, y, z;           // Position offset from aircraft
    float pitch, heading, roll;
    float zoom;
    float focalLength;       // Focal length parameter (mm equivalent)
    float aperture;          // Aperture (f-stop) for DOF simulation
};

// Custom camera path with keyframes
struct CustomCameraPath {
    std::string name;
    CameraType type;
    std::vector<CameraKeyframe> keyframes;
    bool isLooping;
    
    float getTotalDuration() const {
        if (keyframes.empty()) return 0.0f;
        return keyframes.back().time;
    }
};

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

// Flight loop callback
static XPLMFlightLoopID g_flightLoopId = nullptr;

// Predefined camera shots
static std::vector<CameraShot> g_cockpitShots;
static std::vector<CameraShot> g_externalShots;

// Custom camera paths
static std::vector<CustomCameraPath> g_customPaths;
static int g_currentCustomPathIndex = -1;
static bool g_usingCustomPath = false;
static float g_customPathTime = 0.0f;

// Path editor state
static bool g_pathEditorOpen = false;
static CustomCameraPath g_editingPath;
static int g_editingKeyframeIndex = -1;
static char g_pathNameBuffer[64] = "";
static bool g_showTrajectoryPreview = false;  // Toggle for 3D trajectory visualization

// Draw callback for 3D trajectory visualization
static int DrawTrajectoryCallback(XPLMDrawingPhase inPhase, int inIsBefore, void* inRefcon);

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
static float SmoothDrift(float baseValue, float driftAmount, float time, float frequency);
static CameraKeyframe InterpolateKeyframes(const CameraKeyframe& a, const CameraKeyframe& b, float t);
static void SaveCustomPaths();
static void LoadCustomPaths();
static std::string GetPluginPath();

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
    
    // Custom Camera Paths Section
    ImGui::Text("Custom Camera Paths");
    ImGui::SameLine();
    ImGui::TextDisabled("(?)");
    if (ImGui::IsItemHovered()) {
        ImGui::SetTooltip("Create custom camera movement paths with keyframes.\nDefine position, rotation, zoom, focal length and aperture at each keyframe.");
    }
    
    // List existing paths
    if (!g_customPaths.empty()) {
        ImGui::Text("Saved Paths:");
        for (size_t i = 0; i < g_customPaths.size(); ++i) {
            ImGui::PushID(static_cast<int>(i));
            
            bool isSelected = (g_currentCustomPathIndex == static_cast<int>(i) && g_usingCustomPath);
            if (ImGui::Selectable(g_customPaths[i].name.c_str(), isSelected, 0, ImVec2(120, 0))) {
                // Select this path for playback
                g_currentCustomPathIndex = static_cast<int>(i);
                g_usingCustomPath = true;
                g_customPathTime = 0.0f;
                if (!g_functionActive) {
                    StartCameraControl();
                }
            }
            
            ImGui::SameLine();
            if (ImGui::SmallButton("Edit")) {
                g_pathEditorOpen = true;
                g_editingPath = g_customPaths[i];
                snprintf(g_pathNameBuffer, sizeof(g_pathNameBuffer), "%s", g_customPaths[i].name.c_str());
                g_editingKeyframeIndex = -1;
            }
            
            ImGui::SameLine();
            if (ImGui::SmallButton("Del")) {
                g_customPaths.erase(g_customPaths.begin() + static_cast<std::ptrdiff_t>(i));
                SaveCustomPaths();
                if (g_currentCustomPathIndex == static_cast<int>(i)) {
                    g_usingCustomPath = false;
                    g_currentCustomPathIndex = -1;
                }
            }
            
            ImGui::PopID();
        }
    }
    
    // Button to create new path
    if (ImGui::Button("New Path", ImVec2(80, 0))) {
        g_pathEditorOpen = true;
        g_editingPath = CustomCameraPath();
        g_editingPath.name = "New Path";
        g_editingPath.type = CameraType::External;
        g_editingPath.isLooping = false;
        snprintf(g_pathNameBuffer, sizeof(g_pathNameBuffer), "New Path");
        g_editingKeyframeIndex = -1;
        g_showTrajectoryPreview = true;  // Enable trajectory preview by default
        
        // Start with empty keyframes - user will capture positions
        g_editingPath.keyframes.clear();
    }
    
    ImGui::SameLine();
    if (g_usingCustomPath && ImGui::Button("Stop Path", ImVec2(80, 0))) {
        g_usingCustomPath = false;
        g_currentCustomPathIndex = -1;
    }
    
    // Path Editor Window
    if (g_pathEditorOpen) {
        ImGui::Spacing();
        ImGui::Separator();
        ImGui::Text("Path Editor");
        
        // Trajectory preview toggle with help text
        ImGui::Checkbox("Show 3D Trajectory", &g_showTrajectoryPreview);
        ImGui::SameLine();
        ImGui::TextDisabled("(?)");
        if (ImGui::IsItemHovered()) {
            ImGui::SetTooltip("When enabled, draws the camera path in 3D space.\nGreen line = path trajectory\nYellow dots = keyframes\nRed dot = selected keyframe");
        }
        
        ImGui::Spacing();
        
        // Path name
        ImGui::Text("Name:");
        ImGui::SameLine();
        ImGui::SetNextItemWidth(150);
        ImGui::InputText("##pathname", g_pathNameBuffer, sizeof(g_pathNameBuffer));
        
        // Path type
        ImGui::Text("Type:");
        ImGui::SameLine();
        int typeInt = static_cast<int>(g_editingPath.type);
        if (ImGui::RadioButton("Cockpit", &typeInt, 0)) g_editingPath.type = CameraType::Cockpit;
        ImGui::SameLine();
        if (ImGui::RadioButton("External", &typeInt, 1)) g_editingPath.type = CameraType::External;
        
        // Looping
        ImGui::Checkbox("Looping", &g_editingPath.isLooping);
        
        ImGui::Spacing();
        ImGui::Separator();
        
        // Capture current camera position section
        ImGui::TextColored(ImVec4(0.4f, 0.8f, 1.0f, 1.0f), "Capture Camera Position");
        ImGui::SameLine();
        ImGui::TextDisabled("(?)");
        if (ImGui::IsItemHovered()) {
            ImGui::SetTooltip("Move X-Plane's camera to your desired position,\nthen click 'Capture Position' to add it as a keyframe.\nBuild your camera path by capturing multiple positions.");
        }
        
        if (ImGui::Button("Capture Position", ImVec2(120, 0))) {
            // Read current camera position
            XPLMCameraPosition_t camPos;
            XPLMReadCameraPosition(&camPos);
            
            // Get aircraft position to calculate relative offset
            float acfX = XPLMGetDataf(g_drLocalX);
            float acfY = XPLMGetDataf(g_drLocalY);
            float acfZ = XPLMGetDataf(g_drLocalZ);
            float acfHeading = XPLMGetDataf(g_drHeading);
            
            // Transform world camera position to aircraft-relative coordinates
            float rad = acfHeading * PI / 180.0f;
            float cosH = std::cos(rad);
            float sinH = std::sin(rad);
            
            // Calculate offset from aircraft
            float dx = camPos.x - acfX;
            float dy = camPos.y - acfY;
            float dz = camPos.z - acfZ;
            
            // Rotate to aircraft-relative coordinates
            float relX = dx * cosH + dz * sinH;
            float relY = dy;
            float relZ = -dx * sinH + dz * cosH;
            
            // Calculate relative heading
            float relHeading = camPos.heading - acfHeading;
            while (relHeading > 180.0f) relHeading -= 360.0f;
            while (relHeading < -180.0f) relHeading += 360.0f;
            
            // Create new keyframe
            CameraKeyframe newKf;
            newKf.time = g_editingPath.keyframes.empty() ? 0.0f : g_editingPath.keyframes.back().time + 3.0f;
            newKf.x = relX;
            newKf.y = relY;
            newKf.z = relZ;
            newKf.pitch = camPos.pitch;
            newKf.heading = relHeading;
            newKf.roll = camPos.roll;
            newKf.zoom = camPos.zoom;
            newKf.focalLength = 50.0f;
            newKf.aperture = 2.8f;
            
            g_editingPath.keyframes.push_back(newKf);
            g_editingKeyframeIndex = static_cast<int>(g_editingPath.keyframes.size()) - 1;
        }
        
        ImGui::SameLine();
        if (ImGui::Button("Update Selected", ImVec2(110, 0))) {
            if (g_editingKeyframeIndex >= 0 && g_editingKeyframeIndex < static_cast<int>(g_editingPath.keyframes.size())) {
                // Read current camera position
                XPLMCameraPosition_t camPos;
                XPLMReadCameraPosition(&camPos);
                
                // Get aircraft position
                float acfX = XPLMGetDataf(g_drLocalX);
                float acfY = XPLMGetDataf(g_drLocalY);
                float acfZ = XPLMGetDataf(g_drLocalZ);
                float acfHeading = XPLMGetDataf(g_drHeading);
                
                // Transform to aircraft-relative
                float rad = acfHeading * PI / 180.0f;
                float cosH = std::cos(rad);
                float sinH = std::sin(rad);
                
                float dx = camPos.x - acfX;
                float dy = camPos.y - acfY;
                float dz = camPos.z - acfZ;
                
                float relX = dx * cosH + dz * sinH;
                float relY = dy;
                float relZ = -dx * sinH + dz * cosH;
                
                float relHeading = camPos.heading - acfHeading;
                while (relHeading > 180.0f) relHeading -= 360.0f;
                while (relHeading < -180.0f) relHeading += 360.0f;
                
                // Update selected keyframe
                CameraKeyframe& kf = g_editingPath.keyframes[g_editingKeyframeIndex];
                kf.x = relX;
                kf.y = relY;
                kf.z = relZ;
                kf.pitch = camPos.pitch;
                kf.heading = relHeading;
                kf.roll = camPos.roll;
                kf.zoom = camPos.zoom;
            }
        }
        if (ImGui::IsItemHovered()) {
            ImGui::SetTooltip("Update the selected keyframe with current camera position");
        }
        
        ImGui::Spacing();
        ImGui::Separator();
        ImGui::Text("Keyframes (%zu):", g_editingPath.keyframes.size());
        
        // Keyframe list
        for (size_t i = 0; i < g_editingPath.keyframes.size(); ++i) {
            ImGui::PushID(static_cast<int>(i + 1000));
            
            CameraKeyframe& kf = g_editingPath.keyframes[i];
            bool isEditing = (g_editingKeyframeIndex == static_cast<int>(i));
            
            char label[32];
            snprintf(label, sizeof(label), "KF %zu (%.1fs)", i + 1, kf.time);
            
            if (ImGui::Selectable(label, isEditing, 0, ImVec2(100, 0))) {
                g_editingKeyframeIndex = static_cast<int>(i);
            }
            
            ImGui::SameLine();
            if (ImGui::SmallButton("-")) {
                g_editingPath.keyframes.erase(g_editingPath.keyframes.begin() + static_cast<std::ptrdiff_t>(i));
                if (g_editingKeyframeIndex == static_cast<int>(i)) {
                    g_editingKeyframeIndex = -1;
                }
            }
            
            ImGui::PopID();
        }
        
        // Add keyframe button
        if (ImGui::SmallButton("+ Add Keyframe")) {
            CameraKeyframe newKf;
            if (!g_editingPath.keyframes.empty()) {
                newKf = g_editingPath.keyframes.back();
                newKf.time += 2.0f;
            } else {
                newKf = {0.0f, 0.0f, 10.0f, -30.0f, 10.0f, 180.0f, 0.0f, 1.0f, 50.0f, 2.8f};
            }
            g_editingPath.keyframes.push_back(newKf);
            g_editingKeyframeIndex = static_cast<int>(g_editingPath.keyframes.size()) - 1;
        }
        
        // Keyframe editor
        if (g_editingKeyframeIndex >= 0 && g_editingKeyframeIndex < static_cast<int>(g_editingPath.keyframes.size())) {
            ImGui::Spacing();
            ImGui::Text("Edit Keyframe %d:", g_editingKeyframeIndex + 1);
            
            CameraKeyframe& kf = g_editingPath.keyframes[g_editingKeyframeIndex];
            
            ImGui::SetNextItemWidth(60);
            ImGui::InputFloat("Time (s)", &kf.time, 0.5f, 1.0f, "%.1f");
            
            ImGui::Text("Position:");
            ImGui::SetNextItemWidth(60);
            ImGui::InputFloat("X", &kf.x, 1.0f, 5.0f, "%.1f");
            ImGui::SameLine();
            ImGui::SetNextItemWidth(60);
            ImGui::InputFloat("Y", &kf.y, 1.0f, 5.0f, "%.1f");
            ImGui::SameLine();
            ImGui::SetNextItemWidth(60);
            ImGui::InputFloat("Z", &kf.z, 1.0f, 5.0f, "%.1f");
            
            ImGui::Text("Rotation:");
            ImGui::SetNextItemWidth(60);
            ImGui::InputFloat("Pitch", &kf.pitch, 1.0f, 5.0f, "%.1f");
            ImGui::SameLine();
            ImGui::SetNextItemWidth(60);
            ImGui::InputFloat("Heading", &kf.heading, 5.0f, 15.0f, "%.1f");
            ImGui::SameLine();
            ImGui::SetNextItemWidth(60);
            ImGui::InputFloat("Roll", &kf.roll, 1.0f, 5.0f, "%.1f");
            
            ImGui::Text("Camera:");
            ImGui::SetNextItemWidth(60);
            ImGui::InputFloat("Zoom", &kf.zoom, 0.05f, 0.1f, "%.2f");
            ImGui::SameLine();
            ImGui::SetNextItemWidth(60);
            ImGui::InputFloat("Focal (mm)", &kf.focalLength, 5.0f, 10.0f, "%.0f");
            ImGui::SameLine();
            ImGui::SetNextItemWidth(60);
            ImGui::InputFloat("f/", &kf.aperture, 0.5f, 1.0f, "%.1f");
        }
        
        ImGui::Spacing();
        
        // Save/Cancel buttons
        if (ImGui::Button("Save Path", ImVec2(80, 0))) {
            g_editingPath.name = g_pathNameBuffer;
            
            // Check if this is an existing path or new
            bool found = false;
            for (auto& path : g_customPaths) {
                if (path.name == g_editingPath.name) {
                    path = g_editingPath;
                    found = true;
                    break;
                }
            }
            if (!found) {
                g_customPaths.push_back(g_editingPath);
            }
            
            SaveCustomPaths();
            g_pathEditorOpen = false;
        }
        
        ImGui::SameLine();
        if (ImGui::Button("Cancel", ImVec2(80, 0))) {
            g_pathEditorOpen = false;
        }
        
        ImGui::SameLine();
        if (ImGui::Button("Preview", ImVec2(80, 0))) {
            // Find or add this path temporarily for preview
            bool found = false;
            for (size_t i = 0; i < g_customPaths.size(); ++i) {
                if (g_customPaths[i].name == g_editingPath.name) {
                    g_customPaths[i] = g_editingPath;
                    g_currentCustomPathIndex = static_cast<int>(i);
                    found = true;
                    break;
                }
            }
            if (!found) {
                g_customPaths.push_back(g_editingPath);
                g_currentCustomPathIndex = static_cast<int>(g_customPaths.size()) - 1;
            }
            
            g_usingCustomPath = true;
            g_customPathTime = 0.0f;
            if (!g_functionActive) {
                StartCameraControl();
            }
        }
    }
    
    ImGui::Spacing();
    ImGui::Separator();
    ImGui::Spacing();
    
    if (ImGui::Button("Close", ImVec2(80, 0))) {
        SetVisible(false);
    }
}

/**
 * Initialize predefined camera shots with cinematic movement
 * Each shot includes drift parameters for smooth multi-axis camera motion
 */
static void InitializeCameraShots() {
    // Cockpit shots - various instrument views and pilot perspective shots
    // Drift values create slow, cinematic camera movements
    // For cockpit: subtle position/rotation drift + zoom breathing for DOF effect
    g_cockpitShots.clear();
    
    // Center panel view - slow zoom in with slight drift
    // x, y, z, pitch, heading, roll, zoom, duration, name, driftX, driftY, driftZ, driftPitch, driftHeading, driftRoll, driftZoom
    g_cockpitShots.push_back({CameraType::Cockpit, 0.0f, 0.15f, 0.4f, -8.0f, 0.0f, 0.0f, 1.0f, 8.0f, "Center Panel",
                              0.0f, 0.01f, 0.02f, 0.2f, 0.0f, 0.0f, 0.03f});
    
    // Left panel (throttle/navigation) - gentle pan right with zoom
    g_cockpitShots.push_back({CameraType::Cockpit, -0.25f, 0.1f, 0.3f, -12.0f, -25.0f, 0.0f, 1.2f, 7.0f, "Left Panel",
                              0.01f, 0.0f, 0.01f, 0.15f, 1.0f, 0.0f, 0.025f});
    
    // Right panel (radios/autopilot) - gentle pan left with zoom
    g_cockpitShots.push_back({CameraType::Cockpit, 0.25f, 0.1f, 0.3f, -12.0f, 25.0f, 0.0f, 1.2f, 7.0f, "Right Panel",
                              -0.01f, 0.0f, 0.01f, 0.15f, -1.0f, 0.0f, 0.025f});
    
    // Overhead panel view - slow tilt down
    g_cockpitShots.push_back({CameraType::Cockpit, 0.0f, 0.35f, 0.15f, -55.0f, 0.0f, 0.0f, 1.1f, 6.0f, "Overhead Panel",
                              0.0f, -0.01f, 0.01f, 1.5f, 0.0f, 0.0f, 0.02f});
    
    // PFD closeup - slow zoom with subtle drift
    g_cockpitShots.push_back({CameraType::Cockpit, -0.12f, 0.05f, 0.45f, -3.0f, -8.0f, 0.0f, 1.6f, 8.0f, "PFD View",
                              0.005f, 0.005f, 0.015f, 0.1f, 0.3f, 0.0f, 0.04f});
    
    // ND/MFD view - slow zoom with subtle drift  
    g_cockpitShots.push_back({CameraType::Cockpit, 0.12f, 0.05f, 0.45f, -3.0f, 8.0f, 0.0f, 1.6f, 8.0f, "ND/MFD View",
                              -0.005f, 0.005f, 0.015f, 0.1f, -0.3f, 0.0f, 0.04f});
    
    // Pilot's eye view looking out - subtle look around
    g_cockpitShots.push_back({CameraType::Cockpit, -0.1f, 0.25f, -0.1f, 3.0f, 5.0f, 0.0f, 0.9f, 10.0f, "Pilot View",
                              0.005f, 0.0f, 0.0f, 0.0f, 0.8f, 0.0f, 0.0f});
    
    // Co-pilot perspective - looking at captain's side
    g_cockpitShots.push_back({CameraType::Cockpit, 0.35f, 0.2f, 0.0f, 0.0f, -20.0f, 0.0f, 0.95f, 8.0f, "Copilot View",
                              -0.01f, 0.0f, 0.0f, 0.0f, 0.5f, 0.0f, 0.01f});
    
    // Looking out left window - slow pan
    g_cockpitShots.push_back({CameraType::Cockpit, -0.35f, 0.15f, 0.0f, 2.0f, -75.0f, 0.0f, 0.85f, 9.0f, "Left Window",
                              0.0f, 0.01f, 0.0f, -0.3f, 2.0f, 0.0f, 0.0f});
    
    // Looking out right window - slow pan
    g_cockpitShots.push_back({CameraType::Cockpit, 0.35f, 0.15f, 0.0f, 2.0f, 75.0f, 0.0f, 0.85f, 9.0f, "Right Window",
                              0.0f, 0.01f, 0.0f, -0.3f, -2.0f, 0.0f, 0.0f});
    
    // Pedestal/center console view
    g_cockpitShots.push_back({CameraType::Cockpit, 0.0f, 0.0f, 0.35f, -35.0f, 0.0f, 0.0f, 1.4f, 6.0f, "Pedestal View",
                              0.0f, 0.01f, 0.01f, 0.5f, 0.0f, 0.0f, 0.03f});
    
    // External shots - various external angles with cinematic movement
    // External shots have more pronounced position drift for dynamic feel
    // Positions are offset away from aircraft center to avoid clipping
    g_externalShots.clear();
    
    // Front hero shot - positioned further out and slightly offset
    g_externalShots.push_back({CameraType::External, 5.0f, 8.0f, -55.0f, 10.0f, 175.0f, 0.0f, 0.85f, 10.0f, "Front Hero",
                               -0.1f, 0.12f, 0.25f, -0.25f, 0.3f, 0.0f, 0.01f});
    
    // Rear chase - elevated and further back, offset to side
    g_externalShots.push_back({CameraType::External, -8.0f, 12.0f, 65.0f, 15.0f, 8.0f, 0.0f, 0.8f, 10.0f, "Rear Chase",
                               0.15f, 0.08f, -0.18f, -0.15f, -0.4f, 0.0f, 0.0f});
    
    // Left flyby - further out to side, more dramatic sweep
    g_externalShots.push_back({CameraType::External, -50.0f, 5.0f, 15.0f, 5.0f, 82.0f, 2.0f, 0.85f, 12.0f, "Left Flyby",
                               0.5f, 0.1f, -0.6f, 0.0f, 1.0f, -0.1f, 0.0f});
    
    // Right flyby - further out to side, more dramatic sweep
    g_externalShots.push_back({CameraType::External, 50.0f, 5.0f, 15.0f, 5.0f, -82.0f, -2.0f, 0.85f, 12.0f, "Right Flyby",
                               -0.5f, 0.1f, -0.6f, 0.0f, -1.0f, 0.1f, 0.0f});
    
    // High orbit - high above and further back for wide view
    g_externalShots.push_back({CameraType::External, 15.0f, 55.0f, 35.0f, 62.0f, -15.0f, 0.0f, 0.75f, 15.0f, "High Orbit",
                               -0.3f, 0.03f, 0.0f, 0.0f, 2.0f, 0.0f, 0.0f});
    
    // Low angle front - below and forward, offset to avoid fuselage
    g_externalShots.push_back({CameraType::External, 12.0f, -12.0f, -40.0f, -22.0f, 170.0f, 3.0f, 0.9f, 8.0f, "Low Angle Front",
                               -0.1f, 0.15f, 0.2f, 0.4f, 0.5f, -0.2f, 0.0f});
    
    // Quarter front left - wide angle approach shot
    g_externalShots.push_back({CameraType::External, -40.0f, 12.0f, -40.0f, 14.0f, 135.0f, -1.0f, 0.82f, 10.0f, "Quarter FL",
                               0.25f, 0.06f, 0.3f, -0.12f, -0.8f, 0.05f, 0.0f});
    
    // Quarter front right - wide angle approach shot
    g_externalShots.push_back({CameraType::External, 40.0f, 12.0f, -40.0f, 14.0f, -135.0f, 1.0f, 0.82f, 10.0f, "Quarter FR",
                               -0.25f, 0.06f, 0.3f, -0.12f, 0.8f, -0.05f, 0.0f});
    
    // Quarter rear left - elevated departure shot
    g_externalShots.push_back({CameraType::External, -35.0f, 18.0f, 50.0f, 20.0f, 45.0f, 2.0f, 0.8f, 10.0f, "Quarter RL",
                               0.2f, 0.04f, -0.2f, -0.18f, -0.6f, -0.1f, 0.0f});
    
    // Quarter rear right - elevated departure shot  
    g_externalShots.push_back({CameraType::External, 35.0f, 18.0f, 50.0f, 20.0f, -45.0f, -2.0f, 0.8f, 10.0f, "Quarter RR",
                               -0.2f, 0.04f, -0.2f, -0.18f, 0.6f, 0.1f, 0.0f});
    
    // Wing tip left - close wing view with offset
    g_externalShots.push_back({CameraType::External, -28.0f, 5.0f, 8.0f, 10.0f, 60.0f, -3.0f, 1.0f, 8.0f, "Wing Left",
                               0.1f, 0.04f, -0.12f, 0.0f, 0.6f, 0.15f, 0.0f});
    
    // Wing tip right - close wing view with offset
    g_externalShots.push_back({CameraType::External, 28.0f, 5.0f, 8.0f, 10.0f, -60.0f, 3.0f, 1.0f, 8.0f, "Wing Right",
                               -0.1f, 0.04f, -0.12f, 0.0f, -0.6f, -0.15f, 0.0f});
    
    // Engine close-up left - positioned to see engine from outside
    g_externalShots.push_back({CameraType::External, -20.0f, 2.0f, -5.0f, 8.0f, 75.0f, 0.0f, 1.2f, 7.0f, "Engine L",
                               0.06f, 0.03f, -0.1f, 0.0f, 0.4f, 0.0f, 0.0f});
    
    // Engine close-up right - positioned to see engine from outside
    g_externalShots.push_back({CameraType::External, 20.0f, 2.0f, -5.0f, 8.0f, -75.0f, 0.0f, 1.2f, 7.0f, "Engine R",
                               -0.06f, 0.03f, -0.1f, 0.0f, -0.4f, 0.0f, 0.0f});
    
    // Tail view - looking at tail from behind and above
    g_externalShots.push_back({CameraType::External, -10.0f, 15.0f, 70.0f, 28.0f, 10.0f, 0.0f, 0.85f, 9.0f, "Tail View",
                               0.1f, 0.06f, -0.12f, -0.25f, -0.6f, 0.0f, 0.0f});
    
    // Belly view - looking up at aircraft from below
    g_externalShots.push_back({CameraType::External, 8.0f, -18.0f, 0.0f, -35.0f, -5.0f, 0.0f, 0.9f, 8.0f, "Belly View",
                               -0.05f, 0.08f, 0.0f, 0.3f, 0.4f, 0.0f, 0.0f});
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
 * Smooth drift using sinusoidal easing for organic camera movement
 * Creates a gentle, breathing-like motion instead of linear movement
 */
static float SmoothDrift(float baseValue, float driftAmount, float time, float frequency) {
    // Use combination of sine waves at different frequencies for organic feel
    float primary = std::sin(time * frequency * TWO_PI) * 0.5f + 0.5f;  // Main wave
    float secondary = std::sin(time * frequency * 0.7f * TWO_PI) * 0.3f;  // Slower secondary wave
    float tertiary = std::sin(time * frequency * 1.3f * TWO_PI) * 0.2f;   // Faster tertiary wave
    
    // Combine waves for natural-looking motion
    float smoothT = primary + secondary + tertiary;
    smoothT = std::clamp(smoothT, 0.0f, 1.0f);  // Clamp to 0-1
    
    return baseValue + driftAmount * smoothT;
}

/**
 * Ease in-out sine for extra smooth interpolation
 */
static float EaseInOutSine(float t) {
    return -(std::cos(PI * t) - 1.0f) / 2.0f;
}

// Note: CatmullRom spline function may be added in future for advanced path interpolation

/**
 * Interpolate between two keyframes with smooth easing
 */
static CameraKeyframe InterpolateKeyframes(const CameraKeyframe& a, const CameraKeyframe& b, float t) {
    // Use ease-in-out sine for smoother interpolation
    float smoothT = EaseInOutSine(t);
    
    CameraKeyframe result;
    result.time = Lerp(a.time, b.time, t);  // Linear time
    result.x = Lerp(a.x, b.x, smoothT);
    result.y = Lerp(a.y, b.y, smoothT);
    result.z = Lerp(a.z, b.z, smoothT);
    result.pitch = Lerp(a.pitch, b.pitch, smoothT);
    result.heading = LerpAngle(a.heading, b.heading, smoothT);
    result.roll = Lerp(a.roll, b.roll, smoothT);
    result.zoom = Lerp(a.zoom, b.zoom, smoothT);
    result.focalLength = Lerp(a.focalLength, b.focalLength, smoothT);
    result.aperture = Lerp(a.aperture, b.aperture, smoothT);
    
    return result;
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
 * Save custom camera paths to a file
 */
static void SaveCustomPaths() {
    std::string path = GetPluginPath() + "camera_paths.cfg";
    FILE* file = fopen(path.c_str(), "w");
    if (!file) {
        XPLMDebugString("MovieCamera: Failed to save custom paths\n");
        return;
    }
    
    fprintf(file, "# MovieCamera Custom Camera Paths\n");
    fprintf(file, "version 1\n");
    fprintf(file, "paths %zu\n", g_customPaths.size());
    
    for (const auto& cpath : g_customPaths) {
        fprintf(file, "\npath_start\n");
        fprintf(file, "name %s\n", cpath.name.c_str());
        fprintf(file, "type %d\n", static_cast<int>(cpath.type));
        fprintf(file, "looping %d\n", cpath.isLooping ? 1 : 0);
        fprintf(file, "keyframes %zu\n", cpath.keyframes.size());
        
        for (const auto& kf : cpath.keyframes) {
            fprintf(file, "kf %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f\n",
                    kf.time, kf.x, kf.y, kf.z, kf.pitch, kf.heading, kf.roll,
                    kf.zoom, kf.focalLength, kf.aperture);
        }
        fprintf(file, "path_end\n");
    }
    
    fclose(file);
    XPLMDebugString("MovieCamera: Custom paths saved\n");
}

/**
 * Load custom camera paths from a file
 */
static void LoadCustomPaths() {
    std::string path = GetPluginPath() + "camera_paths.cfg";
    FILE* file = fopen(path.c_str(), "r");
    if (!file) {
        // No saved paths file - that's OK
        return;
    }
    
    g_customPaths.clear();
    
    char line[256];
    
    while (fgets(line, sizeof(line), file)) {
        if (line[0] == '#') continue;
        
        if (strncmp(line, "path_start", 10) == 0) {
            CustomCameraPath cpath;
            cpath.isLooping = false;
            cpath.type = CameraType::External;
            
            while (fgets(line, sizeof(line), file)) {
                if (strncmp(line, "path_end", 8) == 0) break;
                
                if (strncmp(line, "name ", 5) == 0) {
                    // Safe string parsing with bounds checking
                    char name[64] = {0};
                    int result = sscanf(line + 5, "%63[^\n]", name);
                    if (result == 1) {
                        cpath.name = name;
                    }
                } else if (strncmp(line, "type ", 5) == 0) {
                    int typeVal = atoi(line + 5);
                    if (typeVal == 0 || typeVal == 1) {
                        cpath.type = static_cast<CameraType>(typeVal);
                    }
                } else if (strncmp(line, "looping ", 8) == 0) {
                    cpath.isLooping = (atoi(line + 8) != 0);
                } else if (strncmp(line, "kf ", 3) == 0) {
                    CameraKeyframe kf = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 50.0f, 2.8f};
                    int parsed = sscanf(line + 3, "%f %f %f %f %f %f %f %f %f %f",
                           &kf.time, &kf.x, &kf.y, &kf.z, &kf.pitch, &kf.heading,
                           &kf.roll, &kf.zoom, &kf.focalLength, &kf.aperture);
                    // Only add keyframe if all 10 values were successfully parsed
                    if (parsed == 10) {
                        cpath.keyframes.push_back(kf);
                    }
                }
            }
            
            if (!cpath.name.empty()) {
                g_customPaths.push_back(cpath);
            }
        }
    }
    
    fclose(file);
    
    char msg[128];
    snprintf(msg, sizeof(msg), "MovieCamera: Loaded %zu custom paths\n", g_customPaths.size());
    XPLMDebugString(msg);
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
    
    // Initialize random seed
    std::srand(static_cast<unsigned>(std::time(nullptr)));
    
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
    
    // Check if using custom camera path
    if (g_usingCustomPath && g_currentCustomPathIndex >= 0 && 
        g_currentCustomPathIndex < static_cast<int>(g_customPaths.size())) {
        
        const CustomCameraPath& cpath = g_customPaths[g_currentCustomPathIndex];
        if (cpath.keyframes.size() >= 2) {
            float pathTime = g_customPathTime;
            float totalDuration = cpath.getTotalDuration();
            
            // Handle looping
            if (cpath.isLooping && totalDuration > 0.0f) {
                pathTime = std::fmod(pathTime, totalDuration);
            }
            
            // Find the two keyframes to interpolate between
            size_t kfIndex = 0;
            for (size_t i = 0; i < cpath.keyframes.size() - 1; ++i) {
                if (pathTime >= cpath.keyframes[i].time && pathTime < cpath.keyframes[i + 1].time) {
                    kfIndex = i;
                    break;
                }
                if (i == cpath.keyframes.size() - 2) {
                    kfIndex = i;
                }
            }
            
            const CameraKeyframe& kf1 = cpath.keyframes[kfIndex];
            const CameraKeyframe& kf2 = cpath.keyframes[(std::min)(kfIndex + 1, cpath.keyframes.size() - 1)];
            
            // Calculate interpolation factor
            float segmentDuration = kf2.time - kf1.time;
            float t = (segmentDuration > 0.001f) ? (pathTime - kf1.time) / segmentDuration : 0.0f;
            t = (std::max)(0.0f, (std::min)(1.0f, t));
            
            // Interpolate keyframes with smooth easing
            CameraKeyframe interpolated = InterpolateKeyframes(kf1, kf2, t);
            
            // Transform to world coordinates
            float rad = acfHeading * PI / 180.0f;
            float cosH = std::cos(rad);
            float sinH = std::sin(rad);
            
            outCameraPosition->x = acfX + interpolated.x * cosH - interpolated.z * sinH;
            outCameraPosition->y = acfY + interpolated.y;
            outCameraPosition->z = acfZ + interpolated.x * sinH + interpolated.z * cosH;
            outCameraPosition->pitch = interpolated.pitch;
            outCameraPosition->heading = acfHeading + interpolated.heading;
            outCameraPosition->roll = interpolated.roll;
            outCameraPosition->zoom = interpolated.zoom;
            
            return 1;
        }
    }
    
    if (g_inTransition) {
        // Smooth transition between shots using ease-in-out
        float t = EaseInOutCubic(g_transitionProgress);
        
        outCameraPosition->x = Lerp(g_startPos.x, g_targetPos.x, t);
        outCameraPosition->y = Lerp(g_startPos.y, g_targetPos.y, t);
        outCameraPosition->z = Lerp(g_startPos.z, g_targetPos.z, t);
        outCameraPosition->pitch = Lerp(g_startPos.pitch, g_targetPos.pitch, t);
        outCameraPosition->heading = LerpAngle(g_startPos.heading, g_targetPos.heading, t);
        outCameraPosition->roll = Lerp(g_startPos.roll, g_targetPos.roll, t);
        outCameraPosition->zoom = Lerp(g_startPos.zoom, g_targetPos.zoom, t);
    } else {
        // Apply shot with SMOOTH cinematic drift using sinusoidal easing
        float rad = acfHeading * PI / 180.0f;
        float cosH = std::cos(rad);
        float sinH = std::sin(rad);
        
        // Calculate drift offset using smooth sinusoidal motion instead of linear
        float driftTime = g_shotElapsedTime;
        float driftFrequency = 0.1f;  // Slow frequency for gentle motion
        
        // Position drift with smooth oscillation (creates floating effect)
        float driftedX = SmoothDrift(g_currentShot.x, g_currentShot.driftX * g_currentShot.duration, driftTime, driftFrequency);
        float driftedY = SmoothDrift(g_currentShot.y, g_currentShot.driftY * g_currentShot.duration, driftTime, driftFrequency * 0.8f);
        float driftedZ = SmoothDrift(g_currentShot.z, g_currentShot.driftZ * g_currentShot.duration, driftTime, driftFrequency * 1.1f);
        
        // Rotation drift with different frequencies for organic feel
        float driftedPitch = SmoothDrift(g_currentShot.pitch, g_currentShot.driftPitch * g_currentShot.duration, driftTime, driftFrequency * 0.9f);
        float driftedHeading = SmoothDrift(g_currentShot.heading, g_currentShot.driftHeading * g_currentShot.duration, driftTime, driftFrequency * 0.7f);
        float driftedRoll = SmoothDrift(g_currentShot.roll, g_currentShot.driftRoll * g_currentShot.duration, driftTime, driftFrequency * 1.2f);
        
        // Zoom drift with smooth breathing effect
        float driftedZoom = SmoothDrift(g_currentShot.zoom, g_currentShot.driftZoom * g_currentShot.duration, driftTime, driftFrequency * 0.5f);
        
        // Transform position offset from aircraft-relative to world coordinates
        outCameraPosition->x = acfX + driftedX * cosH - driftedZ * sinH;
        outCameraPosition->y = acfY + driftedY;
        outCameraPosition->z = acfZ + driftedX * sinH + driftedZ * cosH;
        outCameraPosition->pitch = driftedPitch;
        outCameraPosition->heading = acfHeading + driftedHeading;
        outCameraPosition->roll = driftedRoll;
        outCameraPosition->zoom = driftedZoom;
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
        // Update custom path time if using custom path
        if (g_usingCustomPath && g_currentCustomPathIndex >= 0 && 
            g_currentCustomPathIndex < static_cast<int>(g_customPaths.size())) {
            
            g_customPathTime += inElapsedSinceLastCall;
            
            const CustomCameraPath& cpath = g_customPaths[g_currentCustomPathIndex];
            float totalDuration = cpath.getTotalDuration();
            
            // Check if path has finished (non-looping)
            if (!cpath.isLooping && g_customPathTime >= totalDuration) {
                g_usingCustomPath = false;
                g_currentCustomPathIndex = -1;
                // Continue with normal shot rotation
            }
        } else if (g_inTransition) {
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
 * Draw callback to visualize camera trajectory in 3D space
 */
static int DrawTrajectoryCallback(XPLMDrawingPhase inPhase, int inIsBefore, void* inRefcon) {
    (void)inPhase;
    (void)inIsBefore;
    (void)inRefcon;
    
    // Only draw if trajectory preview is enabled and we have a path being edited
    if (!g_showTrajectoryPreview || !g_pathEditorOpen || g_editingPath.keyframes.size() < 2) {
        return 1;
    }
    
    // Get aircraft position
    float acfX = XPLMGetDataf(g_drLocalX);
    float acfY = XPLMGetDataf(g_drLocalY);
    float acfZ = XPLMGetDataf(g_drLocalZ);
    float acfHeading = XPLMGetDataf(g_drHeading);
    
    float rad = acfHeading * PI / 180.0f;
    float cosH = std::cos(rad);
    float sinH = std::sin(rad);
    
    // Set up OpenGL state for drawing lines
    XPLMSetGraphicsState(0, 0, 0, 0, 1, 1, 0);
    
    // Draw trajectory line (green color)
    glColor4f(0.0f, 1.0f, 0.3f, 0.8f);
    glLineWidth(3.0f);
    glBegin(GL_LINE_STRIP);
    
    // Sample trajectory at regular intervals
    float totalDuration = g_editingPath.getTotalDuration();
    int numSamples = static_cast<int>(totalDuration * 10.0f);  // 10 samples per second
    if (numSamples < 20) numSamples = 20;
    if (numSamples > 200) numSamples = 200;
    
    for (int i = 0; i <= numSamples; ++i) {
        float t = static_cast<float>(i) / static_cast<float>(numSamples);
        float pathTime = t * totalDuration;
        
        // Find keyframes to interpolate
        size_t kfIndex = 0;
        for (size_t j = 0; j < g_editingPath.keyframes.size() - 1; ++j) {
            if (pathTime >= g_editingPath.keyframes[j].time && 
                pathTime < g_editingPath.keyframes[j + 1].time) {
                kfIndex = j;
                break;
            }
            if (j == g_editingPath.keyframes.size() - 2) {
                kfIndex = j;
            }
        }
        
        const CameraKeyframe& kf1 = g_editingPath.keyframes[kfIndex];
        const CameraKeyframe& kf2 = g_editingPath.keyframes[(std::min)(kfIndex + 1, g_editingPath.keyframes.size() - 1)];
        
        float segDuration = kf2.time - kf1.time;
        float segT = (segDuration > 0.001f) ? (pathTime - kf1.time) / segDuration : 0.0f;
        segT = std::clamp(segT, 0.0f, 1.0f);
        
        // Interpolate position with easing
        float smoothT = EaseInOutSine(segT);
        float posX = Lerp(kf1.x, kf2.x, smoothT);
        float posY = Lerp(kf1.y, kf2.y, smoothT);
        float posZ = Lerp(kf1.z, kf2.z, smoothT);
        
        // Transform to world coordinates
        float worldX = acfX + posX * cosH - posZ * sinH;
        float worldY = acfY + posY;
        float worldZ = acfZ + posX * sinH + posZ * cosH;
        
        glVertex3f(worldX, worldY, worldZ);
    }
    glEnd();
    
    // Draw keyframe points (yellow spheres approximated with points)
    glColor4f(1.0f, 1.0f, 0.0f, 1.0f);
    glPointSize(10.0f);
    glBegin(GL_POINTS);
    for (size_t i = 0; i < g_editingPath.keyframes.size(); ++i) {
        const CameraKeyframe& kf = g_editingPath.keyframes[i];
        float worldX = acfX + kf.x * cosH - kf.z * sinH;
        float worldY = acfY + kf.y;
        float worldZ = acfZ + kf.x * sinH + kf.z * cosH;
        glVertex3f(worldX, worldY, worldZ);
    }
    glEnd();
    
    // Highlight selected keyframe (red)
    if (g_editingKeyframeIndex >= 0 && g_editingKeyframeIndex < static_cast<int>(g_editingPath.keyframes.size())) {
        const CameraKeyframe& kf = g_editingPath.keyframes[g_editingKeyframeIndex];
        float worldX = acfX + kf.x * cosH - kf.z * sinH;
        float worldY = acfY + kf.y;
        float worldZ = acfZ + kf.x * sinH + kf.z * cosH;
        
        glColor4f(1.0f, 0.2f, 0.2f, 1.0f);
        glPointSize(15.0f);
        glBegin(GL_POINTS);
        glVertex3f(worldX, worldY, worldZ);
        glEnd();
    }
    
    glLineWidth(1.0f);
    glPointSize(1.0f);
    
    return 1;
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
    
    // Load custom camera paths
    LoadCustomPaths();
    
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
    
    // Register draw callback for trajectory visualization
    XPLMRegisterDrawCallback(DrawTrajectoryCallback, xplm_Phase_Modern3D, 0, nullptr);
    
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
    
    // Unregister draw callback
    XPLMUnregisterDrawCallback(DrawTrajectoryCallback, xplm_Phase_Modern3D, 0, nullptr);
    
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
