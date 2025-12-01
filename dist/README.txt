MovieCamera - Cinematic Camera Plugin for X-Plane 12
=====================================================

Version: 1.0.0
Author: CCA3370
Website: https://github.com/CCA3370/MovieCamera

DESCRIPTION
-----------
MovieCamera is a cinematic camera plugin for X-Plane 12 that provides 
automatic smooth camera movements with transitions between cockpit and 
external views. Perfect for creating stunning flight videos and screenshots.

FEATURES
--------
- Automatic camera movements with smooth transitions
- 21 predefined camera angles:
  * 9 cockpit views (instruments, panels, windows)
  * 12 external views (various angles around aircraft)
- Three operating modes:
  * Off: Normal X-Plane camera control
  * Manual: User-controlled camera activation
  * Auto: Automatic activation based on flight conditions
- Configurable settings via ImGui interface
- Mouse pause feature - camera pauses when mouse moves

INSTALLATION
------------
1. Extract the MovieCamera folder to:
   X-Plane 12/Resources/plugins/

   Your folder structure should look like:
   X-Plane 12/
     Resources/
       plugins/
         MovieCamera/
           lin_x64/MovieCamera.xpl  (Linux)
           win_x64/MovieCamera.xpl  (Windows)
           mac_x64/MovieCamera.xpl  (macOS)

2. Start X-Plane 12

USAGE
-----
Access the plugin from the menu: Plugins > MovieCamera

Menu Options:
- Auto: Toggle automatic mode (activates based on flight conditions)
- Start: Manually start camera control
- Stop: Stop camera control
- Settings: Open the settings window

Settings:
- Delay (seconds): Time to wait after mouse stops moving before 
  activating camera (default: 60)
- Auto Alt (ft): Altitude threshold above which Auto mode can 
  activate (default: 18000)
- Shot Duration Min/Max: Duration range for each camera shot 
  (default: 3-5 seconds)

Auto Mode Conditions:
- On ground and stationary, OR
- Above configured altitude AND mouse idle for configured delay

REQUIREMENTS
------------
- X-Plane 12
- 64-bit operating system (Windows, Linux, or macOS)

TROUBLESHOOTING
---------------
- If the plugin doesn't appear in the menu, check that the .xpl file 
  is in the correct platform folder (lin_x64, win_x64, or mac_x64)
- Check X-Plane's Log.txt for any error messages

SUPPORT
-------
For bug reports and feature requests, please visit:
https://github.com/CCA3370/MovieCamera/issues

LICENSE
-------
This plugin is released under the GNU General Public License v3 (GPLv3).
See LICENSE.txt for full details.
