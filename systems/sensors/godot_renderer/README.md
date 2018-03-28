# godotvis
This project aims to extract just the graphics rendering part of Godot as a standalone application. 
This is the first step to see if we can use Godot as a graphics renderer inside Drake.
The high-level goal is to have a photo-realisitic rendering system to simulate camera images, and we hope Godot might be powerful enough for that.

## INSTALL
- clone Godot 3.0 from its master branch here: https://github.com/godotengine/godot
- cd godot/modules
- git clone git@github.awsinternal.tri.global:duynguyen-ta/godotvis.git  _# ask Duy to give you the access_
- cd ..  # back to godot
- scons platform=\<your_platform\> tools=no verbose=yes -j21  _# where \<your_platform\> could be either x11 or osx_
- Test the app: bin/godotvis.x11.debug.64
