# How to run godot-based RGB-D camera.

```
mkdir ws/sean-godot -p
cd ws/sean-godot
git clone https://github.com/SeanCurtis-TRI/godot.git
cd godot
git checkout patch_render_compiler_warnings
scons platform=x11
```

```
cd ../../
git clone git@github.com:kunimatsu-tri/drake.git
cd drake
git checkout rgbd_renderer_godot
```

The next line gives you some hints about where you should change.
```
git diff 0be42e2c..949c581cf7
```

```
cd systems/sensors
bazel build :rgbd_camera_rendering_example
```

The following should work as-is.
```
bazel run :rgbd_camera_rendering_example -- --sdf_dir=`pwd`/gltf --output_dir=`pwd` --num=37
```

You may want to specify sdf_fixed as well.
```
bazel run :rgbd_camera_rendering_example -- --sdf_dir=<DIR-TO-YOUR-SDF> --sdf_fixed=<YOUR-FIXED-SDF.sdf> --output_dir=<WHEREVER-YOU-LINE> --num=<START-INDEX-FOR-SDF>
```