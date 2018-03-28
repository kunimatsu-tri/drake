godot_srcs = glob(["thirdparty/misc/*.h",
                   "thirdparty/misc/*.c",
                   "thirdparty/misc/*.cpp",

                   "thirdparty/minizip/*.h",
                   "thirdparty/minizip/*.c",

                   "thirdparty/zstd/zstd.h",
                   "thirdparty/zstd/**/*.h",
                   "thirdparty/zstd/**/*.c",
                   "thirdparty/zstd/**/*.cpp",

                   "thirdparty/glad/*.c",
                   "thirdparty/glad/**/*.h",

                   "core/*.cpp",
                   "core/**/*.cpp",

                   "servers/*.cpp",
                   "servers/**/*.cpp",

                   "scene/*.cpp",
                   "scene/**/*.cpp",

                   "drivers/*.cpp",
                   "drivers/unix/dir_access_unix.cpp",
                   "drivers/unix/file_access_unix.cpp",
                   "drivers/gles3/*.cpp",
                   "drivers/gl_context/*.cpp",
                   "drivers/png/*.h",
                   "drivers/png/*.cpp",

                   "editor/*.h",
                   "editor/**/*.h",
                   "platform/**/**/*.h",
                   "platform/**/globals/global_defaults.cpp"
])

godot_hdrs = glob([
                  "core/*.h",
                  "core/**/*.h",
                  "core/**/*.inc",
                  "servers/*.h",
                  "servers/**/*.h",
                  "scene/*.h",
                  "scene/**/*.h",
                  "scene/**/**/*.inc",
                  "drivers/*.h",
                  "drivers/unix/*.h",
                  "drivers/gles3/*.h",
                  "drivers/gles3/**/*.h",
                  "drivers/gl_context/*.h",
])

cc_library(
    name = "godot",
    srcs = godot_srcs ,
    hdrs = godot_hdrs,
    includes = ["core", "core/os", "core/io", "core/math", "core/bind", "core/helper",
                "drivers", "drivers/gles3",
                "platform/x11",
                "thirdparty/zstd/common", "thirdparty/zstd", "thirdparty/glad", "."],
    defines = ["GLAD_ENABLED", "GLES_OVER_GL", "ZSTD_STATIC_LINKING_ONLY", "UNIX_ENABLED"],
    deps = [],
    copts = [],
    visibility = ["//visibility:public"],
)

