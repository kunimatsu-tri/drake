#pragma once

namespace drake {
namespace systems {
namespace sensors {
namespace shaders {

constexpr char kDepthVS[] =
    "//VTK::System::Dec\n"  // always start with this line
    "attribute vec4 vertexMC;\n"
    "attribute vec3 normalMC;\n"
    "uniform mat3 normalMatrix;\n"
    "uniform mat4 MCDCMatrix;\n"
    "uniform mat4 MCVCMatrix;\n"
    "varying vec3 normalVCVSOutput;\n"
    "varying vec4 vertexVCVSOutput;\n"
    "attribute vec2 tcoordMC;\n"
    "varying vec2 tcoordVCVSOutput;\n"
    "void main () {\n"
    "  normalVCVSOutput = normalMatrix * normalMC;\n"
    "  tcoordVCVSOutput = tcoordMC;\n"
    "  vertexVCVSOutput = MCVCMatrix * vertexMC;\n"
    "  gl_Position = MCDCMatrix * vertexMC;\n"
    "}\n";

constexpr char kDepthFS[] =
    "//VTK::System::Dec\n"  // always start with this line
    "//VTK::Output::Dec\n"  // always have this line in your FS
    "varying vec3 normalVCVSOutput;\n"
    "varying vec4 vertexVCVSOutput;\n"
    "varying vec2 tcoordVCVSOutput;\n"
    "out vec4 colorOut;\n"
    "uniform float z_near;\n"
    "uniform float z_far;\n"
    "uniform sampler2D texture_0;\n"
    "void main () {\n"
    "  float z = -vertexVCVSOutput.z;  // In meters.\n"
    "  float random_noise = texture(texture_0, vec2(gl_FragCoord.x / 640., gl_FragCoord.y / 480.)).x;\n"
    // "  float f = pow(dot(normalVCVSOutput, normalize(vertexVCVSOutput.xyz)), 5.) / pow(z, 2.);\n"
    // "  float angle = dot(normalVCVSOutput, normalize(-vertexVCVSOutput.xyz)) / 2. + 0.5;\n"
    // "  float distance = 1. / pow(z, 2.);\n"
    // "  float dnoise = distance * (1. + 0.01 * random_noise);\n"
    // "  float f = angle / pow(z, 2.) + random_noise;\n"
    // "  float z = gl_FragCoord.z; // [0, 1].\n"
    "  float angle = dot(normalVCVSOutput, normalize(-vertexVCVSOutput.xyz));\n"
    "  if (angle + random_noise * 0.2 > 0.3 * min(1.0, z * 2.5 / 5.)) {\n"
    "    float z_noise = z * (1. + 0.01 * random_noise);\n"
    "    float z_norm = (z_noise - z_near) / (z_far - z_near); // From meters to [0, 1].\n"
    "    colorOut = vec4(z_norm, z_norm, z_norm, 1.);\n"
    "  } else {\n"
    "    colorOut = vec4(0, 0, 0, 1.);\n"
    "  }\n"
    "  float z_norm = (z - z_near) / (z_far - z_near); // From meters to [0, 1].\n"
    "  z_norm = clamp(z_norm, 0.f, 1.f);\n"
    "  colorOut = vec4(z_norm, z_norm, z_norm, 1.);\n"
    "}\n";

}  // namespace shaders
}  // namespace sensors
}  // namespace systems
}  // namespace drake
