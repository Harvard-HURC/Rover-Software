"""Atmosphere patches for a copy of gz-rendering's ogre2 media (GZ_RENDERING_RESOURCE_PATH):
procedural clear sky (replaces the skybox fragment shader), distance haze (Pbs + Terra custom piece),
Terra roughness fix. And lighting presets as world-SDF replacements."""
import math
from pathlib import Path

HERE = Path(__file__).resolve().parent
MEDIA = HERE / "gzr" / "ogre2" / "media"


def srgb_lin(c):
    c = c / 255.0
    return c / 12.92 if c <= 0.04045 else ((c + 0.055) / 1.055) ** 2.4


def lin3(rgb):
    return tuple(round(srgb_lin(v), 4) for v in rgb)


def sun_dir(el_deg, az_deg):
    """Unit vector towards the sun (ENU) from elevation and azimuth (clockwise from north)."""
    el, az = math.radians(el_deg), math.radians(az_deg)
    return (math.sin(az) * math.cos(el), math.cos(az) * math.cos(el), math.sin(el))


SKY_FS = """#include <metal_stdlib>
using namespace metal;
struct PS_INPUT {{ float3 cameraDir; }};
// ROVER PATCH: procedural clear desert sky (gz world frame: x east, y north, z up), linear output.
fragment float4 main_metal( PS_INPUT inPs [[stage_in]], texturecube<float> skyCubemap [[texture(0)]],
                            sampler samplerState [[sampler(0)]] )
{{
  float3 d = normalize( inPs.cameraDir );
  float3 sunDir = normalize( float3( {sx}, {sy}, {sz} ) );
  float mu = d.z;
  float s = max( dot( d, sunDir ), 0.0 );
  float3 zenith = float3( {zr}, {zg}, {zb} );
  float3 horizon = float3( {hr}, {hg}, {hb} );
  float h = pow( 1.0 - clamp( mu, 0.0, 1.0 ), {hexp} );
  float3 c = mix( zenith, horizon, h );
  // Mie glow around the sun, stronger near the horizon; the sun disk (0.27 deg radius) saturates.
  c += float3( 1.0, 0.92, 0.80 ) * ( 0.10 * pow( s, 8.0 ) + 0.35 * pow( s, 200.0 ) ) * ( 0.6 + 0.4 * h );
  c += float3( 30.0 ) * smoothstep( 0.999985, 0.999992, s );
  // Below the horizon: a darker haze band (normally hidden by terrain).
  if( mu < 0.0 ) c = mix( horizon, horizon * float3( {gr}, {gg}, {gb} ), clamp( -mu * 8.0, 0.0, 1.0 ) );
  return float4( c, 1.0 );
}}
"""

SKY_FS_GLSL = """#version ogre_glsl_ver_330
vulkan_layout( ogre_t0 ) uniform textureCube skyCubemap;
vulkan( layout( ogre_s0 ) uniform sampler texSampler );
vulkan_layout( location = 0 ) in block {{ vec3 cameraDir; }} inPs;
vulkan_layout( location = 0 ) out vec3 fragColour;
// ROVER PATCH: procedural clear desert sky, see skybox_fs.metal (UNTESTED: written for Linux/GL, only Metal was run)
void main()
{{
  vec3 d = normalize( inPs.cameraDir );
  vec3 sunDir = normalize( vec3( {sx}, {sy}, {sz} ) );
  float mu = d.z;
  float s = max( dot( d, sunDir ), 0.0 );
  vec3 zenith = vec3( {zr}, {zg}, {zb} );
  vec3 horizon = vec3( {hr}, {hg}, {hb} );
  float h = pow( 1.0 - clamp( mu, 0.0, 1.0 ), {hexp} );
  vec3 c = mix( zenith, horizon, h );
  c += vec3( 1.0, 0.92, 0.80 ) * ( 0.10 * pow( s, 8.0 ) + 0.35 * pow( s, 200.0 ) ) * ( 0.6 + 0.4 * h );
  c += vec3( 30.0 ) * smoothstep( 0.999985, 0.999992, s );
  if( mu < 0.0 ) c = mix( horizon, horizon * vec3( {gr}, {gg}, {gb} ), clamp( -mu * 8.0, 0.0, 1.0 ) );
  fragColour = c;
}}
"""

HAZE = """// ROVER PATCH: distance haze (aerial perspective) for Pbs and Terra
@piece( custom_ps_posExecution_haze )
@property( !hlms_shadowcaster && !hlms_prepass && !hlms_render_depth_only && (hlms_normal || hlms_qtangent || detail_maps_diffuse || detail_maps_normal) )
	{{
		float hazeF = {fmax} * ( 1.0 - exp( -length( inPs.pos ) * {beta} ) );
		outPs_colour0.xyz = lerp( outPs_colour0.xyz, float3( {r}, {g}, {b} ), hazeF );
	}}
@end
@end
"""


def write_atmosphere(sun, zenith_srgb=(58, 110, 190), horizon_srgb=(190, 204, 222), hexp=4.0, beta=4.0e-5,
                     haze_srgb=None, fmax=0.85, ground=(0.75, 0.72, 0.68), media=MEDIA):
    z, h = lin3(zenith_srgb), lin3(horizon_srgb)
    hz = lin3(haze_srgb) if haze_srgb else h
    args = dict(sx=sun[0], sy=sun[1], sz=sun[2], zr=z[0], zg=z[1], zb=z[2], hr=h[0], hg=h[1], hb=h[2], hexp=hexp,
                gr=ground[0], gg=ground[1], gb=ground[2])
    (media / "materials/programs/Metal/skybox_fs.metal").write_text(SKY_FS.format(**args))
    (media / "materials/programs/GLSL/skybox_fs.glsl").write_text(SKY_FS_GLSL.format(**args))
    # Haze works on linear colour before the sRGB write (hw gamma), like the sky.
    (media / "Hlms/Gz/Pbs/900.RoverHaze_piece_ps.any").write_text(
        HAZE.format(beta=f"{beta:.3e}", r=hz[0], g=hz[1], b=hz[2], fmax=fmax))


def lighting(sun_el, sun_az, sun_rgb=(1.0, 0.96, 0.9), intensity=1.0, ambient=(0.5, 0.48, 0.46),
             background=(0.62, 0.74, 0.9), specular=(0.3, 0.3, 0.3)):
    """World-SDF replacements for the scene ambient/background and the sun."""
    s = sun_dir(sun_el, sun_az)
    d = tuple(-v for v in s)
    f = lambda t: " ".join(f"{v:.4g}" for v in t)  # noqa: E731
    return [
        (r"<ambient>[^<]*</ambient>", f"<ambient>{f(ambient)} 1</ambient>"),
        (r"<background>[^<]*</background>", f"<background>{f(background)} 1</background>"),
        (r'(<light type="directional" name="sun">.*?)<diffuse>[^<]*</diffuse>',
         rf"\g<1><diffuse>{f(sun_rgb)} 1</diffuse><intensity>{intensity}</intensity>"),
        (r'(<light type="directional" name="sun">.*?)<specular>[^<]*</specular>',
         rf"\g<1><specular>{f(specular)} 1</specular>"),
        (r'(<light type="directional" name="sun">.*?)<direction>[^<]*</direction>',
         rf"\g<1><direction>{f(d)}</direction>"),
    ], s
