shader_type spatial;
render_mode diffuse_burley;
uniform sampler2D road_mask : filter_linear_mipmap, repeat_disable;
uniform vec4 mask_bounds;
uniform bool road_overlay = false;
uniform sampler2D soil_color : source_color, filter_linear_mipmap_anisotropic, repeat_enable;
uniform sampler2D soil_normal : hint_normal, filter_linear_mipmap_anisotropic, repeat_enable;
uniform sampler2D grass_color : source_color, filter_linear_mipmap_anisotropic, repeat_enable;
uniform sampler2D road_color : source_color, filter_linear_mipmap_anisotropic, repeat_enable;
uniform sampler2D road_normal : hint_normal, filter_linear_mipmap_anisotropic, repeat_enable;
uniform sampler2D rock_color : source_color, filter_linear_mipmap_anisotropic, repeat_enable;
uniform sampler2D rock_normal : hint_normal, filter_linear_mipmap_anisotropic, repeat_enable;
uniform sampler2D snow_color : source_color, filter_linear_mipmap_anisotropic, repeat_enable;
uniform sampler2D snow_normal : hint_normal, filter_linear_mipmap_anisotropic, repeat_enable;
varying vec3 wp;varying vec3 wn;varying float recess;
void vertex(){recess=1.0-COLOR.r;wp=(MODEL_MATRIX*vec4(VERTEX,1)).xyz;wn=normalize(MODEL_NORMAL_MATRIX*NORMAL);}
vec3 tri(sampler2D tex,vec3 p,vec3 w){return texture(tex,p.yz).rgb*w.x+texture(tex,p.xz).rgb*w.y+texture(tex,p.xy).rgb*w.z;}
void fragment(){
 vec3 n=normalize(wn);vec3 weights=pow(abs(n),vec3(4));weights/=max(dot(weights,vec3(1)),0.001);
 vec2 map_uv=(wp.xz-mask_bounds.xy)/mask_bounds.zw;
 vec3 mask=texture(road_mask,map_uv).rgb;
 float at_height=1.0-smoothstep(0.4,2.0,abs(wp.y-(mask.b*140.0-5.0)));
 float road=mask.r*at_height;
 // The soil/grass terrain carries the outer texture blend. Fade the thin road
 // overlay inside that region so polygon boundaries never form a hard edge.
 float coverage=smoothstep(0.55,0.95,road);
 float dither=fract(sin(dot(floor(wp.xz*90.0),vec2(12.9898,78.233)))*43758.5453);
 if(road_overlay && coverage<dither){discard;}
 float tracks=mask.g*at_height;
 vec3 soil=texture(soil_color,wp.xz*0.58).rgb;
 vec3 grass=texture(grass_color,wp.xz*0.08).rgb;
 float patch=0.5+0.5*sin(wp.x*0.16+sin(wp.z*0.21))*sin(wp.z*0.18);
 vec3 earth=mix(soil*vec3(0.92,0.97,0.87),grass*0.8,patch*0.45);
 vec3 detail=texture(soil_normal,wp.xz*0.58).rgb*2.0-1.0;
 vec3 world_detail=normalize(n+vec3(detail.x,0,detail.y)*0.65);
 float slope=smoothstep(0.24,0.64,1.0-n.y);
 if(slope>0.01){
  vec3 cliff=tri(rock_color,wp*0.23,weights)*0.7;
  earth=mix(earth,cliff,slope);
 }
 float snow=smoothstep(78.0,110.0,wp.y)*(1.0-slope*0.7);
 if(snow>0.01){earth=mix(earth,texture(snow_color,wp.xz*0.22).rgb*0.82,snow);}
 vec3 gravel=texture(road_color,wp.xz*0.63).rgb;
 float gray=dot(gravel,vec3(0.299,0.587,0.114));
 gravel=mix(gravel,vec3(gray)*vec3(1.05,1.03,0.98),0.5)*0.91;
 // Compaction changes texture and roughness subtly; no bright painted stripes.
 gravel*=mix(0.95,1.05,tracks);
 float winter=smoothstep(78.0,104.0,wp.y);
 vec3 snow_surface=texture(snow_color,wp.xz*0.22).rgb;
 vec3 packed_snow=snow_surface*mix(vec3(0.78,0.82,0.86),vec3(0.66,0.73,0.80),tracks*0.75);
 packed_snow=mix(packed_snow,gravel,0.055);
 gravel=mix(gravel,packed_snow,winter);
 gravel*=mix(1.0,0.73,clamp(recess,0.0,1.0));
 ALBEDO=mix(earth,gravel,road);
 vec3 road_bump=texture(road_normal,wp.xz*0.63).rgb*2.0-1.0;
 vec3 road_n=normalize(n+vec3(road_bump.x,0,road_bump.y)*mix(0.75,0.38,tracks));
 vec3 snow_bump=texture(snow_normal,wp.xz*0.22).rgb*2.0-1.0;
 vec3 snow_n=normalize(n+vec3(snow_bump.x,0,snow_bump.y)*0.34);
 world_detail=normalize(mix(world_detail,snow_n,snow));
 road_n=normalize(mix(road_n,snow_n,winter*0.85));
 vec3 final_n=normalize(mix(world_detail,road_n,road));
 NORMAL=normalize((VIEW_MATRIX*vec4(final_n,0)).xyz);
 ROUGHNESS=mix(0.96,mix(0.90,0.67,winter),tracks*road);SPECULAR=0.18;
}
