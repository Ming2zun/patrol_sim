shader_type spatial;
render_mode diffuse_burley;
uniform mat4 vehicle_inverse;
uniform sampler2D dust_tex : source_color,filter_linear_mipmap,repeat_enable;
varying vec3 body_pos;
void vertex(){body_pos=(vehicle_inverse*MODEL_MATRIX*vec4(VERTEX,1)).xyz;}
void fragment(){
 vec3 dust=texture(dust_tex,body_pos.xz*2.1+body_pos.y*0.13).rgb;
 float grain=dot(dust,vec3(0.333));
 float skirt=(1.0-smoothstep(0.52,1.06,body_pos.y))*clamp(0.32+grain*0.4,0.0,0.45);
 ALBEDO=mix(vec3(0.64,0.66,0.66)*(0.98+grain*0.04),vec3(0.18,0.15,0.105),skirt);
 METALLIC=0.12;ROUGHNESS=mix(0.29+grain*0.09,0.74,skirt);
 CLEARCOAT=0.55*(1.0-skirt);CLEARCOAT_ROUGHNESS=0.18;SPECULAR=0.46;
}
