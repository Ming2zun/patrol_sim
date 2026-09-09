shader_type spatial;
render_mode diffuse_burley;
uniform sampler2D gravel : source_color, filter_linear_mipmap_anisotropic, repeat_enable;
uniform sampler2D bumps : hint_normal, filter_linear_mipmap_anisotropic, repeat_enable;
uniform sampler2D grit : source_color, filter_linear_mipmap_anisotropic, repeat_enable;
varying vec3 wp;varying vec3 wn;
void vertex(){wp=(MODEL_MATRIX*vec4(VERTEX,1)).xyz;wn=normalize(MODEL_NORMAL_MATRIX*NORMAL);}
void fragment(){
 float edge_noise=0.5+0.5*sin(wp.x*32.0+sin(wp.z*27.0))*sin(wp.z*43.0);
 if(abs(UV.x)>1.85 && COLOR.r<0.18 && edge_noise>(COLOR.r/0.18)){discard;}
 vec2 uv=wp.xz*0.6;vec3 a=texture(gravel,uv).rgb;vec3 b=texture(grit,wp.xz*0.48).rgb;
 float lightness=dot(a,vec3(0.299,0.587,0.114));
 vec3 dust=mix(a,vec3(lightness)*vec3(1.02,1.0,0.91),0.72);
 float marks=sin(UV.y*1.3)*0.027+sin(UV.y*0.37)*0.022;
 float track=(1.0-smoothstep(0.20,0.48,abs(abs(UV.x+marks)-0.95)));
 float broken=0.84+0.16*sin(wp.x*2.3+wp.z*0.73)*sin(wp.z*1.7);
 track*=broken;
 vec3 loose=mix(dust,b*1.4,0.32)*0.78;
 ALBEDO=mix(loose,dust*1.23,track);
 ALBEDO*=0.94+0.06*sin(wp.x*0.21+wp.z*0.12);
 vec3 normal_tex=texture(bumps,uv).rgb*2.0-1.0;
 vec3 n=normalize(wn+vec3(normal_tex.x,0,normal_tex.y)*mix(0.85,0.22,track));
 NORMAL=normalize((VIEW_MATRIX*vec4(n,0)).xyz);
 ROUGHNESS=mix(0.98,0.85,track);SPECULAR=0.2;
}
