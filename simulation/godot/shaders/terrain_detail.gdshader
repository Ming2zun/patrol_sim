shader_type spatial;
render_mode diffuse_burley;
uniform sampler2D macro_color : source_color, filter_linear_mipmap_anisotropic;
uniform sampler2D soil_color : source_color, filter_linear_mipmap_anisotropic, repeat_enable;
uniform sampler2D soil_normal : hint_normal, filter_linear_mipmap_anisotropic, repeat_enable;
uniform sampler2D rock_color : source_color, filter_linear_mipmap_anisotropic, repeat_enable;
uniform sampler2D rock_normal : hint_normal, filter_linear_mipmap_anisotropic, repeat_enable;
uniform sampler2D snow_color : source_color, filter_linear_mipmap_anisotropic, repeat_enable;
uniform sampler2D snow_normal : hint_normal, filter_linear_mipmap_anisotropic, repeat_enable;
varying vec3 wp;
varying vec3 wn;
void vertex(){ wp=(MODEL_MATRIX*vec4(VERTEX,1.0)).xyz; wn=normalize(MODEL_NORMAL_MATRIX*NORMAL); }
vec3 tri(sampler2D tex,vec3 p,vec3 w){return texture(tex,p.yz).rgb*w.x+texture(tex,p.xz).rgb*w.y+texture(tex,p.xy).rgb*w.z;}
vec3 tri_normal(sampler2D tex,vec3 p,vec3 w,vec3 n){
 vec3 a=texture(tex,p.yz).rgb*2.0-1.0; vec3 b=texture(tex,p.xz).rgb*2.0-1.0; vec3 c=texture(tex,p.xy).rgb*2.0-1.0;
 return normalize(vec3(a.z*sign(n.x),a.x,a.y)*w.x+vec3(b.x,b.z*sign(n.y),b.y)*w.y+vec3(c.x,c.y,c.z*sign(n.z))*w.z);
}
void fragment(){
 vec3 n=normalize(wn);vec3 w=pow(abs(n),vec3(4.0));w/=max(dot(w,vec3(1)),0.001);
 vec3 base=texture(macro_color,UV).rgb;
 float white=smoothstep(0.40,0.78,max(base.r,max(base.g,base.b)))*(1.0-smoothstep(0.04,0.16,abs(base.g-base.r)));
 float rock=smoothstep(0.22,0.67,1.0-abs(n.y));
 vec3 soil=tri(soil_color,wp*0.55,w);vec3 stone=tri(rock_color,wp*0.22,w);vec3 snow=tri(snow_color,wp*0.24,w);
 float macro=0.84+0.16*sin(wp.x*0.043+sin(wp.z*0.069))*sin(wp.z*0.027);
 float far_blend=smoothstep(25.0,110.0,length(wp-CAMERA_POSITION_WORLD));
 float grass_patch=smoothstep(-0.4,0.65,sin(wp.x*0.17+sin(wp.z*0.25))*sin(wp.z*0.14));
 vec3 earthy=soil*mix(vec3(0.92,0.95,0.87),vec3(0.64,0.74,0.43),grass_patch);
 vec3 ground=mix(earthy,mix(earthy,base*0.85,0.48),far_blend)*macro;
 ALBEDO=mix(mix(ground,stone*0.85,rock),snow*0.88,white*(1.0-rock*0.7));
 vec3 detail=mix(tri_normal(soil_normal,wp*0.55,w,n),tri_normal(rock_normal,wp*0.22,w,n),rock);
 detail=mix(detail,tri_normal(snow_normal,wp*0.24,w,n),white);
 NORMAL=normalize((VIEW_MATRIX*vec4(normalize(mix(n,detail,0.55)),0)).xyz);
 ROUGHNESS=0.93;SPECULAR=0.23;
}
