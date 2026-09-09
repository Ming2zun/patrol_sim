shader_type spatial;
render_mode cull_disabled,diffuse_burley;
uniform sampler2D leaf_tex : source_color,filter_linear_mipmap_anisotropic,repeat_enable;
uniform vec4 tint : source_color=vec4(1.0);
uniform bool textured=false;
varying vec3 wn;varying vec3 wp;
void vertex(){wn=normalize(MODEL_NORMAL_MATRIX*NORMAL);wp=(MODEL_MATRIX*vec4(VERTEX,1)).xyz;}
void fragment(){
 vec4 c=tint;if(textured){c*=texture(leaf_tex,UV);}if(c.a<0.4){discard;}
 float upward=smoothstep(-0.12,0.45,wn.y);
 float variation=0.86+0.14*sin(wp.x*9.1+wp.z*7.8+wp.y*2.3);
 float snow=mix(0.15,0.92,upward)*variation;
 ALBEDO=mix(c.rgb,vec3(0.78,0.83,0.88),snow);ROUGHNESS=0.93;SPECULAR=0.2;
}
