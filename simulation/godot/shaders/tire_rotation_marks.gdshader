shader_type spatial;
render_mode diffuse_burley, cull_disabled;
uniform mat4 wheel_inverse;
varying vec3 wheel_p;
void vertex(){wheel_p=(wheel_inverse*MODEL_MATRIX*vec4(VERTEX,1.0)).xyz;}
float stripe(float angle,float center,float width){
 float d=abs(mod(angle-center+3.14159265,6.2831853)-3.14159265);
 float aa=max(fwidth(d),0.003);
 return 1.0-smoothstep(width-aa,width+aa,d);
}
void fragment(){
 float angle=atan(wheel_p.z,wheel_p.y);
 float ring=length(wheel_p.yz);
 float ivory=max(max(stripe(angle,1.12,.065),stripe(angle,2.58,.075)),max(stripe(angle,3.72,.06),stripe(angle,5.05,.07)));
 float amber=stripe(angle,0.0,.095);
 float region=smoothstep(.285,.315,ring);
 float grain=fract(sin(dot(floor(wheel_p*900.0),vec3(12.9898,78.233,37.719)))*43758.5453);
 vec3 rubber=vec3(.035,.032,.028)*(0.94+grain*.12);
 vec3 paint=mix(vec3(.68,.64,.53),vec3(.73,.40,.075),amber);
 ALBEDO=mix(rubber,paint,max(ivory,amber)*region);
 ROUGHNESS=.9;SPECULAR=.18;
}
