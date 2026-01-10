
///////////DEFAULT SHADER/////////////////

@vs vs_default

layout(binding=0) uniform vs_params {
    mat4 u_model;
	mat4 u_mvp;
};

in vec3 v_pos;
in vec3 v_norm;
in vec2 v_uv;

out vec3 pos;
out vec3 norm;
out vec2 uv;

void main() {
    gl_Position=u_mvp*vec4(v_pos, 1);
    pos=(u_model*vec4(v_pos, 1)).xyz;
    norm=normalize(mat3(u_model)*v_norm);
    uv=v_uv;
}

@end

@fs fs_default

layout(binding=0) uniform texture2D default_tex;
layout(binding=0) uniform sampler default_smp;

layout(binding=1) uniform fs_params {
	vec2 u_tl;
	vec2 u_br;
	vec3 u_view_pos;
	int u_num_lights;
	vec4 u_light_pos[16];
	vec4 u_light_col[16];

};

in vec3 pos;
in vec3 norm;
in vec2 uv;


out vec4 frag_color;

void main() {
	const float amb_mag=1;//0-1
	const vec3 amb_col=vec3(0.03);
	
	const float shininess=64;//32-256
	const float spec_mag=1;//0-1

	const float att_k0=1.0;
	const float att_k1=0.09;
	const float att_k2=0.032;

	//srgb->linear
	//vec3 base_col_srgb=texture(sampler2D(default_tex, default_smp),u_tl + uv * (u_br - u_tl)).rgb;
	vec3 base_col_srgb=texture(sampler2D(default_tex, default_smp), uv).rgb;
	vec3 base_col=pow(base_col_srgb, vec3(2.2));

	vec3 n=normalize(norm);
	vec3 v=normalize(u_view_pos- pos);

	//start with ambient
	vec3 col=amb_col*base_col*amb_mag;
	for(int i=0; i<u_num_lights; i++) {
		vec3 l_pos=u_light_pos[i].xyz;
		vec3 l_col=u_light_col[i].rgb;

		vec3 l=l_pos-pos;
		float dist=length(l);
		l/=dist;

		//diffuse(Lambert)
		float n_dot_l=max(0, dot(n, l));
		vec3 diffuse=l_col*base_col*n_dot_l;

		//blinn-phong specular
		vec3 h=normalize(l+v);//half-vector
		float n_dot_h=max(0, dot(n, h));
		float spec_factor=pow(n_dot_h, shininess);
		vec3 specular=l_col*spec_factor*spec_mag;

		//attenuation (apply equally to all channels)
		float att=1/(att_k0+att_k1*dist+att_k2*dist*dist);

		col+=att*(diffuse+specular);
	}
	
	//linear->srgb
	vec3 col_srgb=pow(col, vec3(1/2.2));
	frag_color=vec4(col_srgb, 1);

}

@end

@program default vs_default fs_default

///////// 2d texture with/without animation //////////////////

@vs vs_texview

in vec2 v_pos;
in vec2 v_uv;

out vec2 uv;

void main()
{
	gl_Position = vec4(v_pos, .5, 1);
	uv.x = v_uv.x;
	uv.y = -v_uv.y;
}

@end

@fs fs_texview

layout(binding = 0) uniform texture2D texview_tex;
layout(binding = 0) uniform sampler texview_smp;

layout(binding = 0) uniform fs_texview_params
{
	vec2 u_tl;
	vec2 u_br;
};

in vec2 uv;

out vec4 frag_color;

void main()
{
	vec4 col = texture(sampler2D(texview_tex, texview_smp), u_tl + uv * (u_br - u_tl));
	frag_color = vec4(col.rgb, 1);
}

@end

@program texview vs_texview fs_texview

//////// LINE RENDERING SHADER////////////////////

@vs vs_line

layout(binding=0) uniform vs_line_params {
	mat4 u_mvp;
};

in vec3 v_pos;
in vec4 v_col;

out vec4 col;

void main()
{
	gl_Position = u_mvp * vec4(v_pos, 1);
	col = v_col;
}

@end

@fs fs_line

in vec4 col;

out vec4 frag_color;

void main()
{
	frag_color = col;
}

@end

@program line vs_line fs_line


