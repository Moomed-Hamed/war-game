#version 330 core

out vec4 frag_color;

uniform vec4 color;

// the instance's albedo multiplies the overlay colour : the collider wires leave it
// white, the skeleton uses it to tint each bone by how deep it sits in the chain
flat in vec3 tint;

void main()
{
   frag_color = vec4(color.rgb * tint, color.a);
}
