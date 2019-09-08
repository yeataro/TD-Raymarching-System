out Vertex
{
	vec2 texCoord;

} oVert;

void main() {
	// Scale vertex attribute to [0-1] range
	const vec2 madd = vec2(0.5, 0.5);
	oVert.texCoord = P.xy * madd + madd;
	#ifdef _InvY
	oVert.texCoord.y = 1.0 - texCoord.y;
	#endif

	gl_Position = vec4(P.xy, 0.0, 1.0);
}
