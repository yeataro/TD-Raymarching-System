// A flat rectangle full of camera
out Vertex
{
	flat int cameraIndex;
	vec2 ViewUV;
	//mat4 WorldInvMat;
} oVert;

void main() 
{
	int cameraIndex = TDCameraIndex();
	oVert.cameraIndex = cameraIndex;
	oVert.ViewUV = uv[0].xy;
	//oVert.WorldInvMat = uTDMats[cameraIndex].worldInverse;
	gl_Position = vec4(P*2,1);
}


