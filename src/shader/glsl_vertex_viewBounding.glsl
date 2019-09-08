out Vertex
{
	flat int cameraIndex;
	noperspective vec2 ViewUV;
	//mat4 WorldInvMat;
} oVert;

void main() 
{
	vec4 worldSpacePos = TDDeform(P);
	vec4 ViewPos = TDWorldToProj(worldSpacePos);
	int cameraIndex = TDCameraIndex();
	oVert.cameraIndex = cameraIndex;
	oVert.ViewUV = (ViewPos.xy/ViewPos.w+1)/2;
	//oVert.WorldInvMat = uTDMats[cameraIndex].worldInverse;
	gl_Position = ViewPos;
}
