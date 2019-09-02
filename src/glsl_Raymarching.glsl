// Code adapted from http://jamie-wong.com/2016/07/15/ray-marching-signed-distance-functions/
// Inspired by exsstas https://github.com/exsstas/Raymarching-in-TD
// TD-Raymarching-System by Yea Chen  https://github.com/yeataro/

out vec4 fragColor;

// raymarcher parameters
uniform int uSteps;						// the max steps before giving up
uniform float uMinDist;					// the starting distance away from the eye
uniform float uMaxDist;					// the max distance away from the eye to march before giving up

// scene parameters
uniform int uNum;						// number of primitives in array

// Primitive's "Type" & combinations("Uni", "K")
uniform samplerBuffer SDFsPara;

// Primitive's matrix
uniform samplerBuffer Mat0;
uniform samplerBuffer Mat1;
uniform samplerBuffer Mat2;
uniform samplerBuffer Mat3;

// Colors
uniform float uAlpha;
uniform vec4 uDiffuseColor;
uniform vec4 uAmbientColor;
uniform vec3 uSpecularColor;
uniform float uShininess;
uniform float uShadowStrength;
uniform vec3 uShadowColor;

in Vertex
{
	flat int cameraIndex;
	vec2 texCoord0;
} iVert;


//------------------------------------------------------------
// SDF functions - add below all primitives and blending functions you need
// http://iquilezles.org/www/articles/distfunctions/distfunctions.htm
//------------------------------------------------------------

float cubeSDF(vec3 p) {
    // If d.x < 0, then -1 < p.x < 1, and same logic applies to p.y, p.z
    // So if all components of d are negative, then p is inside the unit cube
    vec3 d = abs(p) - vec3(1.0, 1.0, 1.0);
    
    // Assuming p is inside the cube, how far is it from the surface?
    // Result will be negative or zero.
    float insideDistance = min(max(d.x, max(d.y, d.z)), 0.0);
    
    // Assuming p is outside the cube, how far is it from the surface?
    // Result will be positive or zero.
    float outsideDistance = length(max(d, 0.0));
    
    return insideDistance + outsideDistance;
}


float sphereDf(vec3 p)
{
    return length(p)-1;
}

float sdPlane( vec3 p)
{
   vec4 n = vec4(.5);
    // n must be normalized
    return dot( p, n.xyz ) + n.w;
}

float sdTorus( vec3 p,vec2 t = vec2(1,0.5))
{
  vec2 q = vec2(length(p.xz)-t.x,p.y);
  return length(q)-t.y;
}

#define Type0 sphereDf(Pos); 
#define Type1 cubeSDF(Pos); 
#define Type2 sdPlane(Pos); 
#define Type3 sdTorus(Pos); 



/* 
# Primitive combinations
http://iquilezles.org/www/articles/distfunctions/distfunctions.htm
0 = Union
1 = Subtraction
2 = Intersection
3 = Smooth Union
4 = Smooth Subtraction
5 = Smooth Intersection
*/

#define Combie0 opUnion(scene, sph)
#define Combie1 opSubtraction(sph, scene)
#define Combie2 opIntersection(scene, sph)
#define Combie3 opSmoothUnion(scene, sph, K)
#define Combie4 opSmoothSubtraction(sph, scene, K)
#define Combie5 opSmoothIntersection(scene, sph, K)

/// Union, Subtraction, Intersection - exact, bound, bound
float opUnion( float d1, float d2 ) { return min(d1,d2); }

float opSubtraction( float d1, float d2 ) { return max(-d1,d2); }

float opIntersection( float d1, float d2 ) { return max(d1,d2); }

// Smooth Union, Subtraction and Intersection - exact, bound, bound
float opSmoothUnion( float d1, float d2, float k =1) {
    float h = clamp( 0.5 + 0.5*(d2-d1)/k, 0.0, 1.0 );
    return mix( d2, d1, h ) - k*h*(1.0-h); }

float opSmoothSubtraction( float d1, float d2, float k =1 ) {
    float h = clamp( 0.5 - 0.5*(d2+d1)/k, 0.0, 1.0 );
    return mix( d2, -d1, h ) + k*h*(1.0-h); }

float opSmoothIntersection( float d1, float d2, float k =1 ) {
    float h = clamp( 0.5 - 0.5*(d2-d1)/k, 0.0, 1.0 );
    return mix( d2, d1, h ) + k*h*(1.0-h); }

//Get Matrix from Primitives.

mat4 SdfDeform(int i){
    mat4 TheMAT;
    TheMAT[0] = texelFetchBuffer(Mat0, i);
    TheMAT[1] = texelFetchBuffer(Mat1, i);
    TheMAT[2] = texelFetchBuffer(Mat2, i);
    TheMAT[3] = texelFetchBuffer(Mat3, i);
    return TheMAT;
}

//------------------------------------------------------------
// Describe scene here
//------------------------------------------------------------

float sceneSDF(vec3 p)
{
    
    float scene = uMaxDist;  // for empty start

    for (int i = 0; i < uNum; i++) {

        vec4 SDFsParameter = texelFetchBuffer(SDFsPara, i);
        int type = int(SDFsParameter.x);
        int Uni = int(SDFsParameter.y);
        float K =SDFsParameter.z;
        
        mat4 SdfDeform = SdfDeform(i);
        vec4 Posv4 = vec4(p,1.);
        Posv4 = SdfDeform*Posv4;
        vec3 Pos = Posv4.xyz;
        float sph;
        
        

        switch(type){
            default:sph = Type0; break;
            case 0: sph = Type0; break;
            case 1: sph = Type1; break;
            case 2: sph = Type2; break;
            case 3: sph = Type3; break;
        }

        switch(Uni){
            default:scene = Combie0 ; break;
            case 0: scene = Combie0 ; break;
            case 1: scene = Combie1 ; break;
            case 2: scene = Combie2 ; break;
            case 3: scene = Combie3 ; break;
            case 4: scene = Combie4 ; break;
            case 5: scene = Combie5 ; break;
        }

    }



    return scene;
}
//------------------------------------------------------------



//------------------------------------------------------------
// Distance and direction
//------------------------------------------------------------
/**
 * eye: the eye point, acting as the origin of the ray
 * marchingDirection: the normalized direction to march in
 * start: the starting distance away from the eye
 * end: the max distance away from the eye to march before giving up
 */
float shortestDistanceToSurface(vec3 eye, vec3 marchingDirection, float start, float end) {
    float depth = start;
    for (int i = 0; i < uSteps; i++) {
        float dist = sceneSDF(eye + depth * marchingDirection);
        if (dist < start) {
            return depth;
        }
        depth += dist;
        if (depth >= end) {
            return end;
        }
    }
    return end;
}


vec3 rayDirection(vec2 fragCoord) {
    vec2 pos = (fragCoord-0.5)*2;
    vec4 Dir = uTDMats[iVert.cameraIndex].projInverse * vec4(pos,0,-1);
    Dir = normalize(vec4(Dir.xy,-Dir.z,1));
    vec4 wDir = Dir *uTDMats[iVert.cameraIndex].cam;
    return wDir.xyz;
}


float map(float value, float min1, float max1, float min2, float max2) {
  return min2 + (value - min1) * (max2 - min2) / (max1 - min1);
}

float procDepth(vec3 localPos){
    vec4 frag = uTDMats[iVert.cameraIndex].camProj*vec4(localPos,1);
    frag/=frag.w;
    return (frag.z+1)/2;
}


//------------------------------------------------------------
// Normals
//------------------------------------------------------------
/**
 * Using the gradient of the SDF, estimate the normal on the surface at point p.
 */
vec3 estimateNormal(vec3 p) {
    return normalize(vec3(
        sceneSDF(vec3(p.x + uMinDist, p.y, p.z)) - sceneSDF(vec3(p.x - uMinDist, p.y, p.z)),
        sceneSDF(vec3(p.x, p.y + uMinDist, p.z)) - sceneSDF(vec3(p.x, p.y - uMinDist, p.z)),
        sceneSDF(vec3(p.x, p.y, p.z  + uMinDist)) - sceneSDF(vec3(p.x, p.y, p.z - uMinDist))
    ));
}

//Lighting with TD's way
vec4 TDLightingSDF(vec3 p,vec3 eye){
    vec4 outcol = vec4(0.0, 0.0, 0.0, uAlpha);
	vec3 diffuseSum = vec3(0.0, 0.0, 0.0);
	vec3 specularSum = vec3(0.0, 0.0, 0.0);
    vec3 normal = estimateNormal(p);
    vec3 viewVec = normalize(eye - p);

    for (int i = 0; i < TD_NUM_LIGHTS; i++)
	{
		vec3 diffuseContrib = vec3(0);
		vec3 specularContrib = vec3(0);
		TDLighting(diffuseContrib,
			specularContrib,
			i,
			p,
			normal,
			uShadowStrength, uShadowColor,
			viewVec,
			uShininess);
		diffuseSum += diffuseContrib;
		specularSum += specularContrib;
	}
    // Final Diffuse Contribution
	diffuseSum *= uDiffuseColor.rgb;
	vec3 finalDiffuse = diffuseSum;
	outcol.rgb += finalDiffuse;

	// Final Specular Contribution
	vec3 finalSpecular = vec3(0.0);
	specularSum *= uSpecularColor;
	finalSpecular += specularSum;

	outcol.rgb += finalSpecular;

	// Ambient Light Contribution
	outcol.rgb += vec3(uTDGeneral.ambientColor.rgb * uAmbientColor.rgb);
    return outcol;
}



void main()
{
    TDCheckDiscard();
    vec2 CanvasUV = iVert.texCoord0;

    // setting camera(eye)
    vec3 eye = vec4(uTDMats[iVert.cameraIndex].camInverse*vec4(0,0,0,1)).xyz;
    vec3 worldDir = rayDirection(CanvasUV);

    float dist = shortestDistanceToSurface(eye, worldDir, uMinDist, uMaxDist);
    
    if (dist > uMaxDist - uMinDist) {
        discard;
        //fragColor = vec4(0.0);          // Didn't hit anything return transparent background
        //gl_FragDepth = 1;
        //return;
    }
    
    // The closest point on the surface to the eyepoint along the view ray
    vec3 p = eye + dist * worldDir;         
    float Depth = procDepth(p);

    //TD lighting
    vec4 fColor = TDLightingSDF(p,eye);

    //Output Color
    fColor = TDDither(fColor);
    fColor = TDOutputSwizzle(fColor);
    fragColor = fColor;
    gl_FragDepth = Depth;
    
}