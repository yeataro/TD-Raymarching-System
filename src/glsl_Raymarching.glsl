// Code adapted from http://jamie-wong.com/2016/07/15/ray-marching-signed-distance-functions/
// TouchDesigner version by exsstas https://github.com/exsstas/Raymarching-in-TD

out vec4 fragColor;

// raymarcher parameters
uniform int uSteps;						// the max steps before giving up
uniform float uMinDist;					// the starting distance away from the eye
uniform float uMaxDist;					// the max distance away from the eye to march before giving up

// scene parameters
uniform int uNum;						// number of primitives in array
uniform samplerBuffer uPrim;			// textrure buffer name defined on Arrays page of the node
uniform float uSmoothK;					// smooth distance for blending primitives

// Camera and color parameters
uniform vec3 uLight1Pos;				// Light position
uniform vec3 uLight1Col;				// Light color

uniform vec3 uAmbient;
uniform vec3 uDiffuse;
uniform vec3 uSpecular;
uniform float uShine;					// Shininess coefficient

//New Code
uniform samplerBuffer SDFsType;
uniform samplerBuffer SDFsT;
uniform samplerBuffer SDFsR;
uniform samplerBuffer SDFsS;

in Vertex
{
	flat int cameraIndex;
	mat4 WProj;
	mat4 Proj;
	mat4 invProj;
    mat4 worldCam;
    mat4 worldCamInverse;
    mat4 cam;
    mat4 camInverse;
    mat4 camProj;
	mat4 camProjInverse;
	vec2 texCoord0;
} iVert;

//New Code

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


float sphereDf(vec3 p, vec3 spherePos, float radius)
{
    return length(p - spherePos) - radius;
}

float sdPlane( vec3 p, vec4 n )
{
    // n must be normalized
    return dot( p, n.xyz ) + n.w;
}

float sdTorus( vec3 p, vec2 t )
{
  vec2 q = vec2(length(p.xz)-t.x,p.y);
  return length(q)-t.y;
}

#define Type01 sphereDf
#define Type02 cubeSDF
#define Type03 sdPlane
#define Type04 sdTorus



/* 
# Primitive combinations
0 = Union
1 = Subtraction
2 = Intersection
3 = Smooth Union
4 = Smooth Subtraction
5 = Smooth Intersection
*/

#define Combie01 opUnion
#define Combie02 opSubtraction
#define Combie03 opIntersection
#define Combie04 opSmoothUnion
#define Combie05 opSmoothSubtraction
#define Combie06 opSmoothIntersection

/// Union, Subtraction, Intersection - exact, bound, bound
float opUnion( float d1, float d2 ) { return min(d1,d2); }

float opSubtraction( float d1, float d2 ) { return max(-d1,d2); }

float opIntersection( float d1, float d2 ) { return max(d1,d2); }

// Smooth Union, Subtraction and Intersection - exact, bound, bound
float opSmoothUnion( float d1, float d2) {
    float k = uSmoothK;
    float h = clamp( 0.5 + 0.5*(d2-d1)/k, 0.0, 1.0 );
    return mix( d2, d1, h ) - k*h*(1.0-h); }

float opSmoothSubtraction( float d1, float d2, float k ) {
    float h = clamp( 0.5 - 0.5*(d2+d1)/k, 0.0, 1.0 );
    return mix( d2, -d1, h ) + k*h*(1.0-h); }

float opSmoothIntersection( float d1, float d2, float k ) {
    float h = clamp( 0.5 - 0.5*(d2-d1)/k, 0.0, 1.0 );
    return mix( d2, d1, h ) + k*h*(1.0-h); }

//------------------------------------------------------------
// Describe your scene here
//------------------------------------------------------------
float sceneSDF(vec3 p)
{
    
    float scene = uMaxDist;  // for empty start
    //float scene = sdPlane(p, uPlane);

    /* Oringinal Code
    // Loop for creating spheres at every sample of an array
    for (int i = 0; i < uNum; i++) {
        vec4 smpl = texelFetchBuffer(uPrim, i);
        vec3 sphere = vec3(smpl.xyz);	// position
        float radius = smpl.w;			// radius
        float sph = sphereDf(p, sphere, radius); 
        scene = opSmoothUnion(scene, sph);
    }
    */

    //  New Code
    for (int i = 0; i < uNum; i++) {

        int type = int(texelFetchBuffer(SDFsType, i));
        vec3 T = texelFetchBuffer(SDFsT, i).xyz;
        vec3 R = texelFetchBuffer(SDFsR, i).xyz;
        vec3 S = texelFetchBuffer(SDFsS, i).xyz;
        vec4 smpl = texelFetchBuffer(uPrim, i);

        if (type == 0){
        vec3 sphere = vec3(smpl.xyz);	// position
        float radius = smpl.w;			// radius
        float sph = Type01(p, sphere, radius); 
        scene = opSmoothUnion(scene, sph);
        }
        else if (type == 1){

        }
    }




    return scene;
}

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
/**
**
 * Return the normalized direction to march in from the eye point for a single pixel.
 * 
 * fieldOfView: vertical field of view in degrees
 * size: resolution of the output image
 * fragCoord: the x,y coordinate of the pixel in the output image
 */

vec3 rayDirection(vec2 fragCoord) {
    vec2 pos = (fragCoord-0.5)*2;
    vec4 Dir = iVert.invProj * vec4(pos,0,-1);
    Dir = normalize(vec4(Dir.xy,-Dir.z,1));
    vec4 wDir = Dir * iVert.cam;
    return wDir.xyz;
}


float map(float value, float min1, float max1, float min2, float max2) {
  return min2 + (value - min1) * (max2 - min2) / (max1 - min1);
}

float procDepth(vec3 localPos){
    vec4 frag = iVert.camProj*vec4(localPos,1);
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



//------------------------------------------------------------
// Light + coloring + shadows
//------------------------------------------------------------
/**
 * Lighting contribution of a single point light source via Phong illumination.
 * 
 * The vec3 returned is the RGB color of the light's contribution.
 *
 * k_a: Ambient color
 * k_d: Diffuse color
 * k_s: Specular color
 * alpha: Shininess coefficient
 * p: position of point being lit
 * eye: the position of the camera
 * lightPos: the position of the light
 * lightIntensity: color/intensity of the light
 *
 * See https://en.wikipedia.org/wiki/Phong_reflection_model#Description
 */
vec3 phongContribForLight(vec3 k_d, vec3 k_s, float alpha, vec3 p, vec3 eye,
                          vec3 lightPos, vec3 lightIntensity) {
    vec3 N = estimateNormal(p);
    vec3 L = normalize(lightPos - p);
    vec3 V = normalize(eye - p);
    vec3 R = normalize(reflect(-L, N));
    
    float dotLN = dot(L, N);
    float dotRV = dot(R, V);
    
    if (dotLN < 0.0) {
        // Light not visible from this point on the surface, so add no color
        return vec3(0.0);
    } 
    
    if (dotRV < 0.0) {
        // Light reflection in opposite direction as viewer, apply only diffuse
        // component
        return lightIntensity * (k_d * dotLN);
    }
    return lightIntensity * (k_d * dotLN + k_s * pow(dotRV, alpha));
}

/**
 * Lighting via Phong illumination.
 * 
 * The vec3 returned is the RGB color of that point after lighting is applied.
 * k_a: Ambient color
 * k_d: Diffuse color
 * k_s: Specular color
 * alpha: Shininess coefficient
 * p: position of point being lit
 * eye: the position of the camera
 *
 * See https://en.wikipedia.org/wiki/Phong_reflection_model#Description
 */
vec3 phongIllumination(vec3 k_a, vec3 k_d, vec3 k_s, float alpha, vec3 p, vec3 eye) {
    const vec3 ambientLight = 0.5 * vec3(1.0, 1.0, 1.0);
    vec3 color = ambientLight * k_a;
    color += phongContribForLight(k_d, k_s, alpha, p, eye,
                                  uLight1Pos,
                                  uLight1Col);
    
 
    // Example of hardcoded (second) light:
    //
    // vec3 light2Pos = vec3(2.0 * sin(0.37 * uTime),
    //                       2.0 * cos(0.37 * uTime),
    //                       2.0);
    // vec3 light2Intensity = vec3(0.4, 0.4, 0.4);   
    // color += phongContribForLight(k_d, k_s, alpha, p, eye,
    //                               light2Pos,
    //                               light2Intensity);    
    return color;
}


//------------------------------------------------------------
// "Look at ..." matrix
//------------------------------------------------------------
/**
 * Return a transform matrix that will transform a ray from view space
 * to world coordinates, given the eye point, the camera target, and an up vector.
 *
 * This assumes that the center of the camera is aligned with the negative z axis in
 * view space when calculating the ray marching direction. See rayDirection.
 */

//------------------------------------------------------------
// Put everything together
//------------------------------------------------------------





void main()
{
    //TDCheckDiscard();
    vec2 uv = iVert.texCoord0;

    // setting camera(eye)
    vec3 eye = vec4(iVert.camInverse*vec4(0,0,0,1)).xyz;
    vec3 worldDir = rayDirection(uv);

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

    // coloring (details at line 165)
    vec3 color = phongIllumination(uAmbient, uDiffuse, uSpecular, uShine, p, eye); 
    // alpha set to 1.0, try change it to 0.0 instead:
    vec4 fColor =vec4(color, 1.0);
    fColor = TDDither(fColor);
    fColor = TDOutputSwizzle(fColor);
    fragColor = fColor;
    gl_FragDepth = Depth;
    
}