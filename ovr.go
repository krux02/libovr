package ovr

//#cgo LDFLAGS: ../../../github.com/krux02/libovr/Lib/Linux/Release/x86_64/libovr.a -ludev -lGL -lX11 -lm -lstdc++ -lXrandr
//#include "OVR_CAPI.h"
//#include "stdlib.h"
import "C"

import "unsafe"

func vector2f(that C.ovrVector2f) (this Vector2f) {
	this.X = float32(that.x)
	this.Y = float32(that.y)
	return
}

func c_vector2f(that Vector2f) (this C.ovrVector2f) {
	this.x = C.float(that.X)
	this.y = C.float(that.Y)
	return
}

func vector3f(that C.ovrVector3f) (this Vector3f) {
	this.X = float32(that.x)
	this.Y = float32(that.y)
	this.Z = float32(that.z)
	return
}

func c_vector3f(that Vector3f) (this C.ovrVector3f) {
	this.x = C.float(that.X)
	this.y = C.float(that.Y)
	this.z = C.float(that.Z)
	return
}

func vector2i(that C.ovrVector2i) (this Vector2i) {
	this.X = int32(that.x)
	this.Y = int32(that.y)
	return
}

func c_vector2i(that Vector2i) (this C.ovrVector2i) {
	this.x = C.int(that.X)
	this.y = C.int(that.Y)
	return
}

func quatf(that C.ovrQuatf) (this Quatf) {
	this.X = float32(that.x)
	this.Y = float32(that.y)
	this.Z = float32(that.z)
	this.W = float32(that.w)
	return
}

func c_quatf(that Quatf) (this C.ovrQuatf) {
	this.x = C.float(that.X)
	this.y = C.float(that.Y)
	this.z = C.float(that.Z)
	this.w = C.float(that.W)
	return
}

func matrix4f(that C.ovrMatrix4f) (this Matrix4f) {
	return *(*Matrix4f)(unsafe.Pointer(&that))
}

func (mat Matrix4f) FlatArray() (out [16]float32) {
	return *(*[16]float32)(unsafe.Pointer(&mat))
}

func c_matrix4f(that Matrix4f) (this C.ovrMatrix4f) {
	for i := 0; i < 4; i++ {
		for j := 0; j < 4; j++ {
			this.M[i][j] = C.float(that.M[i][j])
		}
	}
	return
}

func recti(that C.ovrRecti) (this Recti) {
	this.Pos = vector2i(that.Pos)
	this.Size = sizei(that.Size)
	return
}

func c_recti(that Recti) (this C.ovrRecti) {
	this.Pos = c_vector2i(that.Pos)
	this.Size = c_sizei(that.Size)
	return
}

func fovPort(that C.ovrFovPort) (this FovPort) {
	this.DownTan = float32(that.DownTan)
	this.LeftTan = float32(that.LeftTan)
	this.RightTan = float32(that.RightTan)
	this.UpTan = float32(that.UpTan)
	return
}

func c_fovPort(that FovPort) (this C.ovrFovPort) {
	this.DownTan = C.float(that.DownTan)
	this.LeftTan = C.float(that.LeftTan)
	this.RightTan = C.float(that.RightTan)
	this.UpTan = C.float(that.UpTan)
	return
}

func sensorState(that C.ovrSensorState) (this SensorState) {
	this.Predicted = poseState(that.Predicted)
	this.Recorded = poseState(that.Recorded)
	this.StatusFlags = StatusBits(that.StatusFlags)
	this.Temperature = float32(that.Temperature)
	return
}

func poseState(that C.ovrPoseStatef) (this PoseStatef) {
	this.Pose = posef(that.Pose)
	this.AngularVelocity = vector3f(that.AngularVelocity)
	this.LinearVelocity = vector3f(that.LinearVelocity)
	this.AngularAcceleration = vector3f(that.AngularAcceleration)
	this.LinearAcceleration = vector3f(that.LinearAcceleration)
	this.TimeInSeconds = float64(that.TimeInSeconds)
	return
}

func posef(that C.ovrPosef) (this Posef) {
	this.Orientation = quatf(that.Orientation)
	this.Position = vector3f(that.Position)
	return
}

func c_posef(that Posef) (this C.ovrPosef) {
	this.Orientation = c_quatf(that.Orientation)
	this.Position = c_vector3f(that.Position)
	return
}

func sizei(that C.ovrSizei) (this Sizei) {
	this.W = int32(that.w)
	this.H = int32(that.h)
	return
}

func c_sizei(that Sizei) (this C.ovrSizei) {
	this.w = C.int(that.W)
	this.h = C.int(that.H)
	return
}

func renderApiConfigHeader(that C.ovrRenderAPIConfigHeader) (this RenderApiConfigHeader) {
	this.API = RenderApiType(that.API)
	this.RTSize = sizei(that.RTSize)
	this.Multisample = int32(that.Multisample)
	return
}

func c_renderApiConfigHeader(that RenderApiConfigHeader) (this C.ovrRenderAPIConfigHeader) {
	this.API = C.ovrRenderAPIType(that.API)
	this.RTSize = c_sizei(that.RTSize)
	this.Multisample = C.int(that.Multisample)
	return
}

func renderApiConfig(that C.ovrRenderAPIConfig) (this RenderApiConfig) {
	this.Header = renderApiConfigHeader(that.Header)
	for i := 0; i < len(this.PlatformData); i++ {
		this.PlatformData[i] = uint64(that.PlatformData[i])
	}
	return
}

func c_renderApiConfig(that RenderApiConfig) (this C.ovrRenderAPIConfig) {
	this.Header = c_renderApiConfigHeader(that.Header)
	for i := 0; i < len(that.PlatformData); i++ {
		this.PlatformData[i] = C.uintptr_t(that.PlatformData[i])
	}
	return
}

func frameTiming(that C.ovrFrameTiming) (this FrameTiming) {
	this.DeltaSeconds = float32(that.DeltaSeconds)
	this.ThisFrameSeconds = float64(that.ThisFrameSeconds)
	this.TimewarpPointSeconds = float64(that.TimewarpPointSeconds)
	this.NextFrameSeconds = float64(that.NextFrameSeconds)
	this.ScanoutMidpointSeconds = float64(that.ScanoutMidpointSeconds)
	this.EyeScanoutSeconds[0] = float64(that.EyeScanoutSeconds[0])
	this.EyeScanoutSeconds[1] = float64(that.EyeScanoutSeconds[1])
	return
}

func eyeRenderDesc(that C.ovrEyeRenderDesc) (this EyeRenderDesc) {
	this.Eye = EyeType(that.Eye)
	this.Fov = fovPort(that.Fov)
	this.DistortedViewport = recti(that.DistortedViewport)
	this.PixelsPerTanAngleAtCenter = vector2f(that.PixelsPerTanAngleAtCenter)
	this.ViewAdjust = vector3f(that.ViewAdjust)
	return
}

func (this *Matrix4f) cptr() *C.ovrMatrix4f {
	return (*C.ovrMatrix4f)(unsafe.Pointer(this))
}

func (this *Vector2f) cptr() *C.ovrVector2f {
	return (*C.ovrVector2f)(unsafe.Pointer(this))
}

func (this *SensorState) cptr() *C.ovrSensorState {
	return (*C.ovrSensorState)(unsafe.Pointer(this))
}

func (this *SensorDesc) cptr() *C.ovrSensorDesc {
	return (*C.ovrSensorDesc)(unsafe.Pointer(this))
}

func (this *FrameTiming) cptr() *C.ovrFrameTiming {
	return (*C.ovrFrameTiming)(unsafe.Pointer(this))
}

func (this *EyeRenderDesc) cptr() *C.ovrEyeRenderDesc {
	return (*C.ovrEyeRenderDesc)(unsafe.Pointer(this))
}

func (this *RenderApiConfigHeader) cptr() *C.ovrRenderAPIConfigHeader {
	return (*C.ovrRenderAPIConfigHeader)(unsafe.Pointer(this))
}

func (this *RenderApiConfig) cptr() *C.ovrRenderAPIConfig {
	return (*C.ovrRenderAPIConfig)(unsafe.Pointer(this))
}

func (this *TextureHeader) cptr() *C.ovrTextureHeader {
	return (*C.ovrTextureHeader)(unsafe.Pointer(this))
}

func (this *Texture) cptr() *C.ovrTexture {
	return (*C.ovrTexture)(unsafe.Pointer(this))
}

// -----------------------------------------------------------------------------------
// ***** API Interfaces

// Basic steps to use the API:
//
// Setup:
//  1. ovrInitialize();
//  2. ovrHMD hmd = ovrHmd_Create(0);  ovrHmd_GetDesc(hmd, &hmdDesc);
//  3. Use hmdDesc and ovrHmd_GetFovTextureSize() to determine graphics configuration.
//  4. Call ovrHmd_StartSensor() to configure and initialize tracking.
//  5. Call ovrHmd_ConfigureRendering() to setup graphics for SDK rendering,
//     which is the preferred approach.
//     Please refer to "Game-Side Rendering" below if you prefer to do that instead.
//  5. Allocate textures as needed.
//
// Game Loop:
//  6. Call ovrHmd_BeginFrame() to get frame timing and orientation information.
//  7. Render each eye in between ovrHmd_BeginEyeRender and ovrHmd_EndEyeRender calls,
//     providing the result texture to the API.
//  8. Call ovrHmd_EndFrame() to render distorted textures to the back buffer
//     and present them on the Hmd.
//
// Shutdown:
//  9. ovrHmd_Destroy(hmd)
//  10. ovr_Shutdown()
//

// Library init/shutdown, must be called around all other OVR code.
// No other functions calls are allowed before ovr_Initialize succeeds or after ovr_Shutdown.

func Initialize() bool {
	return C.ovr_Initialize() != 0
}

func Shutdown() {
	C.ovr_Shutdown()
}

// Detects or re-detects HMDs and reports the total number detected.
// Users can get information about each HMD by calling ovrHmd_Create with an index.
func HmdDetect() int {
	return int(C.ovrHmd_Detect())
}

// Creates a handle to an HMD and optionally fills in data about it.
// Index can [0 .. ovrHmd_Detect()-1]; index mappings can cange after each ovrHmd_Detect call.
// If not null, returned handle must be freed with ovrHmd_Destroy.
func HmdCreate(index int) *Hmd {
	return hmd(C.ovrHmd_Create(C.int(index)))
}

func (this *Hmd) Destroy() {
	C.ovrHmd_Destroy(this.cptr())
}

//ovrHmd   ovrHmd_CreateDebug(ovrHmdType type);
func HmdCreateDebug(hmdType HmdType) *Hmd {
	return hmd(C.ovrHmd_CreateDebug(C.ovrHmdType(hmdType)))
}

// Returns last error for HMD state. Returns null for no error.
// String is valid until next call or GetLastError or HMD is destroyed.
// Pass null hmd to get global error (for create, etc).
func (this *Hmd) GetLastError() string {
	return C.GoString(C.ovrHmd_GetLastError(this.cptr()))
}

//-------------------------------------------------------------------------------------

// Returns capability bits that are enabled at this time; described by ovrHmdCaps.
// Note that this value is different font ovrHmdDesc::HmdCaps, which describes what
// capabilities are available.
func (this *Hmd) GetEnabledCaps() HmdCaps {
	return HmdCaps(C.ovrHmd_GetEnabledCaps(this.cptr()))
}

// Modifies capability bits described by ovrHmdCaps that can be modified,
// such as ovrHmd_LowPersistance.
func (this *Hmd) SetEnabledCaps(hmdCaps HmdCaps) {
	C.ovrHmd_SetEnabledCaps(this.cptr(), C.uint(hmdCaps))
}

//-------------------------------------------------------------------------------------
// ***** Sensor Interface

// All sensor interface functions are thread-safe, allowing sensor state to be sampled
// from different threads.
// Starts sensor sampling, enabling specified capabilities, described by ovrSensorCaps.
//  - supportedSensorCaps specifies support that is requested. The function will succeed
//	  even if these caps are not available (i.e. sensor or camera is unplugged). Support
//    will automatically be enabled if such device is plugged in later. Software should
//    check ovrSensorState.StatusFlags for real-time status.
//  - requiredSensorCaps specify sensor capabilities required at the time of the call.
//    If they are not available, the function will fail. Pass 0 if only specifying
//    supportedSensorCaps.
func (this *Hmd) StartSensor(supportedSensorCaps, requiredSensorCaps SensorCaps) bool {
	return 0 != C.ovrHmd_StartSensor(this.cptr(), C.uint(supportedSensorCaps), C.uint(requiredSensorCaps))
}

// Stops sensor sampling, shutting down internal resources.

func (this *Hmd) StopSensor() {
	C.ovrHmd_StopSensor(this.cptr())
}

// Resets sensor orientation.
func (this *Hmd) ResetSensor() {
	C.ovrHmd_ResetSensor(this.cptr())
}

// Returns sensor state reading based on the specified absolute system time.
// Pass absTime value of 0.0 to request the most recent sensor reading; in this case
// both PredictedPose and SamplePose will have the same value.
// ovrHmd_GetEyePredictedSensorState relies on this internally.
// This may also be used for more refined timing of FrontBuffer rendering logic, etc.
func (hmd *Hmd) GetSensorState(absTime float64) SensorState {
	return sensorState(C.ovrHmd_GetSensorState(hmd.cptr(), C.double(absTime)))
}

// Returns information about a sensor.
// Only valid after StartSensor.
func (this *Hmd) GetSensorDesc(descOut *SensorDesc) bool {
	return 0 != C.ovrHmd_GetSensorDesc(this.cptr(), descOut.cptr())
}

func (this *Hmd) GetDesc() (desc HmdDesc) {
	var cdesc C.ovrHmdDesc
	C.ovrHmd_GetDesc(this.cptr(), &cdesc)
	return hmdDesc(cdesc)
}

// Calculates texture size recommended for rendering one eye within HMD, given FOV cone.
// Higher FOV will generally require larger textures to maintain quality.
//  - pixelsPerDisplayPixel specifies that number of render target pixels per display
//    pixel at center of distortion; 1.0 is the default value. Lower values
//    can improve performance.
func (hmd *Hmd) GetFovTextureSize(eye EyeType, fov FovPort, pixelsPerDisplayPixel float32) Sizei {
	var cFov C.ovrFovPort
	cFov.DownTan = C.float(fov.DownTan)
	cFov.LeftTan = C.float(fov.LeftTan)
	cFov.RightTan = C.float(fov.RightTan)
	cFov.UpTan = C.float(fov.UpTan)
	return sizei(C.ovrHmd_GetFovTextureSize(hmd.cptr(), C.ovrEyeType(eye), cFov, C.float(pixelsPerDisplayPixel)))
}

//-------------------------------------------------------------------------------------
// *****  Rendering API Thread Safety

//  All of rendering APIs, inclusing Configure and frame functions are *NOT
//  Thread Safe*.  It is ok to use ConfigureRendering on one thread and handle
//  frames on another thread, but explicit synchronization must be done since
//  functions that depend on configured state are not reentrant.
//
//  As an extra requirement, any of the following calls must be done on
//  the render thread, which is the same thread that calls ovrHmd_BeginFrame
//  or ovrHmd_BeginFrameTiming.
//    - ovrHmd_EndFrame
//    - ovrHmd_BeginEyeRender
//    - ovrHmd_EndEyeRender
//    - ovrHmd_GetFramePointTime
//    - ovrHmd_GetEyePose
//    - ovrHmd_GetEyeTimewarpMatrices

//-------------------------------------------------------------------------------------
// *****  SDK-Rendering Functions

// These functions support rendering of distortion by the SDK through direct
// access to the underlying rendering HW, such as D3D or GL.
// This is the recommended approach, as it allows for better support or future
// Oculus hardware and a range of low-level optimizations.

// Configures rendering; fills in computed render parameters.
// This function can be called multiple times to change rendering settings.
// The users pass in two eye view descriptors that are used to
// generate complete rendering information for each eye in eyeRenderDescOut[2].
//
//  - apiConfig provides D3D/OpenGL specific parameters. Pass null
//    to shutdown rendering and release all resources.
//  - distortionCaps describe distortion settings that will be applied.
//

func (this *Hmd) ConfigureRendering(apiConfig *RenderApiConfig, distortionCaps DistortionCaps, eyeFovIn [2]FovPort) (eyeRenderDescOut [2]EyeRenderDesc, ok bool) {
	//config := c_renderApiConfig(apiConfig)
	dc := C.uint(distortionCaps)
	efi := [2]C.ovrFovPort{
		c_fovPort(eyeFovIn[0]),
		c_fovPort(eyeFovIn[1]),
	}
	ok = 0 != C.ovrHmd_ConfigureRendering(this.cptr(), apiConfig.cptr(), dc, &efi[0], eyeRenderDescOut[0].cptr())
	return
}

// Begins a frame, returning timing and orientation information useful for simulation.
// This should be called in the beginning of game rendering loop (on render thread).
// This function relies on ovrHmd_BeginFrameTiming for some of its functionality.
// Pass 0 for frame index if not using GetFrameTiming.

func (this *Hmd) BeginFrame(frameIndex int) FrameTiming {
	return frameTiming(C.ovrHmd_BeginFrame(this.cptr(), C.uint(frameIndex)))
}

// Ends frame, rendering textures to frame buffer. This may perform distortion and scaling
// internally, assuming is it not delegated to another thread.
// Must be called on the same thread as BeginFrame. Calls ovrHmd_BeginEndTiming internally.
// *** This Function will to Present/SwapBuffers and potentially wait for GPU Sync ***.

func (this *Hmd) EndFrame() {
	C.ovrHmd_EndFrame(this.cptr())
}

// Marks beginning of eye rendering. Must be called on the same thread as BeginFrame.
// This function uses ovrHmd_GetEyePose to predict sensor state that should be
// used rendering the specified eye.
// This combines current absolute time with prediction that is appropriate for this HMD.
// It is ok to call ovrHmd_BeginEyeRender() on both eyes before calling ovrHmd_EndEyeRender.
// If rendering one eye at a time, it is best to render eye specified by
// HmdDesc.EyeRenderOrder[0] first.

func (this *Hmd) BeginEyeRender(eye EyeType) Posef {
	return posef(C.ovrHmd_BeginEyeRender(this.cptr(), C.ovrEyeType(eye)))
}

// Marks the end of eye rendering and submits the eye texture for display after it is ready.
// Rendering viewport within the texture can change per frame if necessary.
// Specified texture may be presented immediately or wait until ovrHmd_EndFrame based
// on the implementation. The API performs distortion and scaling internally.
// 'renderPose' will typically be the value returned from ovrHmd_BeginEyeRender, but can
// be different if a different pose was used for rendering.

func (this *Hmd) EndEyeRender(eye EyeType, renderPose Posef, eyeTexture *Texture) {
	C.ovrHmd_EndEyeRender(this.cptr(), C.ovrEyeType(eye), c_posef(renderPose), eyeTexture.cptr())
}

//-------------------------------------------------------------------------------------
// *****  Game-Side Rendering Functions

// These functions provide distortion data and render timing support necessary to allow
// game rendering of distortion. Game-side rendering involves the following steps:
//
//  1. Setup ovrEyeDesc based on desired texture size and Fov.
//     Call ovrHmd_GetRenderDesc to get the necessary rendering parameters for each eye.
//
//  2. Use ovrHmd_CreateDistortionMesh to generate distortion mesh.
//
//  3. Use ovrHmd_BeginFrameTiming, ovrHmd_GetEyePose and ovrHmd_BeginFrameTiming
//     in the rendering loop to obtain timing and predicted view orientation for
//     each eye.
//      - If relying on timewarp, use ovr_WaitTillTime after rendering+flush, followed
//        by ovrHmd_GetEyeTimewarpMatrices to obtain timewarp matrices used
//        in distortion pixel shader to reduce latency.
//

// Computes distortion viewport, view adjust and other rendering for the specified
// eye. This can be used instead of ovrHmd_ConfigureRendering to help setup rendering on
// the game side.

func (this *Hmd) GetRenderDesc(eyeType EyeType, fov FovPort) EyeRenderDesc {
	return eyeRenderDesc(C.ovrHmd_GetRenderDesc(this.cptr(), C.ovrEyeType(eyeType), c_fovPort(fov)))
}

/*

TODO add support for Distortion Vertex
// Describes a vertex used for distortion; this is intended to be converted into
// the engine-specific format.
// Some fields may be unused based on ovrDistortionCaps selected. TexG and TexB, for example,
// are not used if chromatic correction is not requested.
typedef struct ovrDistortionVertex_
{
    ovrVector2f Pos;
    float       TimeWarpFactor;  // Lerp factor between time-warp matrices. Can be encoded in Pos.z.
    float       VignetteFactor;  // Vignette fade factor. Can be encoded in Pos.w.
    ovrVector2f TexR;
    ovrVector2f TexG;
    ovrVector2f TexB;
} ovrDistortionVertex;

// Describes a full set of distortion mesh data, filled in by ovrHmd_CreateDistortionMesh.
// Contents of this data structure, if not null, should be freed by ovrHmd_DestroyDistortionMesh.
typedef struct ovrDistortionMesh_
{
    ovrDistortionVertex* pVertexData;
    unsigned short*      pIndexData;
    unsigned int         VertexCount;
    unsigned int         IndexCount;
} ovrDistortionMesh;

// Generate distortion mesh per eye.
// Distortion capabilities will depend on 'distortionCaps' flags; user should rely on
// appropriate shaders based on their settings.
// Distortion mesh data will be allocated and stored into the ovrDistortionMesh data structure,
// which should be explicitly freed with ovrHmd_DestroyDistortionMesh.
// Users should call ovrHmd_GetRenderScaleAndOffset to get uvScale and Offset values for rendering.
// The function shouldn't fail unless theres is a configuration or memory error, in which case
// ovrDistortionMesh values will be set to null.
OVR_EXPORT ovrBool  ovrHmd_CreateDistortionMesh( ovrHmd hmd,ovrEyeType eyeType, ovrFovPort fov,
                                                 unsigned int distortionCaps,
                                                 ovrDistortionMesh *meshData );

OVR_EXPORT ovrBool  ovrHmd_CreateDistortionMesh( ovrHmd hmd,
                                                 ovrEyeType eyeType, ovrFovPort fov,
                                                 unsigned int distortionCaps,
                                                 ovrDistortionMesh *meshData );



// Frees distortion mesh allocated by ovrHmd_GenerateDistortionMesh. meshData elements
// are set to null and zeroes after the call.
func (hmd *Hmd) DestroyDistortionMesh(ovrDistortionMesh* meshData );
OVR_EXPORT void     ovrHmd_DestroyDistortionMesh( ovrDistortionMesh* meshData );

*/

// Computes updated 'uvScaleOffsetOut' to be used with a distortion if render target size or
// viewport changes after the fact. This can be used to adjust render size every frame, if desired.
func (hmd *Hmd) GetRenderScaleAndOffset(fov FovPort, textureSize Sizei, renderViewport Recti) (uvScaleOffsetOut [2]Vector2f) {
	C.ovrHmd_GetRenderScaleAndOffset(c_fovPort(fov), c_sizei(textureSize), c_recti(renderViewport), uvScaleOffsetOut[0].cptr())
	return
}

// Thread-safe timing function for the main thread. Caller should increment frameIndex
// with every frame and pass the index to RenderThread for processing.
func (this *Hmd) GetFrameTiming(frameIndex int) FrameTiming {
	return frameTiming(C.ovrHmd_GetFrameTiming(this.cptr(), C.uint(frameIndex)))
}

// Called at the beginning of the frame on the Render Thread.
// Pass frameIndex == 0 if ovrHmd_GetFrameTiming isn't being used. Otherwise,
// pass the same frame index as was used for GetFrameTiming on the main thread.
func (hmd *Hmd) BeginFrameTiming(frameIndex int) FrameTiming {
	return frameTiming(C.ovrHmd_BeginFrameTiming(hmd.cptr(), C.uint(frameIndex)))
}

// Marks the end of game-rendered frame, tracking the necessary timing information. This
// function must be called immediately after Present/SwapBuffers + GPU sync. GPU sync is important
// before this call to reduce latency and ensure proper timing.
func (hmd *Hmd) EndFrameTiming() {
	C.ovrHmd_EndFrameTiming(hmd.cptr())
}

// Initializes and resets frame time tracking. This is typically not necessary, but
// is helpful if game changes vsync state or video mode. vsync is assumed to be on if this
// isn't called. Resets internal frame index to the specified number.
func (hmd *Hmd) ResetFrameTiming(frameIndex int) {
	C.ovrHmd_ResetFrameTiming(hmd.cptr(), C.uint(frameIndex))
}

// Predicts and returns Pose that should be used rendering the specified eye.
// Must be called between ovrHmd_BeginFrameTiming & ovrHmd_EndFrameTiming.
func (this *Hmd) GetEyePose(eye EyeType) Posef {
	return posef(C.ovrHmd_GetEyePose(this.cptr(), C.ovrEyeType(eye)))
}

// Computes timewarp matrices used by distortion mesh shader, these are used to adjust
// for orientation change since the last call to ovrHmd_GetEyePose for this eye.
// The ovrDistortionVertex::TimeWarpFactor is used to blend between the matrices,
// usually representing two different sides of the screen.
// Must be called on the same thread as ovrHmd_BeginFrameTiming.
func (hmd *Hmd) GetEyeTimewarpMatrices(eye EyeType, renderPose Posef) (twmOut [2]Matrix4f) {
	C.ovrHmd_GetEyeTimewarpMatrices(hmd.cptr(), C.ovrEyeType(eye), c_posef(renderPose), twmOut[0].cptr())
	return
}

//-------------------------------------------------------------------------------------
// ***** Stateless math setup functions

// Used to generate projection from ovrEyeDesc::Fov.
func MatrixProjection(fov FovPort, znear, zfar float32, rightHanded bool) Matrix4f {
	if rightHanded {
		return matrix4f(C.ovrMatrix4f_Projection(c_fovPort(fov), C.float(znear), C.float(zfar), 1))
	} else {
		return matrix4f(C.ovrMatrix4f_Projection(c_fovPort(fov), C.float(znear), C.float(zfar), 0))
	}
}

// Used for 2D rendering, Y is down
// orthoScale = 1.0f / pixelsPerTanAngleAtCenter
// orthoDistance = distance from camera, such as 0.8m
func MatrixOrthoSubProjection(projection Matrix4f, orthoScale Vector2f, orthoDistance, eyeViewAdjustX float32) Matrix4f {
	return matrix4f(C.ovrMatrix4f_OrthoSubProjection(c_matrix4f(projection), c_vector2f(orthoScale), C.float(orthoDistance), C.float(eyeViewAdjustX)))
}

// Returns global, absolute high-resolution time in seconds. This is the same
// value as used in sensor messages.
func GetTimeInSeconds() float64 {
	return float64(C.ovr_GetTimeInSeconds())
}

// Waits until the specified absolute time.
func WaitTillTime(absTime float64) float64 {
	return float64(C.ovr_WaitTillTime(C.double(absTime)))
}

// -----------------------------------------------------------------------------------
// ***** Latency Test interface

// Does latency test processing and returns 'TRUE' if specified rgb color should
// be used to clear the screen.
/*
func (hmd *Hmd) Hmd_ProcessLatencyTest(r, g, b byte) bool {
	rgbColorOut := [3]C.uchar{r, g, b}
	return 0 != C.ovrHmd_ProcessLatencyTest(hmd.cptr(), &rgbColorOut)
}
*/

// Returns non-null string once with latency test result, when it is available.
// Buffer is valid until next call.
func (hmd *Hmd) GetLatencyTestResult() string {
	return C.GoString(C.ovrHmd_GetLatencyTestResult(hmd.cptr()))
}

func (hmd *Hmd) GetMeasuredLatencyTest2() float64 {
	return float64(C.ovrHmd_GetMeasuredLatencyTest2(hmd.cptr()))
}

const (
	KeyUser                    = "User"
	KeyName                    = "Name"
	KeyGender                  = "Gender"
	KeyPlayerHeight            = "PlayerHeight"
	KeyEyeHeight               = "EyeHeight"
	KeyIPD                     = "IPD"
	KeyNeckToEyeHorizontal     = "NeckEyeHori"
	KeyNeckToEyeVertical       = "NeckEyeVert"
	DefaultGender              = "Male"
	DefaultPlayerHeight        = 1.778
	DefaultEyeHeight           = 1.675
	DefaultIPD                 = 0.064
	DefaultNeckToEyeHorizental = 0.12
	DefaultNeckToEyeVertical   = 0.12
)

func (hmd *Hmd) GetFloat(propertyName string, defaultVal float32) float32 {
	_propertyName := C.CString(propertyName)
	defer C.free(unsafe.Pointer(_propertyName))
	return float32(C.ovrHmd_GetFloat(hmd.cptr(), _propertyName, C.float(defaultVal)))
}

func (hmd *Hmd) SetFloat(propertyName string, value float32) bool {
	_propertyName := C.CString(propertyName)
	defer C.free(unsafe.Pointer(_propertyName))
	return 0 != C.ovrHmd_SetFloat(hmd.cptr(), _propertyName, C.float(value))
}

// Get float[] property. Returns the number of elements filled in, 0 if property doesn't exist.
// Maximum of arraySize elements will be written.
func (hmd *Hmd) GetFloatArray(propertyName string) []float32 {
	_propertyName := C.CString(propertyName)
	defer C.free(unsafe.Pointer(_propertyName))
	arraySize := C.ovrHmd_GetArraySize(hmd.cptr(), _propertyName)
	values := make([]float32, arraySize)
	arrayPtr := (*C.float)(&values[0])
	C.ovrHmd_GetFloatArray(hmd.cptr(), _propertyName, arrayPtr, arraySize)
	return values
}

func (hmd *Hmd) GetFloatArray2(propertyName string, values []float32) []float32 {
	_propertyName := C.CString(propertyName)
	defer C.free(unsafe.Pointer(_propertyName))
	arraySize := C.uint(cap(values))
	arrayPtr := (*C.float)(&values[0])
	size := C.ovrHmd_GetFloatArray(hmd.cptr(), _propertyName, arrayPtr, arraySize)
	return values[:size]
}

func (hmd *Hmd) SetFloatArray(propertyName string, values []float32) bool {
	arraySize := C.uint(len(values))
	_values := (*C.float)(&values[0])
	_propertyName := C.CString(propertyName)
	defer C.free(unsafe.Pointer(_propertyName))
	return 0 != C.ovrHmd_SetFloatArray(hmd.cptr(), _propertyName, _values, arraySize)
}

func (hmd *Hmd) GetString(propertyName string, defaultValue string) string {
	_propertyName := C.CString(propertyName)
	defer C.free(unsafe.Pointer(_propertyName))
	str := C.ovrHmd_GetString(hmd.cptr(), _propertyName, nil)
	if str == nil {
		return defaultValue
	} else {
		return C.GoString(str)
	}
}
