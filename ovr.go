package ovr

//#cgo LDFLAGS: -lovr -ludev -lGL -lX11 -lm -lstdc++ -lXrandr -LLib/Linux/Release/x86_64/
//#include "OVR_CAPI.h"
//
import "C"

import "unsafe"

// 2D integer
type Vector2i struct { X,Y int32 }
type Sizei struct { W,H int32 }
type Recti struct {	Pos Vector2i;	Size Sizei}
type Quatf struct {    X,Y,Z,W float32}
type Vector2f struct {X,Y float32}
type Vector3f struct {X,Y,Z float32}
type Matrix4f [4][4]float32
type Posef struct{
	Orientation Quatf
	Position Vector3f
}
type PoseStatef struct {
    Pose  ovrPosef     
    AngularVelocity  Vector3f  
    LinearVelocity  
    AngularAcceleration  
    LinearAcceleration  
    TimeInSeconds float64         // Absolute time of this state sample.  double       
}


// Specifies which eye is being used for rendering.
// This type explicitly does not include a third "NoStereo" option, as such is
// not required for an HMD-centered API.
type EyeType uint
const ( 
    EyeLeft  EyeType = 0
    EyeRight EyeType = 1
    EyeCount EyeType = 2
)


type FovPort struct {
     UpTan
     DownTan
     LeftTan
     RightTan float32
}

type HmdType uint

const (
    Hmd_None = HmdType(C.ovrHmd_None)
    Hmd_DK1 = HmdType(C.ovrHmd_DK1)
    Hmd_DKHD = HmdType(C.ovrHmd_DKHD)
    Hmd_CrystalCoveProto = HmdType(C.ovrHmd_CrystalCoveProto)
    Hmd_DK2 = HmdType(C.ovrHmd_DK2)
    Hmd_Other = HmdType(C.ovrHmd_Other)
)

type HmdCaps uint

const (
    HmdCap_Present = HmdCaps(C.ovrHmdCap_Present)
    HmdCap_Available = HmdCaps(C.ovrHmdCap_Available)
    HmdCap_LowPersistence = HmdCaps(C.ovrHmdCap_LowPersistence)
	HmdCap_LatencyTest = HmdCaps(C.ovrHmdCap_LatencyTest)
    HmdCap_DynamicPrediction = HmdCaps(C.ovrHmdCap_DynamicPrediction)
    HmdCap_NoVSync = HmdCaps(C.ovrHmdCap_NoVSync)
	HmdCap_NoRestore = HmdCaps(C.ovrHmdCap_NoRestore)
    HmdCap_Writable_Mask = HmdCaps(C.ovrHmdCap_Writable_Mask)
)


// Sensor capability bits reported by device.
// Used with ovrHmd_StartSensor.
type SensorCaps uint

const (
    SensorCap_Orientation = SensorCaps(C.ovrSensorCap_Orientation)
    SensorCap_YawCorrection = SensorCaps(C.ovrSensorCap_YawCorrection)
    SensorCap_Position = SensorCaps(C.ovrSensorCap_Position)
)

type DistortionCaps uint

const (        
    DistortionCap_Chromatic = DistortionCaps(C.ovrDistortionCap_Chromatic)
    DistortionCap_TimeWarp = DistortionCaps(C.ovrDistortionCap_TimeWarp)
    DistortionCap_Vignette = DistortionCaps(C.ovrDistortionCap_Vignette)
)


// This is a complete descriptor of the HMD.
type HmdDesc struct {
    Handle *Hmd;  // Handle of this HMD.
    Type HmdType;
    
    ProductName, Manufacturer string

    // Capability bits described by ovrHmdCaps.
    HmdCaps HmdCaps
	// Capability bits described by ovrSensorCaps.
    SensorCaps SensorCaps
    // Capability bits described by ovrDistortionCaps.
    DistortionCaps DistortionCaps;

    // Resolution of the entire HMD screen (for both eyes) in pixels.
    Sizei    Resolution;
    Vector2i WindowsPos;     

    // These define the recommended and maximum optical FOVs for the HMD.    
    DefaultEyeFov [Eye_Count]FovPort
    MaxEyeFov [Eye_Count]FovPort
    EyeRenderOrder [ovrEye_Count]EyeType
    
    DisplayDeviceName string
    DisplayId int
}

func (this *HmdDesc) Init(that *C.ovrHmdDesc) {
	this.Handle = that.Handle
	this.Type = HmdType(that.Type)
	this.ProductName = C.GoString(that.ProductName)
	this.Manufacturer = C.GoString(that.Manufacturer)
	this.HmdCaps = HmdCaps(that.HmdCaps)
	this.SensorCaps = SensorCaps(that.SensorCaps)
	this.Resolution = Sizei(that.Resolution)
	this.WindowPos = Vector2i(that.WindowPos)
	for i:=0;i<2;i++{
		this.DefaultEyeFov[i] = that.DefaultEyeFov[i]
		this.MaxEyeFov[i] = that.MaxEyeFov[i]
		this.EyeRenderOrder[i] = that.EyeRenderOrder[i]
	}
	this.DisplayDeviceName = C.GoString(that.DisplayDeviceName)
	DisplayId = int(that.DisplayId)
}

type StatusBits uint

const (
    Status_OrientationTracked = StatusBits(C.ovrStatus_OrientationTracked)
    Status_PositionTracked = StatusBits(C.ovrStatus_PositionTracked)
    Status_PositionConnected = StatusBits(C.ovrStatus_PositionConnected)
    Status_HmdConnected = StatusBits(C.ovrStatus_HmdConnected)
)


type SensorState struct {
   Predicted, Recorded PoseStatef
   Temperature float32
   StatusFlags StatusBits;
}

func (this *SensorState) cptr() *C.ovrSensorState {
	return (*C.ovrSensorState)(this)
}


// For now.
// TBD: Decide if this becomes a part of HMDDesc
type SensorDesc struct {
    VendorId,   ProductId int16
    SerialNumber [24]byte;
}

func (this *SensorDesc) cptr() *C.ovrSensorDesc {
	return (*C.ovrSensorDesc)(this)
}

// Frame data reported by ovrHmd_BeginFrameTiming().
type FrameTiming struct {
    DeltaSeconds float32;
    ThisFrameSeconds float64
    TimewarpPointSeconds float64
    NextFrameSeconds float64
    ScanoutMidpointSeconds float64
    EyeScanoutSeconds [2]float64
}

func (this *FrameTiming) cptr() *C.ovrFrameTiming {
	return (*C.ovrFrameTiming)(this)
}



// Rendering information for each eye, computed by either ovrHmd_ConfigureRendering().
// or ovrHmd_GetRenderDesc() based on the specified Fov.
// Note that the rendering viewport is not included here as it can be 
// specified separately and modified per frame though:
//    (a) calling ovrHmd_GetRenderScaleAndOffset with game-rendered api,
// or (b) passing different values in ovrTexture in case of SDK-rendered distortion.
type EyeRenderDesc struct {    
      Eye EyeType
      Fov FovPort
		DistortedViewport Recti 	        // Distortion viewport 
     PixelsPerTanAngleAtCenter Vector2f  // How many display pixels will fit in tan(angle) = 1.
     ViewAdjust Vector3f  		        // Translation to be applied to view matrix.
}

func (this *EyeRenderDesc) cptr() *C.ovrEyeRenderDesc {
	return (*C.EyeRenderDesc)(this)
}

//-----------------------------------------------------------------------------------
// ***** Platform-independent Rendering Configuration

// These types are used to hide platform-specific details when passing
// render device, OS and texture data to the APIs.
//
// The benefit of having these wrappers vs. platform-specific API functions is
// that they allow game glue code to be portable. A typical example is an
// engine that has multiple back ends, say GL and D3D. Portable code that calls
// these back ends may also use LibOVR. To do this, back ends can be modified
// to return portable types such as ovrTexture and ovrRenderAPIConfig.

type RenderApiType uint
const (
    RenderAPI_None = RenderApiType(C.ovrRenderAPI_None)
    RenderAPI_OpenGL = RenderApiType(C.ovrRenderAPI_OpenGL)
    RenderAPI_Android_GLES = RenderApiType(C.ovrRenderAPI_Android_GLES)
    RenderAPI_D3D9 = RenderApiType(C.ovrRenderAPI_D3D9)
    RenderAPI_D3D10 = RenderApiType(C.ovrRenderAPI_D3D10)
    RenderAPI_D3D11 = RenderApiType(C.ovrRenderAPI_D3D11)
    RenderAPI_Count = RenderApiType(C.ovrRenderAPI_Count)
)

type RenderAPIConfigHeader struct {
     API RenderAPIType
             RTSize Sizei
    Multisample int32
}

func (this *RenderAPIConfigHeader) cptr() *C.RenderAPIConfigHeader {
	return (*C.ovrRenderAPIConfigHeader)(this)
}


type RenderAPIConfig struct {
    Header RenderAPIConfigHeader
    PlatformData [8]uintptr
}

func (this *RenderAPIConfig) cptr() *C.ovrRenderAPIConfig {
	return (*C.ovrRenderAPIConfig)(this)
}


type TextureHeader struct {
     API     RenderAPIType
             TextureSize Sizei
             RenderViewport Recti
}

func (this *TextureHeader) cptr() *C.ovrTextureHeader {
	return (*C.ovrTextureHeader)(this)
}


type Texture struct {
    Header TextureHeader
    PlatformData [8]uintptr
}

func (this *Texture) cptr() *C.ovrTexture {
	return (*C.ovrTexture)(this)
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
func Hmd_Detect() int {
	return int(C.ovrHmd_Detect())
}

type Hmd C.struct_ovrHmdStruct

func hmd(h C.ovrHmd) *Hmd {
	return (*Hmd)((*C.struct_ovrHmdStruct)(h))
}

func (this *Hmd) hmd() C.ovrHmd {
	return C.ovrHmd((*C.struct_ovrHmdStruct)(this))
}

// Creates a handle to an HMD and optionally fills in data about it.
// Index can [0 .. ovrHmd_Detect()-1]; index mappings can cange after each ovrHmd_Detect call.
// If not null, returned handle must be freed with ovrHmd_Destroy.
func HmdCreate(index int) *Hmd {
	return hmd(C.ovrHmd_Create(C.int(index)))
}

func (this *Hmd) Destroy() {
	C.ovrHmd_Destroy(this.hmd())
}


//ovrHmd   ovrHmd_CreateDebug(ovrHmdType type);


// Returns last error for HMD state. Returns null for no error.
// String is valid until next call or GetLastError or HMD is destroyed.
// Pass null hmd to get global error (for create, etc).
func (this *Hmd) GetLastError() string {
	return C.GoString(C.ovrHmd_GetLastError(this.hmd()))
}

//-------------------------------------------------------------------------------------

// Returns capability bits that are enabled at this time; described by ovrHmdCaps.
// Note that this value is different font ovrHmdDesc::HmdCaps, which describes what
// capabilities are available.
func (this *Hmd) GetEnabledCaps(hmd overHmd) uint {
	return uint(C.ovrHmd_GetEnabledCaps(hmd.hmd()))
}

// Modifies capability bits described by ovrHmdCaps that can be modified,
// such as ovrHmd_LowPersistance.
func (this *Hmd) SetEnabledCaps(hmdCaps uint) {
	C.ovrHmd_SetEnabledCaps(this.hmd(), C.uint(hmdCaps))
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
func (this *Hmd) StartSensor(supportedSensorCaps, requiredSensorCaps uint) bool {
	return 0 != C.ovrHmd_StartSensor(hmd.hmd(),	C.uint(supportedSensorCaps), C.uint(requiredSensorCaps))
}

// Stops sensor sampling, shutting down internal resources.

func (this *Hmd) StopSensor() {
	C.ovrHmd_StopSensor(hmd.hmd())
}

// Resets sensor orientation.
func (this *Hmd) ResetSensor() {
	C.ovrHmd_ResetSensor(hmd.hmd())
}

// Returns sensor state reading based on the specified absolute system time.
// Pass absTime value of 0.0 to request the most recent sensor reading; in this case
// both PredictedPose and SamplePose will have the same value.
// ovrHmd_GetEyePredictedSensorState relies on this internally.
// This may also be used for more refined timing of FrontBuffer rendering logic, etc.
func (hmd *Hmd) GetSensorState(absTime float64) SensorState {
	return SensorState(C.ovrHmd_GetSensorState(hmd.hmd(), C.double(absTime)))
}

// Returns information about a sensor.
// Only valid after StartSensor.
func (this *Hmd) GetSensorDesc(descOut *SensorDesc) {
	return bool(C.ovrHmd_GetSensorDesc(hmd.hmd(), descOut.cptr()))
}


OVR_EXPORT void     ovrHmd_GetDesc(ovrHmd hmd, ovrHmdDesc* desc);
OVR_EXPORT void     ovrHmd_GetDesc(ovrHmd hmd, ovrHmdDesc* desc);

// Calculates texture size recommended for rendering one eye within HMD, given FOV cone.
// Higher FOV will generally require larger textures to maintain quality.
//  - pixelsPerDisplayPixel specifies that number of render target pixels per display
//    pixel at center of distortion; 1.0 is the default value. Lower values
//    can improve performance.
OVR_EXPORT ovrSizei ovrHmd_GetFovTextureSize(ovrHmd hmd, ovrEyeType eye, ovrFovPort fov, float pixelsPerDisplayPixel);
OVR_EXPORT ovrSizei ovrHmd_GetFovTextureSize(ovrHmd hmd, ovrEyeType eye, ovrFovPort fov, float pixelsPerDisplayPixel);
                                             
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
OVR_EXPORT ovrBool ovrHmd_ConfigureRendering( ovrHmd hmd, 
											const ovrRenderAPIConfig* apiConfig,                                              
                                            unsigned int distortionCaps,
                                            const ovrFovPort eyeFovIn[2],
                                            ovrEyeRenderDesc eyeRenderDescOut[2] );

OVR_EXPORT ovrBool ovrHmd_ConfigureRendering( ovrHmd hmd,
                                            const ovrRenderAPIConfig* apiConfig,                                              
                                            unsigned int distortionCaps,
                                            const ovrFovPort eyeFovIn[2],
                                            ovrEyeRenderDesc eyeRenderDescOut[2] );


// Begins a frame, returning timing and orientation information useful for simulation.
// This should be called in the beginning of game rendering loop (on render thread).
// This function relies on ovrHmd_BeginFrameTiming for some of its functionality.
// Pass 0 for frame index if not using GetFrameTiming.
OVR_EXPORT ovrFrameTiming ovrHmd_BeginFrame(ovrHmd hmd, unsigned int frameIndex);
OVR_EXPORT ovrFrameTiming ovrHmd_BeginFrame(ovrHmd hmd, unsigned int frameIndex);

// Ends frame, rendering textures to frame buffer. This may perform distortion and scaling
// internally, assuming is it not delegated to another thread. 
// Must be called on the same thread as BeginFrame. Calls ovrHmd_BeginEndTiming internally.
// *** This Function will to Present/SwapBuffers and potentially wait for GPU Sync ***.
OVR_EXPORT void     ovrHmd_EndFrame(ovrHmd hmd);
OVR_EXPORT void     ovrHmd_EndFrame(ovrHmd hmd);


// Marks beginning of eye rendering. Must be called on the same thread as BeginFrame.
// This function uses ovrHmd_GetEyePose to predict sensor state that should be
// used rendering the specified eye.
// This combines current absolute time with prediction that is appropriate for this HMD.
// It is ok to call ovrHmd_BeginEyeRender() on both eyes before calling ovrHmd_EndEyeRender.
// If rendering one eye at a time, it is best to render eye specified by
// HmdDesc.EyeRenderOrder[0] first.
OVR_EXPORT ovrPosef ovrHmd_BeginEyeRender(ovrHmd hmd, ovrEyeType eye);
OVR_EXPORT ovrPosef ovrHmd_BeginEyeRender(ovrHmd hmd, ovrEyeType eye);

// Marks the end of eye rendering and submits the eye texture for display after it is ready.
// Rendering viewport within the texture can change per frame if necessary.
// Specified texture may be presented immediately or wait until ovrHmd_EndFrame based
// on the implementation. The API performs distortion and scaling internally.
// 'renderPose' will typically be the value returned from ovrHmd_BeginEyeRender, but can
// be different if a different pose was used for rendering.
OVR_EXPORT void     ovrHmd_EndEyeRender(ovrHmd hmd, ovrEyeType eye, ovrPosef renderPose, ovrTexture* eyeTexture);
OVR_EXPORT void     ovrHmd_EndEyeRender(ovrHmd hmd, ovrEyeType eye, ovrPosef renderPose, ovrTexture* eyeTexture);
                                        
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
OVR_EXPORT ovrEyeRenderDesc ovrHmd_GetRenderDesc(ovrHmd hmd, ovrEyeType eyeType, ovrFovPort fov);
OVR_EXPORT ovrEyeRenderDesc ovrHmd_GetRenderDesc(ovrHmd hmd, ovrEyeType eyeType, ovrFovPort fov);


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

// Computes updated 'uvScaleOffsetOut' to be used with a distortion if render target size or
// viewport changes after the fact. This can be used to adjust render size every frame, if desired.
func (hmd *Hmd) GetRenderScaleAndOffset(ovrFovPort fov, ovrSizei textureSize, ovrRecti renderViewport, ovrVector2f uvScaleOffsetOut[2] );
OVR_EXPORT void     ovrHmd_GetRenderScaleAndOffset( ovrFovPort fov, ovrSizei textureSize, ovrRecti renderViewport, ovrVector2f uvScaleOffsetOut[2] );


// Thread-safe timing function for the main thread. Caller should increment frameIndex
// with every frame and pass the index to RenderThread for processing.
func (hmd *Hmd) GetFrameTiming(hmd.hmd(), frameIndex uint);
OVR_EXPORT ovrFrameTiming ovrHmd_GetFrameTiming(ovrHmd hmd, unsigned int frameIndex);

// Called at the beginning of the frame on the Render Thread.
// Pass frameIndex == 0 if ovrHmd_GetFrameTiming isn't being used. Otherwise,
// pass the same frame index as was used for GetFrameTiming on the main thread.
func (hmd *Hmd) BeginFrameTiming(hmd.hmd(), frameIndex uint);
OVR_EXPORT ovrFrameTiming ovrHmd_BeginFrameTiming(ovrHmd hmd, unsigned int frameIndex);

// Marks the end of game-rendered frame, tracking the necessary timing information. This
// function must be called immediately after Present/SwapBuffers + GPU sync. GPU sync is important
// before this call to reduce latency and ensure proper timing.
func (hmd *Hmd) EndFrameTiming() {
	C.ovrHmd_EndFrameTiming(hmd.hmd());
}

// Initializes and resets frame time tracking. This is typically not necessary, but
// is helpful if game changes vsync state or video mode. vsync is assumed to be on if this
// isn't called. Resets internal frame index to the specified number.
func (hmd *Hmd) ResetFrameTiming(frameIndex uint) {
	C.ResetFrameTiming(hmd.hmd(), C.uint(frameIndex))
}

// Predicts and returns Pose that should be used rendering the specified eye.
// Must be called between ovrHmd_BeginFrameTiming & ovrHmd_EndFrameTiming.
func (hmd *Hmd) GetEyePose(ovrEyeType eye){ 
	OVR_EXPORT ovrPosef ovrHmd_GetEyePose(ovrHmd hmd, ovrEyeType eye);
}

// Computes timewarp matrices used by distortion mesh shader, these are used to adjust
// for orientation change since the last call to ovrHmd_GetEyePose for this eye.
// The ovrDistortionVertex::TimeWarpFactor is used to blend between the matrices,
// usually representing two different sides of the screen.
// Must be called on the same thread as ovrHmd_BeginFrameTiming.
func (hmd *Hmd) GetEyeTimewarpMatrices(hmd.hmd(), ovrEyeType eye, ovrPosef renderPose, ovrMatrix4f twmOut[2]);
OVR_EXPORT void     ovrHmd_GetEyeTimewarpMatrices(ovrHmd hmd, ovrEyeType eye, ovrPosef renderPose, ovrMatrix4f twmOut[2]);

//-------------------------------------------------------------------------------------
// ***** Stateless math setup functions

// Used to generate projection from ovrEyeDesc::Fov.
func (hmd *Hmd) ix4f_Projection(ovrFovPort fov, float znear, float zfar, ovrBool rightHanded );
OVR_EXPORT ovrMatrix4f ovrMatrix4f_Projection( ovrFovPort fov, float znear, float zfar, ovrBool rightHanded );
                                               
// Used for 2D rendering, Y is down
// orthoScale = 1.0f / pixelsPerTanAngleAtCenter
// orthoDistance = distance from camera, such as 0.8m
func (hmd *Hmd) ix4f_OrthoSubProjection(ovrMatrix4f projection, ovrVector2f orthoScale, float orthoDistance, float eyeViewAdjustX);
OVR_EXPORT ovrMatrix4f ovrMatrix4f_OrthoSubProjection(ovrMatrix4f projection, ovrVector2f orthoScale, float orthoDistance, float eyeViewAdjustX);

// Returns global, absolute high-resolution time in seconds. This is the same
// value as used in sensor messages.
func WaitTimeInSeconds() float64 {
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
func (hmd *Hmd) Hmd_ProcessLatencyTest(r,g,b byte) bool {
	rgbColorOut := [3]C.uchar{r,g,b}
	return 0 != C.ovrHmd_ProcessLatencyTest(hmd.hmd(), &rgbColorOut)
}

// Returns non-null string once with latency test result, when it is available.
// Buffer is valid until next call.
func (hmd *Hmd)  GetLatencyTestResult() string {
	return C.GoString(C.ovrHmd_GetLatencyTestResult(hmd.hmd()))
}

func (hmd *Hmd) GetMeasuredLatencyTest2() float64 {
	return float64(C.ovrHmd_GetMeasuredLatencyTest2(hmd.hmd()))
}

const (
	KeyUser = "User"
	KeyName = "Name"
	KeyGender = "Gender"
	KeyPlayerHeight = "PlayerHeight"
	KeyEyeHeight = "EyeHeight"
	KeyIPD = "IPD"
	KeyNeckToEyeHorizontal = "NeckEyeHori"
	KeyNeckToEyeVertical = "NeckEyeVert"
	DefaultGender = "Male"

	DefaultPlayerHeight = C.OVR_DEFAULT_PLAYER_HEIGHT           
    DefaultEyeHeight = C.OVR_DEFAULT_EYE_HEIGHT              
    DefaultIPD = C.OVR_DEFAULT_IPD                     
    DefaultNeckToEyeHorizental = C.OVR_DEFAULT_NECK_TO_EYE_HORIZONTAL 
    DefaultNeckToEyeVertical = C.OVR_DEFAULT_NECK_TO_EYE_VERTICAL   
)


func (hmd *Hmd) GetFloat(propertyName string, defaultVal float32) float32 {
	_propertyName := C.CString(propertyName)
	defer C.Free(unsafe.Pointer(_propertyName))
	return float32(C.ovrHmd_GetFloat(hmd.hmd(), _propertyName, float32(defaultVal)))
}


func (hmd *Hmd) SetFloat(propertyName string, value float32) bool {
	_propertyName := C.CString(propertyName)
	defer C.Free(unsafe.Pointer(_propertyName))
	return bool(C.ovrHmd_SetFloat(hmd.hmd(), _propertyName, float32(value)))
}

// Get float[] property. Returns the number of elements filled in, 0 if property doesn't exist.
// Maximum of arraySize elements will be written.
func (hmd *Hmd) GetFloatArray(propertyName string) []float32 {
	_propertyName := C.CString(propertyName)
	defer C.Free(unsafe.Pointer(_propertyName))
	arraySize := C.ovrHmd_GetArraySize(hmd.hmd(), _propertyName)
	values := make([]float32, arraySize)
	arrayPtr := (*C.float)(&values[0])
	return float32(C.ovrHmd_GetFloatArray(hmd.hmd(), _propertyName, float32(defaultVal)))
	C.ovrHmd_GetFloatArray
	return values
}

func (hmd *Hmd) SetFloatArray(propertyName string, values []float32) bool {
	arraySize := C.uint(len(values))
	_values := (*C.float)(&values[0])
	_propertyName := C.CString(propertyName)
	defer C.Free(unsafe.Pointer(_propertyName))
	return bool(ovrHmd_SetFloatArray(hmd.hmd(), _propertyName, values, arraySize))
}

func (hmd *Hmd) GetString(propertyName string) string {
	_propertyName := C.CString(propertyName)
	defer C.Free(unsafe.Pointer(_propertyName))
	return C.GoString(C.ovrHmd_GetString(hmd.hmd(),_propertyName))
}
