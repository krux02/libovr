package ovr

//#include "OVR_CAPI.h"
//#include "OVR_CAPI_GL.h"
import "C"

type Vector2i C.ovrVector2i
type Sizei C.ovrSizei
type Recti C.ovrRecti
type Quatf C.ovrQuatf
type Vector2f C.ovrVector2f
type Vector3f C.ovrVector3f
type Matrix4f C.ovrMatrix4f
type Posef C.ovrPosef
type PoseStatef C.ovrPoseStatef
type FovPort C.ovrFovPort
type HmdType C.ovrHmdType
type HmdCaps C.ovrHmdCaps
type SensorCaps C.ovrSensorCaps
type DistortionCaps C.ovrDistortionCaps
type EyeType C.ovrEyeType
type Hmd C.struct_ovrHmdStruct
type StatusBits C.ovrStatusBits
type SensorState C.ovrSensorState
type SensorDesc C.ovrSensorDesc
type FrameTiming C.ovrFrameTiming
type EyeRenderDesc C.ovrEyeRenderDesc
type RenderApiType C.ovrRenderAPIType
type RenderApiConfigHeader C.ovrRenderAPIConfigHeader
type RenderApiConfig C.ovrRenderAPIConfig
type TextureHeader C.ovrTextureHeader
type Texture C.ovrTexture

type GLConfigData C.ovrGLConfigData
type GLTextureData C.ovrGLTextureData
type GLTexture C.ovrGLTexture
type GLConfig C.union_ovrGLConfig

//type HmdDesc C.ovrHmdDesc
