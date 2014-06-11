package ovr

//#include "OVR_CAPI.h"
import "C"

const (
	Eye_Left  = EyeType(C.ovrEye_Left)
	Eye_Right = EyeType(C.ovrEye_Right)
	Eye_Count = EyeType(C.ovrEye_Count)
)

const (
	Hmd_None             = HmdType(C.ovrHmd_None)
	Hmd_DK1              = HmdType(C.ovrHmd_DK1)
	Hmd_DKHD             = HmdType(C.ovrHmd_DKHD)
	Hmd_CrystalCoveProto = HmdType(C.ovrHmd_CrystalCoveProto)
	Hmd_DK2              = HmdType(C.ovrHmd_DK2)
	Hmd_Other            = HmdType(C.ovrHmd_Other)
)

const (
	HmdCap_Present           = HmdCaps(C.ovrHmdCap_Present)
	HmdCap_Available         = HmdCaps(C.ovrHmdCap_Available)
	HmdCap_LowPersistence    = HmdCaps(C.ovrHmdCap_LowPersistence)
	HmdCap_LatencyTest       = HmdCaps(C.ovrHmdCap_LatencyTest)
	HmdCap_DynamicPrediction = HmdCaps(C.ovrHmdCap_DynamicPrediction)
	HmdCap_NoVSync           = HmdCaps(C.ovrHmdCap_NoVSync)
	HmdCap_NoRestore         = HmdCaps(C.ovrHmdCap_NoRestore)
	HmdCap_Writable_Mask     = HmdCaps(C.ovrHmdCap_Writable_Mask)
)

// Sensor capability bits reported by device.
// Used with ovrHmd_StartSensor.
const (
	SensorCap_Orientation   = SensorCaps(C.ovrSensorCap_Orientation)
	SensorCap_YawCorrection = SensorCaps(C.ovrSensorCap_YawCorrection)
	SensorCap_Position      = SensorCaps(C.ovrSensorCap_Position)
)

const (
	DistortionCap_Chromatic = DistortionCaps(C.ovrDistortionCap_Chromatic)
	DistortionCap_TimeWarp  = DistortionCaps(C.ovrDistortionCap_TimeWarp)
	DistortionCap_Vignette  = DistortionCaps(C.ovrDistortionCap_Vignette)
)

const (
	Status_OrientationTracked = StatusBits(C.ovrStatus_OrientationTracked)
	Status_PositionTracked    = StatusBits(C.ovrStatus_PositionTracked)
	Status_PositionConnected  = StatusBits(C.ovrStatus_PositionConnected)
	Status_HmdConnected       = StatusBits(C.ovrStatus_HmdConnected)
)

const (
	RenderAPI_None         = RenderApiType(C.ovrRenderAPI_None)
	RenderAPI_OpenGL       = RenderApiType(C.ovrRenderAPI_OpenGL)
	RenderAPI_Android_GLES = RenderApiType(C.ovrRenderAPI_Android_GLES)
	RenderAPI_D3D9         = RenderApiType(C.ovrRenderAPI_D3D9)
	RenderAPI_D3D10        = RenderApiType(C.ovrRenderAPI_D3D10)
	RenderAPI_D3D11        = RenderApiType(C.ovrRenderAPI_D3D11)
	RenderAPI_Count        = RenderApiType(C.ovrRenderAPI_Count)
)
