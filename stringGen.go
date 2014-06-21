package ovr

import "strings"

func (value EyeType) String() string {
	switch value {
	case Eye_Left:
		return "Eye_Left"
	case Eye_Right:
		return "Eye_Right"
	case Eye_Count:
		return "Eye_Count"
	default:
		panic("invalid eye type")
	}
}

func (value HmdType) String() string {
	switch value {
	case Hmd_None:
		return "Hmd_None"
	case Hmd_DK1:
		return "Hmd_DK1"
	case Hmd_DKHD:
		return "Hmd_DKHD"
	case Hmd_CrystalCoveProto:
		return "Hmd_CrystalCoveProto"
	case Hmd_DK2:
		return "Hmd_DK2"
	case Hmd_Other:
		return "Hmd_Other"
	default:
		panic("invalid HmdType type")
	}
}

func (value HmdCaps) String() string {

	caps := [...]HmdCaps{
		HmdCap_Present,
		HmdCap_Available,
		HmdCap_LowPersistence,
		HmdCap_LatencyTest,
		HmdCap_DynamicPrediction,
		HmdCap_NoVSync,
		HmdCap_NoRestore,
	}
	names := [...]string{
		"HmdCap_Present",
		"HmdCap_Available",
		"HmdCap_LowPersistence",
		"HmdCap_LatencyTest",
		"HmdCap_DynamicPrediction",
		"HmdCap_NoVSync",
		"HmdCap_NoRestore",
	}

	var filtered []string = nil

	for i, c := range caps {
		if c&value != 0 {
			filtered = append(filtered, names[i])
		}
	}

	return strings.Join(filtered, " | ")
}

func (value SensorCaps) String() string {

	caps := [...]SensorCaps{
		SensorCap_Orientation,
		SensorCap_YawCorrection,
		SensorCap_Position,
	}
	names := [...]string{
		"SensorCap_Orientation",
		"SensorCap_YawCorrection",
		"SensorCap_Position",
	}

	var filtered []string = nil

	for i, c := range caps {
		if c&value != 0 {
			filtered = append(filtered, names[i])
		}
	}

	return strings.Join(filtered, " | ")
}

func (value DistortionCaps) String() string {
	caps := [...]DistortionCaps{
		DistortionCap_Chromatic,
		DistortionCap_TimeWarp,
		DistortionCap_Vignette,
	}
	names := [...]string{
		"DistortionCap_Chromatic",
		"DistortionCap_TimeWarp",
		"DistortionCap_Vignette",
	}

	var filtered []string = nil

	for i, c := range caps {
		if c&value != 0 {
			filtered = append(filtered, names[i])
		}
	}

	return strings.Join(filtered, " | ")

}

func (value RenderApiType) String() string {
	switch value {
	case RenderAPI_None:
		return "RenderAPI_None"
	case RenderAPI_OpenGL:
		return "RenderAPI_OpenGL"
	case RenderAPI_Android_GLES:
		return "RenderAPI_Android_GLES"
	case RenderAPI_D3D9:
		return "RenderAPI_D3D9"
	case RenderAPI_D3D10:
		return "RenderAPI_D3D10"
	case RenderAPI_D3D11:
		return "RenderAPI_D3D11"
	case RenderAPI_Count:
		return "RenderAPI_Count"
	default:
		panic("invalid render api type")
	}
}
