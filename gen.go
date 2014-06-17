// Created by cgo -godefs - DO NOT EDIT
// cgo -godefs=true src4gen/test.go

package ovr

type Vector2i struct {
	X int32
	Y int32
}
type Sizei struct {
	W int32
	H int32
}
type Recti struct {
	Pos  Vector2i
	Size Sizei
}
type Quatf struct {
	X float32
	Y float32
	Z float32
	W float32
}
type Vector2f struct {
	X float32
	Y float32
}
type Vector3f struct {
	X float32
	Y float32
	Z float32
}
type Matrix4f struct {
	M [4][4]float32
}
type Posef struct {
	Orientation Quatf
	Position    Vector3f
}
type PoseStatef struct {
	Pose                Posef
	AngularVelocity     Vector3f
	LinearVelocity      Vector3f
	AngularAcceleration Vector3f
	LinearAcceleration  Vector3f
	Pad_cgo_0           [4]byte
	TimeInSeconds       float64
}
type FovPort struct {
	UpTan    float32
	DownTan  float32
	LeftTan  float32
	RightTan float32
}
type HmdType uint32
type HmdCaps uint32
type SensorCaps uint32
type DistortionCaps uint32
type EyeType uint32
type Hmd [0]byte

type StatusBits uint32
type SensorState struct {
	Predicted   PoseStatef
	Recorded    PoseStatef
	Temperature float32
	StatusFlags StatusBits
}
type SensorDesc struct {
	VendorId     int16
	ProductId    int16
	SerialNumber [24]int8
}
type FrameTiming struct {
	DeltaSeconds           float32
	Pad_cgo_0              [4]byte
	ThisFrameSeconds       float64
	TimewarpPointSeconds   float64
	NextFrameSeconds       float64
	ScanoutMidpointSeconds float64
	EyeScanoutSeconds      [2]float64
}
type EyeRenderDesc struct {
	Eye                       EyeType
	Fov                       FovPort
	DistortedViewport         Recti
	PixelsPerTanAngleAtCenter Vector2f
	ViewAdjust                Vector3f
}

type RenderApiType uint32
type RenderApiConfigHeader struct {
	API         RenderApiType
	RTSize      Sizei
	Multisample int32
}
type RenderApiConfig struct {
	Header       RenderApiConfigHeader
	PlatformData [8]uint64
}
type TextureHeader struct {
	API            RenderApiType
	TextureSize    Sizei
	RenderViewport Recti
}
type Texture struct {
	Header       TextureHeader
	Pad_cgo_0    [4]byte
	PlatformData [8]uint64
}
type GLConfigData struct {
	Header RenderApiConfigHeader
}
type GLTextureData struct {
	Header TextureHeader
	TexId  uint32
}
type GLTexture [96]byte
type GLConfig [80]byte
