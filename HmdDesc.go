package ovr

//#include "OVR_CAPI.h"
import "C"

import "unsafe"

/*
type HmdDesc struct {
	Handle            *Hmd
	Type              HmdType
	Pad_cgo_0         [4]byte
	productName       *int8
	manufacturer      *int8
	HmdCaps           HmdCaps
	SensorCaps        SensorCaps
	DistortionCaps    DistorionCaps
	Resolution        Sizei
	WindowsPos        Vector2i
	DefaultEyeFov     [2]FovPort
	MaxEyeFov         [2]FovPort
	EyeRenderOrder    [2]uint32
	Pad_cgo_1         [4]byte
	displayDeviceName *int8
	DisplayId         int32
	Pad_cgo_2         [4]byte
}

func (this *HmdDesc) ProductName() string {
	return C.GoString(this.productName)
}

func (this *HmdDesc) Manufacturer() string {
	return C.GoString(this.manufacturer)
}

func (this *hmdDesc) DisplayDeviceNamee() string {
	return C.GoString(this.displayDeviceName)
}
*/

// This is a complete descriptor of the HMD.
type HmdDesc struct {
	Handle *Hmd // Handle of this HMD.
	Type   HmdType

	ProductName, Manufacturer string

	// Capability bits described by ovrHmdCaps.
	HmdCaps HmdCaps
	// Capability bits described by ovrSensorCaps.
	SensorCaps SensorCaps
	// Capability bits described by ovrDistortionCaps.
	DistortionCaps DistortionCaps

	// Resolution of the entire HMD screen (for both eyes) in pixels.
	Resolution Sizei
	WindowsPos Vector2i

	// These define the recommended and maximum optical FOVs for the HMD.
	DefaultEyeFov  [Eye_Count]FovPort
	MaxEyeFov      [Eye_Count]FovPort
	EyeRenderOrder [Eye_Count]EyeType

	DisplayDeviceName string
	DisplayId         int
}

//type Hmd C.struct_ovrHmdStruct

func hmd(h C.ovrHmd) *Hmd {
	return (*Hmd)(unsafe.Pointer(h))
}

func (this *Hmd) cptr() C.ovrHmd {
	return C.ovrHmd(unsafe.Pointer(this))
}

func hmdDesc(that C.ovrHmdDesc) (this HmdDesc) {
	this.Handle = hmd(that.Handle)
	this.Type = HmdType(that.Type)
	this.ProductName = C.GoString(that.ProductName)
	this.Manufacturer = C.GoString(that.Manufacturer)
	this.HmdCaps = HmdCaps(that.HmdCaps)
	this.SensorCaps = SensorCaps(that.SensorCaps)
	this.Resolution = sizei(that.Resolution)
	this.WindowsPos = vector2i(that.WindowsPos)
	for i := 0; i < 2; i++ {
		this.DefaultEyeFov[i] = fovPort(that.DefaultEyeFov[i])
		this.MaxEyeFov[i] = fovPort(that.MaxEyeFov[i])
		this.EyeRenderOrder[i] = EyeType(that.EyeRenderOrder[i])
	}
	this.DisplayDeviceName = C.GoString(that.DisplayDeviceName)
	this.DisplayId = int(that.DisplayId)
	return this
}
