package ovr

import "unsafe"

func (this *GLConfig) Config() *RenderApiConfig {
	return (*RenderApiConfig)(unsafe.Pointer(this))
}

func (this *GLConfig) OGL() *GLConfigData {
	return (*GLConfigData)(unsafe.Pointer(this))
}

func (this *GLTexture) Texture() *Texture {
	return (*Texture)(unsafe.Pointer(this))
}

func (this *GLTexture) OGL() *GLTextureData {
	return (*GLTextureData)(unsafe.Pointer(this))
}
