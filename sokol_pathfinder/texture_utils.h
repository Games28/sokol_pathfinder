#pragma once
#ifndef TEXTURE_UTIL_H
#define TEXTURE_UTIL_H

#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"

static sg_view makeTextureFromPixels(std::uint32_t* pixels, int width, int height) {
	sg_image_desc image_desc{};
	image_desc.width=width;
	image_desc.height=height;
	image_desc.data.mip_levels[0].ptr=pixels;
	image_desc.data.mip_levels[0].size=sizeof(std::uint32_t)*width*height;
	sg_image image=sg_make_image(image_desc);

	sg_view_desc view_desc{};
	view_desc.texture.image=image;
	return sg_make_view(view_desc);
}

static sg_view makeBlankTexture() {
	std::uint32_t pixel[1]{0xFFFFFFFF};
	return makeTextureFromPixels(pixel, 1, 1);
}

static sg_view makeUVTexture(int width, int height) {
	std::uint32_t* pixels=new std::uint32_t[width*height];
	for(int j=0; j<height; j++) {
		for(int i=0; i<width; i++) {
			float u=(.5f+i)/width;
			float v=(.5f+j)/height;

			int k=i+width*j;
			int r=255*u;
			int g=255*v;
			int b=127;
			int a=255;
			pixels[k]=(a<<24)|(b<<16)|(g<<8)|r;
		}
	}

	sg_view view=makeTextureFromPixels(pixels, width, height);
	delete[] pixels;

	return view;
}

[[nodiscard]] static ReturnCode makeTextureFromFile(sg_view& view, const std::string& filename) {
	int width, height, channels;
	unsigned char* pixels8=stbi_load(filename.c_str(), &width, &height, &channels, 4);
	if(!pixels8) return {false, "invalid image file"};

	std::uint32_t* pixels32=new std::uint32_t[width*height];
	std::memcpy(pixels32, pixels8, sizeof(std::uint8_t)*4*width*height);
	stbi_image_free(pixels8);

	view=makeTextureFromPixels(pixels32, width, height);
	delete[] pixels32;

	return {true, "success"};
}
#endif