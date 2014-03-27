// Author:  Juergen Gall
// Contact: gall@vision.ee.ethz.ch
// Date: 18.05.2011
// Computer Vision Laboratory, ETH Zurich

// All data is only for research purposes. When using this data, please acknowledge the effort that went into data collection by referencing the corresponding paper:

// Gall J., Fossati A., and van Gool L., Functional Categorization of Objects using Real-time Markerless Motion Capture, IEEE Conference on Computer Vision and Pattern Recognition (CVPR'11), 1969-1976, 2011. 

// ##########################################################################

#include "fbo.h"
#include <iostream>

bool fbo::initialize(int x, int y){

	usecolorbuffer = usedepthbuffer = userenderbuffer = false;
	sizex = x;
	sizey = y;
	
	glGenFramebuffersEXT(1, &framebuffer);
	glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, framebuffer);
	glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, 0);	
	
	return true;
}

bool fbo::addDepthBuffer(){
	
	usedepthbuffer = true;
	glGenTextures(1, &depthbuffer);
	glBindTexture(GL_TEXTURE_RECTANGLE_ARB, depthbuffer);
	glTexImage2D(GL_TEXTURE_RECTANGLE_ARB, 0, GL_DEPTH_COMPONENT24, sizex, sizey, 0, GL_DEPTH_COMPONENT, 
				GL_UNSIGNED_BYTE, NULL);
	glTexParameteri(GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	
	glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, framebuffer);
	glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_DEPTH_ATTACHMENT_EXT, GL_TEXTURE_RECTANGLE_ARB, depthbuffer, 0);
	glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, 0);
	
	return checkFramebufferStatus();
}

bool fbo::checkFramebufferStatus(bool silent){

	GLenum status;
	status = (GLenum) glCheckFramebufferStatusEXT(GL_FRAMEBUFFER_EXT);
	
	switch(status){
		case GL_FRAMEBUFFER_COMPLETE_EXT:
			return true;
		case GL_FRAMEBUFFER_UNSUPPORTED_EXT:
			if (!silent) std::cout << "Framebuffer not supported." << std::endl;
			return false;
		case GL_FRAMEBUFFER_INCOMPLETE_MISSING_ATTACHMENT_EXT:
			if (!silent) std::cout << "Framebuffer incomplete, missing attachment." << std::endl;
			return false;
		case GL_FRAMEBUFFER_INCOMPLETE_ATTACHMENT_EXT:
			if(!silent) std::cout << "Framebuffer incomplete, incomplete attachment." << std::endl;
			return false;
		case GL_FRAMEBUFFER_INCOMPLETE_DIMENSIONS_EXT:
			if(!silent) std::cout << "Framebuffer incomplete, image dimensions wrong." << std::endl;
			return false;
		case GL_FRAMEBUFFER_INCOMPLETE_FORMATS_EXT:
			if (!silent) std::cout << "Framebuffer incomplete, image format wrong." << std::endl;
			return false;
		case GL_FRAMEBUFFER_INCOMPLETE_DRAW_BUFFER_EXT:
			if (!silent) std::cout << "Framebuffer incomplete, missing draw buffer." << std::endl;
			return false;
		case GL_FRAMEBUFFER_INCOMPLETE_READ_BUFFER_EXT:
			if (!silent) std::cout << "Framebuffer incomplete, missing read buffer." << std::endl;
			return false;
		default:
			return false;
	}
	return true;
	
}

bool fbo::addColorBuffer(){
	usecolorbuffer = true;
	
	glGenTextures(1, &colorbuffer);
	glBindTexture(GL_TEXTURE_RECTANGLE_ARB, colorbuffer);
	glTexImage2D(GL_TEXTURE_RECTANGLE_ARB, 0, GL_RGB, sizex, sizey, 0, GL_RGB, 
				GL_UNSIGNED_BYTE, NULL);
	glTexParameteri(GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	
	glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, framebuffer);
	glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT0_EXT, GL_TEXTURE_RECTANGLE_ARB, colorbuffer, 0);
	glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, 0);
	glBindTexture(GL_TEXTURE_RECTANGLE_ARB, 0);
	
	return checkFramebufferStatus();
}

bool fbo::addRenderBuffer(){
	userenderbuffer = true;
	glGenRenderbuffersEXT(1, &renderbuffer);

	glTexImage2D(GL_TEXTURE_RECTANGLE_ARB, 0, GL_RGB, sizex, sizey, 0, GL_RGB, 
				GL_UNSIGNED_BYTE, NULL);
	glTexParameteri(GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	
	glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, framebuffer);
	glBindRenderbufferEXT(GL_RENDERBUFFER_EXT, renderbuffer);
	glRenderbufferStorageEXT(GL_RENDERBUFFER_EXT, GL_DEPTH_COMPONENT24, sizex, sizey);
	glFramebufferRenderbufferEXT(GL_FRAMEBUFFER_EXT, GL_DEPTH_ATTACHMENT_EXT, GL_RENDERBUFFER_EXT, renderbuffer);
	glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, 0);
	
	return checkFramebufferStatus();
}  

void fbo::bind(){
	glViewport(0, 0, sizex, sizey);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, framebuffer);
}

void fbo::unbind(){
	glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, 0);
//	glViewport(0, 0, 800, 600);
}

void fbo::shutdown(){
	
//	if (userenderbuffer)
//		glDeleteRenderbuffersEXT(renderbuffer);
	
//	glDeleteFramebuffersEXT(framebuffer);
}
