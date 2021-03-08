/******************************************************************************
*       SOFA, Simulation Open-Framework Architecture, development version     *
*                (c) 2006-2019 INRIA, USTL, UJF, CNRS, MGH                    *
*                                                                             *
* This program is free software; you can redistribute it and/or modify it     *
* under the terms of the GNU Lesser General Public License as published by    *
* the Free Software Foundation; either version 2.1 of the License, or (at     *
* your option) any later version.                                             *
*                                                                             *
* This program is distributed in the hope that it will be useful, but WITHOUT *
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or       *
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License *
* for more details.                                                           *
*                                                                             *
* You should have received a copy of the GNU Lesser General Public License    *
* along with this program. If not, see <http://www.gnu.org/licenses/>.        *
*******************************************************************************
* Authors: The SOFA Team and external contributors (see Authors.txt)          *
*                                                                             *
* Contact information: contact@sofa-framework.org                             *
******************************************************************************/

#include "ProjectiveSetup.h"
#include <sofa/core/ObjectFactory.h>
#include <iostream>

#include <sstream>
#include <sofa/core/visual/VisualParams.h>
#include <sofa/defaulttype/BoundingBox.h>
#include <limits>


namespace sofa
{

namespace component
{

namespace visualmodel
{

int ProjectiveSetupClass = core::RegisterObject("ProjectiveSetup")
        .add< ProjectiveSetup >()
        ;


ProjectiveSetup::ProjectiveSetup()
	: d_showImg( initData (&d_showImg, bool(true), "showImg", "show/hide background image") )
	, d_viewType( initData (&d_viewType, bool(true), "viewType", "perspective/ortho background image") )
	, d_cameraFactor( initData (&d_cameraFactor, float(1.0), "cameraFactor", "apply scaling factor to the input image and points") )
	, d_filename( initData(&d_filename, "fileName", "Image filename") )
	, d_cameraPosition( initData(&d_cameraPosition, "cameraPosition", "Position of the camera") )
	, d_inputImagePoints( initData(&d_inputImagePoints, "inputImagePoints", "Input vector of image points in pixel coordinates") )
	, d_outputImagePoints( initData(&d_outputImagePoints, "outputImagePoints", "Input vector of model points in metric coordinates") )
	, d_drawSightlines( initData(&d_drawSightlines, "drawSightlines", "Draw the lines from the camera to the image points") )
{

}

ProjectiveSetup::~ProjectiveSetup()
{
}

void ProjectiveSetup::printPretty44Mat(GLdouble M[16])
{
	   std::cout << "[0] " << M[0] << "   ";
	   std::cout << "[4] " << M[4] << "   ";
	   std::cout << "[8] " << M[8] << "   ";
	   std::cout << "[12] " << M[12] << std::endl;
	   std::cout << "[1] " << M[1] << "   ";
	   std::cout << "[5] " << M[5] << "   ";
	   std::cout << "[9] " << M[9] << "   ";
	   std::cout << "[13] " << M[13] << std::endl;
	   std::cout << "[2] " << M[2] << "   ";
	   std::cout << "[6] " << M[6] << "   ";
	   std::cout << "[10] " << M[10] << "   ";
	   std::cout << "[14] " << M[14] << std::endl;
	   std::cout << "[3] " << M[3] << "   ";
	   std::cout << "[7] " << M[7] << "   ";
	   std::cout << "[11] " << M[11] << "   ";
	   std::cout << "[15] " << M[15] << std::endl << std::endl;
}

void ProjectiveSetup::init()
{
    float cameraFactor = d_cameraFactor.getValue();
    const char* filename = d_filename.getFullPath().c_str();
    cv::Mat img = cv::imread(filename,  cv::IMREAD_COLOR);
    int imgWidth = img.cols;
    int imgHeight = img.rows;

    VecCoord inImgPts = d_inputImagePoints.getValue();
    helper::WriteAccessor<Data<VecCoord> > outImgPts = d_outputImagePoints;
    outImgPts.resize(inImgPts.size());	

    // apply metric to pixel transform
    for(unsigned int i=0;i < inImgPts.size();i++) 
    {
	// equivalent to multiply by camera matrix (fx, 0, u, 0, fy, v, 0, 0, 1)
	Coord pt = Coord((inImgPts[i][0] - imgWidth/2)*cameraFactor, (inImgPts[i][1] - imgHeight/2)*(-1)*cameraFactor, 0); 
	outImgPts[i] = pt;
    }
}



void ProjectiveSetup::drawVisual(const core::visual::VisualParams* vparams)
{
    //int vpWidth = vparams->viewport()[2];
    //int vpHeight = vparams->viewport()[3];

    //GLdouble currMat[16];
    //glGetDoublev (GL_PROJECTION_MATRIX, currMat);
    //printPretty44Mat(currMat);

    const char* filename = d_filename.getFullPath().c_str();
    cv::Mat img = cv::imread(filename,  cv::IMREAD_COLOR);
    //cv::flip(img, img, -1);
    int imgWidth = img.cols;
    int imgHeight = img.rows;
    unsigned char* imgData = img.data;

    float cameraFactor = d_cameraFactor.getValue();

    const sofa::defaulttype::Vector3& cameraPos = d_cameraPosition.getValue();
    const sofa::defaulttype::Vec4f& colour = sofa::defaulttype::Vec4f(0.8,0.8,0.1,1);

    VecCoord imgPts = d_inputImagePoints.getValue();
    if (d_drawSightlines.getValue())
    	for(unsigned int i=0; i< imgPts.size(); i++) {
		Coord pt = Coord((imgPts[i][0] - imgWidth/2)*cameraFactor, (imgPts[i][1] - imgHeight/2)*(-1)*cameraFactor, 0); 
		vparams->drawTool()->drawLine( cameraPos, pt, colour);
	}



    if (d_showImg.getValue())
    {
	if(d_viewType.getValue())
	{
	// PERSPECTIVE
	
		glEnable(GL_TEXTURE_2D);	// enable the texture
		glDisable(GL_LIGHTING);		// disable the light

		glBindTexture ( GL_TEXTURE_2D, 0 );  // texture bind
		glTexImage2D (GL_TEXTURE_2D, 0, GL_RGB, imgWidth, imgHeight, 0, GL_BGR_EXT, GL_UNSIGNED_BYTE, imgData );
		//glTexImage2D (GL_TEXTURE_2D, 0, GL_LUMINANCE, m_imageWidth, m_imageHeight, 0, GL_BGR_EXT, GL_UNSIGNED_BYTE, m_imgData.c_str() );

		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);	// Linear Filtering

		float eps = 0.0;
		float z0 = 0.0;

		glBegin(GL_QUADS); //we draw a quad on the entire screen (0,1 - 1,1 - 1,0 - 0,0)
		glColor4f(1.0f,1.0f,1.0f,0.5f);

		float x0 = (float)imgWidth*cameraFactor;
		float y0 = (float)imgHeight*cameraFactor;


		glTexCoord2f(0,1);
		glVertex3f(-x0/2 - eps,-y0/2 - eps, -z0);
		glTexCoord2f(1,1);
		glVertex3f(x0/2 + eps, -y0/2 - eps, -z0);
		glTexCoord2f(1,0);
		glVertex3f(x0/2 + eps, y0/2 + eps, -z0);
		glTexCoord2f(0,0);
		glVertex3f(-x0/2 - eps, y0/2 + eps, -z0);
		glEnd();

/*
		glTexCoord2f(0,1);
		glVertex3f(0,y0,0);
		glTexCoord2f(1,1);
		glVertex3f(x0, y0,0);
		glTexCoord2f(1,0);
		glVertex3f(x0, 0,0);
		glTexCoord2f(0,0);
		glVertex3f(0, 0, 0);
		glEnd();
*/

		// glEnable(GL_DEPTH_TEST);
		glEnable(GL_LIGHTING);		// enable light
		glDisable(GL_TEXTURE_2D);	// disable texture 2D
		//glDepthMask (GL_TRUE);		// enable zBuffer
	}
	else
	{
	// ORTHO

		glMatrixMode(GL_PROJECTION);	//init the projection matrix
		glPushMatrix();
		glLoadIdentity();
		glOrtho(0,1,0,1,-1,1);  // orthogonal view		
		glMatrixMode(GL_MODELVIEW);  
		glPushMatrix(); 
		glLoadIdentity(); 

		// BACKGROUND TEXTURING
		//glDepthMask (GL_FALSE);		// disable the writing of zBuffer
		glDisable(GL_DEPTH_TEST);
		glEnable(GL_TEXTURE_2D);	// enable the texture	
		glDisable(GL_LIGHTING);		// disable the light
				
		glBindTexture ( GL_TEXTURE_2D, 0 );  // texture bind	
		glTexImage2D (GL_TEXTURE_2D, 0, GL_RGB, imgWidth, imgHeight, 0, GL_BGR_EXT, GL_UNSIGNED_BYTE, imgData ); 	

		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);	// Linear Filtering
		 
		// BACKGROUND DRAWING		 
		//glEnable(GL_DEPTH_TEST);		

		glBegin(GL_QUADS); //we draw a quad on the entire screen (0,0 1,0 1,1 0,1)  
			glColor4f(1.0f,1.0f,1.0f,1.0f); 
			glTexCoord2f(0,1);		glVertex2f(0,0);
			glTexCoord2f(1,1);		glVertex2f(1,0);
			glTexCoord2f(1,0);		glVertex2f(1,1);
			glTexCoord2f(0,0);		glVertex2f(0,1);
		glEnd();             

		//glEnable(GL_DEPTH_TEST);
		glEnable(GL_LIGHTING);		// enable light
		glDisable(GL_TEXTURE_2D);	// disable texture 2D	
		glEnable(GL_DEPTH_TEST);
		//glDepthMask (GL_TRUE);		// enable zBuffer
				
		glPopMatrix();
		glMatrixMode(GL_PROJECTION);
		glPopMatrix();
		glMatrixMode(GL_MODELVIEW);
	}

    }

}

void ProjectiveSetup::updateVisual()
{



}



void ProjectiveSetup::initVisual()
{
}


} // namespace visual

} // namespace component

} // namespace sofa
