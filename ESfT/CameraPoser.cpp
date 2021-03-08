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

#include "CameraPoser.h"
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

int CameraPoserClass = core::RegisterObject("CameraPoser")
        .add< CameraPoser >()
        ;


CameraPoser::CameraPoser()
	: d_rotationStep( initData (&d_rotationStep, float(1.0), "rotationStep", "Step for the rotation") )
	, d_translationStep( initData(&d_translationStep, float(1.0), "translationStep", "Step for the translation") )
	, d_outRotation( initData(&d_outRotation, "outRotation", "Output rotation") )
	, d_outTranslation( initData(&d_outTranslation, "d_outTranslation", "Output translation") )
	, d_inTranslation( initData(&d_inTranslation, "inTranslation", "Input translation") )
	, d_inRotation( initData(&d_inRotation, "inRotation", "Input rotation") )
{
    f_listening.setValue(true);
}

CameraPoser::~CameraPoser()
{
}


void CameraPoser::saveImage(int i, int w, int h)
{

    int sofaH = 78;
    int sofaW = 508;
    int sofaHH = 39;

    int newH = h + sofaH; 
    int newW = w + sofaW;

    cv::Mat img(newH, newW, CV_8UC3);
    //glPixelStorei(GL_PACK_ALIGNMENT, (img.step & 3)?1:4);
    glPixelStorei(GL_PACK_ALIGNMENT, 1);
    glPixelStorei(GL_PACK_ROW_LENGTH, img.step/img.elemSize());
    glReadPixels(0, 0, img.cols, img.rows, GL_BGR_EXT, GL_UNSIGNED_BYTE, img.data);
    cv::Mat flipped(img);
    cv::flip(img, flipped, 0);

    cv::Rect cropRegion(sofaW, sofaHH, w, h);
    cv::Mat cropped;
    cropped = img(cropRegion);

    char filename[80];
    sprintf(filename,"./outputTest/proj/proj_%06d.png", i);
    cv::imwrite(filename, cropped);
    //cv::imwrite(filename, img);	
}

void CameraPoser::savePose(int i, helper::Quater< Real > q, Vec3Real t)
{
    char filename[80];
    sprintf(filename,"./outputTest/pose/pose_%06d.txt", i);

    std::ofstream textfile;
    textfile.open(filename);


    textfile << q[0] << " ";
    textfile << q[1] << " ";
    textfile << q[2] << " ";
    textfile << q[3] << " ";
    textfile << t[0] << " ";
    textfile << t[1] << " ";
    textfile << t[2];

    textfile.close();

}

void CameraPoser::printPretty44Mat(GLdouble M[16])
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
/*
void CameraPoser::handleEvent(sofa::core::objectmodel::Event *event)
{

    f_listening.setValue(true);
    

    //if (sofa::core::objectmodel::MouseEvent::checkEventType(event))
    {
	//std::cout << m_im << std::endl;
        sofa::core::objectmodel::MouseEvent *me = static_cast<sofa::core::objectmodel::MouseEvent *>(event);

	//std::cout << me->getState() << std::endl;

        if(me->getState() == sofa::core::objectmodel::MouseEvent::LeftPressed)
	    m_pressed = 1;
        if (me->getState() == sofa::core::objectmodel::MouseEvent::LeftReleased)
	    m_pressed = 0;

        if(m_pressed)
        {
		std::cout << m_im << std::endl;
		//saveImage(m_im, 1200, 1200);        
		//savePose(m_im, modMat);
		m_im++;
	}

    }

}
*/

void CameraPoser::handleEvent(sofa::core::objectmodel::Event *event)
{
    if (simulation::AnimateBeginEvent::checkEventType(event))
    {
	//std::cout << m_im << std::endl;
	if (m_im%10 == 0 && m_im > 0)
		saveImage(m_im, 512, 512);

	m_im++;
    }
}

void CameraPoser::drawVisual(const core::visual::VisualParams* vparams)
{
	GLdouble modMat[16];
	glGetDoublev(GL_MODELVIEW_MATRIX, modMat);
	//printPretty44Mat(modMat);
	defaulttype::Mat<3,3,Real> M;
	M[0][0]=modMat[0];
	M[0][1]=modMat[1];
	M[0][2]=modMat[2];
	M[1][0]=modMat[4];
	M[1][1]=modMat[5];
	M[1][2]=modMat[6];
	M[2][0]=modMat[8];
	M[2][1]=modMat[9];
	M[2][2]=modMat[10];
	
    	helper::Quater< Real > q;
	q.fromMatrix(M);

	Vec3Real t;
	t[0] = modMat[12];
	t[1] = modMat[13];
	t[2] = modMat[14];

	std::cout << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << " " << t[0] << " " << t[1] << " " << t[2] << std::endl;

	if (m_im%10 == 0 && m_im > 0)
		savePose(m_im, q, t);
	
	//int vpWidth = vparams->viewport()[2];
	//int vpHeight = vparams->viewport()[3];

	
	Vec3 inTrans = d_inTranslation.getValue();
	helper::Quater< Real > inQuater = d_inRotation.getValue();
	defaulttype::Mat<3,3,Real> inRotMat;

	inQuater.toMatrix(inRotMat);

	modMat[0] = inRotMat[0][0];
	modMat[1] = inRotMat[0][1];
	modMat[2] = inRotMat[0][2];
	modMat[4] = inRotMat[1][0];
	modMat[5] = inRotMat[1][1];
	modMat[6] = inRotMat[1][2];
	modMat[8] = inRotMat[2][0];
	modMat[9] = inRotMat[2][1];
	modMat[10] = inRotMat[2][2];


	modMat[12] = inTrans[0];
	modMat[13] = inTrans[1];
	modMat[14] = inTrans[2];

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	glLoadMatrixd( modMat );
	
	


}

void CameraPoser::updateVisual()
{
}
void CameraPoser::init()
{

}
void CameraPoser::initVisual()
{

}


} // namespace visual

} // namespace component

} // namespace sofa
