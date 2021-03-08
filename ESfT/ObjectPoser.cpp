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

#include "ObjectPoser.h"
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

int ObjectPoserClass = core::RegisterObject("ObjectPoser")
        .add< ObjectPoser >()
        ;


ObjectPoser::ObjectPoser()
	: d_rotationStep( initData (&d_rotationStep, float(1.0), "rotationStep", "Step for the rotation") )
	, d_translationStep( initData(&d_translationStep, float(1.0), "translationStep", "Step for the translation") )
	, d_outRotation( initData(&d_outRotation, "outRotation", "Output rotation") )
	, d_outTranslation( initData(&d_outTranslation, "d_outTranslation", "Output translation") )
{

}

ObjectPoser::~ObjectPoser()
{
}

void ObjectPoser::saveImage(int i)
{
    int h = 1200;
    int w = 1200;
    int sofaH = 78;
    int sofaW = 508;
    int sofaHH = 39;

    int newH = h + sofaH; 
    int newW = w + sofaW; 

    cv::Mat img(newH, newW, CV_8UC3);
    glPixelStorei(GL_PACK_ALIGNMENT, (img.step & 3)?1:4);
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
}

void ObjectPoser::savePose(int i)
{

}

void ObjectPoser::printPretty44Mat(GLdouble M[16])
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

void ObjectPoser::handleEvent(sofa::core::objectmodel::Event *event)
{

    //helper::WriteAccessor<Data<Vec3Real > > outRotation = d_outRotation;
    //Vec3 outRotation = d_outRotation.getValue();	

    SReal rstep = d_rotationStep.getValue()*m_stepDirection;
    SReal tstep = d_translationStep.getValue()*m_stepDirection;

    f_listening.setValue(true);
    // Key pressed event
    sofa::core::objectmodel::KeypressedEvent* ev = dynamic_cast<sofa::core::objectmodel::KeypressedEvent *>(event);    
    if(ev != NULL)
    {
	//std::cout << ev->getKey() << std::endl;
	switch(ev->getKey())
        {	   
	   case '1':
		_refModel->applyRotation(rstep, 0, 0);
		m_im++;
                break;		
	   case '2':
		_refModel->applyRotation(0, rstep, 0);
                break;
	   case '3':
		_refModel->applyRotation(0, 0, rstep);
                break;
	   case '4':
		_refModel->applyTranslation(tstep, 0, 0);
                break;
	   case '5':
		_refModel->applyTranslation(0, tstep, 0);
                break;
	   case '6':
		_refModel->applyTranslation(0, 0, tstep);
                break;
	   case '7':
		m_stepDirection *= -1; // change the sign of the steps
                break;
           default:
                break;
        }
    }

}

void ObjectPoser::drawVisual(const core::visual::VisualParams* vparams)
{
		
}

void ObjectPoser::updateVisual()
{
}
void ObjectPoser::init()
{

}
void ObjectPoser::initVisual()
{
    _refModel = this->getContext()->get<component::visualmodel::OglModel>();    
}


} // namespace visual

} // namespace component

} // namespace sofa
