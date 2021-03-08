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

#include "SolvePnP.h"
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

int SolvePnPClass = core::RegisterObject("SolvePnP")
        .add< SolvePnP >()
        ;


SolvePnP::SolvePnP()
: d_imagePoints( initData (&d_imagePoints, "imagePoints", "input 2D points") )
, d_objectPoints( initData (&d_objectPoints, "objectPoints", "input 3D points") )
, d_intrinsecParam( initData (&d_intrinsecParam, "intrinsecParam", "camera intrinsec parameters 9x1") )
, d_distortionCoeff( initData (&d_distortionCoeff, "distortionCoeff", "camera distortion coefficients 5x1") )
{

}

SolvePnP::~SolvePnP()
{
}


void SolvePnP::init()
{
	std::vector<cv::Point3f> ocv_objectPoints;
	std::vector<cv::Point2f> ocv_imagePoints;
	cv::Mat ocv_intrinsecParam(3,3,cv::DataType<double>::type);
	cv::Mat ocv_distortionCoeff(5,1,cv::DataType<double>::type);


 
	// object 3D point
	helper::ReadAccessor<Data<helper::vector<Vec3> > > ra_objectPoints = d_objectPoints;
	ocv_objectPoints.resize(ra_objectPoints.size());
	for (unsigned int i = 0 ; i < ra_objectPoints.size(); i++)
		ocv_objectPoints[i] = cv::Point3f(ra_objectPoints[i].at(0), ra_objectPoints[i].at(1), ra_objectPoints[i].at(2));


	// image 2D point
	helper::ReadAccessor<Data<helper::vector<Vec3> > > ra_imagePoints = d_imagePoints;
	ocv_imagePoints.resize(ra_imagePoints.size());
	for (unsigned int i = 0 ; i < ra_imagePoints.size(); i++)
		ocv_imagePoints[i] = cv::Point2f(ra_imagePoints[i].at(0), ra_imagePoints[i].at(1));


	// camera matrix
	helper::ReadAccessor<Data<helper::vector<double> > > ra_intrinsecParam = d_intrinsecParam;

	ocv_intrinsecParam.at<double>(0,0) = ra_intrinsecParam[0];
	ocv_intrinsecParam.at<double>(0,1) = ra_intrinsecParam[1];
	ocv_intrinsecParam.at<double>(0,2) = ra_intrinsecParam[2];
	ocv_intrinsecParam.at<double>(1,0) = ra_intrinsecParam[3];
	ocv_intrinsecParam.at<double>(1,1) = ra_intrinsecParam[4];
	ocv_intrinsecParam.at<double>(1,2) = ra_intrinsecParam[5];
	ocv_intrinsecParam.at<double>(2,0) = ra_intrinsecParam[6];
	ocv_intrinsecParam.at<double>(2,1) = ra_intrinsecParam[7];
	ocv_intrinsecParam.at<double>(2,2) = ra_intrinsecParam[8];

	// distorsion coeff
	helper::ReadAccessor<Data<helper::vector<double> > > ra_distortionCoeff = d_distortionCoeff;

	ocv_distortionCoeff.at<double>(0) = ra_distortionCoeff[0];
	ocv_distortionCoeff.at<double>(1) = ra_distortionCoeff[1];
	ocv_distortionCoeff.at<double>(2) = ra_distortionCoeff[2];
	ocv_distortionCoeff.at<double>(3) = ra_distortionCoeff[3];
	ocv_distortionCoeff.at<double>(4) = ra_distortionCoeff[4];


	// solve EPnP
        cv::solvePnP(ocv_objectPoints, ocv_imagePoints, ocv_intrinsecParam, ocv_distortionCoeff, m_ocv_rvec, m_ocv_tvec, false, cv::SOLVEPNP_UPNP);

	std::cout << m_ocv_rvec << std::endl;
	std::cout << m_ocv_tvec << std::endl;

	std::vector<cv::Point2f> ocv_projectedPoints;
	cv::projectPoints(ocv_objectPoints, m_ocv_rvec, m_ocv_tvec, ocv_intrinsecParam, ocv_distortionCoeff, ocv_projectedPoints);



	cv::Mat im = cv::imread("nazim.png");

	for(unsigned int i=0; i < ocv_projectedPoints.size(); i++)
	{
        	cv::circle(im, ocv_projectedPoints[i], 3, cv::Scalar(0,0,255), -1);
	}

	//cv::imshow("Output", im);
	//cv::waitKey(0);
}

void SolvePnP::printRT()
{
	GLfloat modelviewMatrixData[16]; 
	glGetFloatv (GL_MODELVIEW_MATRIX, modelviewMatrixData);

	   std::cout << "the current modelview matrix is:" << std::endl;
	   std::cout << modelviewMatrixData[0] << "   ";
	   std::cout << modelviewMatrixData[4] << "   ";
	   std::cout << modelviewMatrixData[8] << "   ";
	   std::cout << modelviewMatrixData[12] << std::endl;
	   std::cout << modelviewMatrixData[1] << "   ";
	   std::cout << modelviewMatrixData[5] << "   ";
	   std::cout << modelviewMatrixData[9] << "   ";
	   std::cout << modelviewMatrixData[13] << std::endl;
	   std::cout << modelviewMatrixData[2] << "   ";
	   std::cout << modelviewMatrixData[6] << "   ";
	   std::cout << modelviewMatrixData[10] << "   ";
	   std::cout << modelviewMatrixData[14] << std::endl;
	   std::cout << modelviewMatrixData[3] << "   ";
	   std::cout << modelviewMatrixData[7] << "   ";
	   std::cout << modelviewMatrixData[11] << "   ";
	   std::cout << modelviewMatrixData[15] << std::endl << std::endl;

}


void SolvePnP::draw(const core::visual::VisualParams* vparams)
{

	double focal = 800; // TIP: could be equal to image width

	double zNear = vparams->zNear();
	double zFar = vparams->zFar();
	double width = vparams->viewport()[2];
	double height = vparams->viewport()[3];
	double cx = width/2;
	double cy = height/2;

	double projectionMat[16];
	projectionMat[0] = 2.0 * focal / width;
	projectionMat[1] = 0.0;
	projectionMat[2] = 0.0;
	projectionMat[3] = 0.0;

	projectionMat[4] = 0.0;
	projectionMat[5] = 2.0 * focal / height;
	projectionMat[6] = 0.0;
	projectionMat[7] = 0.0;
	
	projectionMat[8] = 2.0 * ( cx / width) - 1.0; 
	projectionMat[9] = 2.0 * ( cy / height ) - 1.0;	
	projectionMat[10] = -( zFar + zNear ) / ( zFar - zNear );
	projectionMat[11] = -1.0;

	projectionMat[12] = 0.0;
	projectionMat[13] = 0.0;
	projectionMat[14] = -2.0 * zFar * zNear / ( zFar - zNear );		
	projectionMat[15] = 0.0;


	glMatrixMode(GL_PROJECTION);	
	glLoadIdentity();
	//gluPerspective(fovy, (float) (height/width), zNear, zFar);
	glLoadMatrixd(projectionMat);


	cv::Mat ocv_rmat;
	cv::Rodrigues(m_ocv_rvec, ocv_rmat);

	double modelviewMat[16];

	modelviewMat[0]  = ocv_rmat.at<double>(0,0);
	modelviewMat[1]  = -ocv_rmat.at<double>(1,0);
	modelviewMat[2]  = -ocv_rmat.at<double>(2,0);
	modelviewMat[3]  = 0;
	modelviewMat[4]  = ocv_rmat.at<double>(0,1);
	modelviewMat[5]  = -ocv_rmat.at<double>(1,1);
	modelviewMat[6]  = -ocv_rmat.at<double>(2,1);
	modelviewMat[7]  = 0;
	modelviewMat[8]  = ocv_rmat.at<double>(0,2);
	modelviewMat[9]  = -ocv_rmat.at<double>(1,2);
	modelviewMat[10] = -ocv_rmat.at<double>(2,2);
	modelviewMat[11] = 0;
	modelviewMat[12] = m_ocv_tvec.at<double>(0,0);
	modelviewMat[13] = -m_ocv_tvec.at<double>(1,0);
	modelviewMat[14] = -m_ocv_tvec.at<double>(2,0);
	modelviewMat[15] = 1;

	//glMatrixMode(GL_MODELVIEW);
	//glLoadIdentity();
	//glLoadMatrixd(modelviewMat);
        //printRT();


}


} // namespace visual

} // namespace component

} // namespace sofa
