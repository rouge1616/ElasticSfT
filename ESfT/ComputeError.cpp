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

#include "ComputeError.h"
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

int ComputeErrorClass = core::RegisterObject("ComputeError")
        .add< ComputeError >()
        ;


ComputeError::ComputeError()
	: d_pos1( initData(&d_pos1, "pos1", "Position of point 1") )
	, d_pos2( initData(&d_pos2, "pos2", "Position of point 2") )
	, d_title( initData(&d_title, "title", "Numbering for the text") )
{

}

ComputeError::~ComputeError()
{
}


void ComputeError::init()
{

}

void ComputeError::drawVisual(const core::visual::VisualParams* vparams)
{
	int t = d_title.getValue()[0];
	Coord pt1 = d_pos1.getValue();
	Coord pt2 = d_pos2.getValue();
	double dist = sqrt((pt1[0] - pt2[0])*(pt1[0] - pt2[0]) + (pt1[1] - pt2[1])*(pt1[1] - pt2[1]) + (pt1[2] - pt2[2])*(pt1[2] - pt2[2]) );
	std::cout << "Dist Point " << t << " = "<< dist << std::endl; 
	//std::cout << "Pos Point " << t << " = "<< pt2 << std::endl; 
}

} // namespace visual

} // namespace component

} // namespace sofa
