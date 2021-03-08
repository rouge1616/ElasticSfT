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

#include "Rigid2Vec3d.h"
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

int Rigid2Vec3dClass = core::RegisterObject("Rigid2Vec3d")
        .add< Rigid2Vec3d >()
        ;


Rigid2Vec3d::Rigid2Vec3d()
	: d_inputPositions( initData(&d_inputPositions, "inputPositions", "Input vector of Rigid points") )
	, d_outputPositions( initData(&d_outputPositions, "outputPositions", "Output vector of Vec3 points") )
	, d_restOutputPositions( initData(&d_restOutputPositions, "restOutputPositions", "Output vector of Vec3 points at rest") )
{

}

Rigid2Vec3d::~Rigid2Vec3d()
{
}


void Rigid2Vec3d::init()
{

    helper::ReadAccessor<Data<VecCoord> > in = d_inputPositions;
    helper::WriteAccessor<Data<VecCoord> > out = d_restOutputPositions;
    out.resize(in.size());
    for (unsigned int i = 0; i < in.size(); i++)	
	out[i] = Coord(in[i][0],in[i][1],in[i][2]);
}

void Rigid2Vec3d::drawVisual(const core::visual::VisualParams* vparams)
{
    helper::ReadAccessor<Data<VecCoord> > in = d_inputPositions;
    helper::WriteAccessor<Data<VecCoord> > out = d_outputPositions;
    out.resize(in.size());
    for (unsigned int i = 0; i < in.size(); i++)	
	out[i] = Coord(in[i][0],in[i][1],in[i][2]);

}

} // namespace visual

} // namespace component

} // namespace sofa
