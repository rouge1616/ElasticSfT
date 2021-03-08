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
#ifndef SOFA_RENDERDEPTHMAP_H
#define SOFA_RENDERDEPTHMAP_H

#include <sofa/core/visual/VisualModel.h>
#include <SofaOpenglVisual/OglModel.h>
#include <sofa/core/topology/BaseMeshTopology.h>
#include <sofa/core/behavior/MechanicalState.h>
#include <sofa/defaulttype/Vec.h>
#include <sofa/defaulttype/VecTypes.h>
#include <sofa/helper/vector.h>
#include <sofa/core/objectmodel/DataFileName.h>
#include <sofa/core/objectmodel/KeypressedEvent.h>

#include <map>

#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/flann/miniflann.hpp"

namespace sofa
{

namespace component
{

namespace visualmodel
{

class Rigid2Vec3d : public core::visual::VisualModel
{
public:

    typedef sofa::defaulttype::ExtVec3fTypes DataTypes;
    typedef typename DataTypes::Coord Coord;
    typedef DataTypes::Real Real;
    typedef sofa::defaulttype::Vec<2, float> TexCoord;
    typedef sofa::defaulttype::Vec<3,Real> Vec3Real;
    typedef typename DataTypes::VecCoord VecCoord;
    typedef typename sofa::defaulttype::Vector3 Vec3;
    typedef sofa::defaulttype::Mat<3,3,double> Matrix3;

    Rigid2Vec3d();
    ~Rigid2Vec3d();

    void init();
    void drawVisual(const core::visual::VisualParams* vparams);


    sofa::Data<VecCoord> d_inputPositions;
    sofa::Data<VecCoord> d_outputPositions;
    sofa::Data<VecCoord> d_restOutputPositions;

protected:



private:

};

} // namespace visual

} // namespace component

} // namespace sofa

#endif 
