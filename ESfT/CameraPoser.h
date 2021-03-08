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
#include <SofaLoader/MeshObjLoader.h>
#include <SofaOpenglVisual/OglModel.h>
#include <sofa/core/topology/BaseMeshTopology.h>
#include <sofa/core/behavior/MechanicalState.h>
#include <sofa/defaulttype/Vec.h>
#include <sofa/defaulttype/VecTypes.h>

#include <sofa/core/objectmodel/KeypressedEvent.h>
#include <sofa/core/objectmodel/MouseEvent.h>
#include <sofa/simulation/AnimateBeginEvent.h>
#include <sofa/simulation/AnimateEndEvent.h>


#include <sofa/helper/system/FileRepository.h>
#include <sofa/helper/system/SetDirectory.h>

#include <sofa/helper/Quater.h>

#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"

#include <fstream>

namespace sofa
{

namespace component
{

namespace visualmodel
{

class CameraPoser : public core::visual::VisualModel
{

protected:
    typedef sofa::defaulttype::ExtVec3fTypes DataTypes;
    typedef DataTypes::Real Real;
    typedef sofa::defaulttype::Vec<2, float> TexCoord;
    typedef sofa::defaulttype::Vec<3,Real> Vec3Real;
    typedef typename sofa::defaulttype::Vector3 Vec3;
    typedef sofa::core::topology::BaseMeshTopology::Triangle Triangle;

public:

    CameraPoser();
    ~CameraPoser();

    //VisualModel functions
    void drawVisual(const core::visual::VisualParams* vparams);
    void handleEvent(sofa::core::objectmodel::Event *event);
    void updateVisual();
    void init();
    void initVisual();
    void printPretty44Mat(GLdouble[16]);
    void saveImage(int, int, int);
    void savePose(int, helper::Quater< Real >, Vec3Real);
 
    sofa::Data< float > d_rotationStep;
    sofa::Data< float > d_translationStep;

    sofa::Data< Vec3Real > d_outRotation;
    sofa::Data< Vec3 > d_outTranslation;

    sofa::Data< Vec3 > d_inTranslation;
    sofa::Data< helper::Quater< Real > > d_inRotation;

    int m_im = 0;
    int m_stepDirection = 1;
    int m_pressed = 0;
		

private:

};

} // namespace visual

} // namespace component

} // namespace sofa

#endif 
