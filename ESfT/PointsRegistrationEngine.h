/******************************************************************************
*       SOFA, Simulation Open-Framework Architecture, version 1.0 RC 1        *
*                (c) 2006-2011 MGH, INRIA, USTL, UJF, CNRS                    *
*                                                                             *
* This library is free software; you can redistribute it and/or modify it     *
* under the terms of the GNU Lesser General Public License as published by    *
* the Free Software Foundation; either version 2.1 of the License, or (at     *
* your option) any later version.                                             *
*                                                                             *
* This library is distributed in the hope that it will be useful, but WITHOUT *
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or       *
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License *
* for more details.                                                           *
*                                                                             *
* You should have received a copy of the GNU Lesser General Public License    *
* along with this library; if not, write to the Free Software Foundation,     *
* Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301 USA.          *
*******************************************************************************
*                               SOFA :: Modules                               *
*                                                                             *
* Authors: The SOFA Team and external contributors (see Authors.txt)          *
*                                                                             *
* Contact information: contact@sofa-framework.org                             *
******************************************************************************/
#ifndef SOFA_COMPONENT_ENGINE_POINTSREGISTRATIONENGINE_H
#define SOFA_COMPONENT_ENGINE_POINTSREGISTRATIONENGINE_H

#if !defined(__GNUC__) || (__GNUC__ > 3 || (_GNUC__ == 3 && __GNUC_MINOR__ > 3))
#pragma once
#endif

#include <iostream>
#include <fstream>

// sofa
#include <sofa/core/component/component.h>
#include <sofa/core/DataEngine.h>
#include <sofa/core/objectmodel/BaseObject.h>
#include <sofa/core/visual/VisualModel.h>
#include <sofa/defaulttype/BaseVector.h>
#include <sofa/helper/vector.h>
#include <sofa/defaulttype/VecTypes.h>
#include <sofa/defaulttype/Mat.h>
#include <sofa/helper/accessor.h>
#include "SofaARCapsule.h"
#include <sofa/helper/Quater.h>
#include <sofa/core/visual/DrawToolGL.h>
#include <sofa/helper/system/gl.h>
#include <sofa/helper/system/glut.h>
#include <sofa/helper/gl/RAII.h>

// boost
#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/serialization/string.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/asio.hpp>




namespace sofa
{

namespace component
{

namespace engine
{

/** 
 * This class registers a subset of a cloud of points to a set of mechanical positions
 */
template <class DataTypes>
class SOFA_SOFAARPLUGIN_API PointsRegistrationEngine : public core::DataEngine
{
public:
    SOFA_CLASS(SOFA_TEMPLATE(PointsRegistrationEngine,DataTypes),core::DataEngine);
    typedef typename DataTypes::VecCoord VecCoord;
    typedef typename DataTypes::Coord Coord;
    typedef typename DataTypes::Real Real;
    typedef typename sofa::defaulttype::Vector3 Vec3;
    typedef sofa::defaulttype::Mat<3,3,double> Matrix3;

public:

    PointsRegistrationEngine();

    ~PointsRegistrationEngine();

    void init();

    void reinit();

    void update();

    void draw(const core::visual::VisualParams *vparams);

    void getNN(const VecCoord&, const VecCoord&, int);


    virtual std::string getTemplateName() const
    {
        return templateName(this);
    }

    static std::string templateName(const PointsRegistrationEngine<DataTypes>* = NULL)
    {
        return DataTypes::Name();
    }


protected:
    Data<VecCoord> f_inputX; ///< input positions
    Data<VecCoord> f_outputX; ///< ouput positions
    Data<VecCoord> f_initPositions; ///< init input point positions
    Data<VecCoord> f_cp; ///< init input point positions
    Data<helper::vector<int> > f_outputIndices; ///< vector of indices of the ouput points
    Data<helper::vector<int> > f_lastLostIndices; ///< vector of indices of the last losted points
    Data< Vec3 > f_scalePt; ///< scaling factor for output position
    Data< float > f_scaleImg; ///< scaling factor for output position
    Data< int > f_dt; ///< time in the tracking framework (related to video fps)
    Data< bool > f_showImg; ///< show/hide background image
    Data< bool > f_persView; ///< perspective/ortho
    Data<std::string> f_host; ///< host IP address
    Data<std::string> f_port; ///< port number
    Data<std::string> f_cameraMatrix; ///< intrinsic camera matrix 'K'
    Data<helper::vector<double> > f_qualities;
    Data<helper::vector<int> > f_status;
    Data<helper::vector<int> > f_subset;
    Data<helper::vector<double> > f_rt;
    Data<std::string > f_outputImageData;

    bool m_hostFound;
    int m_fps;
    std::ofstream m_fileLog;
    std::string m_imgData;
    int m_imageWidth;
    int m_imageHeight;
    int m_flagReceived;
    boost::asio::ip::tcp::iostream m_dataStream;
    std::vector<float> m_currentPositionsX;
    std::vector<float> m_currentPositionsY;
    std::vector<float> m_currentPositionsZ;
    std::vector<int> m_currentStatus;
    std::vector<double> m_initQ;
    std::vector<int> m_initSubset;
    std::vector<int> m_allLostIndices;
};

#if defined(SOFA_EXTERN_TEMPLATE) && !defined(SOFA_COMPONENT_ENGINE_POINTSREGISTRATIONENGINE_CPP)

#ifndef SOFA_FLOAT
extern template class SOFA_ENGINE_API PointsRegistrationEngine<defaulttype::Vec3dTypes>;
#endif //SOFA_FLOAT
#ifndef SOFA_DOUBLE
extern template class SOFA_ENGINE_API PointsRegistrationEngine<defaulttype::Vec3fTypes>;
#endif //SOFA_DOUBLE
#endif

} // namespace engine

} // namespace component

} // namespace sofa

#endif
