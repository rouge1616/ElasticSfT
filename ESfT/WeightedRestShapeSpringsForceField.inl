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
#ifndef SOFA_COMPONENT_FORCEFIELD_WeightedRESTSHAPESPRINGFORCEFIELD_INL
#define SOFA_COMPONENT_FORCEFIELD_WeightedRESTSHAPESPRINGFORCEFIELD_INL

#include "WeightedRestShapeSpringsForceField.h"
#include <sofa/core/visual/VisualParams.h>
#include <sofa/helper/system/config.h>
#include <sofa/defaulttype/VecTypes.h>
#include <sofa/defaulttype/RigidTypes.h>

#include <sofa/defaulttype/RGBAColor.h>

namespace sofa
{

namespace component
{

namespace forcefield
{

using helper::WriteAccessor;
using helper::ReadAccessor;
using core::behavior::BaseMechanicalState;
using core::behavior::MultiMatrixAccessor;
using core::behavior::ForceField;
using defaulttype::BaseMatrix;
using core::VecCoordId;
using core::MechanicalParams;
using defaulttype::Vector3;
using defaulttype::Vec4f;
using helper::vector;
using core::visual::VisualParams;

template<class DataTypes>
WeightedRestShapeSpringsForceField<DataTypes>::WeightedRestShapeSpringsForceField()
    : points(initData(&points, "points", "points controlled by the rest shape springs"))
    , stiffness(initData(&stiffness, "stiffness", "stiffness values between the actual position and the rest shape position"))
    , angularStiffness(initData(&angularStiffness, "angularStiffness", "angularStiffness assigned when controlling the rotation of the points"))
    , pivotPoints(initData(&pivotPoints, "pivot_points", "global pivot points used when translations instead of the rigid mass centers"))
    , external_points(initData(&external_points, "external_points", "points from the external Mechancial State that define the rest shape springs"))
    , recompute_indices(initData(&recompute_indices, true, "recompute_indices", "Recompute indices (should be false for BBOX)"))
    , drawSpring(initData(&drawSpring,false,"drawSpring","draw Spring"))
    , springColor(initData(&springColor, defaulttype::RGBAColor(0.0,1.0,0.0,1.0), "springColor","spring color. (default=[0.0,1.0,0.0,1.0])"))
    , restMState(initLink("external_rest_shape", "rest_shape can be defined by the position of an external Mechanical State"))
    , springDirections( initData(&springDirections, VecBool(1,1,1) ,"springDirections","for each direction, 1 if applied, 0 if free") )
{
}

template<class DataTypes>
void WeightedRestShapeSpringsForceField<DataTypes>::parse(core::objectmodel::BaseObjectDescription *arg)
{
    const char* attr = arg->getAttribute("external_rest_shape") ;
    if( attr != nullptr && attr[0] != '@')
    {
        msg_error() << "WeightedRestShapeSpringsForceField have changed since 17.06. The parameter 'external_rest_shape' is now a Link. To fix your scene you need to add and '@' in front of the provided path. See PR#315" ;
    }

    Inherit::parse(arg) ;
}

template<class Type>
inline bool needAngularStiffness() { return false; }

template<>
inline bool needAngularStiffness<sofa::defaulttype::Rigid3Types>(){ return true; }

template<class DataTypes>
void WeightedRestShapeSpringsForceField<DataTypes>::bwdInit()
{
    ForceField<DataTypes>::init();

    if (stiffness.getValue().empty())
    {
        msg_info() << "No stiffness is defined, assuming equal stiffness on each node, k = 100.0 ";

        VecReal stiffs;
        stiffs.push_back(100.0);
        stiffness.setValue(stiffs);
    }

    if (restMState.get() == NULL)
    {
        useRestMState = false;
        msg_info() << "no external rest shape used";

        if(!restMState.empty())
        {
            msg_warning() << "external_rest_shape in node " << this->getContext()->getName() << " not found";
        }
    }
    else
    {
        msg_info() << "external rest shape used";
        useRestMState = true;
    }

    recomputeIndices();

    BaseMechanicalState* state = this->getContext()->getMechanicalState();
    if(!state)
    {
        msg_warning() << "MechanicalState of the current context returns null pointer";
    }
    else
    {
        assert(state);
        matS.resize(state->getMatrixSize(),state->getMatrixSize());
    }

    /// Compile time condition to check if we are working with a Rigid3Types or a type that does not
    /// need the Angular Stiffness parameters.
    if(needAngularStiffness<DataTypes>())
    {
        sofa::helper::ReadAccessor<Data<VecReal>> s = stiffness;
        sofa::helper::WriteOnlyAccessor<Data<VecReal>> as = angularStiffness;

        if (as.size() < s.size())
        {
            msg_info() << "'stiffness' is larger than 'angularStiffness', add the default value (100.0) to the missing entries.";

            for(size_t i = as.size();i<s.size();i++)
            {
                as.push_back(100.0);
            }
        }else if (as.size() > s.size())
        {
            msg_info() << "'stiffness' is smaller than 'angularStiffness', clamp the extra values in angularStiffness.";
            as.resize(s.size());
        }
    }

    lastUpdatedStep = -1.0;
}


template<class DataTypes>
void WeightedRestShapeSpringsForceField<DataTypes>::reinit()
{
    if (!checkOutOfBoundsIndices())
    {
        m_indices.clear();
    }
    else
    {
        msg_info() << "Indices successfully checked";
    }

    if (stiffness.getValue().empty())
    {
        msg_info() << "No stiffness is defined, assuming equal stiffness on each node, k = 100.0 " ;

        VecReal stiffs;
        stiffs.push_back(100.0);
        stiffness.setValue(stiffs);
    }
    else
    {
        const VecReal &k = stiffness.getValue();
        if ( k.size() != m_indices.size() )
        {
            msg_warning() << "Size of stiffness vector is not correct (" << k.size() << "), should be either 1 or " << m_indices.size() << msgendl
                          << "First value of stiffness will be used";
        }
    }

}

template<class DataTypes>
void WeightedRestShapeSpringsForceField<DataTypes>::recomputeIndices()
{
    m_indices.clear();
    m_ext_indices.clear();

    for (unsigned int i = 0; i < points.getValue().size(); i++)
    {
        m_indices.push_back(points.getValue()[i]);
    }

    for (unsigned int i = 0; i < external_points.getValue().size(); i++)
    {
        m_ext_indices.push_back(external_points.getValue()[i]);
    }

    if (m_indices.empty())
    {
        // no point are defined, default case: points = all points
        for (unsigned int i = 0; i < (unsigned)this->mstate->getSize(); i++)
        {
            m_indices.push_back(i);
        }
    }

    if (m_ext_indices.empty())
    {
        if (useRestMState)
        {
            for (unsigned int i = 0; i < getExtPosition()->getValue().size(); i++)
            {
                m_ext_indices.push_back(i);
            }
        }
        else
        {
            for (unsigned int i = 0; i < m_indices.size(); i++)
            {
                m_ext_indices.push_back(m_indices[i]);
            }
        }
    }

    if (!checkOutOfBoundsIndices())
    {
        msg_error() << "The dimension of the source and the targeted points are different ";
        m_indices.clear();
    }
    else
    {
        msg_info() << "Indices successfully checked";
    }
}

template<class DataTypes>
bool WeightedRestShapeSpringsForceField<DataTypes>::checkOutOfBoundsIndices()
{
    if (!checkOutOfBoundsIndices(m_indices, this->mstate->getSize()))
    {
        msg_error() << "Out of Bounds m_indices detected. ForceField is not activated.";
        return false;
    }
    if (!checkOutOfBoundsIndices(m_ext_indices, getExtPosition()->getValue().size()))
    {
        msg_error() << "Out of Bounds m_ext_indices detected. ForceField is not activated.";
        return false;
    }
    if (m_indices.size() != m_ext_indices.size())
    {
        msg_error() << "Dimensions of the source and the targeted points are different. ForceField is not activated.";
        return false;
    }
    return true;
}

template<class DataTypes>
bool WeightedRestShapeSpringsForceField<DataTypes>::checkOutOfBoundsIndices(const VecIndex &indices, const unsigned int dimension)
{
    for (unsigned int i = 0; i < indices.size(); i++)
    {
        if (indices[i] >= dimension)
        {
            return false;
        }
    }
    return true;
}

template<class DataTypes>
const typename WeightedRestShapeSpringsForceField<DataTypes>::DataVecCoord* WeightedRestShapeSpringsForceField<DataTypes>::getExtPosition() const
{
    return (useRestMState ? restMState->read(VecCoordId::position()) : this->mstate->read(VecCoordId::restPosition()));
}

template<class DataTypes>
void WeightedRestShapeSpringsForceField<DataTypes>::addForce(const MechanicalParams*  mparams , DataVecDeriv& f, const DataVecCoord& x, const DataVecDeriv&  v )
{
    SOFA_UNUSED(mparams);
    SOFA_UNUSED(v);

    WriteAccessor< DataVecDeriv > f1 = f;
    ReadAccessor< DataVecCoord > p1 = x;
    ReadAccessor< DataVecCoord > p0 = *getExtPosition();
    const VecReal& k = stiffness.getValue();


    const VecBool& springDirection = springDirections.getValue();


    f1.resize(p1.size());

    if (recompute_indices.getValue())
    {
        recomputeIndices();
    }

    if ( k.size()!= m_indices.size() )
    {
        // Stiffness is not defined on each point, first stiffness is used
        const Real k0 = k[0];

        for (unsigned int i=0; i<m_indices.size(); i++)
        {



            const unsigned int index = m_indices[i];
            const unsigned int ext_index = m_ext_indices[i];
	
            Deriv dx = p1[index] - p0[ext_index];

	    for (unsigned int dim = 0; dim<3; dim++)
    	    	if (!springDirection[dim])
			dx[dim] = 0;	

            f1[index] -=  dx * k0 ;

        }
    }
    else
    {
        // Heterogeneous stiffness for each spring
        for (unsigned int i=0; i<m_indices.size(); i++)
        {
            const unsigned int index = m_indices[i];
            unsigned int ext_index = m_ext_indices[i];

            Deriv dx = p1[index] - p0[ext_index];

	    for (unsigned int dim = 0; dim<3; dim++)
    	    	if (!springDirection[dim])
			dx[dim] = 0;

            f1[index] -=  dx * k[i];
        }
    }
}

template<class DataTypes>
void WeightedRestShapeSpringsForceField<DataTypes>::addDForce(const MechanicalParams* mparams, DataVecDeriv& df, const DataVecDeriv& dx)
{
    WriteAccessor< DataVecDeriv > df1 = df;
    ReadAccessor< DataVecDeriv > dx1 = dx;
    Real kFactor = (Real)mparams->kFactorIncludingRayleighDamping(this->rayleighStiffness.getValue());
    const VecReal& k = stiffness.getValue();

    if (k.size() != m_indices.size())
    {
        // Stiffness is not defined on each point, first stiffness is used
        const Real k0 = k[0] * (Real)kFactor;

        for (unsigned int i=0; i<m_indices.size(); i++)
        {
            df1[m_indices[i]] -=  dx1[m_indices[i]] * k0;
        }
    }
    else
    {
        for (unsigned int i=0; i<m_indices.size(); i++)
        {
            df1[m_indices[i]] -=  dx1[m_indices[i]] * k[i] * kFactor ;
        }
    }
}

// draw for standard types (i.e Vec<1,2,3>)
template<class DataTypes>
void WeightedRestShapeSpringsForceField<DataTypes>::draw(const VisualParams *vparams)
{
    if (!vparams->displayFlags().getShowForceFields() || !drawSpring.getValue())
        return;  /// \todo put this in the parent class

    if(DataTypes::spatial_dimensions > 3)
    {
        msg_error() << "Draw function not implemented for this DataType";
        return;
    }

    vparams->drawTool()->saveLastState();
    vparams->drawTool()->setLightingEnabled(false);

    ReadAccessor< DataVecCoord > p0 = *getExtPosition();
    ReadAccessor< DataVecCoord > p  = this->mstate->read(VecCoordId::position());

    const VecIndex& indices = m_indices;
    const VecIndex& ext_indices = (useRestMState ? m_ext_indices : m_indices);

    vector<Vector3> vertices;

    for (unsigned int i=0; i<indices.size(); i++)
    {
        const unsigned int index = indices[i];
        const unsigned int ext_index = ext_indices[i];

        Vector3 v0(0.0, 0.0, 0.0);
        Vector3 v1(0.0, 0.0, 0.0);
        for(unsigned int j=0 ; j<DataTypes::spatial_dimensions ; j++)
        {
            v0[j] = p[index][j];
            v1[j] = p0[ext_index][j];
        }

        vertices.push_back(v0);
        vertices.push_back(v1);
    }

    //todo(dmarchal) because of https://github.com/sofa-framework/sofa/issues/64
    vparams->drawTool()->drawLines(vertices,5,Vec4f(springColor.getValue()));
    vparams->drawTool()->restoreLastState();
}

template<class DataTypes>
void WeightedRestShapeSpringsForceField<DataTypes>::addKToMatrix(const MechanicalParams* mparams, const MultiMatrixAccessor* matrix )
{
    MultiMatrixAccessor::MatrixRef mref = matrix->getMatrix(this->mstate);
    BaseMatrix* mat = mref.matrix;
    unsigned int offset = mref.offset;
    Real kFact = (Real)mparams->kFactorIncludingRayleighDamping(this->rayleighStiffness.getValue());

    const VecReal& k = stiffness.getValue();
    const int N = Coord::total_size;

    unsigned int curIndex = 0;

    if (k.size() != m_indices.size())
    {
        const Real k0 = -k[0] * (Real)kFact;

        for (unsigned int index = 0; index < m_indices.size(); index++)
        {
            curIndex = m_indices[index];

            for(int i = 0; i < N; i++)
            {
                mat->add(offset + N * curIndex + i, offset + N * curIndex + i, k0);
            }
        }
    }
    else
    {
        for (unsigned int index = 0; index < m_indices.size(); index++)
        {
            curIndex = m_indices[index];

            for(int i = 0; i < N; i++)
            {
                mat->add(offset + N * curIndex + i, offset + N * curIndex + i, -kFact * k[index]);
            }
        }
    }
}

template<class DataTypes>
void WeightedRestShapeSpringsForceField<DataTypes>::addSubKToMatrix(const MechanicalParams* mparams, const MultiMatrixAccessor* matrix, const vector<unsigned> & addSubIndex )
{
    MultiMatrixAccessor::MatrixRef mref = matrix->getMatrix(this->mstate);
    BaseMatrix* mat = mref.matrix;
    unsigned int offset = mref.offset;
    Real kFact = (Real)mparams->kFactorIncludingRayleighDamping(this->rayleighStiffness.getValue());

    const VecReal& k = stiffness.getValue();
    const int N = Coord::total_size;

    unsigned int curIndex = 0;

    if (k.size() != m_indices.size())
    {
        const Real k0 = -k[0] * (Real)kFact;

        for (unsigned int index = 0; index < m_indices.size(); index++)
        {
            curIndex = m_indices[index];

            if (std::find(addSubIndex.begin(), addSubIndex.end(), curIndex) == addSubIndex.end())
                continue;

            for(int i = 0; i < N; i++)
            {
                mat->add(offset + N * curIndex + i, offset + N * curIndex + i, k0);
            }
        }
    }
    else
    {
        for (unsigned int index = 0; index < m_indices.size(); index++)
        {
            curIndex = m_indices[index];

            if (std::find(addSubIndex.begin(), addSubIndex.end(), curIndex) == addSubIndex.end())
                continue;

            for(int i = 0; i < N; i++)
            {
                mat->add(offset + N * curIndex + i, offset + N * curIndex + i, -kFact * k[index]);
            }
        }
    }
}

template<class DataTypes>
void WeightedRestShapeSpringsForceField<DataTypes>::updateForceMask()
{
    for (unsigned int i=0; i<m_indices.size(); i++)
        this->mstate->forceMask.insertEntry(m_indices[i]);
}


} // namespace forcefield

} // namespace component

} // namespace sofa

#endif // SOFA_COMPONENT_FORCEFIELD_RESTSHAPESPRINGFORCEFIELD_INL



