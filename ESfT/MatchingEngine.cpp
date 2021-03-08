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

#include "MatchingEngine.h"
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

int MatchingEngineClass = core::RegisterObject("MatchingEngine")
        .add< MatchingEngine >()
        ;


MatchingEngine::MatchingEngine()
	: d_cameraPosition( initData(&d_cameraPosition, "cameraPosition", "Position of the camera") )
	, d_inputImagePoints( initData(&d_inputImagePoints, "inputImagePoints", "Input vector of image points") )
	, d_inputModelPoints( initData(&d_inputModelPoints, "inputModelPoints", "Input vector of model points") )
	, d_outputSourcePoints( initData(&d_outputSourcePoints, "outputSourcePoints", "Output vector of source points") )
	, d_outputTargetPoints( initData(&d_outputTargetPoints, "outputTargetPoints", "Output vector of target points") )
{

}

MatchingEngine::~MatchingEngine()
{
}


void MatchingEngine::init()
{
    unsigned int sightlineSampling = 100; // number of points per sightline

    const sofa::defaulttype::Vector3& cameraPos = d_cameraPosition.getValue();

    helper::WriteAccessor<Data<VecCoord> > outSrcPts = d_outputSourcePoints;
    helper::WriteAccessor<Data<VecCoord> > outTrgPts = d_outputTargetPoints;
    VecCoord inModPts = d_inputModelPoints.getValue();
    helper::WriteAccessor<Data<VecCoord> > inImgPts = d_inputImagePoints;

    std::vector<VecCoord> sightlines; // vector of of sightlines
    sightlines.resize(inImgPts.size());

    // build the sighlines points (need optimization)
    for (unsigned int i = 0; i < sightlines.size(); i++) {	
	const sofa::defaulttype::Vector3& pt = sofa::defaulttype::Vector3(inImgPts[i]);	
	sofa::defaulttype::Vector3 pt_tmp = cameraPos - pt;
	VecCoord tmpVec;
	for (unsigned int k = 0; k < sightlineSampling; k++ )
	{
		float t = (float)k/sightlineSampling; 
		float x = pt[0] + t*pt_tmp[0];  	
		float y = pt[1] + t*pt_tmp[1];  	
		float z = pt[2] + t*pt_tmp[2];
		tmpVec.push_back(sofa::defaulttype::Vector3(x, y, z));
	}
	sightlines[i] = tmpVec;
	tmpVec.clear();
    }


    // build a kdTree using OpenCV Flann
    std::vector<cv::Point3f> kdpoints;
    for(unsigned int i=0;i < inModPts.size();i++) 
	kdpoints.push_back(cv::Point3f(inModPts[i][0], inModPts[i][1], inModPts[i][2])); // Coord to OCV
	
    cv::flann::KDTreeIndexParams indexParams;
    cv::flann::Index kdtree(cv::Mat(kdpoints).reshape(1), indexParams);

    // initialize temp vector for NN Search
    VecCoord tmp_outSrcPts;
    tmp_outSrcPts.resize(inImgPts.size());
    VecCoord tmp_outTrgPts;
    tmp_outTrgPts.resize(inImgPts.size());
    std::vector<int> tmp_indices;
    tmp_indices.resize(inImgPts.size());

    for(unsigned int i=0; i < sightlines.size(); i++)
    {
	float dist = 1e6;
    	for(unsigned int j=0; j < sightlines[i].size(); j++)
    	{
		Coord spt = sightlines[i][j];  //target point		 
		std::vector<float> query;
		query.push_back(spt[0]);
		query.push_back(spt[1]);
		query.push_back(spt[2]);

    		std::vector<int> indices;
		std::vector<float> dists;

		kdtree.knnSearch(query, indices, dists, 1); // nn search
		Coord tpt = Coord(kdpoints[indices[0]].x, kdpoints[indices[0]].y, kdpoints[indices[0]].z) ; // OCV to Coord

		if (dists[0] < dist) 
		{
			dist = dists[0];
			tmp_outSrcPts[i] = spt;
			tmp_outTrgPts[i] = tpt;
			tmp_indices[i] = indices[0];
		}
			
	}	
    }

    // extra step to remove duplicates
		  
    std::map<int, int> countMap;
    for (unsigned int i = 0; i < tmp_indices.size(); i++) {
	//countMap[tmp_indices[i]]++;
	//if (countMap[tmp_indices[i]] == 1) {
		outSrcPts.push_back(tmp_outSrcPts[i]);
		outTrgPts.push_back(tmp_outTrgPts[i]);
		//std::cout << tmp_outSrcPts[i] << " 0 0 0 1 ";
	//}

    }


/*
    std::vector<int> points2sightlinesIndices; // vector to find which point correspond to which line

    int c = 0;
    for (unsigned int i = 0; i < inImgPts.size(); i++) {	
	const sofa::defaulttype::Vector3& pt = sofa::defaulttype::Vector3(inImgPts[i]);	
	sofa::defaulttype::Vector3 pt_tmp = cameraPos - pt;
	for (unsigned int k = 0; k < sightlineSampling; k++ )
	{
		float t = (float)k/sightlineSampling; 
		float x = pt[0] + t*pt_tmp[0];  	
		float y = pt[1] + t*pt_tmp[1];  	
		float z = pt[2] + t*pt_tmp[2];
		sightlines.push_back(sofa::defaulttype::Vector3(x, y, z));
		points2sightlinesIndices.push_back(i); 
	}
    }

    helper::WriteAccessor<Data<VecCoord> > outSrcPts = d_outputSourcePoints;
    helper::WriteAccessor<Data<VecCoord> > outTrgPts = d_outputTargetPoints;

    // build a kdTree using OpenCV Flann
    std::vector<cv::Point3f> OCVSightlines;
    for(unsigned int i=0;i < sightlines.size();i++) 
	OCVSightlines.push_back(cv::Point3f(sightlines[i][0], sightlines[i][1], sightlines[i][2])); // Coord to OCV
	
    cv::flann::KDTreeIndexParams indexParams;
    cv::flann::Index kdtree(cv::Mat(OCVSightlines).reshape(1), indexParams);

    for(unsigned int i=0; i < inModPts.size(); i++)
    {
	Coord tpt = inModPts[i];		
	std::vector<float> query; // source point

	query.push_back(tpt[0]);
	query.push_back(tpt[1]);
	query.push_back(tpt[2]);

    	std::vector<int> indices;
	std::vector<float> dists;

	kdtree.knnSearch(query, indices, dists, 1); // nn search

	Coord spt = Coord(OCVSightlines[indices[0]].x, OCVSightlines[indices[0]].y, OCVSightlines[indices[0]].z) ; // OCV to Coord
	outSrcPts.push_back(spt);
	outTrgPts.push_back(tpt);
	//cvsLines.erase(srcIdx.begin() + i); // remove this point from the pool
   	//cv::flann::Index kdtree(cv::Mat(cvsLines).reshape(1), indexParams); // reindex kdtree
    }	
*/

}


} // namespace visual

} // namespace component

} // namespace sofa
