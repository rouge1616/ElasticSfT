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
#ifndef SOFA_COMPONENT_ENGINE_POINTSREGISTRATIONENGINE_INL
#define SOFA_COMPONENT_ENGINE_POINTSREGISTRATIONENGINE_INL

#include <PointsRegistrationEngine.h>
#include <sofa/helper/rmath.h> //M_PI
#include <sofa/core/visual/VisualParams.h>
#include <sofa/helper/gl/BasicShapes.h>

namespace sofa
{

namespace component
{

namespace engine
{


using boost::asio::ip::tcp;

template <class DataTypes>
PointsRegistrationEngine<DataTypes>::PointsRegistrationEngine()
: f_inputX ( initData (&f_inputX, "inputPosition", "input array of 3D points") )
, f_outputX( initData (&f_outputX, "outputPosition", "output array of 3D points") )
, f_initPositions( initData (&f_initPositions, "initPositions", "init positions of 3D points") )
, f_cp( initData (&f_cp, "cp", "control points") )
, f_outputIndices( initData (&f_outputIndices, "outputIndices", "indices of the output points") )
, f_lastLostIndices( initData (&f_lastLostIndices, "lastLostIndices", "output array of indices of the last lost points per frame") )
, f_scalePt( initData (&f_scalePt, Vec3(1.0,1.0,1.0), "scalePt", "apply scaling factor to the input 3D points") )
, f_showImg( initData (&f_showImg, bool(true), "showImg", "show/hide background image") )
, f_persView( initData (&f_persView, bool(true), "persView", "perspective/ortho background image") )
, f_scaleImg( initData (&f_scaleImg, float(1.0), "scaleImg", "apply scaling factor to the input image") )
, f_dt( initData (&f_dt, "dt", "time in the tracking framework") )
, f_host( initData (&f_host, "host", "IP address of server") )
, f_port( initData (&f_port, "port", "port number") )
, f_cameraMatrix( initData (&f_cameraMatrix, "cameraMatrix", "instrinsic camera matrix K") )
, f_qualities( initData (&f_qualities, "qualities", "init array of 3D point qualities") )
, f_status( initData (&f_status, "status", "status of the 3D point (lost or not)") )
, f_subset( initData (&f_subset, "subset", "groups of point (1 for instrument, 2 for surface)") )
, f_rt( initData (&f_rt, "rt", "rotation and translation matrices of the real cameras (camera pose estimation)") )
, f_outputImageData( initData (&f_outputImageData, "outputImageData", "outputImageData") )
{
	f_outputImageData.setDisplayed(false);
    m_flagReceived = 0;
    m_fps = 29;
}

template <class DataTypes>
PointsRegistrationEngine<DataTypes>::~PointsRegistrationEngine()
{
	    //m_fileLog.close();    
}


template <class DataTypes>
void PointsRegistrationEngine<DataTypes>::init()
{
    addInput(&f_inputX);
    addOutput(&f_outputX);
    setDirtyValue();
    //m_fileLog.open("error2.txt");
}

template <class DataTypes>
void PointsRegistrationEngine<DataTypes>::reinit()
{
    update();
}

bool findInVector(std::vector<int> v, int value)
{
    for (unsigned int i = 0; i < v.size(); i++)
        if (v[i] == value) return true;
    return false;
}

template <class DataTypes>
void PointsRegistrationEngine<DataTypes>::update()
{

    //sofa::component::visualmodel::BaseCamera bcamera;

    //Vec3 camPos = bcamera->getPosition();
    //Quat camRot = bcamera->getOrientation();

    cleanDirty();

    helper::ReadAccessor<Data<VecCoord> > in = f_inputX;
    helper::WriteAccessor<Data<VecCoord> > out = f_outputX;
    helper::WriteAccessor<Data<helper::vector<int> > > stat = f_status;
    helper::WriteAccessor<Data<VecCoord> > init = f_initPositions;
    helper::WriteAccessor<Data<VecCoord> > cp = f_cp;
    helper::WriteAccessor<Data<helper::vector<int> > > outputIndices = f_outputIndices;
    helper::WriteAccessor<Data<helper::vector<int> > > lastLostIndices = f_lastLostIndices;
    helper::WriteAccessor<Data<helper::vector<double> > > qualities = f_qualities;
    helper::WriteAccessor<Data<helper::vector<double> > > RT = f_rt;
    helper::WriteAccessor<Data<helper::vector<int> > > subset = f_subset;
    Vec3 scalePt = f_scalePt.getValue();

    try
    {
        // stream connection
        boost::asio::ip::tcp::iostream data_stream(f_host.getValue(), f_port.getValue());

        // receiving
        SofaARCapsule cap_recv;
        boost::archive::binary_iarchive ia(data_stream);
        ia >> cap_recv; // unserialization
        m_imgData = cap_recv.getImg();			// buffer image
        m_imageWidth = cap_recv.getImgWidth();		// image width
        m_imageHeight = cap_recv.getImgHeight();		// image height
        m_currentPositionsX = cap_recv.getCurrentPositionX(); // current positions
        m_currentPositionsY = cap_recv.getCurrentPositionY(); // current positions
        m_currentPositionsZ = cap_recv.getCurrentPositionZ(); // current positions
        m_currentStatus = cap_recv.getStatus(); // current positions status
	m_initQ = cap_recv.getCurrentQualities();

	m_initSubset = cap_recv.getSubset();
	f_dt = cap_recv.getDT();
        // resize the vector with the number of initial control point
        if (!m_flagReceived) {
            out.resize(m_currentPositionsX.size());
	    init.resize(m_currentPositionsX.size());
	    qualities.resize(m_initQ.size());
	    subset.resize(m_initSubset.size());
	    stat.resize(m_currentStatus.size());
            std::cout << "1st passage : Out Vector resized to : " << init.size() << " - Init Vector initialized"<< std::endl;
	  
		for(int l = 0 ; l < init.size(); l++) {
			// init positions

			init[l] = Coord(     (m_currentPositionsX.at(l) + RT[12])*scalePt[0], 
					     (m_currentPositionsY.at(l) + RT[13])*scalePt[1], 
					     (m_currentPositionsZ.at(l) + RT[14])*scalePt[2]
					);

//			init[l] = Coord(m_currentPositionsX.at(l), m_currentPositionsY.at(l), m_currentPositionsZ.at(l));

			// init positions qualities
			qualities[l] = m_initQ[l];
			subset[l] = m_initSubset[l];
			//std::cout << "v " << (m_currentPositionsX.at(l) + RT[12])*scalePt[0] << " " << (m_currentPositionsY.at(l) + RT[13])*scalePt[1] << " " << (m_currentPositionsZ.at(l) + RT[14])*scalePt[2] << std::endl;
			//std::cout << "v " << qualities[l] << std::endl;
		}
        }




        // used for resizing vectors and to prevent from opengl empty buffer
        m_flagReceived = 1;

        // sending back
        // Example of a process on the received data
        // the current positions received from server are
        // sent back after the 99.99 setting of the first element of a vector...
        // currentPositionsX.at(0) = 99.99;
	
        SofaARCapsule cap_sent(cap_recv.getImg(), cap_recv.getImgDepth(), cap_recv.getImgNC(), cap_recv.getImgWidth(), cap_recv.getImgHeight(), cap_recv.getDT(),
                               m_currentPositionsX, m_currentPositionsY, m_currentPositionsZ, m_currentStatus, m_initQ, m_initSubset);

        boost::archive::binary_oarchive oa(data_stream);
        oa << cap_sent; // serialization
	
        // stream closing
        data_stream.close();

        //std::cout << "Client <<< Server :\t"<< sizeof(cap_recv) << std::endl;
        //std::cout << "Client >>> Server :\t" << sizeof(cap_sent) << std::endl;


        // update the positions
        // since Boost::asio only support basic vector (ie : vector<float>, vector<int> ...)
        // we define a vector for each coordinate x,y,z.
        // we resize the out vector relativly to the state vector
        // and we save the indices of the LAST lost points

        unsigned int i,k;
        for (i = k = 0; i < m_currentStatus.size(); i++) {
	    stat[i] = m_currentStatus[i];
            // if we loose a point
            if ( !m_currentStatus[i] ) {
                // if it's the first lost indice we save it
                if (m_allLostIndices.size() == 0) {
                    lastLostIndices.push_back(i);
                    std::cout << "Last point lost has the indice : " << i << std::endl;
                    m_allLostIndices.push_back(i);
                }
                else if (!findInVector(m_allLostIndices, i)) {
                    // if not we test if it's a new lost indice
                    // if the indice is not in the lost indices vector, so it's the last one
                    lastLostIndices.push_back(i);
                    // stock the indice lost
                    m_allLostIndices.push_back(i);
                    std::cout << "Last point lost has the indice : " << i << std::endl;

                }
                // continue and get out of the FOR loop so the size of the OUT vector is not incremented
                continue;

            }
            out[k++] = Coord( (m_currentPositionsX.at(i) + RT[12])*scalePt[0], 
			     (m_currentPositionsY.at(i) + RT[13])*scalePt[1], 
			     (m_currentPositionsZ.at(i) + RT[14])*scalePt[2]
			);
//            out[k++] = Coord(m_currentPositionsX.at(i), m_currentPositionsY.at(i), m_currentPositionsZ.at(i));
        }
        
        if (out.size() != k) {
            out.resize(k);
            std::cout << "Output Engine Vector resized to : " << out.size() << std::endl;
            std::cout << "Number of lost points : "<< m_allLostIndices.size() << std::endl;
        }

        // compute the list of indices of the output points
        outputIndices.clear();
        outputIndices.resize(m_currentStatus.size() - m_allLostIndices.size());
        for (i = k = 0; i < m_currentStatus.size(); i++) {
            if (m_currentStatus[i]) {
                outputIndices[k] = i;
                k++;
            }
        }

        cp.resize((int)init.size()/10);

        for(int i = 0; i < cp.size(); i++) {
	    cp[i] = init[i+10];
	    //std::cout << cp[i] << std::endl;
	}

	const VecCoord& initPoints = f_initPositions.getValue();
        const VecCoord& controlPoints = f_cp.getValue();

	//getNN(initPoints, controlPoints, 5);
	

    }
    catch (std::exception& e)
    {
        //std::cerr << e.what() << std::endl;
    }

    // this clear is to keep only the last indice
    // std::cout << "Number of LAST lost point : " << lastLostIndices.size() << std::endl;
    lastLostIndices.clear();


    f_outputImageData = m_imgData;

}


template <class DataTypes>
void PointsRegistrationEngine<DataTypes>::draw(const core::visual::VisualParams* vparams)
{

//	GLfloat projectionMatrixData[16]; 
//	glGetFloatv (GL_PROJECTION_MATRIX, projectionMatrixData);
//	std::cout << "Focal =" << projectionMatrixData[0]*800 << std::endl;


	    
    if(m_flagReceived && f_showImg.getValue())
    {

	if(f_persView.getValue())
	{
	// PERSPECTIVE
	
		glEnable(GL_TEXTURE_2D);	// enable the texture
		glDisable(GL_LIGHTING);		// disable the light

		glBindTexture ( GL_TEXTURE_2D, 0 );  // texture bind
		glTexImage2D (GL_TEXTURE_2D, 0, GL_RGB, m_imageWidth, m_imageHeight, 0, GL_BGR_EXT, GL_UNSIGNED_BYTE, m_imgData.c_str() );
		//glTexImage2D (GL_TEXTURE_2D, 0, GL_LUMINANCE, m_imageWidth, m_imageHeight, 0, GL_BGR_EXT, GL_UNSIGNED_BYTE, m_imgData.c_str() );

		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);	// Linear Filtering

		float eps = 0.0;
		float z0 = 0.0;

		glBegin(GL_QUADS); //we draw a quad on the entire screen (0,1 - 1,1 - 1,0 - 0,0)
		glColor4f(1.0f,1.0f,1.0f,0.5f);

		float x0 = (float)m_imageWidth*f_scaleImg.getValue();
		float y0 = (float)m_imageHeight*f_scaleImg.getValue();

/*
		glTexCoord2f(0,1);
		glVertex3f(-x0/2 - eps,-y0/2 - eps, -z0);
		glTexCoord2f(1,1);
		glVertex3f(x0/2 + eps, -y0/2 - eps, -z0);
		glTexCoord2f(1,0);
		glVertex3f(x0/2 + eps, y0/2 + eps, -z0);
		glTexCoord2f(0,0);
		glVertex3f(-x0/2 - eps, y0/2 + eps, -z0);
		glEnd();
*/

		glTexCoord2f(0,1);
		glVertex3f(0,y0,0);
		glTexCoord2f(1,1);
		glVertex3f(x0, y0,0);
		glTexCoord2f(1,0);
		glVertex3f(x0, 0,0);
		glTexCoord2f(0,0);
		glVertex3f(0, 0, 0);
		glEnd();


		// glEnable(GL_DEPTH_TEST);
		glEnable(GL_LIGHTING);		// enable light
		glDisable(GL_TEXTURE_2D);	// disable texture 2D
		//glDepthMask (GL_TRUE);		// enable zBuffer
	}
	else
	{
	// ORTHO

		glMatrixMode(GL_PROJECTION);	//init the projection matrix
		glPushMatrix();
		glLoadIdentity();
		glOrtho(0,1,0,1,-1,1);  // orthogonal view		
		glMatrixMode(GL_MODELVIEW);  
		glPushMatrix(); 
		glLoadIdentity(); 

		// BACKGROUND TEXTURING
		//glDepthMask (GL_FALSE);		// disable the writing of zBuffer
		glDisable(GL_DEPTH_TEST);
		glEnable(GL_TEXTURE_2D);	// enable the texture	
		glDisable(GL_LIGHTING);		// disable the light
				
		glBindTexture ( GL_TEXTURE_2D, 0 );  // texture bind	
		glTexImage2D (GL_TEXTURE_2D, 0, GL_RGB, m_imageWidth, m_imageHeight, 0, GL_BGR_EXT, GL_UNSIGNED_BYTE, m_imgData.c_str() ); 	

		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);	// Linear Filtering
		 
		// BACKGROUND DRAWING		 
		//glEnable(GL_DEPTH_TEST);		

		glBegin(GL_QUADS); //we draw a quad on the entire screen (0,0 1,0 1,1 0,1)  
			glColor4f(1.0f,1.0f,1.0f,1.0f); 
			glTexCoord2f(0,1);		glVertex2f(0,0);
			glTexCoord2f(1,1);		glVertex2f(1,0);
			glTexCoord2f(1,0);		glVertex2f(1,1);
			glTexCoord2f(0,0);		glVertex2f(0,1);
		glEnd();             

		//glEnable(GL_DEPTH_TEST);
		glEnable(GL_LIGHTING);		// enable light
		glDisable(GL_TEXTURE_2D);	// disable texture 2D	
		glEnable(GL_DEPTH_TEST);
		//glDepthMask (GL_TRUE);		// enable zBuffer
				
		glPopMatrix();
		glMatrixMode(GL_PROJECTION);
		glPopMatrix();
		glMatrixMode(GL_MODELVIEW);
	}

    }
    else
        std::cout <<  "Waiting for data... " << std::endl;

}

} // namespace engine

} // namespace component

} // namespace sofa

#endif
