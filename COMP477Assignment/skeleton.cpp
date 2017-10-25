#include "skeleton.h"
#include "splitstring.h"
#include "Eigen\Geometry"
#include <fstream>
#include <cmath>

/*
 * Load skeleton file
 */
void Skeleton::loadSkeleton(std::string skelFileName)
{
    std::string strBone;
    std::ifstream skelFile(skelFileName.c_str());
    if (skelFile.is_open())
    {
        while ( std::getline(skelFile, strBone)) { //Read a line to build a bone
            std::vector<std::string> boneParams;
            splitstring splitStr(strBone);
            boneParams = splitStr.split(' ');
            Joint temp;
            temp.position.x = std::atof(boneParams[1].c_str());
            temp.position.y = std::atof(boneParams[2].c_str());
            temp.position.z = std::atof(boneParams[3].c_str());
            temp.parentID = std::atoi(boneParams[4].c_str());
            if (std::atoi(boneParams[0].c_str()) != joints.size())
            {
                std::cout<<"[Warning!!!] Bone index not match\n";
            }
            joints.push_back(temp);
        }
    }
	std::vector<Eigen::Matrix4f> temp_matrix;
	for (unsigned i = 1; i < joints.size(); i++)
	{
		temp_matrix.push_back(Eigen::Matrix4f::Identity());
	}
	key_frame.push_back(temp_matrix);
}

void Skeleton::convert_eigen_to_float_matrix(float *local_t)
{
	
}

void Skeleton::save_key_frame()
{
	std::vector<Eigen::Matrix4f> temp;
	for (unsigned i = 1; i < joints.size(); i++)
	{
		Eigen::Matrix4f temp_transform;
		temp_transform << joints[i].local_t[0], joints[i].local_t[4], joints[i].local_t[8], joints[i].local_t[12],
			joints[i].local_t[1], joints[i].local_t[5], joints[i].local_t[9], joints[i].local_t[13],
			joints[i].local_t[2], joints[i].local_t[6], joints[i].local_t[10], joints[i].local_t[14],
			joints[i].local_t[3], joints[i].local_t[7], joints[i].local_t[11], joints[i].local_t[15];
		temp.push_back(temp_transform);
	}
	key_frame.push_back(temp);
}

void Skeleton::delete_key_frame()
{
	key_frame.pop_back();
}

void Skeleton::clear_key_frame()
{
	key_frame.erase(key_frame.begin()+1, key_frame.end());
}

std::vector<std::vector<Eigen::Quaternionf>> Skeleton::matrix_to_quaternion(std::vector<std::vector<Eigen::Matrix4f>> m_vector)
{
	std::vector<std::vector<Eigen::Quaternionf>> q_vector;
	for (unsigned i = 0; i < m_vector.size(); i++)
	{
		std::vector<Eigen::Quaternionf> q;
		for (unsigned j = 0; j < m_vector[i].size(); j++)
		{
			Eigen::Matrix3f temp_matrix;
			temp_matrix << key_frame[i][j](0, 0), key_frame[i][j](0, 1), key_frame[i][j](0, 2),
				key_frame[i][j](1, 0), key_frame[i][j](1, 1), key_frame[i][j](1, 2),
				key_frame[i][j](2, 0), key_frame[i][j](2, 1), key_frame[i][j](2, 2);
			temp_matrix.normalize();
			Eigen::Quaternionf temp(temp_matrix);
			temp.normalize();
			q.push_back(temp);
		}
		q_vector.push_back(q);
	}
	return q_vector;
}

std::vector<std::vector<Eigen::Matrix4f>> quaternion_to_matrix(std::vector<std::vector<Eigen::Quaternionf>> q_vector)
{
	std::vector<std::vector<Eigen::Matrix4f>> m_vector;
	for (unsigned i = 0; i < q_vector.size(); i++)
	{
		vector<Eigen::Matrix4f> m;
		for (unsigned j = 0; j < q_vector[i].size(); j++)
		{
			Eigen::Matrix4f temp_m4 = Eigen::Matrix4f::Identity();
			Eigen::Matrix3f temp_m3 = q_vector[i][j].toRotationMatrix();
			temp_m4 << temp_m3(0, 0), temp_m3(0, 1), temp_m3(0, 2), 0,
				temp_m3(1, 0), temp_m3(1, 1), temp_m3(1, 2), 0,
				temp_m3(2, 0), temp_m3(2, 2), temp_m3(2, 3), 0,
				0, 0, 0, 1;
			m.push_back(temp_m4);
		}
		m_vector.push_back(m);
	}
	return m_vector;
}

void Skeleton::save_animation(std::string animationFileName)
{
	std::ofstream animation(animationFileName.c_str());
	for (unsigned i = 0; i < key_frame.size(); i++)
	{
		animation << i << " ";
		for (unsigned j = 0; j < key_frame[i].size(); j++)
		{
			Eigen::Matrix3f temp_matrix;
			temp_matrix << key_frame[i][j](0,0), key_frame[i][j](0,1), key_frame[i][j](0,2),
				key_frame[i][j](1,0), key_frame[i][j](1,1), key_frame[i][j](1,2),
				key_frame[i][j](2,0), key_frame[i][j](2,1), key_frame[i][j](2,2);
			temp_matrix.normalize();
			Eigen::Quaternionf temp(temp_matrix);
			temp.normalize();
			animation << temp.w() << " " << temp.x() << " " << temp.y() << " " << temp.z() << " ";
		}
		animation << std::endl;
	}

}

/*
 * Load Animation
 */
void Skeleton::loadAnimation(std::string skelFileName)
{
}


/*
 * Draw skeleton with OpenGL
 */
void Skeleton::glDrawSkeleton()
{
    //Rigging skeleton
    glDisable(GL_DEPTH_TEST);
    
    glPushMatrix();
    glTranslatef(-0.9,-0.9,-0.9);
	glScalef(1.8,1.8,1.8);
	glPointSize(6);
	glColor3f(1,0,0);
    updateScreenCoord();
    
    for (unsigned i=0; i<joints.size(); i++)
    {
        glPushMatrix();
        if (joints[i].isPicked)
            glColor3f(1.0, 0.0, 0.0);
        else if (joints[i].isHovered)
            glColor3f(0.7, 0.7, 0.7);
        else
            glColor3f(0.3, 0.3, 0.3);

        glMultMatrixf(joints[i].global_t);
        
        glTranslated(joints[i].position.x, joints[i].position.y, joints[i].position.z);
        glutSolidSphere(0.01, 15, 15);
        glTranslated(-joints[i].position.x, -joints[i].position.y, -joints[i].position.z);
        
        if (joints[i].parentID !=-1)
        {
            glColor3f(0.7, 0.3, 0.3);
            glBegin(GL_LINES);
                glVertex3d(joints[i].position.x, joints[i].position.y, joints[i].position.z);
                glVertex3d(joints[joints[i].parentID].position.x,
                        joints[joints[i].parentID].position.y,
                        joints[joints[i].parentID].position.z);
            glEnd();
        }
        glPopMatrix();
    }
    glPopMatrix();
    
    glEnable(GL_DEPTH_TEST);
}

void Skeleton::updateScreenCoord()
{
    GLint viewport[4];
    GLdouble modelview[16];
    GLdouble projection[16];
    GLdouble winX, winY, winZ;

    glGetDoublev( GL_MODELVIEW_MATRIX, modelview );
    glGetDoublev( GL_PROJECTION_MATRIX, projection );
    glGetIntegerv( GL_VIEWPORT, viewport );
    for (unsigned i=0; i<joints.size(); i++)
    {
        double mm[16];
        mult(modelview, joints[i].global_t, mm);
        
        gluProject((GLdouble)joints[i].position.x, (GLdouble)joints[i].position.y, (GLdouble)joints[i].position.z,
                mm, projection, viewport,
                &winX, &winY, &winZ );
        joints[i].screenCoord.x = winX;
        joints[i].screenCoord.y = (double)glutGet(GLUT_WINDOW_HEIGHT)-winY;
    }
}
void Skeleton::checkHoveringStatus(int x, int y)
{
    double distance = 0.0f;
    double minDistance = 1000.0f;
    int hoveredJoint = -1;
    for(unsigned i=0; i < joints.size(); i++)
    {
        joints[i].isHovered = false;
        distance = sqrt((x - joints[i].screenCoord.x)*(x - joints[i].screenCoord.x) 
                + (y - joints[i].screenCoord.y)*(y - joints[i].screenCoord.y));
        if (distance > 50) continue;
        if (distance < minDistance)
        {
            hoveredJoint = i;
            minDistance = distance;
        }
    }
    if (hoveredJoint != -1) joints[hoveredJoint].isHovered = true;
}

void Skeleton::release()
{
    hasJointSelected = false;
    for (unsigned i=0; i<joints.size(); i++)
    {
        joints[i].isPicked = false;
    }
}

void Skeleton::addRotation(float* q)
{
    for(int i=0; i<joints.size(); ++i)
    {
        if(joints[i].isPicked)
        {
            mult(joints[i].local_t, q, joints[i].local_t);
            break;
        }
    }
    
}

void Skeleton::updateGlobal()
{
    for(int i=0; i<joints.size(); ++i)
    {
        if(joints[i].parentID!=-1)
        {
            float g[16];
            int p=joints[i].parentID;
            
            float p_pos[3];
            p_pos[0]=joints[p].position.x;
            p_pos[1]=joints[p].position.y;
            p_pos[2]=joints[p].position.z;
            
            
            float tr[16];
            
            trans(joints[p].global_t, p_pos, g);
            
            mult(g, joints[i].local_t, g);
            
            p_pos[0]=-p_pos[0];
            p_pos[1]=-p_pos[1];
            p_pos[2]=-p_pos[2];
            
            trans(g, p_pos, joints[i].global_t);
        }
    }
}

void Skeleton::selectOrReleaseJoint()
{
    bool hasHovered=false;
    for (unsigned i=0; i<joints.size(); i++)
    {
        joints[i].isPicked = false;
        if (joints[i].isHovered)
        {
            hasHovered = true;
            joints[i].isPicked = true;
            hasJointSelected = true;
        }
    }
    if (!hasHovered)    //Release joint
        hasJointSelected = false;
}

