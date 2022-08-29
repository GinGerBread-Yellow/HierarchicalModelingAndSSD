#include "SkeletalModel.h"

#include <FL/Fl.H>

using namespace std;

void SkeletalModel::load(const char *skeletonFile, const char *meshFile, const char *attachmentsFile)
{
	loadSkeleton(skeletonFile);

	m_mesh.load(meshFile);
	m_mesh.loadAttachments(attachmentsFile, m_joints.size());

	computeBindWorldToJointTransforms();
	updateCurrentJointToWorldTransforms();
}

void SkeletalModel::draw(Matrix4f cameraMatrix, bool skeletonVisible)
{
	// draw() gets called whenever a redraw is required
	// (after an update() occurs, when the camera moves, the window is resized, etc)

	m_matrixStack.clear();
	m_matrixStack.push(cameraMatrix);

	if( skeletonVisible )
	{
		drawJoints();

		drawSkeleton();
	}
	else
	{
		// Clear out any weird matrix we may have been using for drawing the bones and revert to the camera matrix.
		glLoadMatrixf(m_matrixStack.top());

		// Tell the mesh to draw itself.
		m_mesh.draw();
	}
}

void SkeletalModel::loadSkeleton( const char* filename )
{
	// Load the skeleton from file here.
	float x, y, z;
	unsigned parent;

	ifstream fin(filename);

	// m_rootJoint
	// m_joints

	while(fin >> x >> y >> z >> parent) {
		Joint *temp = new Joint;
		temp->transform = Matrix4f::translation(x,y,z);

		if(parent != -1)
			m_joints[parent]->children.push_back(temp);
		m_joints.push_back(temp);
	}
	// init root
	m_rootJoint = m_joints[0];

	fin.close();
}

void SkeletalModel::drawJoints( )
{
	// Draw a sphere at each joint. You will need to add a recursive helper function to traverse the joint hierarchy.
	//
	// We recommend using glutSolidSphere( 0.025f, 12, 12 )
	// to draw a sphere of reasonable size.
	//
	// You are *not* permitted to use the OpenGL matrix stack commands
	// (glPushMatrix, glPopMatrix, glMultMatrix).
	// You should use your MatrixStack class
	// and use glLoadMatrix() before your drawing call.

	drawJoints_helper(m_rootJoint);
}

void SkeletalModel::drawJoints_helper(Joint *joint) {

	m_matrixStack.push(joint->transform);

	for(int i = 0, n = joint->children.size(); i < n; ++i) {
		drawJoints_helper(joint->children[i]);
	}

	glLoadMatrixf(m_matrixStack.top());
	glutSolidSphere( 0.025f, 12, 12 );

	m_matrixStack.pop();

}

void SkeletalModel::drawSkeleton( )
{
	// Draw boxes between the joints. You will need to add a recursive helper function to traverse the joint hierarchy.

	drawSkeleton_helper(m_rootJoint);
}

void SkeletalModel::drawSkeleton_helper(Joint *joint) {

	m_matrixStack.push(joint->transform);

	
	for(int i = 0, n = joint->children.size(); i < n; ++i) {		
		drawSkeleton_helper(joint->children[i]);
	}

	m_matrixStack.pop();

	if(joint == m_rootJoint) return;
	// translate z+0.5
	Matrix4f translateZ = Matrix4f::translation(0.f, 0.f, 0.5f); // move up 5

	Vector3f parentOffset = joint->transform.getCol(3).xyz();
	float l = parentOffset.abs();
	// scale to 0.5 -> 0.025, 1 -> l
	Matrix4f scale = Matrix4f::scaling( 0.05f, 0.05f, l);

	// rotate z = parentOffset, y = (z × rnd).normalized(), and x = (y × z).normalized(),
	Vector3f rnd = Vector3f(0.f, 0.f, 1.f);
	Vector3f rotZ = parentOffset.normalized();
	Vector3f rotY = Vector3f::cross(rotZ, rnd).normalized();
	Vector3f rotX = Vector3f::cross(rotY, rotZ).normalized();

	Matrix4f rotate = Matrix4f(rotX[0], rotY[0], rotZ[0], 0.f, 
		rotX[1], rotY[1], rotZ[1], 0.f, 
		rotX[2], rotY[2], rotZ[2], 0.f, 
		0.f, 0.f, 0.f, 1.f);

	m_matrixStack.push(rotate);
	m_matrixStack.push(scale);
	m_matrixStack.push(translateZ);

	glLoadMatrixf(m_matrixStack.top());
	glutSolidCube( 1.0f );

	m_matrixStack.pop();
	m_matrixStack.pop();
	m_matrixStack.pop();

}

void SkeletalModel::setJointTransform(int jointIndex, float rX, float rY, float rZ)
{
	// Set the rotation part of the joint's transformation matrix based on the passed in Euler angles.
	Matrix3f rot = Matrix3f::rotateX(rX) * Matrix3f::rotateY(rY) * Matrix3f::rotateZ(rZ);
	m_joints[jointIndex]->transform.setSubmatrix3x3( 0, 0, rot);

}


void SkeletalModel::computeBindWorldToJointTransforms()
{
	// 2.3.1. Implement this method to compute a per-joint transform from
	// world-space to joint space in the BIND POSE.
	//
	// Note that this needs to be computed only once since there is only
	// a single bind pose.
	//
	// This method should update each joint's bindWorldToJointTransform.
	// You will need to add a recursive helper function to traverse the joint hierarchy.

	m_matrixStack.clear();
	computeBind_helper(m_rootJoint);
}

void SkeletalModel::computeBind_helper(Joint *joint)
{
	m_matrixStack.push(joint->transform);
	for(int i = 0, n = joint->children.size(); i < n; ++i) {		
		computeBind_helper(joint->children[i]);
	}
	joint->bindWorldToJointTransform = m_matrixStack.top().inverse();
	
	m_matrixStack.pop();

	return;
}

void SkeletalModel::updateCurrentJointToWorldTransforms()
{
	// 2.3.2. Implement this method to compute a per-joint transform from
	// joint space to world space in the CURRENT POSE.
	//
	// The current pose is defined by the rotations you've applied to the
	// joints and hence needs to be *updated* every time the joint angles change.
	//
	// This method should update each joint's bindWorldToJointTransform.
	// You will need to add a recursive helper function to traverse the joint hierarchy.
	m_matrixStack.clear();
	updateCurrent_helper(m_rootJoint);
}

void SkeletalModel::updateCurrent_helper(Joint *joint)
{
	m_matrixStack.push(joint->transform);
	for(int i = 0, n = joint->children.size(); i < n; ++i) {		
		updateCurrent_helper(joint->children[i]);
	}
	joint->currentJointToWorldTransform = m_matrixStack.top();
	
	m_matrixStack.pop();

	return;
}

void SkeletalModel::updateMesh()
{
	// 2.3.2. This is the core of SSD.
	// Implement this method to update the vertices of the mesh
	// given the current state of the skeleton.
	// You will need both the bind pose world --> joint transforms.
	// and the current joint --> world transforms.
	/** mesh {currentVertices, faces, } */

	int n = m_mesh.currentVertices.size();

	for(int i = 0; i < n; i++) {
		// calculate p_i = convex combination of T B^(-1) p_i
		std::vector< float > &weights = m_mesh.attachments[i];
		Vector4f pos(m_mesh.bindVertices[i], 1.f);
		Vector4f npos(0.f);
		for(int j = 0; j < 17; j++) {
			// for each joint
			if(weights[j] <= 0) continue;
			// assert(weights[j] <= 1);
			npos = npos + weights[j] *
					(m_joints[j+1]->currentJointToWorldTransform *
					m_joints[j+1]->bindWorldToJointTransform *
					pos);
		}
		m_mesh.currentVertices[i] = npos.xyz();
	}
}

