#include <fstream>
#include <iostream>
#include <functional>
#include <cctype>
#include <locale>
#include <sstream>
#include <ostream>
#include <algorithm>

#include <osg/Matrix>
#include <osg/ShapeDrawable>
#include <osg/Shape>
#include <osg/Quat>

#include "parser.hpp"


class getWorldCoordOfNodeVisitor : public osg::NodeVisitor
{
	public:
		getWorldCoordOfNodeVisitor() :

		osg::NodeVisitor(NodeVisitor::TRAVERSE_PARENTS), done(false)
		{
			wcMatrix = new osg::Matrix();
		}

		virtual void apply(osg::Node& node)
		{
		 if (!done)
		 {
			if ( node.getNumParents() == 0 ) // no parents
			{
			   wcMatrix->set( osg::computeLocalToWorld(this->getNodePath()) );
			   done = true;
			}
			traverse(node);
		 }
		}

		osg::Matrix* giveUpDaMat()
		{
			return wcMatrix;
		}

	private:
	    bool done;
	    osg::Matrix* wcMatrix;
};

osg::Matrix* getWorldCoords(osg::ref_ptr<osg::Node> node)
{
   getWorldCoordOfNodeVisitor* ncv = new getWorldCoordOfNodeVisitor();
   if (node && ncv)
   {
      node->accept(*ncv);
      return ncv->giveUpDaMat();
   }
   else
   {
      return NULL;
   }
}


// ----------------------- SUPPORT -------------------------------

// Trim from start
static inline std::string &ltrim(std::string& s) {
        s.erase(s.begin(), std::find_if(s.begin(), s.end(), std::not1(std::ptr_fun<int, int>(std::isspace))));
        return s;
}

// Trim from end
static inline std::string &rtrim(std::string& s)
{
        s.erase(std::find_if(s.rbegin(), s.rend(), std::not1(std::ptr_fun<int, int>(std::isspace))).base(), s.end());
        return s;
}

// Trim from both ends
static inline std::string& trim(std::string& s)
{
    return ltrim(rtrim(s));
}

// Split string into tokens
static inline std::vector<std::string>& split(const std::string& s, char delim, std::vector<std::string> &elems)
{
    std::stringstream ss(s);
    std::string item;
    while (std::getline(ss, item, delim)) {
        elems.push_back(item);
    }
    return elems;
}

static inline std::vector<std::string> split(const std::string &s, char delim)
{
    std::vector<std::string> elems;
    split(s, delim, elems);
    return elems;
}

static inline std::istream& getline_trimmed(std::istream& is, std::string& str)
{
	getline(is, str);
	str = trim(str);
	return is;
}

static inline float lerp(float l, float r, float t)
{
    return (l + ( (r - l) * t));
}

static inline osg::Vec3f lerp(const osg::Vec3f& l, const osg::Vec3f& r, float t)
{
    return (l + ( (r - l) * t));
}

static inline osg::Matrixf lerp(const osg::Matrixf& l, const osg::Matrixf& r, float t)
{
    return (l + ( (r + (l*(-1))) * t));
}

// ----------------------- END SUPPORT -------------------------------


std::ostream& operator<< (std::ostream& out, Joint& joint)
{
	out << "NAME: " << joint.name << std::endl;
	Joint* parent = dynamic_cast<Joint*>(joint.getParent(0));
	out << "PARENT: " << (parent ? parent->getName() : "NULL") << std::endl;

	out << "POSITION:";
	for (size_t i = 0; i < 3; ++i)
		 out << " " << joint.getPosition()[i];
	out << std::endl;

	out << "CHANNELS: " << joint.numChannels;
	for (size_t i = 0; i < joint.channels.size(); ++i)
		 out << " " << joint.channels[i];
	out << std::endl;

	return out;
}


osg::ref_ptr<BVH> BVHParser::parse(const std::string& filename)
{
	osg::ref_ptr<Hierarchy> hierarchy = BVHParser::readHierarchy(filename);
	if (!hierarchy)
	{
		std::cout << "Failed to create the hierarchy, exiting.." << std::endl;
		return NULL;
	}

	osg::ref_ptr<BVH> bvh = BVHParser::readMotion(filename, hierarchy);

	if (!bvh)
	{
		std::cout << "Failed to create the BVH, exiting.." << std::endl;
		return NULL;
	}

	return bvh;
}


osg::ref_ptr<Hierarchy> BVHParser::readHierarchy(const std::string& filename)
{
	// Open file
	std::ifstream mocapfile(filename.c_str());

	if ( mocapfile.is_open() )
	{
		std::string line;
		getline_trimmed(mocapfile, line);

		// Check that the file is a valid BVH file, i.e. begins with HIERARCHY
		if ( line.find("HIERARCHY") == std::string::npos )
		{
			std::cout << "Not a valid BVH file, exiting.." << std::endl;
			mocapfile.close();
			return NULL;
		}

		// Read the root and build the hierarchy
		getline_trimmed(mocapfile, line);
		osg::ref_ptr<Joint> rootJoint = readJoint(mocapfile, NULL, line, 0, 0);
		mocapfile.close();

		return new Hierarchy(rootJoint);
	}
	else
	{
		std::cout << "Unable to open file: " << filename << " for reading" << std::endl;
		return NULL;
	}
}


osg::ref_ptr<Joint> BVHParser::readJoint(std::ifstream& mocapfile, osg::ref_ptr<Joint> parent, const std::string& name, size_t dataIndex, size_t treeIndex)
{
    size_t idx = dataIndex, tidx = treeIndex;
    std::cout << "* Creating a new joint: " << split(name, ' ')[1] << " didx: " << idx << " tidx: " << treeIndex << std::endl;
    osg::ref_ptr<Joint> joint = new Joint(split(name, ' ')[1], parent, idx, tidx);
    if (parent)
	{
		std::cout << "****** Adding child for joint: " << parent->getName() << " : " << joint->getName() << std::endl;
		parent->addChild(joint);
		//segments.push_back(Segment(parent, joint, root));
	}

	std::string line;
	while ( getline_trimmed(mocapfile, line) )
	{
		// Read: OFFSET x y z
		if ( line.find("OFFSET") != std::string::npos )
		{
			// Read the offset from the stream and set the position attribute of the
			// PositionAttitudeTransform
			std::stringstream stream(line);
			std::string str;
			osg::Vec3f offset;
			stream >> str >> offset[0] >> offset[1] >> offset[2];
			joint->setPosition(offset);
			joint->offset = offset;

			// Set the rotation attribute to identity
			osg::Matrixf identity;
			identity.makeIdentity();
			joint->setAttitude(identity.getRotate());

			// If the joint isn't an end site - draw it with a sphere
			if (joint->getName() != "Site")
				BVHParser::drawJoint(joint);
			// If the joint has a parent draw a bone in between
			if (parent)
				BVHParser::drawBone(parent, joint);
		}
		// Read: CHANNELS numChannels channel1 channel2 ... channelN
		else if ( line.find("CHANNELS") != std::string::npos ) {
			std::stringstream stream(line);
			std::string str;
			size_t numChannels;
			stream >> str >> numChannels;

			joint->setNumChannels(numChannels);
			std::vector<std::string> channels(numChannels);
			for (size_t i = 0; i < numChannels; ++i)
			{
				stream >> str;
				channels[i] = str;
			}
			joint->setChannels(channels);
		}

		else if ( line.find("ROOT") != std::string::npos || line.find("JOINT") != std::string::npos)
		{
			readJoint(mocapfile, joint, line, ++idx, ++tidx);
		}
		else if (line.find("End Site") != std::string::npos)
		{
			readJoint(mocapfile, joint, line, idx, ++tidx);
		}
		else if ( line == "}" )
		{
			osg::ref_ptr<Joint> temp = joint;
			while ( getline_trimmed(mocapfile, line) )
			{
				if ( line == "}" )
					temp = dynamic_cast<Joint*>(temp->getParent(0));
				else
					break;
			}
			if ( line.find("ROOT") != std::string::npos || line.find("JOINT") != std::string::npos )
				readJoint(mocapfile, dynamic_cast<Joint*>(temp->getParent(0)), line, ++idx, ++tidx);
			else
				return joint;
		}
	}
	return joint;
}

osg::ref_ptr<BVH> BVHParser::readMotion(const std::string& filename, osg::ref_ptr<Hierarchy> hierarchy)
{
	std::ifstream mocapfile(filename.c_str());
	if ( mocapfile.is_open() )
	{
		std::string line;
		while ( getline_trimmed(mocapfile, line) )
			if (line == "MOTION") break;

		// Read the number of frames and frame time
		std::string f;
		size_t frames;
		float ftime;
		mocapfile >> f >> frames >> f >> f >> ftime;

		// Read the motion data
		std::vector<std::vector<float> > motiondata(frames);
		for (size_t i = 0; i < frames; ++i)
		{
			getline_trimmed(mocapfile, line);
			if (line == "") {
				i--;
				continue;
			}
			std::istringstream iss(line);
			float n;
			while (iss >> n)
				motiondata[i].push_back(n);
		}

		mocapfile.close();
		osg::ref_ptr<BVH> bvh =  new BVH(hierarchy, motiondata, frames, ftime);
		return bvh;
	}
	else
	{
		std::cout << "Unable to open file: " << filename << " for reading" << std::endl;
		return NULL;
	}
}

void BVHParser::drawJoint(osg::ref_ptr<Joint> joint)
{

	osg::ref_ptr<osg::Sphere> sphere = new osg::Sphere( osg::Vec3(0.0f,0.0f,0.0f), 3.0f);
	osg::ref_ptr<osg::ShapeDrawable> sphereDrawable = new osg::ShapeDrawable(sphere);
	osg::ref_ptr<osg::Geode> sphereGeode = new osg::Geode();

	if (joint->getName() == "Head") {
		sphereDrawable->setColor(osg::Vec4f(0.0f, 0.0f, 1.0f, 1.0f));
		sphere->setRadius(6.0f);
	}
	else if (joint->getName() == "Neck")
		sphereDrawable->setColor(osg::Vec4f(1.0f, 1.0f, 0.0f, 1.0f));
	else if (joint->getName() == "LeftWrist")
		sphereDrawable->setColor(osg::Vec4f(1.0f, 0.0f, 0.0f, 1.0f));
	else if (joint->getName() == "RightWrist")
		sphereDrawable->setColor(osg::Vec4f(1.0f, 0.0f, 0.0f, 1.0f));
	else if (joint->getName() == "LeftAnkle")
		sphereDrawable->setColor(osg::Vec4f(0.0f, 1.0f, 0.0f, 1.0f));
	else if (joint->getName() == "RightAnkle")
		sphereDrawable->setColor(osg::Vec4f(0.0f, 1.0f, 0.0f, 1.0f));

	sphereGeode->addDrawable(sphereDrawable);
	joint->addChild(sphereGeode);
}

void BVHParser::drawBone(osg::ref_ptr<Joint> parent, osg::ref_ptr<Joint> child)
{
	// Don't draw a bone between the head and the end site
	if (parent->getName() == "Head")
		return;

	static float radius = 2.0f;

	osg::Vec3f center;
	float height;

	osg::Cylinder* cylinder;
	osg::ShapeDrawable* cylinderDrawable;
	osg::Geode* geode;

	osg::Vec3f start = osg::Vec3f(0.0f, 0.0f, 0.0f);
	osg::Vec3f end = child->getPosition();

	// Calculate the height and center of the cylinder
	height = (start - end).length();
	center = osg::Vec3( (start.x() + end.x()) / 2.0f,
						(start.y() + end.y()) / 2.0f,
						(start.z() + end.z()) / 2.0f );

	// This is the default direction for the cylinders to face in OpenGL
	osg::Vec3f z = osg::Vec3(0,0,1);

	// Get diff between two points you want cylinder along
	osg::Vec3f p = (end - start);

	// Get CROSS product (the axis of rotation)
	osg::Vec3f t = z^p;

	// Get angle. length is magnitude of the vector
	double angle = acos( (z * p) / p.length() );

	// Create a cylinder between the two points with the given radius
	cylinder = new osg::Cylinder(center, radius, height);
	cylinder->setRotation(osg::Quat(angle, osg::Vec3(t.x(), t.y(), t.z())));

	// A Geode to hold our cylinder
	geode = new osg::Geode;
	cylinderDrawable = new osg::ShapeDrawable(cylinder);
	geode->addDrawable(cylinderDrawable);

	// A PositionAttitudeTransform to hold our Geode
	osg::ref_ptr<osg::PositionAttitudeTransform> pat = new osg::PositionAttitudeTransform();

	// Add the PositionAttitude as a child of the parent
	pat->addChild(geode);
	parent->addChild(pat);
}



BVH::BVH(osg::ref_ptr<Hierarchy> hierarchy, std::vector<std::vector<float> > motiondata, size_t frames, float ftime)
	: osg::Referenced(), hierarchy(hierarchy), motionData(motiondata), frames(frames), frameTime(ftime), currentTime(0.0f), interpolationMethod(None)
{
	// Generate world coordinate data of the joints in order to accommodate lerp interpolation
	// :'((((((((
	this->motionDataWc.resize(frames);
	this->trajectories.resize(100); // TODO: fix to accommodate only number of joints

/*
	this->lerpPoints = new osg::Vec3Array();
	this->lerpColors = new osg::Vec4Array();

	this->lerpGeom = new osg::Geometry();
	this->lerpGeom->setVertexArray(this->lerpPoints);
	   lerpPoints->push_back( osg::Vec3( 0, 0, 0) ); // front left
	   lerpPoints->push_back( osg::Vec3(10, 0, 0) ); // front right
	   lerpPoints->push_back( osg::Vec3(10,10, 0) ); // back right
	   lerpPoints->push_back( osg::Vec3( 0,10, 0) ); // back left
	   lerpPoints->push_back( osg::Vec3( 5, 5,10) ); // peak

	   osg::DrawElementsUInt* pyramidBase =
	        new osg::DrawElementsUInt(osg::PrimitiveSet::QUADS, 0);
	     pyramidBase->push_back(3);
	     pyramidBase->push_back(2);
	     pyramidBase->push_back(1);
	     pyramidBase->push_back(0);
	     lerpGeom->addPrimitiveSet(pyramidBase);

	     osg::DrawElementsUInt* pyramidFaceOne =
	        new osg::DrawElementsUInt(osg::PrimitiveSet::TRIANGLES, 0);
	     pyramidFaceOne->push_back(0);
	     pyramidFaceOne->push_back(1);
	     pyramidFaceOne->push_back(4);
	     lerpGeom->addPrimitiveSet(pyramidFaceOne);

	     osg::DrawElementsUInt* pyramidFaceTwo =
	        new osg::DrawElementsUInt(osg::PrimitiveSet::TRIANGLES, 0);
	     pyramidFaceTwo->push_back(1);
	     pyramidFaceTwo->push_back(2);
	     pyramidFaceTwo->push_back(4);
	     lerpGeom->addPrimitiveSet(pyramidFaceTwo);

	     osg::DrawElementsUInt* pyramidFaceThree =
	        new osg::DrawElementsUInt(osg::PrimitiveSet::TRIANGLES, 0);
	     pyramidFaceThree->push_back(2);
	     pyramidFaceThree->push_back(3);
	     pyramidFaceThree->push_back(4);
	     lerpGeom->addPrimitiveSet(pyramidFaceThree);

	     osg::DrawElementsUInt* pyramidFaceFour =
	        new osg::DrawElementsUInt(osg::PrimitiveSet::TRIANGLES, 0);
	     pyramidFaceFour->push_back(3);
	     pyramidFaceFour->push_back(0);
	     pyramidFaceFour->push_back(4);
	     lerpGeom->addPrimitiveSet(pyramidFaceFour);

		lerpColors->push_back(osg::Vec4(1.0f, 0.0f, 0.0f, 1.0f) ); //index 0 red
		   lerpColors->push_back(osg::Vec4(0.0f, 1.0f, 0.0f, 1.0f) ); //index 1 green
		   lerpColors->push_back(osg::Vec4(0.0f, 0.0f, 1.0f, 1.0f) ); //index 2 blue
		   lerpColors->push_back(osg::Vec4(1.0f, 1.0f, 1.0f, 1.0f) ); //index 3 white
		   lerpColors->push_back(osg::Vec4(1.0f, 0.0f, 0.0f, 1.0f) ); //index 4 red

    this->lerpGeom->setColorArray(this->lerpColors, osg::Array::BIND_PER_VERTEX);
	//this->lerpGeom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINES, 0, this->lerpPoints->size()));
*/



	this->lerpGeode = new osg::Geode();
	this->lerpGeode->addDrawable(this->lerpGeom);

	this->update(0);
	this->recordWorldCoordinates(hierarchy->getRoot(), 0);
	for (size_t i = 1; i < frames; ++i)
	{
		this->update(ftime);
		this->recordWorldCoordinates(hierarchy->getRoot(), i);
	}

	/*
	for (size_t i = 0; i < frames; ++i)
	{
		std::cout << "Frame " << i << " - size: " << motionDataWc[i].size() << std::endl;
		for (size_t j = 0; j < motionDataWc[i].size(); ++j)
		{
			osg::Vec4d v;
			v = osg::Vec4(1, 0, 0, 0) * motionDataWc[i][j];
			std::cout << v.x() << " " << v.y() << " " << v.z() << " " << v.w() << std::endl;
			v = osg::Vec4(0, 1, 0, 0) * motionDataWc[i][j];
			std::cout << v.x() << " " << v.y() << " " << v.z() << " " << v.w() << std::endl;
			v = osg::Vec4(0, 0, 1, 0) * motionDataWc[i][j];
			std::cout << v.x() << " " << v.y() << " " << v.z() << " " << v.w() << std::endl;
			v = osg::Vec4(0, 0, 0, 1) * motionDataWc[i][j];
			std::cout << v.x() << " " << v.y() << " " << v.z() << " " << v.w() << std::endl << std::endl;
		}
	}
	 */
}

void BVH::recordWorldCoordinates(osg::ref_ptr<Joint> joint, size_t frame)
{
	Joint* parent = NULL;
	if(joint->getNumParents()){
	    parent = dynamic_cast<Joint*>(joint->getParent(0));
	}
	else{
    	//std::cout << "No parent\n";
	}
    osg::Matrix oMatrix;
    oMatrix.makeIdentity();
    joint->computeLocalToWorldMatrix(oMatrix, NULL);

    if(parent) {
        osg::Matrix pMatrix = motionDataWc[frame][parent->getDataIndex()];
        oMatrix = oMatrix * pMatrix;
    }

	if (&oMatrix != NULL)
	{
		this->motionDataWc[frame].push_back(oMatrix);
	}

	for (size_t i = 1; i < joint->getNumChildren(); ++i)
	{
		if (dynamic_cast<Joint*>(joint->getChild(i)))
		{
			recordWorldCoordinates(dynamic_cast<Joint*>(joint->getChild(i)), frame);
		}
	}
}

osg::Vec3f BVH::interpolate(const osg::Vec3f& l, const osg::Vec3f& r, float t, BVH::InterpolationType type)
{
	// Used axes and radian scaling factor
	static osg::Vec3d axisX = osg::Vec3d(1,0,0);
	static osg::Vec3d axisY = osg::Vec3d(0,1,0);
	static osg::Vec3d axisZ = osg::Vec3d(0,0,1);
	static float torad = 0.0174532925;

	switch (this->interpolationMethod)
	{
		case None: // No interpolation - show the previous frame
		{
			return l;
			break;
		}
		case Lerp: // Lerp interpolates only position
		{
			if (type == Translation)
				return lerp(l, r, t);
			else
				return l;
			break;
		}
		case Slerp: // TODO: fix this with a more generic interpolate function
		{
			if (type == Translation)
			{
				return lerp(l, r, t);
			}
			else
			{
				osg::Quat lrot( l[1] * torad, axisY, l[0] * torad, axisX, l[2] * torad, axisZ );
				osg::Quat rrot( r[1] * torad, axisY, r[0] * torad, axisX, r[2] * torad, axisZ );
				osg::Quat result;
				result.slerp(t, lrot, rrot);
				return result.asVec3();
			}
			break;
		}
		case Euler: // Euler lerp interpolates both position and rotation
		{
			return lerp(l, r, t);
			break;
		}
	}

	std::cout << "Error: invalid interpolation method" << std::endl;
	return osg::Vec3f(0.0f, 0.0f, 0.0f);
}

void BVH::update(float dt)
{
	//this->lerpPoints->clear();
	//this->lerpColors->clear();

	// Keep track of time
	currentTime += dt;

	// Calculate the next and previous frames' indices
	// in order to interpolate, the condition exists in order to avoid overflow
	// in frames, TODO: FIX THIS
	float frame = currentTime / frameTime;
	if (frame >= this->frames-1)
	{
		while (frame >= this->frames-1)
			frame -= (this->frames-1);
	    currentTime = 0;
	}

	size_t previous = size_t(frame);
	size_t next = (previous+1)%this->frames;
	float t = frame - previous;


	const std::vector<float>& framedata_prev = motionData[previous];
	const std::vector<float>& framedata_next = motionData[next];

	//std::cout << previous << " " << next << std::endl;

	osg::ref_ptr<Joint> root = hierarchy->getRoot();

	if (interpolationMethod != Lerp)
	{
		osg::Vec3f position = interpolate(osg::Vec3f(framedata_prev[0], framedata_prev[1], framedata_prev[2]),
										  osg::Vec3f(framedata_next[0], framedata_next[1], framedata_next[2]), t, Translation);
		root->setPosition(position);
	}

	// Used axes and deg to radian scaling factor
	static osg::Vec3f axisX = osg::Vec3d(1,0,0);
	static osg::Vec3f axisY = osg::Vec3d(0,1,0);
	static osg::Vec3f axisZ = osg::Vec3d(0,0,1);
	static float torad = 0.0174532925;

	if (interpolationMethod == Slerp)
	{
		osg::Quat lrot( framedata_prev[5] * torad, axisY,
						framedata_prev[4] * torad, axisX,
						framedata_prev[3] * torad, axisZ );
		osg::Quat rrot( framedata_next[5] * torad, axisY,
						framedata_next[4] * torad, axisX,
						framedata_next[3] * torad, axisZ );
		osg::Quat result;
		result.slerp(t, lrot, rrot);
		root->setAttitude(result);
	}
	else if (interpolationMethod == Lerp)
	{
		osg::Matrixf transform = lerp(motionDataWc[previous][0], motionDataWc[previous][0], t);
		root->interpolated = transform;
		root->interpolatedInverse = osg::Matrixf::inverse(transform);

		//transform.getRotate();
		osg::Vec3f translate;
		osg::Vec3f sc;
		osg::Quat rotation, so;
		transform.decompose(translate, rotation, sc, so);

		rotation *= so;
		double rotY, rotX, rotZ;
		rotation.getRotate(rotY, axisY);
		rotation.getRotate(rotX, axisX);
		rotation.getRotate(rotZ, axisZ);

		root->setPosition(translate);
		root->setAttitude(osg::Quat(rotY*torad, axisY, rotX*torad, axisX, rotZ*torad, axisZ));
		//root->setScale(sc);

		//this->lerpPoints->push_back(transform.getTrans());
		//this->lerpColors->push_back(osg::Vec4f(1.0f, 0.0, 0.0f, 1.0f));
	}
	else
	{
		// The rotations are given to the interpolate function in the YXZ order
		osg::Vec3f rotation = interpolate(osg::Vec3f(framedata_prev[5], framedata_prev[4], framedata_prev[3]),
						       	   	  	  osg::Vec3f(framedata_next[5], framedata_next[4], framedata_next[3]), t, Rotation);

		root->setAttitude( osg::Quat(rotation[0] * torad, axisY,
									 rotation[1] * torad, axisX,
									 rotation[2] * torad, axisZ) );
	}

	for (size_t i = 1; i < root->getNumChildren(); ++i)
	{
		if (dynamic_cast<Joint*>(root->getChild(i)))
		{
			updateRecurse(dynamic_cast<Joint*>(root->getChild(i)), previous, next, t);
		}
	}

	//this->lerpGeom->removePrimitiveSet(0);
	//this->lerpGeom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINE_STRIP, 0, this->lerpPoints->size()));
}

void BVH::updateRecurse(osg::ref_ptr<Joint> joint, size_t prevFrameIdx, size_t nextFrameIdx, float t)
{
	// Used axes and radian scaling factor
	static osg::Vec3f axisX = osg::Vec3d(1,0,0);
	static osg::Vec3f axisY = osg::Vec3d(0,1,0);
	static osg::Vec3f axisZ = osg::Vec3d(0,0,1);
	static float torad = 0.0174532925;

	// Parent
	osg::ref_ptr<Joint> parent = dynamic_cast<Joint*>(joint->getParent(0));

	const std::vector<float>& framedata_prev = motionData[prevFrameIdx];
	const std::vector<float>& framedata_next = motionData[nextFrameIdx];

	size_t offset = (joint->getDataIndex()-1) * 3;

	if (interpolationMethod == Slerp)
	{
		osg::Quat lrot( framedata_prev[6+offset+2] * torad, axisY,
						framedata_prev[6+offset+1] * torad, axisX,
						framedata_prev[6+offset] * torad, axisZ );
		osg::Quat rrot( framedata_next[6+offset+2] * torad, axisY,
						framedata_next[6+offset+1] * torad, axisX,
						framedata_next[6+offset] * torad, axisZ );
		osg::Quat result;
		result.slerp(t, lrot, rrot);
		joint->setAttitude(result);
	}
	else if (interpolationMethod == Lerp)
	{
		osg::Matrixf transform = lerp(motionDataWc[prevFrameIdx][joint->getDataIndex()], motionDataWc[nextFrameIdx][joint->getDataIndex()], t);
		joint->interpolated = transform;
		joint->interpolatedInverse = osg::Matrixf::inverse(transform);
		osg::Matrixf local = parent->interpolatedInverse * joint->interpolated;

		osg::Vec3f translate, scale;
		osg::Quat rotation, so;
		local.decompose(translate, rotation, scale, so);

		rotation = local.getRotate();
		double rotY, rotX, rotZ;
		rotation.getRotate(rotY, axisY);
		rotation.getRotate(rotX, axisX);
		rotation.getRotate(rotZ, axisZ);

		joint->setAttitude(osg::Quat(rotY*torad, axisY, rotX*torad, axisX, rotZ*torad, axisZ));
		joint->setPosition(local.getTrans());
	}
	else
	{
		// The rotations are given to the interpolate function in the YXZ order
		osg::Vec3f rotation = interpolate(osg::Vec3f(framedata_prev[6+offset+2], framedata_prev[6+offset+1], framedata_prev[6+offset]),
										  osg::Vec3f(framedata_next[6+offset+2], framedata_next[6+offset+1], framedata_next[6+offset]), t, Rotation);

		joint->setAttitude( osg::Quat(rotation[0] * torad, axisY,
									  rotation[1] * torad, axisX,
									  rotation[2] * torad, axisZ) );
	}


	// Visualization of trajectories: CRUFTY :XXXXXXX HAJOTKAA SIIHEN
	// Kun luette tätä koodia minä -> http://www.youtube.com/watch?v=wl_auj1ybVA
	if (prevFrameIdx == 0)
		trajectories[joint->getDataIndex()].clear();

	if (joint->getNumChildren() == 0)
	{
		osg::ref_ptr<osg::Sphere> sphere = new osg::Sphere( osg::Vec3(0.0f,0.0f,0.0f), 1.0f);
		osg::ref_ptr<osg::ShapeDrawable> sphereDrawable = new osg::ShapeDrawable(sphere);
		sphereDrawable->setColor(osg::Vec4f(0.0f, 1.0f, 0.0f, 1.0f));
		osg::ref_ptr<osg::Geode> sphereGeode = new osg::Geode();
		sphereGeode->addDrawable(sphereDrawable);
		osg::ref_ptr<osg::PositionAttitudeTransform> pt = new osg::PositionAttitudeTransform();
		pt->addChild(sphereGeode);

		osg::Matrix* mat;
		mat = getWorldCoords(joint);
		pt->setPosition(mat->getTrans());
		pt->setAttitude(mat->getRotate());

		if (trajectories[joint->getDataIndex()].size() < 60)
		{
			trajectories[joint->getDataIndex()].push_back(pt);
		}
		else
		{
			trajectories[joint->getDataIndex()].pop_front();
			trajectories[joint->getDataIndex()].push_back(pt);
		}
	}




	for (size_t i = 1; i < joint->getNumChildren(); ++i)
	{
		if (dynamic_cast<Joint*>(joint->getChild(i)))
		{
			updateRecurse(dynamic_cast<Joint*>(joint->getChild(i)), prevFrameIdx, nextFrameIdx, t);
		}
	}
}
