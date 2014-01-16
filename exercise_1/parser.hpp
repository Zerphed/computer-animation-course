/*
 * parser.hpp
 *
 *  Created on: 3.10.2013
 *      Author: jhnissin
 */

#ifndef PARSER_HPP_
#define PARSER_HPP_

#include <string>
#include <vector>
#include <deque>

#include <osg/PositionAttitudeTransform>
#include <osg/Geode>
#include <osg/Vec3>
#include <osg/Matrix>
#include <osg/Geometry>


class Joint : virtual public osg::PositionAttitudeTransform
{
public:
	Joint(std::string name, Joint* parent, size_t dataIndex, size_t treeIndex) : osg::PositionAttitudeTransform(), name(name), dataIndex(dataIndex), treeIndex(treeIndex), numChannels(0)
	{

	}

	~Joint()
	{

	}

	// -------------- GETTERS AND SETTERS ----------------

	const std::string& getName()
	{
		return this->name;
	}

	void setName(const std::string& name)
	{
		this->name = name;
	}

	size_t getDataIndex()
	{
		return this->dataIndex;
	}

	void setDataIndex(size_t index)
	{
		this->dataIndex = index;
	}

	size_t getTreeIndex()
	{
		return this->treeIndex;
	}

	void setTreeIndex(size_t index)
	{
		this->treeIndex = index;
	}

	size_t getNumChannels()
	{
		return this->numChannels;
	}

	void setNumChannels(size_t numChannels)
	{
		this->numChannels = numChannels;
	}

	const std::vector<std::string>& getChannels()
	{
		return this->channels;
	}

	void setChannels(std::vector<std::string>& channels)
	{
		this->channels = channels;
	}

	// ------------- END GETTERS AND SETTERS -------------

	friend std::ostream& operator<< (std::ostream& out, Joint& joint);

	// Interpolation matrices in world coordinates for the lerp interpolation
	osg::Matrixf interpolatedInverse;
	osg::Matrixf interpolated;
	osg::Vec3f offset;

private:
	std::string name;
	size_t dataIndex;
	size_t treeIndex;

	size_t numChannels;
	std::vector<std::string> channels;
};


class Hierarchy : virtual public osg::Referenced
{
	public:
		Hierarchy(osg::ref_ptr<Joint> root) : osg::Referenced(), root(root)
		{

		}

		~Hierarchy()
		{

		}

		osg::ref_ptr<Joint> getRoot()
		{
			return this->root;
		}

	private:

		osg::ref_ptr<Joint> root;
};


class BVH : virtual public osg::Referenced
{
	public:
		enum InterpolationMethod { None = 0,
								   Lerp,
								   Slerp,
								   Euler };

		enum InterpolationType { Translation = 0,
								 Rotation };


		BVH(osg::ref_ptr<Hierarchy> hierarchy, std::vector<std::vector<float> > motiondata, size_t frames, float ftime);

		~BVH()
		{

		}

		void recordWorldCoordinates(osg::ref_ptr<Joint> joint, size_t frame);

		osg::ref_ptr<Joint> getRoot()
		{
			return this->hierarchy->getRoot();
		}

		osg::ref_ptr<osg::Geode> getLerpGraphics()
		{
			return this->lerpGeode;
		}

		void toggleInterpolationMethod(BVH::InterpolationMethod method)
		{
			this->interpolationMethod = method;
		}

		osg::Vec3f interpolate(const osg::Vec3f& l, const osg::Vec3f& r, float t, BVH::InterpolationType type);

		void update(float dt);

		void updateRecurse(osg::ref_ptr<Joint> joint, size_t prevFrameIdx, size_t nextFrameIdx, float t);

		std::vector<std::deque<osg::ref_ptr<osg::PositionAttitudeTransform> > > trajectories;

	private:

		osg::ref_ptr<Hierarchy> hierarchy;
		std::vector<std::vector<float> > motionData;
		std::vector<std::vector<osg::Matrix> > motionDataWc;
		size_t frames;
		float frameTime;
		float currentTime;
		BVH::InterpolationMethod interpolationMethod;

		osg::ref_ptr<osg::DrawElementsUInt> drawelements;
		osg::ref_ptr<osg::Geode> lerpGeode;
		osg::ref_ptr<osg::Geometry> lerpGeom;
		osg::ref_ptr<osg::Vec3Array> lerpPoints;
		osg::ref_ptr<osg::Vec4Array> lerpColors;
};


class BVHParser
{
	public:
		static osg::ref_ptr<BVH> parse(const std::string& filename);

	private:
		static osg::ref_ptr<Hierarchy> readHierarchy(const std::string& filename);
		static osg::ref_ptr<Joint> readJoint(std::ifstream& mocapfile, osg::ref_ptr<Joint> parent, const std::string& name, size_t dataIndex, size_t treeIndex);
		static osg::ref_ptr<BVH> readMotion(const std::string& filename, osg::ref_ptr<Hierarchy> hierarchy);

		static void drawJoint(osg::ref_ptr<Joint> joint);
		static void drawBone(osg::ref_ptr<Joint> parent, osg::ref_ptr<Joint> child);
};


#endif /* PARSER_HPP_ */
