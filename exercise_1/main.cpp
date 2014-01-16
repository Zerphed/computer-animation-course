#include <iostream>

#include <osg/MatrixTransform>
#include <osg/ShapeDrawable>
#include <osgViewer/Viewer>
#include <osgViewer/ViewerEventHandlers>
#include <osgGA/TrackballManipulator>
#include <osg/Timer>
#include <osgWidget/WindowManager>
#include <osg/NodeCallback>

#include "parser.hpp"

const unsigned int WINDOW_WIDTH  = 800;
const unsigned int WINDOW_HEIGHT = 600;
const unsigned int MASK_2D       = 0xF0000000;
const unsigned int MASK_3D       = 0x0F000000;

osg::ref_ptr<BVH> bvh;
osg::ref_ptr<osg::Group> scene;
osg::ref_ptr<osg::Group> trajectories;

class JointNodeCallback : public osg::NodeCallback
{
public:
	JointNodeCallback() : t0(osg::Timer::instance()->tick()), t1(osg::Timer::instance()->tick()), dt(0)
	{
	}

	virtual void operator()(osg::Node* node, osg::NodeVisitor* nv)
	{
		osg::ref_ptr<Joint> joint = dynamic_cast<Joint*> (node);
		if (joint)
		{
			t1 = osg::Timer::instance()->tick();
			dt = osg::Timer::instance()->delta_s (t0, t1);
			t0 = osg::Timer::instance()->tick();
			bvh->update(dt);

			while (trajectories->removeChild(0, 1));

			for (size_t i = 0; i < bvh->trajectories.size(); ++i)
			{
				for (size_t j = 0; j < bvh->trajectories[i].size(); ++j)
				{
					trajectories->addChild(bvh->trajectories[i][j]);
				}
			}

		}
		traverse(node, nv);
	}

private:
	osg::Timer_t t0, t1;
	float dt;
};


struct ColorLabel: public osgWidget::Label
{
    ColorLabel(const char* label) : osgWidget::Label(label, "")
    {
        setFont("fonts/VeraMono.ttf");
        setFontSize(14);
        setFontColor(1.0f, 1.0f, 1.0f, 1.0f);

        setColor(0.3f, 0.3f, 0.3f, 1.0f);
        setPadding(2.0f);
        setCanFill(true);

        addSize(150.0f, 25.0f);

        setLabel(label);
        setEventMask(osgWidget::EVENT_MOUSE_PUSH | osgWidget::EVENT_MASK_MOUSE_MOVE);
    }

    bool mousePush(double, double, const osgWidget::WindowManager*)
    {
        osg::ref_ptr<osgWidget::Table> p = dynamic_cast<osgWidget::Table*>(_parent);

        if(!p)
        	return false;
        p->hide();

        const std::string& name = getName();

        if (!name.compare("None"))
        {
        	std::cout << "Changing interpolation method to: None" << std::endl;
        	bvh->toggleInterpolationMethod((BVH::InterpolationMethod)0);
        	//scene->replaceChild(bvh->getLerpGraphics(), bvh->getRoot());
        }
        else if (!name.compare("Lerp"))
        {
        	std::cout << "Changing interpolation method to: Lerp" << std::endl;
        	bvh->toggleInterpolationMethod((BVH::InterpolationMethod)1);
        	//scene->replaceChild(bvh->getRoot(), bvh->getLerpGraphics());
        }
        else if (!name.compare("Slerp"))
        {
        	std::cout << "Changing interpolation method to: Slerp" << std::endl;
        	bvh->toggleInterpolationMethod((BVH::InterpolationMethod)2);
        	scene->replaceChild(bvh->getLerpGraphics(), bvh->getRoot());
        }
        else if (!name.compare("Euler"))
        {
        	std::cout << "Changing interpolation method to: Euler" << std::endl;
        	bvh->toggleInterpolationMethod((BVH::InterpolationMethod)3);
        	scene->replaceChild(bvh->getLerpGraphics(), bvh->getRoot());
        }
        else
        	std::cout << "Error: unknown interpolation option" << std::endl;

        return true;
    }

    bool mouseEnter(double, double, const osgWidget::WindowManager*)
    {
        setColor(0.9f, 0.6f, 0.1f, 1.0f);
        return true;
    }

    bool mouseLeave(double, double, const osgWidget::WindowManager*)
    {
        setColor(0.3f, 0.3f, 0.3f, 1.0f);
        return true;
    }
};

class ColorLabelMenu: public ColorLabel
{
public:
    ColorLabelMenu(const char* label) : ColorLabel(label)
    {
        _window = new osgWidget::Table(std::string("Menu_") + label, 6, 5);

        _window->addWidget(new ColorLabel("None"), 0, 0);
        _window->addWidget(new ColorLabel("Lerp"), 0, 1);
        _window->addWidget(new ColorLabel("Slerp"), 0, 2);
        _window->addWidget(new ColorLabel("Euler"), 0, 3);

        _window->resize();
    }

    void managed(osgWidget::WindowManager* wm)
    {
        osgWidget::Label::managed(wm);
        wm->addChild(_window.get());
        _window->hide();
    }

    void positioned()
    {
        osgWidget::Label::positioned();
        _window->setOrigin(_parent->getX(), _parent->getY() +  _parent->getHeight());
    }

    bool mousePush(double, double, const osgWidget::WindowManager*)
    {
        if(!_window->isVisible())
        	_window->show();
        else
        	_window->hide();
        return true;
    }

private:
    osg::ref_ptr<osgWidget::Table> _window;
};


int main(int argc, char** argv)
{
	// Sanity check for the command line arguments
	if (argc < 2)
	{
		std::cout << "Invalid number of arguments, use: " << argv[0] << " mocapfile.bvh" << std::endl;
		return EXIT_FAILURE;
	}

	// Setup the GUI components
	osgViewer::Viewer viewer;
	viewer.setCameraManipulator(new osgGA::TrackballManipulator);

	osgWidget::WindowManager* wm = new osgWidget::WindowManager(&viewer, WINDOW_WIDTH, WINDOW_HEIGHT, MASK_2D);

	osgWidget::Window* menu = new osgWidget::Box("menu", osgWidget::Box::HORIZONTAL);
	menu->addWidget(new ColorLabelMenu("Interpolation method"));
	menu->getBackground()->setColor(1.0f, 1.0f, 1.0f, 1.0f);
	menu->setPosition(15.0f, 15.0f, 0.0f);

	wm->addChild(menu);

	// Declare a group to act as a root node of a scene, setup scene data
	bvh = BVHParser::parse(argv[1]);
	bvh->getRoot()->setUpdateCallback(new JointNodeCallback);

	scene = new osg::Group();
	trajectories = new osg::Group();
	trajectories->setDataVariance(osg::Object::DYNAMIC);

	scene->addChild(bvh->getRoot());
	scene->addChild(trajectories);

	viewer.setSceneData( scene );
	viewer.home();

	return osgWidget::createExample(viewer, wm, scene);
}
