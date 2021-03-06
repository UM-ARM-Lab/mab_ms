// ***************************************************************************
//
//   Generated automatically by genwrapper.
//   Please DO NOT EDIT this file!
//
// ***************************************************************************

#include <osgIntrospection/ReflectionMacros>
#include <osgIntrospection/TypedMethodInfo>
#include <osgIntrospection/StaticMethodInfo>
#include <osgIntrospection/Attributes>

#include <osg/Drawable>
#include <osg/Geode>
#include <osg/GraphicsContext>
#include <osg/Node>
#include <osg/RenderInfo>
#include <osg/State>
#include <osg/StateSet>
#include <osgUtil/GLObjectsVisitor>

// Must undefine IN and OUT macros defined in Windows headers
#ifdef IN
#undef IN
#endif
#ifdef OUT
#undef OUT
#endif

BEGIN_OBJECT_REFLECTOR(osgUtil::GLObjectsOperation)
	I_DeclaringFile("osgUtil/GLObjectsVisitor");
	I_BaseType(osg::GraphicsOperation);
	I_ConstructorWithDefaults1(IN, osgUtil::GLObjectsVisitor::Mode, mode, osgUtil::GLObjectsVisitor::COMPILE_DISPLAY_LISTS|osgUtil::GLObjectsVisitor::COMPILE_STATE_ATTRIBUTES|osgUtil::GLObjectsVisitor::CHECK_BLACK_LISTED_MODES,
	                           Properties::NON_EXPLICIT,
	                           ____GLObjectsOperation__GLObjectsVisitor_Mode,
	                           "",
	                           "");
	I_ConstructorWithDefaults2(IN, osg::Node *, subgraph, , IN, osgUtil::GLObjectsVisitor::Mode, mode, osgUtil::GLObjectsVisitor::COMPILE_DISPLAY_LISTS|osgUtil::GLObjectsVisitor::COMPILE_STATE_ATTRIBUTES|osgUtil::GLObjectsVisitor::CHECK_BLACK_LISTED_MODES,
	                           ____GLObjectsOperation__osg_Node_P1__GLObjectsVisitor_Mode,
	                           "",
	                           "");
END_REFLECTOR

TYPE_NAME_ALIAS(unsigned int, osgUtil::GLObjectsVisitor::Mode)

BEGIN_ENUM_REFLECTOR(osgUtil::GLObjectsVisitor::ModeValues)
	I_DeclaringFile("osgUtil/GLObjectsVisitor");
	I_EnumLabel(osgUtil::GLObjectsVisitor::SWITCH_ON_DISPLAY_LISTS);
	I_EnumLabel(osgUtil::GLObjectsVisitor::SWITCH_OFF_DISPLAY_LISTS);
	I_EnumLabel(osgUtil::GLObjectsVisitor::COMPILE_DISPLAY_LISTS);
	I_EnumLabel(osgUtil::GLObjectsVisitor::COMPILE_STATE_ATTRIBUTES);
	I_EnumLabel(osgUtil::GLObjectsVisitor::RELEASE_DISPLAY_LISTS);
	I_EnumLabel(osgUtil::GLObjectsVisitor::RELEASE_STATE_ATTRIBUTES);
	I_EnumLabel(osgUtil::GLObjectsVisitor::SWITCH_ON_VERTEX_BUFFER_OBJECTS);
	I_EnumLabel(osgUtil::GLObjectsVisitor::SWITCH_OFF_VERTEX_BUFFER_OBJECTS);
	I_EnumLabel(osgUtil::GLObjectsVisitor::CHECK_BLACK_LISTED_MODES);
END_REFLECTOR

BEGIN_OBJECT_REFLECTOR(osgUtil::GLObjectsVisitor)
	I_DeclaringFile("osgUtil/GLObjectsVisitor");
	I_BaseType(osg::NodeVisitor);
	I_ConstructorWithDefaults1(IN, osgUtil::GLObjectsVisitor::Mode, mode, osgUtil::GLObjectsVisitor::COMPILE_DISPLAY_LISTS|osgUtil::GLObjectsVisitor::COMPILE_STATE_ATTRIBUTES|osgUtil::GLObjectsVisitor::CHECK_BLACK_LISTED_MODES,
	                           Properties::NON_EXPLICIT,
	                           ____GLObjectsVisitor__Mode,
	                           "Construct a GLObjectsVisitor to traverse all children, operating on node according to specified mode, such as to compile or release display list/texture objects etc. ",
	                           "Default mode is to compile GL objects. ");
	I_Method0(const char *, libraryName,
	          Properties::VIRTUAL,
	          __C5_char_P1__libraryName,
	          "return the library name/namespapce of the visitor's. ",
	          "Should be defined by derived classes. ");
	I_Method0(const char *, className,
	          Properties::VIRTUAL,
	          __C5_char_P1__className,
	          "return the name of the visitor's class type. ",
	          "Should be defined by derived classes. ");
	I_Method0(void, reset,
	          Properties::VIRTUAL,
	          __void__reset,
	          "Method to call to reset visitor. ",
	          "Useful if your visitor accumulates state during a traversal, and you plan to reuse the visitor. To flush that state for the next traversal: call reset() prior to each traversal. ");
	I_Method1(void, setMode, IN, osgUtil::GLObjectsVisitor::Mode, mode,
	          Properties::NON_VIRTUAL,
	          __void__setMode__Mode,
	          "Set the operational mode of what operations to do on the scene graph. ",
	          "");
	I_Method0(osgUtil::GLObjectsVisitor::Mode, getMode,
	          Properties::NON_VIRTUAL,
	          __Mode__getMode,
	          "Get the operational mode. ",
	          "");
	I_Method1(void, setState, IN, osg::State *, state,
	          Properties::NON_VIRTUAL,
	          __void__setState__osg_State_P1,
	          "Set the State to use during traversal. ",
	          "");
	I_Method0(osg::State *, getState,
	          Properties::NON_VIRTUAL,
	          __osg_State_P1__getState,
	          "",
	          "");
	I_Method1(void, setRenderInfo, IN, osg::RenderInfo &, renderInfo,
	          Properties::NON_VIRTUAL,
	          __void__setRenderInfo__osg_RenderInfo_R1,
	          "",
	          "");
	I_Method0(osg::RenderInfo &, getRenderInfo,
	          Properties::NON_VIRTUAL,
	          __osg_RenderInfo_R1__getRenderInfo,
	          "",
	          "");
	I_Method1(void, apply, IN, osg::Node &, node,
	          Properties::VIRTUAL,
	          __void__apply__osg_Node_R1,
	          "Simply traverse using standard NodeVisitor traverse method. ",
	          "");
	I_Method1(void, apply, IN, osg::Geode &, node,
	          Properties::VIRTUAL,
	          __void__apply__osg_Geode_R1,
	          "For each Geode visited set the display list usage according to the _displayListMode. ",
	          "");
	I_Method1(void, apply, IN, osg::Drawable &, drawable,
	          Properties::NON_VIRTUAL,
	          __void__apply__osg_Drawable_R1,
	          "",
	          "");
	I_Method1(void, apply, IN, osg::StateSet &, stateset,
	          Properties::NON_VIRTUAL,
	          __void__apply__osg_StateSet_R1,
	          "",
	          "");
	I_SimpleProperty(osgUtil::GLObjectsVisitor::Mode, Mode, 
	                 __Mode__getMode, 
	                 __void__setMode__Mode);
	I_SimpleProperty(osg::RenderInfo &, RenderInfo, 
	                 __osg_RenderInfo_R1__getRenderInfo, 
	                 __void__setRenderInfo__osg_RenderInfo_R1);
	I_SimpleProperty(osg::State *, State, 
	                 __osg_State_P1__getState, 
	                 __void__setState__osg_State_P1);
END_REFLECTOR

