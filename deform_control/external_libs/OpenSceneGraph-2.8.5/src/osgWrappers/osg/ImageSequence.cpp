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

#include <osg/CopyOp>
#include <osg/Image>
#include <osg/ImageSequence>
#include <osg/NodeVisitor>
#include <osg/Object>
#include <osg/StateAttribute>

// Must undefine IN and OUT macros defined in Windows headers
#ifdef IN
#undef IN
#endif
#ifdef OUT
#undef OUT
#endif

TYPE_NAME_ALIAS(std::vector< osg::ref_ptr< osg::Image > >, osg::ImageSequence::Images)

TYPE_NAME_ALIAS(std::vector< std::string >, osg::ImageSequence::FileNames)

BEGIN_ENUM_REFLECTOR(osg::ImageSequence::Mode)
	I_DeclaringFile("osg/ImageSequence");
	I_EnumLabel(osg::ImageSequence::PRE_LOAD_ALL_IMAGES);
	I_EnumLabel(osg::ImageSequence::PAGE_AND_RETAIN_IMAGES);
	I_EnumLabel(osg::ImageSequence::PAGE_AND_DISCARD_USED_IMAGES);
END_REFLECTOR

BEGIN_OBJECT_REFLECTOR(osg::ImageSequence)
	I_DeclaringFile("osg/ImageSequence");
	I_BaseType(osg::ImageStream);
	I_Constructor0(____ImageSequence,
	               "",
	               "");
	I_ConstructorWithDefaults2(IN, const osg::ImageSequence &, ImageSequence, , IN, const osg::CopyOp &, copyop, osg::CopyOp::SHALLOW_COPY,
	                           ____ImageSequence__C5_ImageSequence_R1__C5_CopyOp_R1,
	                           "Copy constructor using CopyOp to manage deep vs shallow copy. ",
	                           "");
	I_Method0(osg::Object *, cloneType,
	          Properties::VIRTUAL,
	          __Object_P1__cloneType,
	          "Clone the type of an object, with Object* return type. ",
	          "Must be defined by derived classes. ");
	I_Method1(osg::Object *, clone, IN, const osg::CopyOp &, copyop,
	          Properties::VIRTUAL,
	          __Object_P1__clone__C5_CopyOp_R1,
	          "Clone an object, with Object* return type. ",
	          "Must be defined by derived classes. ");
	I_Method1(bool, isSameKindAs, IN, const osg::Object *, obj,
	          Properties::VIRTUAL,
	          __bool__isSameKindAs__C5_Object_P1,
	          "",
	          "");
	I_Method0(const char *, libraryName,
	          Properties::VIRTUAL,
	          __C5_char_P1__libraryName,
	          "return the name of the object's library. ",
	          "Must be defined by derived classes. The OpenSceneGraph convention is that the namespace of a library is the same as the library name. ");
	I_Method0(const char *, className,
	          Properties::VIRTUAL,
	          __C5_char_P1__className,
	          "return the name of the object's class type. ",
	          "Must be defined by derived classes. ");
	I_Method1(int, compare, IN, const osg::Image &, rhs,
	          Properties::VIRTUAL,
	          __int__compare__C5_Image_R1,
	          "Return -1 if *this < *rhs, 0 if *this==*rhs, 1 if *this>*rhs. ",
	          "");
	I_Method1(void, setReferenceTime, IN, double, t,
	          Properties::VIRTUAL,
	          __void__setReferenceTime__double,
	          "",
	          "");
	I_Method0(double, getReferenceTime,
	          Properties::VIRTUAL,
	          __double__getReferenceTime,
	          "",
	          "");
	I_Method1(void, setTimeMultiplier, IN, double, tm,
	          Properties::VIRTUAL,
	          __void__setTimeMultiplier__double,
	          "",
	          "");
	I_Method0(double, getTimeMultiplier,
	          Properties::VIRTUAL,
	          __double__getTimeMultiplier,
	          "",
	          "");
	I_Method1(void, seek, IN, double, time,
	          Properties::VIRTUAL,
	          __void__seek__double,
	          "",
	          "");
	I_Method0(void, play,
	          Properties::VIRTUAL,
	          __void__play,
	          "",
	          "");
	I_Method0(void, pause,
	          Properties::VIRTUAL,
	          __void__pause,
	          "",
	          "");
	I_Method0(void, rewind,
	          Properties::VIRTUAL,
	          __void__rewind,
	          "",
	          "");
	I_Method1(void, setMode, IN, osg::ImageSequence::Mode, mode,
	          Properties::NON_VIRTUAL,
	          __void__setMode__Mode,
	          "",
	          "");
	I_Method0(osg::ImageSequence::Mode, getMode,
	          Properties::NON_VIRTUAL,
	          __Mode__getMode,
	          "",
	          "");
	I_Method1(void, setLength, IN, double, length,
	          Properties::NON_VIRTUAL,
	          __void__setLength__double,
	          "",
	          "");
	I_Method0(double, getLength,
	          Properties::VIRTUAL,
	          __double__getLength,
	          "",
	          "");
	I_Method1(void, addImageFile, IN, const std::string &, fileName,
	          Properties::NON_VIRTUAL,
	          __void__addImageFile__C5_std_string_R1,
	          "",
	          "");
	I_Method2(void, setImageFile, IN, unsigned int, pos, IN, const std::string &, fileName,
	          Properties::NON_VIRTUAL,
	          __void__setImageFile__unsigned_int__C5_std_string_R1,
	          "",
	          "");
	I_Method1(std::string, getImageFile, IN, unsigned int, pos,
	          Properties::NON_VIRTUAL,
	          __std_string__getImageFile__unsigned_int,
	          "",
	          "");
	I_Method0(unsigned int, getNumImageFiles,
	          Properties::NON_VIRTUAL,
	          __unsigned_int__getNumImageFiles,
	          "",
	          "");
	I_Method0(osg::ImageSequence::FileNames &, getFileNames,
	          Properties::NON_VIRTUAL,
	          __FileNames_R1__getFileNames,
	          "",
	          "");
	I_Method0(const osg::ImageSequence::FileNames &, getFileNames,
	          Properties::NON_VIRTUAL,
	          __C5_FileNames_R1__getFileNames,
	          "",
	          "");
	I_Method1(void, addImage, IN, osg::Image *, image,
	          Properties::NON_VIRTUAL,
	          __void__addImage__osg_Image_P1,
	          "",
	          "");
	I_MethodWithDefaults9(void, setImage, IN, int, s, , IN, int, t, , IN, int, r, , IN, GLint, internalTextureformat, , IN, GLenum, pixelFormat, , IN, GLenum, type, , IN, unsigned char *, data, , IN, osg::Image::AllocationMode, mode, , IN, int, packing, 1,
	                      Properties::NON_VIRTUAL,
	                      __void__setImage__int__int__int__GLint__GLenum__GLenum__unsigned_char_P1__AllocationMode__int,
	                      "",
	                      "");
	I_Method2(void, setImage, IN, unsigned int, pos, IN, osg::Image *, image,
	          Properties::NON_VIRTUAL,
	          __void__setImage__unsigned_int__osg_Image_P1,
	          "",
	          "");
	I_Method1(osg::Image *, getImage, IN, unsigned int, pos,
	          Properties::NON_VIRTUAL,
	          __Image_P1__getImage__unsigned_int,
	          "",
	          "");
	I_Method1(const osg::Image *, getImage, IN, unsigned int, pos,
	          Properties::NON_VIRTUAL,
	          __C5_Image_P1__getImage__unsigned_int,
	          "",
	          "");
	I_Method0(unsigned int, getNumImages,
	          Properties::NON_VIRTUAL,
	          __unsigned_int__getNumImages,
	          "",
	          "");
	I_Method0(osg::ImageSequence::Images &, getImages,
	          Properties::NON_VIRTUAL,
	          __Images_R1__getImages,
	          "",
	          "");
	I_Method0(const osg::ImageSequence::Images &, getImages,
	          Properties::NON_VIRTUAL,
	          __C5_Images_R1__getImages,
	          "",
	          "");
	I_Method1(void, update, IN, osg::NodeVisitor *, nv,
	          Properties::VIRTUAL,
	          __void__update__NodeVisitor_P1,
	          "",
	          "");
	I_ProtectedMethod0(void, applyLoopingMode,
	                   Properties::VIRTUAL,
	                   Properties::NON_CONST,
	                   __void__applyLoopingMode,
	                   "",
	                   "");
	I_ProtectedMethod1(void, setImageToChild, IN, const osg::Image *, image,
	                   Properties::NON_VIRTUAL,
	                   Properties::NON_CONST,
	                   __void__setImageToChild__C5_osg_Image_P1,
	                   "",
	                   "");
	I_ProtectedMethod0(void, computeTimePerImage,
	                   Properties::NON_VIRTUAL,
	                   Properties::NON_CONST,
	                   __void__computeTimePerImage,
	                   "",
	                   "");
	I_ProtectedMethod1(int, imageIndex, IN, double, time,
	                   Properties::NON_VIRTUAL,
	                   Properties::NON_CONST,
	                   __int__imageIndex__double,
	                   "",
	                   "");
	I_SimpleProperty(osg::ImageSequence::FileNames &, FileNames, 
	                 __FileNames_R1__getFileNames, 
	                 0);
	I_ArrayProperty(osg::Image *, Image, 
	                __Image_P1__getImage__unsigned_int, 
	                __void__setImage__unsigned_int__osg_Image_P1, 
	                __unsigned_int__getNumImages, 
	                __void__addImage__osg_Image_P1, 
	                0, 
	                0);
	I_ArrayProperty(std::string, ImageFile, 
	                __std_string__getImageFile__unsigned_int, 
	                __void__setImageFile__unsigned_int__C5_std_string_R1, 
	                __unsigned_int__getNumImageFiles, 
	                0, 
	                0, 
	                0);
	I_SimpleProperty(osg::ImageSequence::Images &, Images, 
	                 __Images_R1__getImages, 
	                 0);
	I_SimpleProperty(double, Length, 
	                 __double__getLength, 
	                 __void__setLength__double);
	I_SimpleProperty(osg::ImageSequence::Mode, Mode, 
	                 __Mode__getMode, 
	                 __void__setMode__Mode);
	I_SimpleProperty(double, ReferenceTime, 
	                 __double__getReferenceTime, 
	                 __void__setReferenceTime__double);
	I_SimpleProperty(double, TimeMultiplier, 
	                 __double__getTimeMultiplier, 
	                 __void__setTimeMultiplier__double);
END_REFLECTOR

BEGIN_OBJECT_REFLECTOR(osg::ImageSequence::UpdateCallback)
	I_DeclaringFile("osg/ImageSequence");
	I_BaseType(osg::StateAttributeCallback);
	I_Constructor0(____UpdateCallback,
	               "",
	               "");
END_REFLECTOR

BEGIN_VALUE_REFLECTOR(osg::ref_ptr< osg::Image >)
	I_DeclaringFile("osg/ref_ptr");
	I_Constructor0(____ref_ptr,
	               "",
	               "");
	I_Constructor1(IN, osg::Image *, ptr,
	               Properties::NON_EXPLICIT,
	               ____ref_ptr__T_P1,
	               "",
	               "");
	I_Constructor1(IN, const osg::ref_ptr< osg::Image > &, rp,
	               Properties::NON_EXPLICIT,
	               ____ref_ptr__C5_ref_ptr_R1,
	               "",
	               "");
	I_Method0(osg::Image *, get,
	          Properties::NON_VIRTUAL,
	          __T_P1__get,
	          "",
	          "");
	I_Method0(bool, valid,
	          Properties::NON_VIRTUAL,
	          __bool__valid,
	          "",
	          "");
	I_Method0(osg::Image *, release,
	          Properties::NON_VIRTUAL,
	          __T_P1__release,
	          "",
	          "");
	I_Method1(void, swap, IN, osg::ref_ptr< osg::Image > &, rp,
	          Properties::NON_VIRTUAL,
	          __void__swap__ref_ptr_R1,
	          "",
	          "");
	I_SimpleProperty(osg::Image *, , 
	                 __T_P1__get, 
	                 0);
END_REFLECTOR

STD_VECTOR_REFLECTOR(std::vector< osg::ref_ptr< osg::Image > >)

STD_VECTOR_REFLECTOR(std::vector< std::string >)

