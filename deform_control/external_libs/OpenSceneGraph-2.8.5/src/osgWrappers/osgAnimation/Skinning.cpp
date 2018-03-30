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

#include <osg/Matrix>
#include <osgAnimation/Skinning>
#include <osgAnimation/VertexInfluence>

// Must undefine IN and OUT macros defined in Windows headers
#ifdef IN
#undef IN
#endif
#ifdef OUT
#undef OUT
#endif

TYPE_NAME_ALIAS(osg::Matrix, osgAnimation::TransformVertexFunctor::MatrixType)

TYPE_NAME_ALIAS(osgAnimation::Bone, osgAnimation::TransformVertexFunctor::BoneType)

TYPE_NAME_ALIAS(osgAnimation::Bone::BoneMap, osgAnimation::TransformVertexFunctor::BoneMap)

TYPE_NAME_ALIAS(std::vector< osgAnimation::TransformVertexFunctor::BoneWeight >, osgAnimation::TransformVertexFunctor::BoneWeightList)

TYPE_NAME_ALIAS(std::vector< int >, osgAnimation::TransformVertexFunctor::VertexList)

BEGIN_VALUE_REFLECTOR(osgAnimation::TransformVertexFunctor)
	I_DeclaringFile("osgAnimation/Skinning");
	I_Constructor0(____TransformVertexFunctor,
	               "",
	               "");
	I_Method2(void, init, IN, const osgAnimation::TransformVertexFunctor::BoneMap &, map, IN, const osgAnimation::VertexInfluenceSet::UniqVertexSetToBoneSetList &, influence,
	          Properties::NON_VIRTUAL,
	          __void__init__C5_BoneMap_R1__C5_osgAnimation_VertexInfluenceSet_UniqVertexSetToBoneSetList_R1,
	          "",
	          "");
END_REFLECTOR

BEGIN_VALUE_REFLECTOR(osgAnimation::TransformVertexFunctor::BoneWeight)
	I_DeclaringFile("osgAnimation/Skinning");
	I_Constructor2(IN, osgAnimation::TransformVertexFunctor::BoneType *, bone, IN, float, weight,
	               ____BoneWeight__BoneType_P1__float,
	               "",
	               "");
	I_Method0(const osgAnimation::TransformVertexFunctor::BoneType *, getBone,
	          Properties::NON_VIRTUAL,
	          __C5_BoneType_P1__getBone,
	          "",
	          "");
	I_Method0(float, getWeight,
	          Properties::NON_VIRTUAL,
	          __float__getWeight,
	          "",
	          "");
	I_Method1(void, setWeight, IN, float, w,
	          Properties::NON_VIRTUAL,
	          __void__setWeight__float,
	          "",
	          "");
	I_SimpleProperty(const osgAnimation::TransformVertexFunctor::BoneType *, Bone, 
	                 __C5_BoneType_P1__getBone, 
	                 0);
	I_SimpleProperty(float, Weight, 
	                 __float__getWeight, 
	                 __void__setWeight__float);
END_REFLECTOR

BEGIN_VALUE_REFLECTOR(osgAnimation::TransformVertexFunctor::UniqBoneSetVertexSet)
	I_DeclaringFile("osgAnimation/Skinning");
	I_Constructor0(____UniqBoneSetVertexSet,
	               "",
	               "");
	I_Method0(osgAnimation::TransformVertexFunctor::BoneWeightList &, getBones,
	          Properties::NON_VIRTUAL,
	          __BoneWeightList_R1__getBones,
	          "",
	          "");
	I_Method0(osgAnimation::TransformVertexFunctor::VertexList &, getVertexes,
	          Properties::NON_VIRTUAL,
	          __VertexList_R1__getVertexes,
	          "",
	          "");
	I_Method0(void, resetMatrix,
	          Properties::NON_VIRTUAL,
	          __void__resetMatrix,
	          "",
	          "");
	I_Method3(void, accummulateMatrix, IN, const osg::Matrix &, invBindMatrix, IN, const osg::Matrix &, matrix, IN, osg::Matrix::value_type, weight,
	          Properties::NON_VIRTUAL,
	          __void__accummulateMatrix__C5_osg_Matrix_R1__C5_osg_Matrix_R1__osg_Matrix_value_type,
	          "",
	          "");
	I_Method0(void, computeMatrixForVertexSet,
	          Properties::NON_VIRTUAL,
	          __void__computeMatrixForVertexSet,
	          "",
	          "");
	I_Method0(const osgAnimation::TransformVertexFunctor::MatrixType &, getMatrix,
	          Properties::NON_VIRTUAL,
	          __C5_MatrixType_R1__getMatrix,
	          "",
	          "");
	I_SimpleProperty(osgAnimation::TransformVertexFunctor::BoneWeightList &, Bones, 
	                 __BoneWeightList_R1__getBones, 
	                 0);
	I_SimpleProperty(const osgAnimation::TransformVertexFunctor::MatrixType &, Matrix, 
	                 __C5_MatrixType_R1__getMatrix, 
	                 0);
	I_SimpleProperty(osgAnimation::TransformVertexFunctor::VertexList &, Vertexes, 
	                 __VertexList_R1__getVertexes, 
	                 0);
END_REFLECTOR

STD_VECTOR_REFLECTOR(std::vector< osgAnimation::TransformVertexFunctor::BoneWeight >)

