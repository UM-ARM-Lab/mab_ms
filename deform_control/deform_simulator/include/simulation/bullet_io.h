#pragma once
// reading bullet vectors, and converting between them and vectors

#include <btBulletDynamicsCommon.h>
#include <vector>
#include <iostream>

std::ostream &operator<<(std::ostream &stream, const btVector3& v);

std::ostream &operator<<(std::ostream &stream, const btQuaternion& v);

std::ostream &operator<<(std::ostream &stream, const btTransform& v);

std::ostream &operator<<(std::ostream &stream, const std::vector<btVector3>& vs);

std::ostream &operator<<(std::ostream &stream, const btMatrix3x3& m);

std::istream &operator>>(std::istream &stream, btVector3& v);

std::istream &operator>>(std::istream &stream, btQuaternion& v);

std::istream &operator>>(std::istream &stream, btTransform& v);

std::istream &operator>>(std::istream &stream, btMatrix3x3& v);
