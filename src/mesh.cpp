#include "mesh.h"
#include <Eigen/Geometry>  // Add this for cross product
#include <iostream>
#include <cmath>

namespace ClothOpt {

void SurfaceMesh::clear() {
    vertices_.clear();
    triangles_.clear();
}

void SurfaceMesh::addVertex(const Eigen::Vector3d& position) {
    vertices_.emplace_back(position);
}

void SurfaceMesh::addTriangle(int i0, int i1, int i2) {
    triangles_.emplace_back(i0, i1, i2);
}

void SurfaceMesh::setVertexPosition(size_t index, const Eigen::Vector3d& position) {
    if (index < vertices_.size()) {
        vertices_[index].position = position;
    }
}

void SurfaceMesh::computeNormals() {
    // Compute triangle normals
    for (auto& triangle : triangles_) {
        const Eigen::Vector3d& v0 = vertices_[triangle.indices[0]].position;
        const Eigen::Vector3d& v1 = vertices_[triangle.indices[1]].position;
        const Eigen::Vector3d& v2 = vertices_[triangle.indices[2]].position;
        triangle.normal = MeshUtils::computeTriangleNormal(v0, v1, v2);
    }
    
    // Reset vertex normals
    for (auto& vertex : vertices_) {
        vertex.normal = Eigen::Vector3d::Zero();
    }
    
    // Accumulate triangle normals to vertices
    for (const auto& triangle : triangles_) {
        for (int i = 0; i < 3; ++i) {
            vertices_[triangle.indices[i]].normal += triangle.normal;
        }
    }
    
    // Normalize vertex normals
    for (auto& vertex : vertices_) {
        if (vertex.normal.norm() > 1e-8) {
            vertex.normal.normalize();
        } else {
            vertex.normal = Eigen::Vector3d(0, 1, 0);
        }
    }
}

Eigen::MatrixXd SurfaceMesh::getVertexMatrix() const {
    Eigen::MatrixXd vertices(vertices_.size(), 3);
    for (size_t i = 0; i < vertices_.size(); ++i) {
        vertices.row(i) = vertices_[i].position;
    }
    return vertices;
}

Eigen::MatrixXi SurfaceMesh::getTriangleMatrix() const {
    Eigen::MatrixXi triangles(triangles_.size(), 3);
    for (size_t i = 0; i < triangles_.size(); ++i) {
        triangles.row(i) = Eigen::Vector3i(triangles_[i].indices[0], 
                                          triangles_[i].indices[1], 
                                          triangles_[i].indices[2]);
    }
    return triangles;
}

// Utility functions
namespace MeshUtils {

Eigen::Vector3d computeTriangleNormal(const Eigen::Vector3d& v0, const Eigen::Vector3d& v1, const Eigen::Vector3d& v2) {
    Eigen::Vector3d edge1 = v1 - v0;
    Eigen::Vector3d edge2 = v2 - v0;
    Eigen::Vector3d normal = edge1.cross(edge2);  // This needs Eigen/Geometry
    
    if (normal.norm() > 1e-8) {
        normal.normalize();
    } else {
        normal = Eigen::Vector3d(0, 1, 0);
    }
    return normal;
}

double computeTriangleArea(const Eigen::Vector3d& v0, const Eigen::Vector3d& v1, const Eigen::Vector3d& v2) {
    Eigen::Vector3d edge1 = v1 - v0;
    Eigen::Vector3d edge2 = v2 - v0;
    return 0.5 * edge1.cross(edge2).norm();  // This needs Eigen/Geometry
}

} // namespace MeshUtils

} // namespace ClothOpt