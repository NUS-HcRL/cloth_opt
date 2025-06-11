#include "cloth.h"
#include <cmath>
#include <algorithm>

namespace ClothOpt {

ClothMesh::ClothMesh() : gridWidth_(0), gridHeight_(0), gridSpacing_(0.1) {}

void ClothMesh::createGrid(int width, int height, double spacing) {
    clear();
    gridWidth_ = width;
    gridHeight_ = height;
    gridSpacing_ = spacing;
    
    // Create vertices
    for (int i = 0; i < height; ++i) {
        for (int j = 0; j < width; ++j) {
            Eigen::Vector3d pos(j * spacing, 1.0, i * spacing);
            addVertex(pos);
        }
    }
    
    // Create triangles
    for (int i = 0; i < height - 1; ++i) {
        for (int j = 0; j < width - 1; ++j) {
            int topLeft = getGridIndex(i, j);
            int topRight = getGridIndex(i, j + 1);
            int bottomLeft = getGridIndex(i + 1, j);
            int bottomRight = getGridIndex(i + 1, j + 1);
            
            addTriangle(topLeft, bottomLeft, topRight);
            addTriangle(topRight, bottomLeft, bottomRight);
        }
    }
    
    // Initialize simulation data
    size_t vertexCount = getVertexCount();
    velocities.resize(vertexCount, Eigen::Vector3d::Zero());
    forces.resize(vertexCount, Eigen::Vector3d::Zero());
    pinned.resize(vertexCount, false);
    
    buildConstraints();
    computeNormals();
}

void ClothMesh::buildConstraints() {
    distanceConstraints.clear();
    bendingConstraints.clear();
    
    // Distance constraints (structural springs)
    for (int i = 0; i < gridHeight_; ++i) {
        for (int j = 0; j < gridWidth_; ++j) {
            int current = getGridIndex(i, j);
            
            // Horizontal springs
            if (j < gridWidth_ - 1) {
                int right = getGridIndex(i, j + 1);
                distanceConstraints.emplace_back(current, right, gridSpacing_);
            }
            
            // Vertical springs
            if (i < gridHeight_ - 1) {
                int down = getGridIndex(i + 1, j);
                distanceConstraints.emplace_back(current, down, gridSpacing_);
            }
        }
    }
    
    // Bending constraints (skip one vertex)
    for (int i = 0; i < gridHeight_; ++i) {
        for (int j = 0; j < gridWidth_; ++j) {
            int current = getGridIndex(i, j);
            
            // Horizontal bending
            if (j < gridWidth_ - 2) {
                int right2 = getGridIndex(i, j + 2);
                bendingConstraints.emplace_back(current, right2, 2.0 * gridSpacing_);
            }
            
            // Vertical bending
            if (i < gridHeight_ - 2) {
                int down2 = getGridIndex(i + 2, j);
                bendingConstraints.emplace_back(current, down2, 2.0 * gridSpacing_);
            }
        }
    }
}

void ClothMesh::addSphere(const Eigen::Vector3d& center, double radius) {
    collisionSpheres.emplace_back(center, radius);
}

void ClothMesh::clearSpheres() {
    collisionSpheres.clear();
}

void ClothMesh::pinVertex(int index) {
    if (index >= 0 && static_cast<size_t>(index) < pinned.size()) {
        pinned[index] = true;
        velocities[index] = Eigen::Vector3d::Zero();
    }
}

void ClothMesh::pinCorners() {
    if (gridWidth_ > 0 && gridHeight_ > 0) {
        pinVertex(getGridIndex(0, 0));                              // Top-left
        pinVertex(getGridIndex(0, gridWidth_ - 1));                 // Top-right
        pinVertex(getGridIndex(gridHeight_ - 1, 0));                // Bottom-left
        pinVertex(getGridIndex(gridHeight_ - 1, gridWidth_ - 1));   // Bottom-right
    }
}

} // namespace ClothOpt