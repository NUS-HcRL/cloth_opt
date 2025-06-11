#ifndef CLOTH_H
#define CLOTH_H

#include "mesh.h"
#include <Eigen/Geometry>
#include <vector>

namespace ClothOpt {

// Edge structure for spring constraints
struct Edge {
    int v0, v1;
    double restLength;
    Edge(int vertex0, int vertex1, double length) : v0(vertex0), v1(vertex1), restLength(length) {}
};

// Collision sphere structure
struct CollisionSphere {
    Eigen::Vector3d center;
    double radius;
    CollisionSphere(const Eigen::Vector3d& c, double r) : center(c), radius(r) {}
};

// Cloth properties
struct ClothProperties {
    double mass = 1.0;
    double stiffness = 1000.0;
    double damping = 0.99;
    double bendingStiffness = 100.0;
    double friction = 0.8;  // Friction coefficient for collisions
    Eigen::Vector3d gravity = Eigen::Vector3d(0, 0, -9.81);  // Gravity vector
};

// Cloth simulation class
class ClothMesh : public SurfaceMesh {
public:
    ClothMesh();
    
    // Grid creation
    void createGrid(int width, int height, double spacing = 0.1);
    
    // Simulation state
    std::vector<Eigen::Vector3d> velocities;
    std::vector<Eigen::Vector3d> forces;
    std::vector<bool> pinned;
    
    // Constraints
    std::vector<Edge> distanceConstraints;
    std::vector<Edge> bendingConstraints;
    
    // Collision objects
    std::vector<CollisionSphere> collisionSpheres;
    
    // Properties
    ClothProperties properties;
    
    // Constraint helpers
    void pinVertex(int index);
    void pinCorners();
    
    // Collision helpers
    void addSphere(const Eigen::Vector3d& center, double radius);
    void clearSpheres();
    
    // Grid utility
    int getGridIndex(int i, int j) const { return i * gridWidth_ + j; }
    
private:
    int gridWidth_, gridHeight_;
    double gridSpacing_;
    
    void buildConstraints();
};

} // namespace ClothOpt

#endif // CLOTH_H