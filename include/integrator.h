#ifndef INTEGRATOR_H
#define INTEGRATOR_H

#include "cloth.h"
#include <Eigen/Core>
#include <iostream>
#include <fstream>

namespace ClothOpt {

// Debug information structure
struct DebugInfo {
    double totalForce = 0.0;
    double totalVelocity = 0.0;
    double maxDisplacement = 0.0;
    int activeVertices = 0;
    int pinnedVertices = 0;
    double gravityMagnitude = 0.0;
    double springForceMagnitude = 0.0;
    
    void print() const {
        std::cout << "=== DEBUG INFO ===" << std::endl;
        std::cout << "Active vertices: " << activeVertices << "/" << (activeVertices + pinnedVertices) << std::endl;
        std::cout << "Total force magnitude: " << totalForce << std::endl;
        std::cout << "Total velocity magnitude: " << totalVelocity << std::endl;
        std::cout << "Max displacement: " << maxDisplacement << std::endl;
        std::cout << "Gravity force magnitude: " << gravityMagnitude << std::endl;
        std::cout << "Spring force magnitude: " << springForceMagnitude << std::endl;
        std::cout << "==================" << std::endl;
    }
};

// Base integrator class
class Integrator {
public:
    virtual ~Integrator() = default;
    virtual void step(ClothMesh& mesh, double dt) = 0;
    
    // Debug controls
    void enableDebug(bool enable) { debugEnabled_ = enable; }
    void setDebugFrequency(int frequency) { debugFrequency_ = frequency; }
    void enableVerboseDebug(bool enable) { verboseDebug_ = enable; }
    
protected:
    bool debugEnabled_ = false;
    bool verboseDebug_ = false;
    int debugFrequency_ = 100;  // Print debug info every N steps
    int stepCount_ = 0;
};

// Semi-implicit Euler integrator with debug capabilities
class SemiImplicitEulerIntegrator : public Integrator {
public:
    void step(ClothMesh& mesh, double dt) override;
    
    // Get last debug info
    const DebugInfo& getLastDebugInfo() const { return lastDebugInfo_; }
    
private:
    DebugInfo lastDebugInfo_;
    
    void applyGravity(ClothMesh& mesh, double dt);
    void applySpringForces(ClothMesh& mesh, double dt);
    void updatePositions(ClothMesh& mesh, double dt);
    void satisfyConstraints(ClothMesh& mesh);
    void handleCollisions(ClothMesh& mesh);
   
    bool pointToTriangleDistance(
        const Eigen::Vector3d& point,
        const Eigen::Vector3d& v0,
        const Eigen::Vector3d& v1, 
        const Eigen::Vector3d& v2,
        Eigen::Vector3d& closestPoint,
        double& distance,
        bool& isInside);
    
    // Debug functions
    void debugForces(const ClothMesh& mesh, const std::string& stage);
    void debugVelocities(const ClothMesh& mesh, const std::string& stage);
    void debugPositions(const ClothMesh& mesh, const std::string& stage);
    void computeDebugInfo(const ClothMesh& mesh);
    void printVertexInfo(const ClothMesh& mesh, int vertexIndex);

};

} // namespace ClothOpt

#endif // INTEGRATOR_H