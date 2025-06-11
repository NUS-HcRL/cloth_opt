#pragma once

#include <Eigen/Dense>
#include <vector>
#include <unordered_map>
#include <memory>

namespace ClothOpt {

class ClothMesh; // Forward declaration

struct ControlTarget {
    size_t vertexIndex;
    Eigen::Vector3d targetPosition;
    Eigen::Vector3d targetVelocity;
    Eigen::Vector3d externalForce;
    bool positionControlActive;
    bool velocityControlActive;
    bool forceControlActive;
    double positionGain;    // P gain for position control
    double velocityGain;    // D gain for velocity control
    double maxForce;        // Maximum control force
    
    ControlTarget() 
        : vertexIndex(0)
        , targetPosition(Eigen::Vector3d::Zero())
        , targetVelocity(Eigen::Vector3d::Zero())
        , externalForce(Eigen::Vector3d::Zero())
        , positionControlActive(false)
        , velocityControlActive(false)
        , forceControlActive(false)
        , positionGain(1000.0)
        , velocityGain(100.0)
        , maxForce(100.0) {}
};

class ClothController {
public:
    ClothController();
    ~ClothController() = default;
    
    // === CONTROL TARGET MANAGEMENT ===
    
    // Add control targets
    void addPositionControl(size_t vertexIndex, const Eigen::Vector3d& targetPosition, 
                           double gain = 1000.0, double maxForce = 100.0);
    
    void addVelocityControl(size_t vertexIndex, const Eigen::Vector3d& targetVelocity, 
                           double gain = 100.0, double maxForce = 50.0);
    
    void addForceControl(size_t vertexIndex, const Eigen::Vector3d& force);
    
    // Update existing controls
    void updatePositionTarget(size_t vertexIndex, const Eigen::Vector3d& newTarget);
    void updateVelocityTarget(size_t vertexIndex, const Eigen::Vector3d& newVelocity);
    void updateForceTarget(size_t vertexIndex, const Eigen::Vector3d& newForce);
    
    // Remove controls
    void removeControl(size_t vertexIndex);
    void removeAllControls();
    
    // === CONTROL EXECUTION ===
    
    // Apply all active controls to the cloth mesh
    void applyControls(ClothMesh& mesh, double dt);
    
    // === TRAJECTORY CONTROL ===
    
    // Set a trajectory for a vertex (position over time)
    void setTrajectory(size_t vertexIndex, const std::vector<Eigen::Vector3d>& positions, 
                      const std::vector<double>& times, bool loop = false);
    
    void updateTrajectories(double currentTime);
    
    // === UTILITY FUNCTIONS ===
    
    // Check if a vertex is under control
    bool isControlled(size_t vertexIndex) const;
    
    // Get current control force for a vertex
    Eigen::Vector3d getControlForce(size_t vertexIndex) const;
    
    // Get control status
    size_t getControlCount() const { return controlTargets_.size(); }
    std::vector<size_t> getControlledVertices() const;
    
    // Enable/disable control debugging
    void enableDebug(bool enable) { debugEnabled_ = enable; }
    
    // === PRESET CONTROL PATTERNS ===
    
    // Apply circular motion to a vertex
    void addCircularMotion(size_t vertexIndex, const Eigen::Vector3d& center, 
                          double radius, double frequency, const Eigen::Vector3d& axis = Eigen::Vector3d(0,1,0));
    
    // Apply sinusoidal motion
    void addSinusoidalMotion(size_t vertexIndex, const Eigen::Vector3d& center,
                            const Eigen::Vector3d& amplitude, double frequency);
    
    // Apply wind-like forces to multiple vertices
    void addWindForce(const std::vector<size_t>& vertexIndices, 
                     const Eigen::Vector3d& windDirection, double strength, double turbulence = 0.0);

private:
    std::unordered_map<size_t, ControlTarget> controlTargets_;
    
    // Trajectory data
    struct Trajectory {
        std::vector<Eigen::Vector3d> positions;
        std::vector<double> times;
        bool loop;
        double startTime;
    };
    std::unordered_map<size_t, Trajectory> trajectories_;
    
    // Motion patterns
    struct MotionPattern {
        enum Type { CIRCULAR, SINUSOIDAL } type;
        Eigen::Vector3d center;
        Eigen::Vector3d amplitude;
        Eigen::Vector3d axis;
        double frequency;
        double radius;
        double startTime;
    };
    std::unordered_map<size_t, MotionPattern> motionPatterns_;
    
    double currentTime_;
    bool debugEnabled_;
    
    // Helper functions
    Eigen::Vector3d calculatePositionControl(const ControlTarget& target, 
                                            const Eigen::Vector3d& currentPos) const;
    
    Eigen::Vector3d calculateVelocityControl(const ControlTarget& target, 
                                            const Eigen::Vector3d& currentVel) const;
    
    Eigen::Vector3d interpolateTrajectory(const Trajectory& trajectory, double time) const;
    
    void updateMotionPatterns(double currentTime);
};

} // namespace ClothOpt