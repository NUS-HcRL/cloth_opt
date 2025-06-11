#include "controller.h"
#include "cloth.h"
#include <iostream>
#include <algorithm>
#include <cmath>

namespace ClothOpt {

ClothController::ClothController() 
    : currentTime_(0.0)
    , debugEnabled_(false) {
}

// === CONTROL TARGET MANAGEMENT ===

void ClothController::addPositionControl(size_t vertexIndex, const Eigen::Vector3d& targetPosition, 
                                        double gain, double maxForce) {
    ControlTarget& target = controlTargets_[vertexIndex];
    target.vertexIndex = vertexIndex;
    target.targetPosition = targetPosition;
    target.positionControlActive = true;
    target.positionGain = gain;
    target.maxForce = maxForce;
    
    if (debugEnabled_) {
        std::cout << "Added position control for vertex " << vertexIndex 
                  << " target: " << targetPosition.transpose() << std::endl;
    }
}

void ClothController::addVelocityControl(size_t vertexIndex, const Eigen::Vector3d& targetVelocity, 
                                        double gain, double maxForce) {
    ControlTarget& target = controlTargets_[vertexIndex];
    target.vertexIndex = vertexIndex;
    target.targetVelocity = targetVelocity;
    target.velocityControlActive = true;
    target.velocityGain = gain;
    target.maxForce = maxForce;
    
    if (debugEnabled_) {
        std::cout << "Added velocity control for vertex " << vertexIndex 
                  << " target: " << targetVelocity.transpose() << std::endl;
    }
}

void ClothController::addForceControl(size_t vertexIndex, const Eigen::Vector3d& force) {
    ControlTarget& target = controlTargets_[vertexIndex];
    target.vertexIndex = vertexIndex;
    target.externalForce = force;
    target.forceControlActive = true;
    
    if (debugEnabled_) {
        std::cout << "Added force control for vertex " << vertexIndex 
                  << " force: " << force.transpose() << std::endl;
    }
}

void ClothController::updatePositionTarget(size_t vertexIndex, const Eigen::Vector3d& newTarget) {
    auto it = controlTargets_.find(vertexIndex);
    if (it != controlTargets_.end()) {
        it->second.targetPosition = newTarget;
        it->second.positionControlActive = true;
    }
}

void ClothController::updateVelocityTarget(size_t vertexIndex, const Eigen::Vector3d& newVelocity) {
    auto it = controlTargets_.find(vertexIndex);
    if (it != controlTargets_.end()) {
        it->second.targetVelocity = newVelocity;
        it->second.velocityControlActive = true;
    }
}

void ClothController::updateForceTarget(size_t vertexIndex, const Eigen::Vector3d& newForce) {
    auto it = controlTargets_.find(vertexIndex);
    if (it != controlTargets_.end()) {
        it->second.externalForce = newForce;
        it->second.forceControlActive = true;
    }
}

void ClothController::removeControl(size_t vertexIndex) {
    controlTargets_.erase(vertexIndex);
    trajectories_.erase(vertexIndex);
    motionPatterns_.erase(vertexIndex);
    
    if (debugEnabled_) {
        std::cout << "Removed control for vertex " << vertexIndex << std::endl;
    }
}

void ClothController::removeAllControls() {
    controlTargets_.clear();
    trajectories_.clear();
    motionPatterns_.clear();
    
    if (debugEnabled_) {
        std::cout << "Removed all controls" << std::endl;
    }
}

// === CONTROL EXECUTION ===

void ClothController::applyControls(ClothMesh& mesh, double dt) {
    currentTime_ += dt;
    
    // Update motion patterns
    updateMotionPatterns(currentTime_);
    
    // Update trajectories
    updateTrajectories(currentTime_);
    
    // Apply controls
    for (auto& [vertexIndex, target] : controlTargets_) {
        if (vertexIndex >= mesh.getVertexCount()) continue;
        
        Eigen::Vector3d currentPos = mesh.getVertex(vertexIndex).position;
        Eigen::Vector3d currentVel = mesh.velocities[vertexIndex];
        Eigen::Vector3d totalForce = Eigen::Vector3d::Zero();
        
        // Position control
        if (target.positionControlActive) {
            Eigen::Vector3d posForce = calculatePositionControl(target, currentPos);
            totalForce += posForce;
        }
        
        // Velocity control
        if (target.velocityControlActive) {
            Eigen::Vector3d velForce = calculateVelocityControl(target, currentVel);
            totalForce += velForce;
        }
        
        // External force
        if (target.forceControlActive) {
            totalForce += target.externalForce;
        }
        
        // Clamp total force
        if (totalForce.norm() > target.maxForce) {
            totalForce = totalForce.normalized() * target.maxForce;
        }
        
        // Apply force to mesh (convert to acceleration)
        if (totalForce.norm() > 1e-8) {
            // Assume unit mass for simplicity
            Eigen::Vector3d acceleration = totalForce;
            mesh.velocities[vertexIndex] += acceleration * dt;
        }
        
        if (debugEnabled_ && (int)(currentTime_ * 10) % 30 == 0) { // Debug every 3 seconds
            std::cout << "Vertex " << vertexIndex << " force: " << totalForce.transpose() 
                      << " pos: " << currentPos.transpose() << std::endl;
        }
    }
}

// === TRAJECTORY CONTROL ===

void ClothController::setTrajectory(size_t vertexIndex, const std::vector<Eigen::Vector3d>& positions, 
                                   const std::vector<double>& times, bool loop) {
    if (positions.size() != times.size() || positions.empty()) {
        std::cerr << "Invalid trajectory data for vertex " << vertexIndex << std::endl;
        return;
    }
    
    Trajectory& traj = trajectories_[vertexIndex];
    traj.positions = positions;
    traj.times = times;
    traj.loop = loop;
    traj.startTime = currentTime_;
    
    // Ensure we have position control enabled
    addPositionControl(vertexIndex, positions[0]);
    
    if (debugEnabled_) {
        std::cout << "Set trajectory for vertex " << vertexIndex 
                  << " with " << positions.size() << " waypoints" << std::endl;
    }
}

void ClothController::updateTrajectories(double currentTime) {
    for (auto& [vertexIndex, trajectory] : trajectories_) {
        double relativeTime = currentTime - trajectory.startTime;
        Eigen::Vector3d targetPos = interpolateTrajectory(trajectory, relativeTime);
        updatePositionTarget(vertexIndex, targetPos);
    }
}

// === UTILITY FUNCTIONS ===

bool ClothController::isControlled(size_t vertexIndex) const {
    return controlTargets_.find(vertexIndex) != controlTargets_.end();
}

Eigen::Vector3d ClothController::getControlForce(size_t vertexIndex) const {
    auto it = controlTargets_.find(vertexIndex);
    if (it != controlTargets_.end()) {
        return it->second.externalForce;
    }
    return Eigen::Vector3d::Zero();
}

std::vector<size_t> ClothController::getControlledVertices() const {
    std::vector<size_t> vertices;
    for (const auto& [vertexIndex, target] : controlTargets_) {
        vertices.push_back(vertexIndex);
    }
    return vertices;
}

// === PRESET CONTROL PATTERNS ===

void ClothController::addCircularMotion(size_t vertexIndex, const Eigen::Vector3d& center, 
                                       double radius, double frequency, const Eigen::Vector3d& axis) {
    MotionPattern pattern;
    pattern.type = MotionPattern::CIRCULAR;
    pattern.center = center;
    pattern.radius = radius;
    pattern.frequency = frequency;
    pattern.axis = axis.normalized();
    pattern.startTime = currentTime_;
    
    motionPatterns_[vertexIndex] = pattern;
    addPositionControl(vertexIndex, center + Eigen::Vector3d(radius, 0, 0));
    
    if (debugEnabled_) {
        std::cout << "Added circular motion for vertex " << vertexIndex 
                  << " center: " << center.transpose() << " radius: " << radius << std::endl;
    }
}

void ClothController::addSinusoidalMotion(size_t vertexIndex, const Eigen::Vector3d& center,
                                         const Eigen::Vector3d& amplitude, double frequency) {
    MotionPattern pattern;
    pattern.type = MotionPattern::SINUSOIDAL;
    pattern.center = center;
    pattern.amplitude = amplitude;
    pattern.frequency = frequency;
    pattern.startTime = currentTime_;
    
    motionPatterns_[vertexIndex] = pattern;
    addPositionControl(vertexIndex, center);
    
    if (debugEnabled_) {
        std::cout << "Added sinusoidal motion for vertex " << vertexIndex 
                  << " center: " << center.transpose() << " amplitude: " << amplitude.transpose() << std::endl;
    }
}

void ClothController::addWindForce(const std::vector<size_t>& vertexIndices, 
                                  const Eigen::Vector3d& windDirection, double strength, double turbulence) {
    for (size_t vertexIndex : vertexIndices) {
        Eigen::Vector3d windForce = windDirection.normalized() * strength;
        
        // Add turbulence
        if (turbulence > 0) {
            Eigen::Vector3d randomDir = Eigen::Vector3d::Random().normalized();
            windForce += randomDir * (turbulence * strength * (rand() / (double)RAND_MAX - 0.5));
        }
        
        addForceControl(vertexIndex, windForce);
    }
    
    if (debugEnabled_) {
        std::cout << "Added wind force to " << vertexIndices.size() << " vertices, "
                  << "direction: " << windDirection.transpose() << " strength: " << strength << std::endl;
    }
}

// === PRIVATE HELPER FUNCTIONS ===

Eigen::Vector3d ClothController::calculatePositionControl(const ControlTarget& target, 
                                                         const Eigen::Vector3d& currentPos) const {
    Eigen::Vector3d error = target.targetPosition - currentPos;
    return error * target.positionGain;
}

Eigen::Vector3d ClothController::calculateVelocityControl(const ControlTarget& target, 
                                                         const Eigen::Vector3d& currentVel) const {
    Eigen::Vector3d error = target.targetVelocity - currentVel;
    return error * target.velocityGain;
}

Eigen::Vector3d ClothController::interpolateTrajectory(const Trajectory& trajectory, double time) const {
    if (trajectory.positions.empty()) return Eigen::Vector3d::Zero();
    
    // Handle looping
    double totalTime = trajectory.times.back();
    if (trajectory.loop && time > totalTime) {
        time = fmod(time, totalTime);
    }
    
    // Find the appropriate segment
    for (size_t i = 0; i < trajectory.times.size() - 1; ++i) {
        if (time >= trajectory.times[i] && time <= trajectory.times[i + 1]) {
            double t = (time - trajectory.times[i]) / (trajectory.times[i + 1] - trajectory.times[i]);
            return trajectory.positions[i] * (1.0 - t) + trajectory.positions[i + 1] * t;
        }
    }
    
    // Return last position if time exceeds trajectory
    return trajectory.positions.back();
}

void ClothController::updateMotionPatterns(double currentTime) {
    for (auto& [vertexIndex, pattern] : motionPatterns_) {
        double t = currentTime - pattern.startTime;
        Eigen::Vector3d newTarget;
        
        switch (pattern.type) {
            case MotionPattern::CIRCULAR: {
                double angle = 2.0 * M_PI * pattern.frequency * t;
                
                // Create orthonormal basis
                Eigen::Vector3d u = pattern.axis;
                Eigen::Vector3d v = (u.cross(Eigen::Vector3d(1, 0, 0))).normalized();
                if (v.norm() < 0.1) v = (u.cross(Eigen::Vector3d(0, 1, 0))).normalized();
                Eigen::Vector3d w = u.cross(v);
                
                newTarget = pattern.center + pattern.radius * (v * cos(angle) + w * sin(angle));
                break;
            }
            case MotionPattern::SINUSOIDAL: {
                newTarget = pattern.center + pattern.amplitude * sin(2.0 * M_PI * pattern.frequency * t);
                break;
            }
        }
        
        updatePositionTarget(vertexIndex, newTarget);
    }
}

} // namespace ClothOpt