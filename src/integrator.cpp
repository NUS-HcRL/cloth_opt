#include "integrator.h"
#include <iostream>
#include <iomanip>

// Include parallel processing headers
#ifdef HAVE_TBB
    #include <tbb/parallel_for.h>
    #include <tbb/blocked_range.h>
    #include <tbb/combinable.h>
    #define PARALLEL_BACKEND "Intel oneTBB"
#elif defined(HAVE_OPENMP)
    #include <omp.h>
    #define PARALLEL_BACKEND "OpenMP"
#else
    #define PARALLEL_BACKEND "Sequential"
#endif

namespace ClothOpt {

void SemiImplicitEulerIntegrator::step(ClothMesh& mesh, double dt) {
    stepCount_++;
    
    // Clear forces
    std::fill(mesh.forces.begin(), mesh.forces.end(), Eigen::Vector3d::Zero());
    
    // Apply forces
    applyGravity(mesh, dt);
    applySpringForces(mesh, dt);
    
    // Update velocities and positions in one pass
    for (size_t i = 0; i < mesh.getVertexCount(); ++i) {
        if (mesh.pinned[i]) continue;
        
        // Update velocity
        Eigen::Vector3d acceleration = mesh.forces[i] / mesh.properties.mass;
        mesh.velocities[i] = mesh.velocities[i] * mesh.properties.damping + acceleration * dt;
        
        // Update position
        Eigen::Vector3d newPosition = mesh.getVertex(i).position + mesh.velocities[i] * dt;
        mesh.setVertexPosition(i, newPosition);
    }
    
    // Handle collisions (fast method)
    handleCollisions(mesh);
    
    // Satisfy constraints (reduced iterations for speed)
    satisfyConstraints(mesh);
    
    // Debug only occasionally
    if (debugEnabled_ && stepCount_ % (debugFrequency_ * 5) == 0) {
        computeDebugInfo(mesh);
        lastDebugInfo_.print();
    }
}

void SemiImplicitEulerIntegrator::applyGravity(ClothMesh& mesh, double dt) {
    double vertexMass = mesh.properties.mass / mesh.getVertexCount();
    
    if (debugEnabled_ && stepCount_ % debugFrequency_ == 0) {
        std::cout << "Applying gravity: mass=" << vertexMass 
                  << ", gravity=" << mesh.properties.gravity.transpose() << std::endl;
        std::cout << "Total vertices: " << mesh.getVertexCount() << std::endl;
        
        // Count pinned vertices
        int pinnedCount = 0;
        for (size_t i = 0; i < mesh.getVertexCount(); ++i) {
            if (mesh.pinned[i]) pinnedCount++;
        }
        std::cout << "Pinned vertices: " << pinnedCount << std::endl;
    }
    
    for (size_t i = 0; i < mesh.getVertexCount(); ++i) {
        if (!mesh.pinned[i]) {
            Eigen::Vector3d gravityForce = vertexMass * mesh.properties.gravity;
            mesh.forces[i] += gravityForce;
            
            if (debugEnabled_ && verboseDebug_ && stepCount_ % debugFrequency_ == 0 && i < 3) {
                std::cout << "Vertex " << i << " gravity force: " << gravityForce.transpose() << std::endl;
            }
        }
    }
}

void SemiImplicitEulerIntegrator::applySpringForces(ClothMesh& mesh, double dt) {
    if (debugEnabled_ && stepCount_ % debugFrequency_ == 0) {
        std::cout << "Applying spring forces: " << mesh.distanceConstraints.size() 
                  << " distance constraints, " << mesh.bendingConstraints.size() 
                  << " bending constraints" << std::endl;
        std::cout << "Stiffness: " << mesh.properties.stiffness 
                  << ", Bending stiffness: " << mesh.properties.bendingStiffness << std::endl;
    }
    
    // Distance constraints (structural springs)
    for (const auto& constraint : mesh.distanceConstraints) {
        const Eigen::Vector3d& p1 = mesh.getVertex(constraint.v0).position;
        const Eigen::Vector3d& p2 = mesh.getVertex(constraint.v1).position;
        
        Eigen::Vector3d direction = p2 - p1;
        double currentLength = direction.norm();
        
        if (currentLength > 1e-8) {
            direction /= currentLength;
            double stretch = currentLength - constraint.restLength;
            Eigen::Vector3d springForce = mesh.properties.stiffness * stretch * direction;
            
            mesh.forces[constraint.v0] += springForce;
            mesh.forces[constraint.v1] -= springForce;
            
            if (debugEnabled_ && verboseDebug_ && stepCount_ % debugFrequency_ == 0 && 
                (constraint.v0 < 3 || constraint.v1 < 3)) {
                std::cout << "Spring constraint " << constraint.v0 << "-" << constraint.v1 
                          << ": length=" << currentLength << ", rest=" << constraint.restLength
                          << ", stretch=" << stretch << ", force=" << springForce.norm() << std::endl;
            }
        }
    }
    
    // Bending constraints
    for (const auto& constraint : mesh.bendingConstraints) {
        const Eigen::Vector3d& p1 = mesh.getVertex(constraint.v0).position;
        const Eigen::Vector3d& p2 = mesh.getVertex(constraint.v1).position;
        
        Eigen::Vector3d direction = p2 - p1;
        double currentLength = direction.norm();
        
        if (currentLength > 1e-8) {
            direction /= currentLength;
            double stretch = currentLength - constraint.restLength;
            Eigen::Vector3d bendingForce = mesh.properties.bendingStiffness * stretch * direction;
            
            mesh.forces[constraint.v0] += bendingForce;
            mesh.forces[constraint.v1] -= bendingForce;
        }
    }
}

void SemiImplicitEulerIntegrator::updatePositions(ClothMesh& mesh, double dt) {
    double vertexMass = mesh.properties.mass / mesh.getVertexCount();
    
    if (debugEnabled_ && stepCount_ % debugFrequency_ == 0) {
        std::cout << "Updating positions: vertex mass=" << vertexMass 
                  << ", damping=" << mesh.properties.damping << std::endl;
    }
    
    double maxDisplacement = 0.0;
    
    // Semi-implicit Euler: update velocity first, then position
    for (size_t i = 0; i < mesh.getVertexCount(); ++i) {
        if (!mesh.pinned[i]) {
            Eigen::Vector3d oldPosition = mesh.getVertex(i).position;
            Eigen::Vector3d oldVelocity = mesh.velocities[i];
            
            // Update velocity with forces
            mesh.velocities[i] += dt * mesh.forces[i] / vertexMass;
            
            // Apply damping
            mesh.velocities[i] *= mesh.properties.damping;
            
            // Update position with new velocity
            Eigen::Vector3d newPosition = oldPosition + dt * mesh.velocities[i];
            mesh.setVertexPosition(i, newPosition);
            
            double displacement = (newPosition - oldPosition).norm();
            maxDisplacement = std::max(maxDisplacement, displacement);
            
            if (debugEnabled_ && verboseDebug_ && stepCount_ % debugFrequency_ == 0 && i < 3) {
                std::cout << "Vertex " << i << ":" << std::endl;
                std::cout << "  Old pos: " << oldPosition.transpose() << std::endl;
                std::cout << "  Force: " << mesh.forces[i].transpose() << std::endl;
                std::cout << "  Old vel: " << oldVelocity.transpose() << std::endl;
                std::cout << "  New vel: " << mesh.velocities[i].transpose() << std::endl;
                std::cout << "  New pos: " << newPosition.transpose() << std::endl;
                std::cout << "  Displacement: " << displacement << std::endl;
            }
        }
    }
    
    if (debugEnabled_ && stepCount_ % debugFrequency_ == 0) {
        std::cout << "Max displacement this step: " << maxDisplacement << std::endl;
    }
    
    // Store maxDisplacement in debug info
    lastDebugInfo_.maxDisplacement = maxDisplacement;
}

void SemiImplicitEulerIntegrator::satisfyConstraints(ClothMesh& mesh) {
    const int iterations = 1; // Reduced from 2 to 1 for speed
    
    for (int iter = 0; iter < iterations; ++iter) {
        for (const auto& constraint : mesh.distanceConstraints) {
            if (mesh.pinned[constraint.v0] && mesh.pinned[constraint.v1]) continue;
            
            Eigen::Vector3d p1 = mesh.getVertex(constraint.v0).position;
            Eigen::Vector3d p2 = mesh.getVertex(constraint.v1).position;
            
            Eigen::Vector3d delta = p2 - p1;
            double currentLength = delta.norm();
            
            if (currentLength > 1e-6) { // Slightly larger threshold
                double difference = (currentLength - constraint.restLength) / currentLength;
                Eigen::Vector3d correction = 0.5 * difference * delta;
                
                if (!mesh.pinned[constraint.v0]) {
                    mesh.setVertexPosition(constraint.v0, p1 + correction);
                }
                if (!mesh.pinned[constraint.v1]) {
                    mesh.setVertexPosition(constraint.v1, p2 - correction);
                }
            }
        }
    }
}

void SemiImplicitEulerIntegrator::handleCollisions(ClothMesh& mesh) {
    tbb::combinable<int> vertexCollisionCounter;
    tbb::combinable<int> edgeCollisionCounter;
    tbb::combinable<int> faceCollisionCounter;
    
    // VERTEX-SPHERE COLLISIONS (Parallel)
    tbb::parallel_for(
        tbb::blocked_range<size_t>(0, mesh.getVertexCount()),
        [&](const tbb::blocked_range<size_t>& range) {
            int localCollisions = 0;
            
            for (size_t i = range.begin(); i != range.end(); ++i) {
                if (mesh.pinned[i]) continue;
                
                Eigen::Vector3d position = mesh.getVertex(i).position;
                bool hadCollision = false;
                
                // Ground collision
                if (position.y() < 0.0) {
                    position.y() = 0.0;
                    hadCollision = true;
                    localCollisions++;
                }
                
                // Sphere collisions
                for (const auto& sphere : mesh.collisionSpheres) {
                    Eigen::Vector3d toVertex = position - sphere.center;
                    double distance = toVertex.norm();
                    
                    if (distance < sphere.radius) {
                        if (distance > 1e-8) {
                            Eigen::Vector3d normal = toVertex / distance;
                            position = sphere.center + normal * sphere.radius;
                        } else {
                            position = sphere.center + Eigen::Vector3d(sphere.radius, 0, 0);
                        }
                        hadCollision = true;
                        localCollisions++;
                    }
                }
                
                if (hadCollision) {
                    mesh.setVertexPosition(i, position);
                }
            }
            
            vertexCollisionCounter.local() += localCollisions;
        }
    );
    
    // EDGE-SPHERE COLLISIONS (Parallel)
    tbb::parallel_for(
        tbb::blocked_range<size_t>(0, mesh.distanceConstraints.size()),
        [&](const tbb::blocked_range<size_t>& range) {
            int localEdgeCollisions = 0;
            
            for (size_t idx = range.begin(); idx != range.end(); ++idx) {
                const auto& constraint = mesh.distanceConstraints[idx];
                size_t v0 = constraint.v0;
                size_t v1 = constraint.v1;
                
                if (mesh.pinned[v0] && mesh.pinned[v1]) continue;
                
                Eigen::Vector3d p0 = mesh.getVertex(v0).position;
                Eigen::Vector3d p1 = mesh.getVertex(v1).position;
                
                for (const auto& sphere : mesh.collisionSpheres) {
                    // Fast edge-sphere collision check
                    Eigen::Vector3d edge = p1 - p0;
                    double edgeLength = edge.norm();
                    
                    if (edgeLength < 1e-8) continue;
                    
                    edge /= edgeLength;
                    Eigen::Vector3d toStart = p0 - sphere.center;
                    
                    // Project sphere center onto edge
                    double t = -toStart.dot(edge);
                    t = std::max(0.0, std::min(edgeLength, t));
                    
                    Eigen::Vector3d closestPoint = p0 + t * edge;
                    double distToEdge = (closestPoint - sphere.center).norm();
                    
                    if (distToEdge < sphere.radius) {
                        // Calculate correction
                        Eigen::Vector3d normal = (closestPoint - sphere.center);
                        if (normal.norm() > 1e-8) {
                            normal.normalize();
                        } else {
                            normal = Eigen::Vector3d(1, 0, 0);
                        }
                        
                        double penetration = sphere.radius - distToEdge;
                        Eigen::Vector3d correction = normal * penetration * 0.5;
                        
                        // Apply corrections
                        if (!mesh.pinned[v0]) {
                            mesh.setVertexPosition(v0, p0 + correction);
                        }
                        if (!mesh.pinned[v1]) {
                            mesh.setVertexPosition(v1, p1 + correction);
                        }
                        
                        localEdgeCollisions++;
                    }
                }
            }
            
            edgeCollisionCounter.local() += localEdgeCollisions;
        }
    );
    
    // FACE-SPHERE COLLISIONS (Parallel)
    const auto& triangles = mesh.getTriangleMatrix();
    tbb::parallel_for(
        tbb::blocked_range<size_t>(0, triangles.rows()),
        [&](const tbb::blocked_range<size_t>& range) {
            int localFaceCollisions = 0;
            
            for (size_t faceIdx = range.begin(); faceIdx != range.end(); ++faceIdx) {
                // Get triangle vertices
                int v0 = triangles(faceIdx, 0);
                int v1 = triangles(faceIdx, 1);
                int v2 = triangles(faceIdx, 2);
                
                // Skip if all vertices are pinned
                if (mesh.pinned[v0] && mesh.pinned[v1] && mesh.pinned[v2]) continue;
                
                Eigen::Vector3d p0 = mesh.getVertex(v0).position;
                Eigen::Vector3d p1 = mesh.getVertex(v1).position;
                Eigen::Vector3d p2 = mesh.getVertex(v2).position;
                
                for (const auto& sphere : mesh.collisionSpheres) {
                    // Check if sphere center projects onto triangle
                    Eigen::Vector3d closestPoint;
                    double distToFace;
                    bool isInside;
                    
                    if (pointToTriangleDistance(sphere.center, p0, p1, p2, closestPoint, distToFace, isInside)) {
                        if (distToFace < sphere.radius) {
                            // Face intersects sphere
                            Eigen::Vector3d normal = (closestPoint - sphere.center);
                            if (normal.norm() > 1e-8) {
                                normal.normalize();
                            } else {
                                // Use face normal as fallback
                                Eigen::Vector3d edge1 = p1 - p0;
                                Eigen::Vector3d edge2 = p2 - p0;
                                normal = edge1.cross(edge2).normalized();
                            }
                            
                            double penetration = sphere.radius - distToFace;
                            Eigen::Vector3d correction = normal * penetration / 3.0; // Distribute among 3 vertices
                            
                            // Apply corrections to all triangle vertices
                            if (!mesh.pinned[v0]) {
                                mesh.setVertexPosition(v0, p0 + correction);
                            }
                            if (!mesh.pinned[v1]) {
                                mesh.setVertexPosition(v1, p1 + correction);
                            }
                            if (!mesh.pinned[v2]) {
                                mesh.setVertexPosition(v2, p2 + correction);
                            }
                            
                            localFaceCollisions++;
                        }
                    }
                }
            }
            
            faceCollisionCounter.local() += localFaceCollisions;
        }
    );
    
    int totalVertexCollisions = vertexCollisionCounter.combine([](int a, int b) { return a + b; });
    int totalEdgeCollisions = edgeCollisionCounter.combine([](int a, int b) { return a + b; });
    int totalFaceCollisions = faceCollisionCounter.combine([](int a, int b) { return a + b; });
    int totalCollisions = totalVertexCollisions + totalEdgeCollisions + totalFaceCollisions;
    
    if (debugEnabled_ && stepCount_ % debugFrequency_ == 0 && totalCollisions > 0) {
        std::cout << "Collisions: " << totalVertexCollisions << " vertex, " 
                  << totalEdgeCollisions << " edge, " << totalFaceCollisions 
                  << " face, total=" << totalCollisions << std::endl;
    }
}

// Helper function for point-to-triangle distance calculation
bool SemiImplicitEulerIntegrator::pointToTriangleDistance(
    const Eigen::Vector3d& point,
    const Eigen::Vector3d& v0,
    const Eigen::Vector3d& v1, 
    const Eigen::Vector3d& v2,
    Eigen::Vector3d& closestPoint,
    double& distance,
    bool& isInside) {
    
    // Triangle edges
    Eigen::Vector3d edge0 = v1 - v0;
    Eigen::Vector3d edge1 = v2 - v1;
    Eigen::Vector3d edge2 = v0 - v2;
    
    // Triangle normal
    Eigen::Vector3d normal = edge0.cross(v2 - v0);
    double normalLength = normal.norm();
    
    if (normalLength < 1e-8) {
        // Degenerate triangle
        return false;
    }
    
    normal /= normalLength;
    
    // Project point onto triangle plane
    Eigen::Vector3d toPoint = point - v0;
    double projDistance = toPoint.dot(normal);
    Eigen::Vector3d projectedPoint = point - projDistance * normal;
    
    // Check if projected point is inside triangle using barycentric coordinates
    Eigen::Vector3d v0v1 = v1 - v0;
    Eigen::Vector3d v0v2 = v2 - v0;
    Eigen::Vector3d v0p = projectedPoint - v0;
    
    double dot00 = v0v2.dot(v0v2);
    double dot01 = v0v2.dot(v0v1);
    double dot02 = v0v2.dot(v0p);
    double dot11 = v0v1.dot(v0v1);
    double dot12 = v0v1.dot(v0p);
    
    double invDenom = 1.0 / (dot00 * dot11 - dot01 * dot01);
    double u = (dot11 * dot02 - dot01 * dot12) * invDenom;
    double v = (dot00 * dot12 - dot01 * dot02) * invDenom;
    
    if (u >= 0 && v >= 0 && (u + v) <= 1) {
        // Point projects inside triangle
        closestPoint = projectedPoint;
        distance = std::abs(projDistance);
        isInside = true;
        return true;
    } else {
        // Point projects outside triangle - find closest edge/vertex
        isInside = false;
        
        // Check distances to edges and vertices
        double minDistSq = std::numeric_limits<double>::max();
        
        // Edge v0-v1
        double t = std::max(0.0, std::min(1.0, (point - v0).dot(edge0) / edge0.dot(edge0)));
        Eigen::Vector3d edgePoint = v0 + t * edge0;
        double distSq = (point - edgePoint).squaredNorm();
        if (distSq < minDistSq) {
            minDistSq = distSq;
            closestPoint = edgePoint;
        }
        
        // Edge v1-v2
        t = std::max(0.0, std::min(1.0, (point - v1).dot(edge1) / edge1.dot(edge1)));
        edgePoint = v1 + t * edge1;
        distSq = (point - edgePoint).squaredNorm();
        if (distSq < minDistSq) {
            minDistSq = distSq;
            closestPoint = edgePoint;
        }
        
        // Edge v2-v0
        t = std::max(0.0, std::min(1.0, (point - v2).dot(edge2) / edge2.dot(edge2)));
        edgePoint = v2 + t * edge2;
        distSq = (point - edgePoint).squaredNorm();
        if (distSq < minDistSq) {
            minDistSq = distSq;
            closestPoint = edgePoint;
        }
        
        distance = std::sqrt(minDistSq);
        return true;
    }
}
void SemiImplicitEulerIntegrator::debugForces(const ClothMesh& mesh, const std::string& stage) {
    double totalForceMagnitude = 0.0;
    int nonZeroForces = 0;
    
    for (const auto& force : mesh.forces) {
        double magnitude = force.norm();
        totalForceMagnitude += magnitude;
        if (magnitude > 1e-8) nonZeroForces++;
    }
    
    std::cout << stage << " - Total force magnitude: " << totalForceMagnitude 
              << ", Non-zero forces: " << nonZeroForces << "/" << mesh.forces.size() << std::endl;
}

void SemiImplicitEulerIntegrator::debugVelocities(const ClothMesh& mesh, const std::string& stage) {
    double totalVelocityMagnitude = 0.0;
    int nonZeroVelocities = 0;
    
    for (const auto& velocity : mesh.velocities) {
        double magnitude = velocity.norm();
        totalVelocityMagnitude += magnitude;
        if (magnitude > 1e-8) nonZeroVelocities++;
    }
    
    std::cout << stage << " - Total velocity magnitude: " << totalVelocityMagnitude
              << ", Non-zero velocities: " << nonZeroVelocities << "/" << mesh.velocities.size() << std::endl;
}

void SemiImplicitEulerIntegrator::debugPositions(const ClothMesh& mesh, const std::string& stage) {
    Eigen::Vector3d minPos = mesh.getVertex(0).position;
    Eigen::Vector3d maxPos = mesh.getVertex(0).position;
    
    for (size_t i = 1; i < mesh.getVertexCount(); ++i) {
        const auto& pos = mesh.getVertex(i).position;
        minPos = minPos.cwiseMin(pos);
        maxPos = maxPos.cwiseMax(pos);
    }
    
    std::cout << stage << " - Position bounds: min=" << minPos.transpose() 
              << ", max=" << maxPos.transpose() << std::endl;
}

void SemiImplicitEulerIntegrator::computeDebugInfo(const ClothMesh& mesh) {
    lastDebugInfo_ = DebugInfo();
    
    for (size_t i = 0; i < mesh.getVertexCount(); ++i) {
        if (mesh.pinned[i]) {
            lastDebugInfo_.pinnedVertices++;
        } else {
            lastDebugInfo_.activeVertices++;
            lastDebugInfo_.totalForce += mesh.forces[i].norm();
            lastDebugInfo_.totalVelocity += mesh.velocities[i].norm();
        }
    }
    
    lastDebugInfo_.gravityMagnitude = mesh.properties.gravity.norm();
    
    // Calculate spring force magnitude
    double springForceMag = 0.0;
    for (const auto& constraint : mesh.distanceConstraints) {
        const Eigen::Vector3d& p1 = mesh.getVertex(constraint.v0).position;
        const Eigen::Vector3d& p2 = mesh.getVertex(constraint.v1).position;
        double currentLength = (p2 - p1).norm();
        double stretch = std::abs(currentLength - constraint.restLength);
        springForceMag += mesh.properties.stiffness * stretch;
    }
    lastDebugInfo_.springForceMagnitude = springForceMag;
}

void SemiImplicitEulerIntegrator::printVertexInfo(const ClothMesh& mesh, int vertexIndex) {
    if (vertexIndex >= 0 && vertexIndex < (int)mesh.getVertexCount()) {
        std::cout << "Vertex " << vertexIndex << " detailed info:" << std::endl;
        std::cout << "  Pinned: " << (mesh.pinned[vertexIndex] ? "YES" : "NO") << std::endl;
        std::cout << "  Position: " << mesh.getVertex(vertexIndex).position.transpose() << std::endl;
        std::cout << "  Velocity: " << mesh.velocities[vertexIndex].transpose() << std::endl;
        std::cout << "  Force: " << mesh.forces[vertexIndex].transpose() << std::endl;
        std::cout << "  Force magnitude: " << mesh.forces[vertexIndex].norm() << std::endl;
        std::cout << "  Velocity magnitude: " << mesh.velocities[vertexIndex].norm() << std::endl;
    }
}

} // namespace ClothOpt