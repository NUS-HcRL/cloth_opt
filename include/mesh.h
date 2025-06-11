#ifndef MESH_H
#define MESH_H

#include <Eigen/Core>
#include <Eigen/Geometry>  // Add this for cross product
#include <vector>
#include <array>

namespace ClothOpt {

// Basic vertex structure
struct Vertex {
    Eigen::Vector3d position;
    Eigen::Vector3d normal;
    
    Vertex() : position(0, 0, 0), normal(0, 1, 0) {}
    Vertex(const Eigen::Vector3d& pos) : position(pos), normal(0, 1, 0) {}
};

// Triangle face structure
struct Triangle {
    std::array<int, 3> indices;
    Eigen::Vector3d normal;
    
    Triangle() : indices({0, 0, 0}), normal(0, 1, 0) {}
    Triangle(int i0, int i1, int i2) : indices({i0, i1, i2}), normal(0, 1, 0) {}
};

// Basic surface mesh class
class SurfaceMesh {
public:
    SurfaceMesh() = default;
    virtual ~SurfaceMesh() = default;
    
    // Basic operations
    void clear();
    void addVertex(const Eigen::Vector3d& position);
    void addTriangle(int i0, int i1, int i2);
    
    // Getters
    size_t getVertexCount() const { return vertices_.size(); }
    size_t getTriangleCount() const { return triangles_.size(); }
    const std::vector<Vertex>& getVertices() const { return vertices_; }
    const std::vector<Triangle>& getTriangles() const { return triangles_; }
    const Vertex& getVertex(size_t index) const { return vertices_[index]; }
    
    // Setters
    void setVertexPosition(size_t index, const Eigen::Vector3d& position);
    
    // Mesh analysis
    void computeNormals();
    
    // Export functions
    Eigen::MatrixXd getVertexMatrix() const;
    Eigen::MatrixXi getTriangleMatrix() const;
    
protected:
    std::vector<Vertex> vertices_;
    std::vector<Triangle> triangles_;
};

// Utility functions
namespace MeshUtils {
    Eigen::Vector3d computeTriangleNormal(const Eigen::Vector3d& v0, const Eigen::Vector3d& v1, const Eigen::Vector3d& v2);
    double computeTriangleArea(const Eigen::Vector3d& v0, const Eigen::Vector3d& v1, const Eigen::Vector3d& v2);
}

} // namespace ClothOpt

#endif // MESH_H