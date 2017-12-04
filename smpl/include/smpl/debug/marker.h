#ifndef SMPL_MARKER_H
#define SMPL_MARKER_H

#include <cstdint>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <boost/variant.hpp>

namespace sbpl {
namespace visual {

struct Empty { };
struct Arrow { double length; double width; };
struct Cube { double length; double width; double height; };
struct Sphere { double radius; };
struct Ellipse { double axis_x; double axis_y; double axis_z; };
struct Cylinder { double radius; double height; };
struct LineList { std::vector<Eigen::Vector3d> points; };
struct LineStrip { std::vector<Eigen::Vector3d> points; };
struct CubeList { std::vector<Eigen::Vector3d> points; double size; };
struct PointList { std::vector<Eigen::Vector3d> points; };
struct SphereList { std::vector<Eigen::Vector3d> points; double radius; };
struct TriangleList { std::vector<Eigen::Vector3d> vertices; };
struct BillboardText { std::string text; };
struct MeshResource { std::string uri; Eigen::Vector3d scale; };

using Shape = boost::variant<
    Empty,
    Arrow,
    Cube,
    Sphere,
    Ellipse,
    Cylinder,
    LineList,
    LineStrip,
    CubeList,
    SphereList,
    PointList,
    BillboardText,
    MeshResource,
    TriangleList>;

enum ShapeType {
    SHAPE_EMPTY = 0,
    SHAPE_ARROW,
    SHAPE_CUBE,
    SHAPE_SPHERE,
    SHAPE_ELLIPSE,
    SHAPE_CYLINDER,
    SHAPE_LINE_LIST,
    SHAPE_LINE_STRIP,
    SHAPE_CUBE_LIST,
    SHAPE_SPHERE_LIST,
    SHAPE_POINT_LIST,
    SHAPE_BILLBOARD_TEXT,
    SHAPE_MESH_RESOURCE,
    SHAPE_TRIANGLE_LIST,
    SHAPE_COUNT
};

inline ShapeType type(const Shape& shape) {
    return ShapeType(shape.which());
}

inline auto to_cstring(ShapeType type) -> const char* {
    switch (type) {
    case SHAPE_EMPTY:           return "EMPTY";
    case SHAPE_ARROW:           return "ARROW";
    case SHAPE_CUBE:            return "CUBE";
    case SHAPE_SPHERE:          return "SPHERE";
    case SHAPE_ELLIPSE:         return "ELLIPSE";
    case SHAPE_CYLINDER:        return "CYLINDER";
    case SHAPE_LINE_LIST:       return "LINE_LIST";
    case SHAPE_LINE_STRIP:      return "LINE_STRIP";
    case SHAPE_CUBE_LIST:       return "CUBE_LIST";
    case SHAPE_SPHERE_LIST:     return "SPHERE_LIST";
    case SHAPE_POINT_LIST:      return "POINTS";
    case SHAPE_BILLBOARD_TEXT:  return "TEXT_VIEW_FACING";
    case SHAPE_MESH_RESOURCE:   return "MESH_RESOURCE";
    case SHAPE_TRIANGLE_LIST:   return "TRIANGLE_LIST";
    default:                    return "";
    }
}

struct Color {
    float r;
    float g;
    float b;
    float a;
};

using Colors = boost::variant<Color, std::vector<Color>>;

struct Marker
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    enum Action : std::int8_t {
        ACTION_ADD = 0,
        ACTION_MODIFY,
        ACTION_DELETE,
        ACTION_DELETE_ALL
    };

//    Eigen::AffineCompact3d pose;                     // 96
//    Eigen::AffineCompact3f pose;                     // 48
    struct Pose {                                   // 64, 32 for float
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        Eigen::Vector3d position;
        Eigen::Quaterniond orientation;

        Pose() : position(), orientation() { }

        Pose(const Eigen::Affine3d& T) :
            position(T.translation()),
            orientation(T.rotation())
        { }

        Pose& operator=(const Eigen::Affine3d& T) {
            position = T.translation();
            orientation = T.rotation();
            return *this;
        }
    } pose;

    enum Flags {
        FRAME_LOCKED                = 1 << 0,
        MESH_USE_EMBEDDED_MATERIALS = 1 << 1,
    };

    Shape shape;                                    // 40
    Colors color;
    std::string frame_id;                           // 32
    std::string ns;                                 // 32
    double lifetime;                                // 8
    int id;                                         // 4
    Action action;                                  // 1
    std::uint8_t flags;                             // 1

    Marker() :
        pose(),
        shape(),
        frame_id(),
        ns(),
        color(),
        lifetime(0.0),
        id(0),
        action(ACTION_ADD),
        flags(0)
    { }
};

} // namespace visual
} // namespace sbpl

#endif
