#include <smpl/debug/marker_utils.h>

#include <smpl/debug/colors.h>

namespace sbpl {
namespace visual {

auto MakeEmptyMarker() -> Marker {
    visual::Marker m;
    m.shape = Empty();
    return m;
}

auto MakeSphereMarker(
    double x, double y, double z,
    double radius,
    int hue,
    const std::string& frame_id,
    const std::string& ns,
    int id) -> Marker
{
    visual::Marker m;
    m.pose = Eigen::Affine3d(Eigen::Translation3d(x, y, z));
    m.shape = Sphere{ radius };
    m.color = MakeColorHSV(hue);
    m.frame_id = frame_id;
    m.ns = ns;
    m.id = id;

    return m;
}

auto MakePoseMarkers(
    const Eigen::Affine3d& pose,
    const std::string& frame_id,
    const std::string& ns,
    int id,
    bool text)
    -> std::vector<Marker>
{
    std::vector<Marker> markers(2);

    markers[0].pose = pose;
    markers[0].shape = Arrow{ 0.1, 0.015 };
    markers[0].color = Color{ 0.0f, 0.7f, 0.6f, 0.7f };
    markers[0].frame_id = frame_id;
    markers[0].ns = ns;
    markers[0].id = id;

    markers[1].pose = pose;
    markers[1].shape = Ellipse{ 0.07, 0.07, 0.1 };
    markers[1].color = Color{ };
    markers[1].frame_id = frame_id;
    markers[1].ns = ns;
    markers[1].id = id + 1;

    return markers;
}

auto MakeCubesMarker(
    const std::vector<Eigen::Vector3d>& centers,
    double size,
    const Color& color,
    const std::string& frame_id,
    const std::string& ns,
    int id) -> Marker
{
    visual::Marker marker;
    marker.shape = CubeList{ centers, size };
    marker.color = color;
    marker.frame_id = frame_id;
    marker.ns = ns;
    marker.id = id;
    return marker;
}

auto MakeCubesMarker(
    const std::vector<Eigen::Vector3d>& centers,
    double size,
    const std::vector<Color>& colors,
    const std::string& frame_id,
    const std::string& ns,
    int id) -> Marker
{
    visual::Marker marker;
    marker.shape = CubeList{ centers, size };
    marker.color = Colors{ colors };
    marker.frame_id = frame_id;
    marker.ns = ns;
    marker.id = id;
    return marker;
}

auto MakeCubesMarker(
    std::vector<Eigen::Vector3d>&& centers,
    double size,
    const Color& color,
    const std::string& frame_id,
    const std::string& ns,
    int id) -> Marker
{
    visual::Marker marker;
    marker.shape = CubeList{ std::move(centers), size };
    marker.color = color;
    marker.frame_id = frame_id;
    marker.ns = ns;
    marker.id = id;
    return marker;
}

auto MakeCubesMarker(
    std::vector<Eigen::Vector3d>&& centers,
    double size,
    std::vector<Color>&& colors,
    const std::string& frame_id,
    const std::string& ns,
    int id) -> Marker
{
    visual::Marker marker;
    marker.shape = CubeList{ std::move(centers), size };
    marker.color = Colors{ std::move(colors) };
    marker.frame_id = frame_id;
    marker.ns = ns;
    marker.id = id;
    return marker;
}

} // namespace visual
} // namespace sbpl
