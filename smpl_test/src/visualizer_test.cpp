// system includes
#include <stdio.h>

// system includes
#include <smpl/debug/visualize.h>

class TextVisualizer : public sbpl::visual::VisualizerBase
{
public:

    void visualize(
        sbpl::visual::Level level,
        const visualization_msgs::MarkerArray& markers)
    {
        for (const visualization_msgs::Marker& marker : markers.markers) {
            switch (level) {
            case sbpl::visual::Level::Debug:
                printf("[DEBUG] ");
                break;
            case sbpl::visual::Level::Info:
                printf("[INFO]  ");
                break;
            case sbpl::visual::Level::Warn:
                printf("[WARN]  ");
                break;
            case sbpl::visual::Level::Error:
                printf("[ERROR] ");
                break;
            case sbpl::visual::Level::Fatal:
                printf("[FATAL] ");
                break;
            }
            printf("[%s] ", marker.ns.c_str());
            switch (marker.type) {
            case visualization_msgs::Marker::ARROW:
                printArrow(marker);
                break;
            case visualization_msgs::Marker::CUBE:
                printCube(marker);
                break;
            case visualization_msgs::Marker::CUBE_LIST:
                printCubeList(marker);
                break;
            case visualization_msgs::Marker::CYLINDER:
                printCylinder(marker);
                break;
            case visualization_msgs::Marker::LINE_LIST:
                printLineList(marker);
                break;
            case visualization_msgs::Marker::LINE_STRIP:
                printLineStrip(marker);
                break;
            case visualization_msgs::Marker::MESH_RESOURCE:
                printMeshResource(marker);
                break;
            case visualization_msgs::Marker::POINTS:
                printPoints(marker);
                break;
            case visualization_msgs::Marker::SPHERE:
                printSphere(marker);
                break;
            case visualization_msgs::Marker::SPHERE_LIST:
                printSphereList(marker);
                break;
            case visualization_msgs::Marker::TEXT_VIEW_FACING:
                printTextViewFacing(marker);
                break;
            case visualization_msgs::Marker::TRIANGLE_LIST:
                printTriangleList(marker);
                break;
            }
            printf("\n");
        }
    }

private:

    void printArrow(const visualization_msgs::Marker& m)
    {
        printf("arrow");
    }

    void printCube(const visualization_msgs::Marker& m)
    {
        printf("cube");
    }

    void printCubeList(const visualization_msgs::Marker& m)
    {
        printf("cube list");
    }

    void printCylinder(const visualization_msgs::Marker& m)
    {
        printf("cylinder");
    }

    void printLineList(const visualization_msgs::Marker& m)
    {
        printf("line list");
    }

    void printLineStrip(const visualization_msgs::Marker& m)
    {
        printf("line strip");
    }

    void printMeshResource(const visualization_msgs::Marker& m)
    {
        printf("mesh resource");
    }

    void printPoints(const visualization_msgs::Marker& m)
    {
        printf("points");
    }

    void printSphere(const visualization_msgs::Marker& m)
    {
        printf("sphere");
    }

    void printSphereList(const visualization_msgs::Marker& m)
    {
        printf("sphere list");
    }

    void printTextViewFacing(const visualization_msgs::Marker& m)
    {
        printf("text view facing");
    }

    void printTriangleList(const visualization_msgs::Marker& m)
    {
        printf("triangle list");
    }
};

int main(int argc, char* argv[])
{
    TextVisualizer visual;
    sbpl::visual::set_visualizer(&visual);

    visualization_msgs::MarkerArray ma;
    visualization_msgs::Marker m;
    m.ns = "text";
    m.type = visualization_msgs::Marker::CUBE;
    ma.markers.push_back(m);
    m.type = visualization_msgs::Marker::SPHERE;
    ma.markers.push_back(m);
    m.type = visualization_msgs::Marker::POINTS;
    ma.markers.push_back(m);

    printf("---vanilla---\n");

    SV_SHOW_DEBUG(ma);
    SV_SHOW_INFO(ma);
    SV_SHOW_WARN(ma);
    SV_SHOW_ERROR(ma);
    SV_SHOW_FATAL(ma);

    printf("---cond---\n");

    SV_SHOW_DEBUG_COND(true, ma);
    SV_SHOW_INFO_COND(true, ma);
    SV_SHOW_WARN_COND(true, ma);
    SV_SHOW_ERROR_COND(true, ma);
    SV_SHOW_FATAL_COND(true, ma);

    SV_SHOW_DEBUG_COND(false, ma);
    SV_SHOW_INFO_COND(false, ma);
    SV_SHOW_WARN_COND(false, ma);
    SV_SHOW_ERROR_COND(false, ma);
    SV_SHOW_FATAL_COND(false, ma);

    printf("---once---\n");

    for (int i = 0; i < 10; ++i) {
        SV_SHOW_DEBUG_ONCE(ma);
        SV_SHOW_INFO_ONCE(ma);
        SV_SHOW_WARN_ONCE(ma);
        SV_SHOW_ERROR_ONCE(ma);
        SV_SHOW_FATAL_ONCE(ma);
    }

    printf("---throttle---\n");

    auto start = std::chrono::high_resolution_clock::now();
    while (std::chrono::high_resolution_clock::now() < start + std::chrono::seconds(10)) {
        SV_SHOW_DEBUG_THROTTLE(1, ma);
        SV_SHOW_INFO_THROTTLE(1, ma);
        SV_SHOW_WARN_THROTTLE(1, ma);
        SV_SHOW_ERROR_THROTTLE(1, ma);
        SV_SHOW_FATAL_THROTTLE(1, ma);
    }

    return 0;
}
