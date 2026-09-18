#pragma once
#include <d3d11.h>
#include <vector>

namespace MapCanvas {

    enum class WpType { Standard, Arc3Pt, ArcCenterAnchor, ArcCenterEnd };

    struct Waypoint {
        float raw_x, raw_y;
        int lon_sec, lat_sec;

        // Metadata for the State Machine
        WpType type = WpType::Standard;
        int group_id = -1;       // E.g., '1' for the first arc created (3C1)
        int group_index = 0;     // 1, 2, or 3 for the sequence
        float center_angle = 0.0f; // For multi-turn loitering (e.g., 720.0f)
    };

    extern bool show_minute_grid;
    extern bool show_second_grid;
    const std::vector<Waypoint>& GetWaypoints();

    bool LoadTextureFromFile(ID3D11Device* d3dDevice, const char* filename, ID3D11ShaderResourceView** out_srv, int* out_width, int* out_height, unsigned char** out_cpu_data);
    void Initialize(ID3D11Device* d3dDevice);
    void Shutdown();
    void RenderInteractiveMap();
    void ExportMissionFCode();
    void RenderControlPanelUI();
    void SaveSession();
    void LoadSession();
    std::vector<ImVec2> GenerateFlightPath();

}
