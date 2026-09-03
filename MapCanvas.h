#pragma once
#include <d3d11.h>
#include <vector>

namespace MapCanvas {

    struct Waypoint {
        float raw_x;
        float raw_y;
        int lon_sec;
        int lat_sec;
    };

    extern bool show_minute_grid;
    extern bool show_second_grid;
    const std::vector<Waypoint>& GetWaypoints();

    bool LoadTextureFromFile(ID3D11Device* d3dDevice, const char* filename, ID3D11ShaderResourceView** out_srv, int* out_width, int* out_height, unsigned char** out_cpu_data);
    void Initialize(ID3D11Device* d3dDevice);
    void RenderInteractiveMap();
    void ExportMissionFCode();
    void RenderControlPanelUI();
    void SaveSession();
    void LoadSession();

}
