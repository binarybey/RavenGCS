#define IMGUI_DEFINE_MATH_OPERATORS
#include <windows.h>
#include "imgui.h"
#include "imgui_internal.h"
#include "MapCanvas.h"
#include <cmath>
#include <d3d11.h>
#include "stb_image.h"
#include <vector>
#include <thread>
#include <atomic>
#include <string>
#include <fstream>
#include <commdlg.h>

namespace MapCanvas {

    bool show_minute_grid = true;
    bool show_second_grid = true;
    std::vector<Waypoint> waypoints;

    const std::vector<Waypoint>& GetWaypoints() { return waypoints; }

    static ID3D11ShaderResourceView* texMap[2][19] = { nullptr };

    static std::atomic<int> loaded_chunks = 0;
    static std::atomic<bool> map_loaded = false;
    static HANDLE hMapFile = NULL;
    static float* elevation_data = nullptr; // 32-bit float array!
    static const int TILE_W = 3600;
    static const int TILE_H = 10800;

    static int mode_threshold = 15;
    static int tile_offset = 3;
    static int step_grouping = 3;

    // Virtual Camera State
    static float zoom = 0.012f;
    static ImVec2 scroll_pos = ImVec2(0.0f, 0.0f);

    // Waypoint State
    static bool has_waypoint = false;
    static float wp_raw_x = 0.0f;
    static float wp_raw_y = 0.0f;
    static int wp_lon_sec = 0;
    static int wp_lat_sec = 0;

    // Safety Lock State
    bool precisionLockMet = false;

    // --- The Texture Loader ---
    bool LoadTextureFromFile(ID3D11Device* d3dDevice, const char* filename, ID3D11ShaderResourceView** out_srv, int* out_width, int* out_height, unsigned char** out_cpu_data) {
        int image_width = 0;
        int image_height = 0;
        unsigned char* image_data = stbi_load(filename, &image_width, &image_height, NULL, 4);
        if (image_data == NULL) return false;

        // 2. Setup the DirectX 11 Texture description
        D3D11_TEXTURE2D_DESC desc;
        ZeroMemory(&desc, sizeof(desc));
        desc.Width = image_width;
        desc.Height = image_height;
        desc.MipLevels = 1;
        desc.ArraySize = 1;
        desc.Format = DXGI_FORMAT_R8G8B8A8_UNORM;
        desc.SampleDesc.Count = 1;
        desc.Usage = D3D11_USAGE_DEFAULT;
        desc.BindFlags = D3D11_BIND_SHADER_RESOURCE;
        desc.CPUAccessFlags = 0;

        // 3. Upload the pixel data to the GPU
        ID3D11Texture2D* pTexture = NULL;
        D3D11_SUBRESOURCE_DATA subResource;
        subResource.pSysMem = image_data;
        subResource.SysMemPitch = desc.Width * 4;
        subResource.SysMemSlicePitch = 0;

        HRESULT hr = d3dDevice->CreateTexture2D(&desc, &subResource, &pTexture);
        if (FAILED(hr) || pTexture == nullptr) {
            // If it failed (likely due to image being too large), free RAM and abort cleanly
            stbi_image_free(image_data);
            return false;
        }

        // 4. Create the Shader Resource View (The pointer ImGui needs)
        d3dDevice->CreateShaderResourceView(pTexture, NULL, out_srv);
        pTexture->Release();

        *out_cpu_data = image_data;

        *out_width = image_width;
        *out_height = image_height;
        return true;
    }

    void LoadMapTexturesAsync(ID3D11Device* d3dDevice) {
        for (int r = 0; r < 2; r++) {
            for (int c = 0; c < 19; c++) {
                char filename[256];
                // Formats as "chunk_N_01.png" or "chunk_S_19.png"
                snprintf(filename, sizeof(filename), "map_chunks_19x2\\chunk_%c_%02d.png", r == 0 ? 'N' : 'S', c + 1);

                int w, h;
                unsigned char* dummy_cpu_ram = nullptr;

                LoadTextureFromFile(d3dDevice, filename, &texMap[r][c], &w, &h, &dummy_cpu_ram);

                // We immediately trash the CPU RAM here because we will read 
                // the perfect elevation data from terrain_data.bin later!
                if (dummy_cpu_ram) {
                    stbi_image_free(dummy_cpu_ram);
                }

                loaded_chunks++;
            }
        }
        map_loaded = true;
    }

    void LoadElevationData() {
        HANDLE hFile = CreateFileA("terrain_data.bin", GENERIC_READ, FILE_SHARE_READ, NULL, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL);
        if (hFile != INVALID_HANDLE_VALUE) {
            // Map the massive 5.5GB file into virtual memory instantly
            hMapFile = CreateFileMappingA(hFile, NULL, PAGE_READONLY, 0, 0, NULL);
            if (hMapFile != NULL) {
                elevation_data = (float*)MapViewOfFile(hMapFile, FILE_MAP_READ, 0, 0, 0);
            }
            CloseHandle(hFile);
        }
    }

    // --- Initialization (Called once from main.cpp) ---
    void Initialize(ID3D11Device* d3dDevice) {
        LoadSession();
        LoadElevationData(); // Maps instantly, no thread needed!
        // Detach the thread so the UI can launch instantly while it loads
        std::thread loader(LoadMapTexturesAsync, d3dDevice);
        loader.detach();
    }

    // --- The Render Engine (Called every frame from AppUI) ---
    void RenderInteractiveMap() {
        // 1. Safety check for textures
        if (!map_loaded) {
            float window_width = ImGui::GetWindowSize().x;
            float window_height = ImGui::GetWindowSize().y;

            // Format the text and calculate its exact pixel width
            char text_buf[128];
            snprintf(text_buf, sizeof(text_buf), "Parsing High-Resolution Topography (%d/38 Chunks)...", loaded_chunks.load());
            float text_width = ImGui::CalcTextSize(text_buf).x;
            float bar_width = 400.0f;

            // Push down to the vertical center
            ImGui::SetCursorPosY((window_height * 0.5f) - 30.0f);

            // Center the text
            ImGui::SetCursorPosX((window_width - text_width) * 0.5f);
            ImGui::TextUnformatted(text_buf);

            // Center the progress bar
            ImGui::SetCursorPosX((window_width - bar_width) * 0.5f);
            ImGui::ProgressBar((float)loaded_chunks / 38.0f, ImVec2(bar_width, 30.0f));

            return;
        }

        ImGuiIO& io = ImGui::GetIO();

        if (io.KeyCtrl && ImGui::IsKeyPressed(ImGuiKey_M, false)) {
            show_minute_grid = !show_minute_grid;
            SaveSession();
        }
        if (io.KeyCtrl && ImGui::IsKeyPressed(ImGuiKey_S, false)) {
            show_second_grid = !show_second_grid;
            SaveSession();
        }

        ImDrawList* draw_list = ImGui::GetWindowDrawList();

        // 2. Define the Canvas boundaries
        ImVec2 canvas_p0 = ImGui::GetCursorScreenPos();
        ImVec2 canvas_sz = ImGui::GetContentRegionAvail();
        if (canvas_sz.x < 50.0f) canvas_sz.x = 50.0f;
        if (canvas_sz.y < 50.0f) canvas_sz.y = 50.0f;
        ImVec2 canvas_p1 = ImVec2(canvas_p0.x + canvas_sz.x, canvas_p0.y + canvas_sz.y);

        // 3. Invisible Button to catch Mouse Events
        ImGui::InvisibleButton("canvas", canvas_sz, ImGuiButtonFlags_MouseButtonLeft | ImGuiButtonFlags_MouseButtonRight);
        bool is_hovered = ImGui::IsItemHovered();
        bool is_active = ImGui::IsItemActive();

        if (is_hovered && ImGui::IsMouseDragging(ImGuiMouseButton_Middle)) {
            scroll_pos.x += io.MouseDelta.x / zoom;
            scroll_pos.y += io.MouseDelta.y / zoom;
        }

        // 5. Camera Zooming
        if (is_hovered && io.MouseWheel != 0.0f) {
            ImVec2 mouse_in_canvas = ImVec2(io.MousePos.x - canvas_p0.x, io.MousePos.y - canvas_p0.y);

            // Calculate exactly where the mouse is pointing IN THE RAW IMAGE
            float origin_x = (mouse_in_canvas.x / zoom) - scroll_pos.x;
            float origin_y = (mouse_in_canvas.y / zoom) - scroll_pos.y;

            // Apply zoom
            float zoom_factor = powf(1.1f, io.MouseWheel);
            float new_zoom = zoom * zoom_factor;

            // Clamp it (Max increased to 25x so the grid can physically render)
            if (new_zoom < 0.01f) new_zoom = 0.01f;
            if (new_zoom > 30.0f) new_zoom = 30.0f;

            // Re-anchor the camera so the raw image data stays glued under the cursor
            scroll_pos.x = (mouse_in_canvas.x / new_zoom) - origin_x;
            scroll_pos.y = (mouse_in_canvas.y / new_zoom) - origin_y;

            zoom = new_zoom;
        }



        // --- RENDERING PHASE ---
        draw_list->PushClipRect(canvas_p0, canvas_p1, true);

        // The absolute top-left corner of the entire 68,400 x 21,600 map
        ImVec2 map_origin = ImVec2(canvas_p0.x + scroll_pos.x * zoom, canvas_p0.y + scroll_pos.y * zoom);

        // 6. Draw the 38 Chunks with Frustum Culling
        for (int r = 0; r < 2; r++) {
            for (int c = 0; c < 19; c++) {
                if (texMap[r][c]) {
                    ImVec2 p0 = ImVec2(map_origin.x + (c * TILE_W) * zoom, map_origin.y + (r * TILE_H) * zoom);
                    ImVec2 p1 = ImVec2(p0.x + TILE_W * zoom, p0.y + TILE_H * zoom);

                    // Culling: Only draw this chunk if it physically touches the visible canvas
                    if (p1.x > canvas_p0.x && p0.x < canvas_p1.x &&
                        p1.y > canvas_p0.y && p0.y < canvas_p1.y) {
                        draw_list->AddImage((ImTextureID)texMap[r][c], p0, p1);
                    }
                }
            }
        }

        // 6. Waypoint Selection (Left Click = Add, Right Click = Remove)
        if (is_hovered) {
            // Left Click: Drop waypoint snapped to the arc-second tile
            if (zoom > 15.0f && ImGui::IsMouseClicked(ImGuiMouseButton_Left)) {
                float click_x = (io.MousePos.x - map_origin.x) / zoom;
                float click_y = (io.MousePos.y - map_origin.y) / zoom;

                if (click_x >= 0 && click_x < 68400 && click_y >= 0 && click_y < 21600) {
                    int tile_x = (int)floorf(click_x);
                    int tile_y = (int)floorf(click_y);

                    // Prevent dropping multiple waypoints on the exact same tile
                    bool exists = false;
                    for (const auto& wp : waypoints) {
                        if ((int)wp.raw_x == tile_x && (int)wp.raw_y == tile_y) {
                            exists = true;
                            break;
                        }
                    }

                    if (!exists) {
                        Waypoint wp;
                        wp.raw_x = (float)tile_x; // Lock perfectly to the grid integer
                        wp.raw_y = (float)tile_y;
                        wp.lon_sec = (26 * 3600) + tile_x;
                        wp.lat_sec = (42 * 3600) - tile_y;
                        waypoints.push_back(wp);
                        SaveSession();
                    }
                }
            }

            // Right Click: Remove waypoint
            if (ImGui::IsMouseClicked(ImGuiMouseButton_Right)) {
                float mouse_click_x = (io.MousePos.x - map_origin.x) / zoom;
                float mouse_click_y = (io.MousePos.y - map_origin.y) / zoom;
                int click_tile_x = (int)floorf(mouse_click_x);
                int click_tile_y = (int)floorf(mouse_click_y);

                for (auto it = waypoints.begin(); it != waypoints.end(); ) {
                    bool hit = false;

                    if (zoom > 15.0f) {
                        // Zoomed In: Clean tile-based collision detection
                        if ((int)it->raw_x == click_tile_x && (int)it->raw_y == click_tile_y) hit = true;
                    }
                    else {
                        // Zoomed Out: Radius-based collision for pins
                        float center_x = map_origin.x + (it->raw_x + 0.5f) * zoom;
                        float center_y = map_origin.y + (it->raw_y + 0.5f) * zoom;
                        float dx = io.MousePos.x - center_x;
                        float dy = io.MousePos.y - center_y;
                        if ((dx * dx + dy * dy) < (15.0f * 15.0f)) hit = true;
                    }

                    if (hit) {
                        it = waypoints.erase(it);
                        SaveSession();
                        break; // Only erase one per click
                    }
                    else {
                        ++it;
                    }
                }
            }
            
        }

        // 8. The Real-World Geographic Grid Logic
        float degStep = 3600.0f * zoom;
        float minStep = 60.0f * zoom;
        float secStep = 1.0f * zoom;

        // --- DRAW DEGREES (Labeled) ---
        if (degStep > 20.0f) {
            ImU32 degColor = IM_COL32(255, 255, 255, 180); // Bright white
            ImU32 textColor = IM_COL32(255, 255, 255, 255);

            int first_x = (int)((canvas_p0.x - map_origin.x) / degStep);
            int last_x = (int)((canvas_p1.x - map_origin.x) / degStep);
            for (int px = first_x; px <= last_x; px++) {
                float x = map_origin.x + px * degStep;
                if (x >= canvas_p0.x && x <= canvas_p1.x) {
                    draw_list->AddLine(ImVec2(x, canvas_p0.y), ImVec2(x, canvas_p1.y), degColor, 2.0f);

                    // Add Longitude Label (E 26 to E 45)
                    char label[16];
                    snprintf(label, sizeof(label), "%d\xC2\xB0 E", 26 + px);
                    draw_list->AddText(ImVec2(x + 5, canvas_p0.y + 5), textColor, label);
                }
            }

            int first_y = (int)((canvas_p0.y - map_origin.y) / degStep);
            int last_y = (int)((canvas_p1.y - map_origin.y) / degStep);
            for (int py = first_y; py <= last_y; py++) {
                float y = map_origin.y + py * degStep;
                if (y >= canvas_p0.y && y <= canvas_p1.y) {
                    draw_list->AddLine(ImVec2(canvas_p0.x, y), ImVec2(canvas_p1.x, y), degColor, 2.0f);

                    // Add Latitude Label (N 42 down to N 39)
                    char label[16];
                    snprintf(label, sizeof(label), "%d\xC2\xB0 N", 42 - py);
                    draw_list->AddText(ImVec2(canvas_p0.x + 5, y + 5), textColor, label);
                }
            }
        }// (0, 64, 128, 95)

        // --- DRAW ARC-MINUTES ---
        if (show_minute_grid && minStep > 10.0f) {
            ImU32 minColor = IM_COL32(239, 63, 255, 127);
            ImU32 textColor = IM_COL32(127, 127, 127, 220);

            float center_x = canvas_p0.x + canvas_sz.x * 0.5f;
            float center_y = canvas_p0.y + canvas_sz.y * 0.5f;

            int label_step = 1;
            if (minStep < 60.0f) label_step = 5;
            if (minStep < 20.0f) label_step = 15;

            // Longitude (Vertical Lines)
            int first_x = (int)((canvas_p0.x - map_origin.x) / minStep);
            int last_x = (int)((canvas_p1.x - map_origin.x) / minStep);
            for (int px = first_x; px <= last_x; px++) {
                if (px % 60 == 0) continue;
                float x = map_origin.x + px * minStep;
                if (x >= canvas_p0.x && x <= canvas_p1.x) {
                    draw_list->AddLine(ImVec2(x, canvas_p0.y), ImVec2(x, canvas_p1.y), minColor, 1.5f);

                    if (px % label_step == 0) {
                        int total_lon_min = (26 * 60) + px;
                        char label[32];
                        snprintf(label, sizeof(label), "E %d\xC2\xB0%02d'", total_lon_min / 60, total_lon_min % 60);
                        ImVec2 text_size = ImGui::CalcTextSize(label);
                        draw_list->AddText(ImVec2(x - (text_size.x * 0.5f), center_y - text_size.y - 4.0f), textColor, label);
                    }
                }
            }

            // Latitude (Horizontal Lines)
            int first_y = (int)((canvas_p0.y - map_origin.y) / minStep);
            int last_y = (int)((canvas_p1.y - map_origin.y) / minStep);
            for (int py = first_y; py <= last_y; py++) {
                if (py % 60 == 0) continue;
                float y = map_origin.y + py * minStep;
                if (y >= canvas_p0.y && y <= canvas_p1.y) {
                    draw_list->AddLine(ImVec2(canvas_p0.x, y), ImVec2(canvas_p1.x, y), minColor, 1.5f);

                    if (py % label_step == 0) {
                        int total_lat_min = (42 * 60) - py;
                        char label[32];
                        snprintf(label, sizeof(label), "N %d\xC2\xB0%02d'", total_lat_min / 60, total_lat_min % 60);
                        ImVec2 text_size = ImGui::CalcTextSize(label);
                        draw_list->AddText(ImVec2(center_x + 6.0f, y - (text_size.y * 0.5f)), textColor, label);
                    }
                }
            }
        }

        // --- DRAW ARC-SECONDS & HOVER LOGIC ---
        static float hover_timer = 0.0f;

        if (secStep > 15.0f) {
            precisionLockMet = true; // Still allows waypoint placement even if lines are hidden
            ImGui::SetMouseCursor(ImGuiMouseCursor_Hand);

            // ONLY draw the visual grid if the toggle is checked
            if (show_second_grid) {
                ImU32 secColor = IM_COL32(192, 96, 16, 95);

                int first_x = (int)((canvas_p0.x - map_origin.x) / secStep);
                int last_x = (int)((canvas_p1.x - map_origin.x) / secStep);
                for (int px = first_x; px <= last_x; px++) {
                    if (px % 60 == 0) continue;
                    float x = map_origin.x + px * secStep;
                    if (x >= canvas_p0.x && x <= canvas_p1.x)
                        draw_list->AddLine(ImVec2(x, canvas_p0.y), ImVec2(x, canvas_p1.y), secColor, 1.0f);
                }

                int first_y = (int)((canvas_p0.y - map_origin.y) / secStep);
                int last_y = (int)((canvas_p1.y - map_origin.y) / secStep);
                for (int py = first_y; py <= last_y; py++) {
                    if (py % 60 == 0) continue;
                    float y = map_origin.y + py * secStep;
                    if (y >= canvas_p0.y && y <= canvas_p1.y)
                        draw_list->AddLine(ImVec2(canvas_p0.x, y), ImVec2(canvas_p1.x, y), secColor, 1.0f);
                }
            }

            // Always keep tooltip active when zoomed in
            if (is_hovered) {
                if (io.MouseDelta.x == 0.0f && io.MouseDelta.y == 0.0f) hover_timer += io.DeltaTime;
                else hover_timer = 0.0f;

                if (hover_timer > 0.2f) {
                    int raw_x = (int)floorf((io.MousePos.x - map_origin.x) / zoom);
                    int raw_y = (int)floorf((io.MousePos.y - map_origin.y) / zoom);

                    // UPDATED BOUNDS
                    if (raw_x >= 0 && raw_x < 68400 && raw_y >= 0 && raw_y < 21600) {
                        int total_lon_sec = (26 * 3600) + raw_x;
                        int total_lat_sec = (42 * 3600) - raw_y;

                        // --- Extract Elevation ---
                        float elevation = 0.0f;
                        if (elevation_data != nullptr) {
                            // Cast to size_t to prevent 32-bit integer overflow!
                            size_t pixel_index = (size_t)raw_y * 68400 + (size_t)raw_x;
                            elevation = elevation_data[pixel_index];
                        }

                        ImGui::BeginTooltip();
                        ImGui::Text("%d\xC2\xB0 %d' %d\" N\n%d\xC2\xB0 %d' %d\" E\nElev: %.1f m",
                            total_lat_sec / 3600, (total_lat_sec % 3600) / 60, total_lat_sec % 60,
                            total_lon_sec / 3600, (total_lon_sec % 3600) / 60, total_lon_sec % 60,
                            elevation);
                        ImGui::EndTooltip();
                    }
                }
            }
            else {
                hover_timer = 0.0f;
            }
        }
        else {
            precisionLockMet = false;
            hover_timer = 0.0f;
        }

        // --- DYNAMIC ELEVATION BAND SHADING ---
        // Activate when the arc-minute grid is visible
        if (minStep > 10.0f && elevation_data != nullptr) {

            float meters_per_pixel = 30.0f / zoom;

            // Using your updated 50m minimum modifier!
            int tier_interval = max(50, (int)round(meters_per_pixel / 10.0f) * 50);

            // CPU Optimization: Step size for macro views
            int map_step = (zoom < 1.0f) ? (int)ceilf(1.0f / zoom) : 1;

            int first_x = (int)floorf((canvas_p0.x - map_origin.x) / secStep);
            int last_x = (int)ceilf((canvas_p1.x - map_origin.x) / secStep);
            int first_y = (int)floorf((canvas_p0.y - map_origin.y) / secStep);
            int last_y = (int)ceilf((canvas_p1.y - map_origin.y) / secStep);

            // Clamp to map boundaries
            first_x = max(0, first_x);
            last_x = min(68399, last_x);
            first_y = max(0, first_y);
            last_y = min(21599, last_y);

            // Align the starting grid with the map_step to prevent the bands from "wiggling" when panning
            first_x -= (first_x % map_step);
            first_y -= (first_y % map_step);

            // Safety valve to ensure CPU does not choke
            if ((last_x - first_x) / map_step < 3000 && (last_y - first_y) / map_step < 3000) {
                for (int y = first_y; y < last_y; y += map_step) {

                    // Snap Y coordinates perfectly to the screen pixel grid
                    float screen_y1 = floorf(map_origin.y + (float)y * secStep);
                    float screen_y2 = floorf(map_origin.y + (float)(y + map_step) * secStep);

                    for (int x = first_x; x < last_x; x += map_step) {

                        // Snap X coordinates perfectly to the screen pixel grid
                        float screen_x1 = floorf(map_origin.x + (float)x * secStep);
                        float screen_x2 = floorf(map_origin.x + (float)(x + map_step) * secStep);

                        size_t idx = (size_t)y * 68400 + (size_t)x;
                        float elev = elevation_data[idx];

                        int tier = (int)(elev / tier_interval);

                        if (tier % 2 == 0) {
                            // Draw edge-to-edge with zero sub-pixel overlap!
                            draw_list->AddRectFilled(
                                ImVec2(screen_x1, screen_y1),
                                ImVec2(screen_x2, screen_y2),
                                IM_COL32(0, 0, 0, 35)
                            );
                        }
                    }
                }
            }
        }

        // --- HUD LEGEND ---
        char legend_buf[64];
        float meters_per_pixel_hud = 30.0f / zoom;
        int active_interval = max(50, (int)round(meters_per_pixel_hud / 10.0f) * 50);

        snprintf(legend_buf, sizeof(legend_buf), "Contour Interval: %d m", active_interval);
        ImVec2 legend_pos = ImVec2(canvas_p0.x + 15.0f, canvas_p1.y - 35.0f);

        draw_list->AddRectFilled(legend_pos, ImVec2(legend_pos.x + 180.0f, legend_pos.y + 25.0f), IM_COL32(30, 30, 30, 200), 4.0f);
        draw_list->AddRect(legend_pos, ImVec2(legend_pos.x + 180.0f, legend_pos.y + 25.0f), IM_COL32(100, 100, 100, 255), 4.0f);
        draw_list->AddText(ImVec2(legend_pos.x + 10.0f, legend_pos.y + 4.0f), IM_COL32(255, 255, 255, 255), legend_buf);

        // --- DRAW WAYPOINTS & FLIGHT PATH ---
        ImU32 outlineColor = IM_COL32(255, 255, 255, 200); // White
        ImU32 pathColor = IM_COL32(255, 255, 0, 150); // Yellow Path
        ImU32 markerColor = IM_COL32(255, 50, 50, 255); // Solid red
        ImU32 highlightColor = IM_COL32(255, 255, 0, 100); // Semi-transparent yellow

        float blink_time = (float)ImGui::GetTime();
        int blink_alpha = (int)(120.0f + 60.0f * sinf(blink_time * 6.0f));
        ImU32 blinkingTileColor = IM_COL32(255, 50, 50, blink_alpha);

        // ==========================================
        // PASS 1: DRAW PATHS & HIGHLIGHTS (BOTTOM LAYER)
        // ==========================================
        for (size_t i = 1; i < waypoints.size(); i++) {

            float center_x = map_origin.x + (waypoints[i].raw_x + 0.5f) * zoom;
            float center_y = map_origin.y + (waypoints[i].raw_y + 0.5f) * zoom;

            int start_x = (int)waypoints[i - 1].raw_x;
            int start_y = (int)waypoints[i - 1].raw_y;
            int end_x = (int)waypoints[i].raw_x;
            int end_y = (int)waypoints[i].raw_y;

            int dx = end_x - start_x;
            int dy = end_y - start_y;
            int step_x = (dx > 0) ? 1 : ((dx < 0) ? -1 : 0);
            int step_y = (dy > 0) ? 1 : ((dy < 0) ? -1 : 0);

            bool is_x_parent = abs(dx) >= abs(dy);
            double abs_dx = fabs((double)dx), abs_dy = fabs((double)dy);
            double tDeltaX = (abs_dx == 0) ? 1e30 : 1.0 / abs_dx, tDeltaY = (abs_dy == 0) ? 1e30 : 1.0 / abs_dy;
            double tMaxX = (abs_dx == 0) ? 1e30 : 0.5 / abs_dx, tMaxY = (abs_dy == 0) ? 1e30 : 0.5 / abs_dy;

            int current_x = start_x, current_y = start_y;
            int total_crossings = (int)(abs_dx + abs_dy);

            // --- UI Grouping Logic ---
            struct UITile { int x, y; };
            struct UIFlightStep {
                std::vector<UITile> tiles;
                float sum_elev;
                int tile_count;
            };
            std::vector<UIFlightStep> ui_steps;

            int prev_parent = is_x_parent ? current_x : current_y;
            UIFlightStep current_step;
            current_step.sum_elev = 0.0f;
            current_step.tile_count = 0;

            auto add_ui_tile = [&](int cx, int cy) {
                int current_parent = is_x_parent ? cx : cy;
                if (current_parent != prev_parent) {
                    ui_steps.push_back(current_step);
                    current_step = UIFlightStep();
                    current_step.sum_elev = 0.0f;
                    current_step.tile_count = 0;
                    prev_parent = current_parent;
                }
                float elev = (elevation_data != nullptr) ? elevation_data[(size_t)cy * 68400 + (size_t)cx] : 0.0f;
                current_step.sum_elev += elev;
                current_step.tile_count++;
                current_step.tiles.push_back({ cx, cy });
                };

            add_ui_tile(current_x, current_y);

            for (int step = 0; step < total_crossings; step++) {
                if (current_x == end_x && current_y == end_y) break;
                if (fabs(tMaxX - tMaxY) < 1e-8) {
                    current_x += step_x; current_y += step_y;
                    tMaxX += tDeltaX; tMaxY += tDeltaY;
                    step++; add_ui_tile(current_x, current_y);
                }
                else if (tMaxX < tMaxY) {
                    current_x += step_x; tMaxX += tDeltaX; add_ui_tile(current_x, current_y);
                }
                else {
                    current_y += step_y; tMaxY += tDeltaY; add_ui_tile(current_x, current_y);
                }
            }
            ui_steps.push_back(current_step);

            // --- 1. Mode Calculation (Grouped Step Strict Derivative) ---
            std::vector<int> raw_modes(ui_steps.size(), 0);
            int current_mode = 0;
            float prev_group_elev = 0.0f;

            // Iterate through the path in chunks defined by the slider
            for (size_t i = 0; i < ui_steps.size(); i += step_grouping) {

                float group_sum_elev = 0.0f;
                int group_tile_count = 0;

                // Prevent array out-of-bounds on the final chunk
                size_t end_idx = (i + step_grouping < ui_steps.size()) ? i + step_grouping : ui_steps.size();

                // Aggregate elevations and actual tile counts for the entire group
                for (size_t s = i; s < end_idx; s++) {
                    group_sum_elev += ui_steps[s].sum_elev;
                    group_tile_count += ui_steps[s].tile_count;
                }

                // Average with respect to TOTAL TILE COUNT of the group
                float current_group_elev = group_sum_elev / (float)group_tile_count;

                // Initialize the previous elevation on the very first group
                if (i == 0) {
                    prev_group_elev = current_group_elev;
                }

                float delta_z = current_group_elev - prev_group_elev;

                // Apply your strict entry and half-threshold exit rules
                if (current_mode == 0) { // Flat
                    if (delta_z > (float)mode_threshold) {
                        current_mode = 1; // Start Climb
                    }
                    else if (delta_z < -(float)mode_threshold) {
                        current_mode = 2; // Start Glide
                    }
                }
                else if (current_mode == 1) { // Climb
                    if (delta_z < ((float)mode_threshold * 0.5f)) {
                        current_mode = 0; // Terrain slacked off: Terminate climb, go flat
                    }
                }
                else if (current_mode == 2) { // Glide
                    if (delta_z > -((float)mode_threshold * 0.5f)) {
                        current_mode = 0; // Terrain leveled out: Terminate glide, go flat
                    }
                }

                // Assign the calculated mode to ALL steps inside this specific chunk
                for (size_t s = i; s < end_idx; s++) {
                    raw_modes[s] = current_mode;
                }

                // Carry the current group's average forward for the next derivative check
                prev_group_elev = current_group_elev;
            }

            // --- 2. Dynamic Feed-Forward Offset ---
            std::vector<int> final_modes = raw_modes;
            for (int s = 1; s < (int)ui_steps.size(); s++) {
                if (raw_modes[s] != raw_modes[s - 1]) {
                    int upcoming_mode = raw_modes[s];
                    // Use the dynamic UI slider variable instead of hardcoded 3
                    for (int b = 1; b <= tile_offset; b++) {
                        if (s - b >= 0) final_modes[s - b] = upcoming_mode;
                    }
                }
            }

            // --- Draw the Colored Tiles ---
            if (zoom > 15.0f) {
                for (size_t s = 0; s < ui_steps.size(); s++) {
                    ImU32 tile_color;
                    if (final_modes[s] == 1)      tile_color = IM_COL32(255, 50, 50, 100);   // Red
                    else if (final_modes[s] == 2) tile_color = IM_COL32(50, 150, 255, 100);  // Blue
                    else                          tile_color = IM_COL32(50, 255, 50, 100);   // Green

                    for (const auto& t : ui_steps[s].tiles) {
                        float screen_x = map_origin.x + t.x * zoom;
                        float screen_y = map_origin.y + t.y * zoom;
                        draw_list->AddRectFilled(ImVec2(screen_x, screen_y), ImVec2(screen_x + zoom, screen_y + zoom), tile_color);
                    }
                }
            }

            // --- Draw the Dynamic Multi-Colored Line ---
            float start_cx = map_origin.x + (start_x + 0.5f) * zoom;
            float start_cy = map_origin.y + (start_y + 0.5f) * zoom;
            float end_cx = map_origin.x + (end_x + 0.5f) * zoom;
            float end_cy = map_origin.y + (end_y + 0.5f) * zoom;

            for (size_t s = 0; s < ui_steps.size() - 1; s++) {
                ImU32 line_color;
                if (final_modes[s] == 1)      line_color = IM_COL32(255, 50, 50, 255);   // Solid Red
                else if (final_modes[s] == 2) line_color = IM_COL32(50, 150, 255, 255);  // Solid Blue
                else                          line_color = IM_COL32(50, 255, 50, 255);   // Solid Green

                // Interpolate exact points along the perfect center vector
                float t1 = (float)s / (float)(ui_steps.size() - 1);
                float t2 = (float)(s + 1) / (float)(ui_steps.size() - 1);

                float x1 = start_cx + t1 * (end_cx - start_cx);
                float y1 = start_cy + t1 * (end_cy - start_cy);
                float x2 = start_cx + t2 * (end_cx - start_cx);
                float y2 = start_cy + t2 * (end_cy - start_cy);

                draw_list->AddLine(ImVec2(x1, y1), ImVec2(x2, y2), line_color, 3.0f);
            }

        }
        // ==========================================
        // PASS 2: DRAW WAYPOINT MARKERS (TOP LAYER)
        // ==========================================
        for (size_t i = 0; i < waypoints.size(); i++) {

            float center_x = map_origin.x + (waypoints[i].raw_x + 0.5f) * zoom;
            float center_y = map_origin.y + (waypoints[i].raw_y + 0.5f) * zoom;

            if (center_x >= canvas_p0.x - 50 && center_x <= canvas_p1.x + 50 &&
                center_y >= canvas_p0.y - 50 && center_y <= canvas_p1.y + 50) {

                char wp_num[16];
                snprintf(wp_num, sizeof(wp_num), "WP %zu", i + 1);

                if (zoom > 15.0f) {
                    float p0_x = map_origin.x + waypoints[i].raw_x * zoom;
                    float p0_y = map_origin.y + waypoints[i].raw_y * zoom;

                    draw_list->AddRectFilled(ImVec2(p0_x, p0_y), ImVec2(p0_x + zoom, p0_y + zoom), blinkingTileColor);
                    draw_list->AddRect(ImVec2(p0_x, p0_y), ImVec2(p0_x + zoom, p0_y + zoom), markerColor, 0.0f, 0, 1.5f);

                    ImVec2 text_sz = ImGui::CalcTextSize(wp_num);
                    draw_list->AddText(ImVec2(center_x - text_sz.x * 0.5f, center_y - text_sz.y * 0.5f), outlineColor, wp_num);
                }
                else {
                    draw_list->AddCircleFilled(ImVec2(center_x, center_y), 4.0f, markerColor);
                    draw_list->AddCircle(ImVec2(center_x, center_y), 12.0f, outlineColor, 0, 1.5f);
                    draw_list->AddLine(ImVec2(center_x - 20, center_y), ImVec2(center_x + 20, center_y), markerColor, 2.0f);
                    draw_list->AddLine(ImVec2(center_x, center_y - 20), ImVec2(center_x, center_y + 20), markerColor, 2.0f);
                    draw_list->AddText(ImVec2(center_x + 15, center_y - 25), outlineColor, wp_num);
                }
            }
        }
        draw_list->PopClipRect();
    }

    void RenderControlPanelUI() {
        ImGui::Separator();
        ImGui::Text("Mission Generation:");

        // Trigger SaveSession() whenever a slider is changed
        if (ImGui::SliderInt("Mode Threshold (m)", &mode_threshold, 5, 100, "%d m")) SaveSession();
        if (ImGui::SliderInt("Tile Offset", &tile_offset, 0, 20, "%d tiles")) SaveSession();
        if (ImGui::SliderInt("Step Grouping", &step_grouping, 1, 20, "%d steps")) SaveSession();

        // Safety Lock: Only activate if a path exists AND the async texture loader is finished
        if (waypoints.size() >= 2 && map_loaded.load()) {
            if (ImGui::Button("Export F-Code Mission")) {
                ExportMissionFCode();
            }
        }
        else {
            ImGui::BeginDisabled();
            ImGui::Button("Export F-Code Mission");
            ImGui::EndDisabled();

            // Add a visual indicator if the button is disabled due to loading
            if (!map_loaded.load()) {
                ImGui::SameLine();
                ImGui::TextColored(ImVec4(1.0f, 0.6f, 0.0f, 1.0f), " (Parsing Terrain...)");
            }
        }

        ImGui::Spacing();
        ImGui::Text("Selected Waypoints:");

        // Create a scrollable list box for the waypoints
        if (ImGui::BeginChild("WPList", ImVec2(0, 150), true, ImGuiWindowFlags_HorizontalScrollbar)) {
            for (size_t i = 0; i < waypoints.size(); i++) {
                int lon = (26 * 3600) + (int)waypoints[i].raw_x;
                int lat = (42 * 3600) - (int)waypoints[i].raw_y;

                float elev = 0.0f;
                if (elevation_data != nullptr) {
                    elev = elevation_data[(size_t)waypoints[i].raw_y * 68400 + (size_t)waypoints[i].raw_x];
                }

                // Format: WP 1: 41°08'44"N 33°35'21"E | 942.2m
                ImGui::Text("WP %zu:  %d\xC2\xB0%02d'%02d\"N  %d\xC2\xB0%02d'%02d\"E  |  %.1fm",
                    i + 1,
                    lat / 3600, (lat % 3600) / 60, lat % 60,
                    lon / 3600, (lon % 3600) / 60, lon % 60,
                    elev);
            }
        }
        ImGui::EndChild();

        ImGui::Spacing();
    }

    void SaveSession() {
        std::ofstream file("mission_cache.dat", std::ios::binary);
        if (!file) return;

        // Save UI States
        file.write((char*)&show_minute_grid, sizeof(bool));
        file.write((char*)&show_second_grid, sizeof(bool));
        file.write((char*)&mode_threshold, sizeof(int));
        file.write((char*)&tile_offset, sizeof(int));
        file.write((char*)&step_grouping, sizeof(int));

        // Save Waypoints
        size_t wp_count = waypoints.size();
        file.write((char*)&wp_count, sizeof(size_t));
        file.write((char*)waypoints.data(), wp_count * sizeof(Waypoint));
        file.close();
    }

    void LoadSession() {
        std::ifstream file("mission_cache.dat", std::ios::binary);
        if (!file) return;

        file.read((char*)&show_minute_grid, sizeof(bool));
        file.read((char*)&show_second_grid, sizeof(bool));
        file.read((char*)&mode_threshold, sizeof(int));
        file.read((char*)&tile_offset, sizeof(int));
        file.read((char*)&step_grouping, sizeof(int));

        size_t wp_count;
        file.read((char*)&wp_count, sizeof(size_t));
        waypoints.resize(wp_count);
        file.read((char*)waypoints.data(), wp_count * sizeof(Waypoint));
        file.close();
    }

    void ExportMissionFCode() {
        if (waypoints.size() < 2) return;

        // Format: DDMMSSNDDMMSSE-DDMMSSNDDMMSSE-A.txt
        auto format_wp = [](const Waypoint& wp) {
            int lon = (26 * 3600) + (int)wp.raw_x;
            int lat = (42 * 3600) - (int)wp.raw_y;
            char buf[32];
            snprintf(buf, sizeof(buf), "%02d%02d%02dN%02d%02d%02dE",
                lat / 3600, (lat % 3600) / 60, lat % 60,
                lon / 3600, (lon % 3600) / 60, lon % 60);
            return std::string(buf);
            };

        std::string default_name = format_wp(waypoints.front()) + "-" +
            format_wp(waypoints.back()) + "-" +
            std::to_string(waypoints.size()) + ".txt";

        // Windows Native Save Dialog
        OPENFILENAMEA ofn;
        char szFile[260];
        strncpy(szFile, default_name.c_str(), sizeof(szFile));
        ZeroMemory(&ofn, sizeof(ofn));
        ofn.lStructSize = sizeof(ofn);
        ofn.hwndOwner = NULL;
        ofn.lpstrFile = szFile;
        ofn.nMaxFile = sizeof(szFile);
        ofn.lpstrFilter = "Text Files\0*.txt\0All Files\0*.*\0";
        ofn.nFilterIndex = 1;
        ofn.Flags = OFN_PATHMUSTEXIST | OFN_OVERWRITEPROMPT;

        if (GetSaveFileNameA(&ofn) == TRUE) {
            std::ofstream file(ofn.lpstrFile); // Use the selected directory!
            if (!file.is_open()) return;

        file << "%\n";
        file << "F43; Z50;\n";
        
        for (size_t i = 1; i < waypoints.size(); i++) {

            int start_x = (int)waypoints[i - 1].raw_x;
            int start_y = (int)waypoints[i - 1].raw_y;
            int end_x = (int)waypoints[i].raw_x;
            int end_y = (int)waypoints[i].raw_y;

            int dx = end_x - start_x;
            int dy = end_y - start_y;
            int step_x = (dx > 0) ? 1 : ((dx < 0) ? -1 : 0);
            int step_y = (dy > 0) ? 1 : ((dy < 0) ? -1 : 0);

            bool is_x_parent = abs(dx) >= abs(dy);

            double abs_dx = fabs((double)dx);
            double abs_dy = fabs((double)dy);
            double tDeltaX = (abs_dx == 0) ? 1e30 : 1.0 / abs_dx;
            double tDeltaY = (abs_dy == 0) ? 1e30 : 1.0 / abs_dy;
            double tMaxX = (abs_dx == 0) ? 1e30 : 0.5 / abs_dx;
            double tMaxY = (abs_dy == 0) ? 1e30 : 0.5 / abs_dy;

            int current_x = start_x;
            int current_y = start_y;
            int total_crossings = (int)(abs_dx + abs_dy);

            // --- PARENT AXIS GROUPING LOGIC ---
            struct FlightStep {
                int map_x, map_y;
                float sum_elev;
                int tile_count;
            };
            std::vector<FlightStep> steps;

            FlightStep current_step = { current_x, current_y, 0.0f, 0 };
            int prev_parent = is_x_parent ? current_x : current_y;

            // Lambda to handle 1-tile or 2-tile grouping dynamically
            auto add_tile_to_step = [&](int cx, int cy) {
                int current_parent = is_x_parent ? cx : cy;
                if (current_parent != prev_parent) {
                    steps.push_back(current_step);
                    current_step = { cx, cy, 0.0f, 0 };
                    prev_parent = current_parent;
                }

                float elev = 0.0f;
                if (elevation_data != nullptr) {
                    elev = elevation_data[(size_t)cy * 68400 + (size_t)cx]; //[cite: 2]
                }
                current_step.sum_elev += elev;
                current_step.tile_count++;
                };

            // Add starting tile
            add_tile_to_step(current_x, current_y);

            // Supercover Raycast
            for (int step = 0; step < total_crossings; step++) {
                if (current_x == end_x && current_y == end_y) break;

                if (fabs(tMaxX - tMaxY) < 1e-8) {
                    current_x += step_x;
                    current_y += step_y;
                    tMaxX += tDeltaX;
                    tMaxY += tDeltaY;
                    step++;
                    add_tile_to_step(current_x, current_y);
                }
                else if (tMaxX < tMaxY) {
                    current_x += step_x;
                    tMaxX += tDeltaX;
                    add_tile_to_step(current_x, current_y);
                }
                else {
                    current_y += step_y;
                    tMaxY += tDeltaY;
                    add_tile_to_step(current_x, current_y);
                }
            }
            steps.push_back(current_step); // Push the final grouped step

            // --- 1. Mode Calculation (Grouped Step Strict Derivative) ---
            std::vector<int> raw_modes(steps.size(), 0);
            int current_mode = 0;
            float prev_group_elev = 0.0f;

            // Iterate through the path in chunks defined by the slider
            for (size_t i = 0; i < steps.size(); i += step_grouping) {

                float group_sum_elev = 0.0f;
                int group_tile_count = 0;

                // Prevent array out-of-bounds on the final chunk
                size_t end_idx = (i + step_grouping < steps.size()) ? i + step_grouping : steps.size();

                // Aggregate elevations and actual tile counts for the entire group
                for (size_t s = i; s < end_idx; s++) {
                    group_sum_elev += steps[s].sum_elev;
                    group_tile_count += steps[s].tile_count;
                }

                // Average with respect to TOTAL TILE COUNT of the group
                float current_group_elev = group_sum_elev / (float)group_tile_count;

                // Initialize the previous elevation on the very first group
                if (i == 0) {
                    prev_group_elev = current_group_elev;
                }

                float delta_z = current_group_elev - prev_group_elev;

                // Apply your strict entry and half-threshold exit rules
                if (current_mode == 0) { // Flat
                    if (delta_z > (float)mode_threshold) {
                        current_mode = 1; // Start Climb
                    }
                    else if (delta_z < -(float)mode_threshold) {
                        current_mode = 2; // Start Glide
                    }
                }
                else if (current_mode == 1) { // Climb
                    if (delta_z < ((float)mode_threshold * 0.5f)) {
                        current_mode = 0; // Terrain slacked off: Terminate climb, go flat
                    }
                }
                else if (current_mode == 2) { // Glide
                    if (delta_z > -((float)mode_threshold * 0.5f)) {
                        current_mode = 0; // Terrain leveled out: Terminate glide, go flat
                    }
                }

                // Assign the calculated mode to ALL steps inside this specific chunk
                for (size_t s = i; s < end_idx; s++) {
                    raw_modes[s] = current_mode;
                }

                // Carry the current group's average forward for the next derivative check
                prev_group_elev = current_group_elev;
            }

            // --- 2. Dynamic Feed-Forward Offset ---
            std::vector<int> final_modes = raw_modes;
            for (int s = 1; s < (int)steps.size(); s++) {
                if (raw_modes[s] != raw_modes[s - 1]) {
                    int upcoming_mode = raw_modes[s];
                    // Use the dynamic UI slider variable instead of hardcoded 3
                    for (int b = 1; b <= tile_offset; b++) {
                        if (s - b >= 0) final_modes[s - b] = upcoming_mode;
                    }
                }
            }

            // --- 3. F-CODE STRING GENERATION
            current_mode = -1;
            for (size_t s = 0; s < steps.size(); s++) {

                int next_mode = final_modes[s];

                int total_lon_sec = (26 * 3600) + steps[s].map_x;
                int total_lat_sec = (42 * 3600) - steps[s].map_y;
                int lon_d = total_lon_sec / 3600, lon_m = (total_lon_sec % 3600) / 60, lon_s = total_lon_sec % 60;
                int lat_d = total_lat_sec / 3600, lat_m = (total_lat_sec % 3600) / 60, lat_s = total_lat_sec % 60;

                char line[128];
                if (i == 1 && s == 0) {
                    snprintf(line, sizeof(line), "F91; X%02d%02d%02d; Y%02d%02d%02d; N40;\n", lon_d, lon_m, lon_s, lat_d, lat_m, lat_s);
                    current_mode = next_mode;
                }
                else if (i == waypoints.size() - 1 && s == steps.size() - 1) {
                    snprintf(line, sizeof(line), "F91; X%02d%02d%02d; Y%02d%02d%02d; N39;\n", lon_d, lon_m, lon_s, lat_d, lat_m, lat_s);
                }
                else {
                    if (next_mode != current_mode) {
                        snprintf(line, sizeof(line), "F91; X%02d%02d%02d; Y%02d%02d%02d; N%02d;\n", lon_d, lon_m, lon_s, lat_d, lat_m, lat_s, next_mode);
                        current_mode = next_mode;
                    }
                    else {
                        snprintf(line, sizeof(line), "F91; X%02d%02d%02d; Y%02d%02d%02d;\n", lon_d, lon_m, lon_s, lat_d, lat_m, lat_s);
                    }
                }
                file << line;
            }
        }

        file << "%\n";
        file.close();
    }

    }
}
