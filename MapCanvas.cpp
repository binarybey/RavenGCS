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

    enum class AppState {
        IDLE,             // Default Line mode
        WAITING_3PT_1,    // Waiting for WP 1 (Start)
        WAITING_3PT_2,    // Waiting for WP 2 (Mid/Pass-through)
        WAITING_3PT_3,    // Waiting for WP 3 (End)
        WAITING_CENTER_1, // Waiting for Start Anchor
        WAITING_CENTER_2,  // Waiting for True Center
        ADJUSTING_CENTER_ARC // Real-time slider state
    };

    static AppState current_state = AppState::IDLE;
    static int current_arc_group = 1;
    static std::vector<Waypoint> temp_arc_points;

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
    static int path_geometry_mode = 0; // 0: Straight, 1: 3-Point Arc, 2: Center Arc

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
            snprintf(text_buf, sizeof(text_buf), "Parsing Topography Elements (%d/38)", loaded_chunks.load());
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

        // --- CAMERA CLAMPING ---
        // Prevent the center of the screen from leaving the map boundaries
        float half_w = (canvas_sz.x * 0.5f) / zoom;
        float half_h = (canvas_sz.y * 0.5f) / zoom;

        if (scroll_pos.x > half_w) scroll_pos.x = half_w;
        if (scroll_pos.x < -68400.0f + half_w) scroll_pos.x = -68400.0f + half_w;

        if (scroll_pos.y > half_h) scroll_pos.y = half_h;
        if (scroll_pos.y < -21600.0f + half_h) scroll_pos.y = -21600.0f + half_h;

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
        // Handle Escape Key: Cancel ongoing operations
        if (ImGui::IsKeyPressed(ImGuiKey_Escape) && current_state != AppState::IDLE) {
            current_state = AppState::IDLE;
            temp_arc_points.clear();
        }
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

                        if (current_state == AppState::IDLE) {
                            // Standard Line Mode
                            wp.type = WpType::Standard;
                            waypoints.push_back(wp);
                            SaveSession();
                        }
                        else if (current_state >= AppState::WAITING_3PT_1 && current_state <= AppState::WAITING_3PT_3) {
                            // 3-Point Arc Mode
                            wp.type = WpType::Arc3Pt;
                            wp.group_id = current_arc_group;

                            if (current_state == AppState::WAITING_3PT_1) {
                                int max_id = 0;
                                for (const auto& w : waypoints) {
                                    if (w.group_id > max_id) max_id = w.group_id;
                                }
                                current_arc_group = max_id + 1;
                                wp.group_id = current_arc_group; 
                                wp.group_index = 1;
                                temp_arc_points.push_back(wp);
                                current_state = AppState::WAITING_3PT_2;
                            }
                            else if (current_state == AppState::WAITING_3PT_2) {
                                wp.group_index = 2;
                                temp_arc_points.push_back(wp);
                                current_state = AppState::WAITING_3PT_3;
                            }
                            else if (current_state == AppState::WAITING_3PT_3) {
                                wp.group_index = 3;
                                temp_arc_points.push_back(wp);

                                // Finalize the arc: move from temp buffer to main array
                                for (const auto& p : temp_arc_points) waypoints.push_back(p);

                                current_arc_group++;
                                temp_arc_points.clear();
                                current_state = AppState::IDLE; // Snap back to standard line mode
                                SaveSession();
                            }
                        }
                        else if (current_state == AppState::WAITING_CENTER_1) {
                            int max_id = 0;
                            for (const auto& w : waypoints) {
                                if (w.group_id > max_id) max_id = w.group_id;
                            }
                            current_arc_group = max_id + 1;

                            wp.type = WpType::ArcCenterAnchor;
                            wp.group_id = current_arc_group;
                            wp.group_index = 1;
                            temp_arc_points.push_back(wp);
                            current_state = AppState::WAITING_CENTER_2;
                        }
                        else if (current_state == AppState::WAITING_CENTER_2) {
                            wp.type = WpType::ArcCenterEnd;
                            wp.group_id = current_arc_group;
                            wp.group_index = 2;
                            wp.center_angle = 90.0f; // Default live starting angle
                            temp_arc_points.push_back(wp);

                            current_state = AppState::ADJUSTING_CENTER_ARC; // Trigger the slider
                        }
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
                        if ((int)it->raw_x == click_tile_x && (int)it->raw_y == click_tile_y) hit = true;
                    }
                    else {
                        float center_x = map_origin.x + (it->raw_x + 0.5f) * zoom;
                        float center_y = map_origin.y + (it->raw_y + 0.5f) * zoom;
                        float dx = io.MousePos.x - center_x;
                        float dy = io.MousePos.y - center_y;
                        if ((dx * dx + dy * dy) < (15.0f * 15.0f)) hit = true;
                    }

                    if (hit) {
                        // --- SMART DELETION LOGIC ---
                        if (it->type != WpType::Standard && it->group_id != -1) {
                            // It's an Arc: Nuke the entire connected group
                            int target_id = it->group_id;
                            waypoints.erase(
                                std::remove_if(waypoints.begin(), waypoints.end(),
                                    [target_id](const Waypoint& wp) { return wp.group_id == target_id; }),
                                waypoints.end()
                            );
                        }
                        else {
                            // It's a Standard Line: Surgically remove just this one point
                            waypoints.erase(it);
                        }

                        SaveSession();
                        break;
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
        }

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
        std::vector<ImVec2> flight_path = GenerateFlightPath();

        if (flight_path.size() > 1) {
            // --- UI Grouping Logic ---
            struct UITile { int x, y; };
            struct UIFlightStep {
                std::vector<UITile> tiles;
                float sum_elev;
                int tile_count;
                size_t path_idx;
                float t_start;
                float t_end;
            };
            std::vector<UIFlightStep> ui_steps;

            UIFlightStep current_step;
            current_step.sum_elev = 0.0f;
            current_step.tile_count = 0;
            int prev_parent = -1;

            auto add_ui_tile = [&](int cx, int cy, bool is_x_parent, size_t path_idx, float t) {
                int current_parent = is_x_parent ? cx : cy;

                if (prev_parent == -1) {
                    prev_parent = current_parent;
                    current_step.path_idx = path_idx;
                    current_step.t_start = t;
                }

                if (current_parent != prev_parent) {
                    // Close the old step bridging perfectly to 't'
                    current_step.t_end = t;
                    if (current_step.tile_count > 0) ui_steps.push_back(current_step);

                    // Open the new step starting perfectly at 't'
                    current_step = UIFlightStep();
                    current_step.sum_elev = 0.0f;
                    current_step.tile_count = 0;
                    current_step.path_idx = path_idx;
                    current_step.t_start = t;
                    prev_parent = current_parent;
                }

                float elev = (elevation_data != nullptr) ? elevation_data[(size_t)cy * 68400 + (size_t)cx] : 0.0f;
                current_step.sum_elev += elev;
                current_step.tile_count++;
                current_step.tiles.push_back({ cx, cy });

                // Keep stretching the end of the current block
                current_step.t_end = t;
                };

            for (size_t i = 1; i < flight_path.size(); i++) {
                float fx1 = flight_path[i - 1].x;
                float fy1 = flight_path[i - 1].y;
                float fx2 = flight_path[i].x;
                float fy2 = flight_path[i].y;

                int start_x = (int)fx1;
                int start_y = (int)fy1;
                int end_x = (int)fx2;
                int end_y = (int)fy2;

                if (start_x == end_x && start_y == end_y) {
                    add_ui_tile(start_x, start_y, true, i, 0.0f);
                }
                else {
                    int dx = end_x - start_x;
                    int dy = end_y - start_y;
                    int step_x = (dx > 0) ? 1 : ((dx < 0) ? -1 : 0);
                    int step_y = (dy > 0) ? 1 : ((dy < 0) ? -1 : 0);
                    bool is_x_parent = std::abs(dx) >= std::abs(dy);

                    double abs_dx = std::abs((double)dx), abs_dy = std::abs((double)dy);
                    double tDeltaX = (abs_dx == 0) ? 1e30 : 1.0 / abs_dx;
                    double tDeltaY = (abs_dy == 0) ? 1e30 : 1.0 / abs_dy;
                    double tMaxX = (abs_dx == 0) ? 1e30 : 0.5 / abs_dx;
                    double tMaxY = (abs_dy == 0) ? 1e30 : 0.5 / abs_dy;

                    int current_x = start_x, current_y = start_y;
                    int total_crossings = (int)(abs_dx + abs_dy);

                    add_ui_tile(current_x, current_y, is_x_parent, i, 0.0f);

                    for (int step = 0; step < total_crossings; step++) {
                        if (current_x == end_x && current_y == end_y) break;
                        if (std::abs(tMaxX - tMaxY) < 1e-8) {
                            current_x += step_x; current_y += step_y;
                            tMaxX += tDeltaX; tMaxY += tDeltaY;
                            step++;
                        }
                        else if (tMaxX < tMaxY) {
                            current_x += step_x; tMaxX += tDeltaX;
                        }
                        else {
                            current_y += step_y; tMaxY += tDeltaY;
                        }

                        float t = (float)(step + 1) / (float)total_crossings;
                        add_ui_tile(current_x, current_y, is_x_parent, i, t);
                    }
                }

                // Force close the step at the exact end of the segment to prevent cross-segment contamination
                if (current_step.tile_count > 0) {
                    current_step.t_end = 1.0f;
                    ui_steps.push_back(current_step);
                    current_step = UIFlightStep();
                    current_step.sum_elev = 0.0f;
                    current_step.tile_count = 0;
                    prev_parent = -1;
                }
            }

            // --- 1. Mode Calculation ---
            std::vector<int> raw_modes(ui_steps.size(), 0);
            int current_mode = 0;
            float prev_group_elev = 0.0f;
            for (size_t i = 0; i < ui_steps.size(); i += step_grouping) {
                float group_sum_elev = 0.0f;
                int group_tile_count = 0;
                size_t end_idx = (i + step_grouping < ui_steps.size()) ? i + step_grouping : ui_steps.size();
                for (size_t s = i; s < end_idx; s++) {
                    group_sum_elev += ui_steps[s].sum_elev;
                    group_tile_count += ui_steps[s].tile_count;
                }
                float current_group_elev = group_sum_elev / (float)group_tile_count;
                if (i == 0) prev_group_elev = current_group_elev;
                float delta_z = current_group_elev - prev_group_elev;

                if (current_mode == 0) {
                    if (delta_z > (float)mode_threshold) current_mode = 1;
                    else if (delta_z < -(float)mode_threshold) current_mode = 2;
                }
                else if (current_mode == 1) {
                    if (delta_z < ((float)mode_threshold * 0.5f)) current_mode = 0;
                }
                else if (current_mode == 2) {
                    if (delta_z > -((float)mode_threshold * 0.5f)) current_mode = 0;
                }
                for (size_t s = i; s < end_idx; s++) raw_modes[s] = current_mode;
                prev_group_elev = current_group_elev;
            }

            // --- 2. Dynamic Feed-Forward Offset ---
            std::vector<int> final_modes = raw_modes;
            for (int s = 1; s < (int)ui_steps.size(); s++) {
                if (raw_modes[s] != raw_modes[s - 1]) {
                    int upcoming_mode = raw_modes[s];
                    for (int b = 1; b <= tile_offset; b++) {
                        if (s - b >= 0) final_modes[s - b] = upcoming_mode;
                    }
                }
            }

            // --- Draw the Colored Tiles ---
            if (zoom > 15.0f) {
                int last_drawn_x = -1;
                int last_drawn_y = -1;

                for (size_t s = 0; s < ui_steps.size(); s++) {
                    ImU32 tile_color = IM_COL32(50, 255, 50, 100);
                    if (final_modes[s] == 1) tile_color = IM_COL32(255, 50, 50, 100);
                    else if (final_modes[s] == 2) tile_color = IM_COL32(50, 150, 255, 100);

                    for (const auto& t : ui_steps[s].tiles) {
                        // Deduplication: Prevent alpha-stacking on segment boundaries
                        if (t.x == last_drawn_x && t.y == last_drawn_y) continue;

                        float screen_x = map_origin.x + t.x * zoom;
                        float screen_y = map_origin.y + t.y * zoom;
                        draw_list->AddRectFilled(ImVec2(screen_x, screen_y), ImVec2(screen_x + zoom, screen_y + zoom), tile_color);

                        last_drawn_x = t.x;
                        last_drawn_y = t.y;
                    }
                }
            }

            // --- 3. Draw the Exact Mathematical Line ---
            for (size_t s = 0; s < ui_steps.size(); s++) {
                ImU32 segment_color = IM_COL32(50, 255, 50, 255);
                if (!final_modes.empty()) {
                    if (final_modes[s] == 1) segment_color = IM_COL32(255, 50, 50, 255);
                    else if (final_modes[s] == 2) segment_color = IM_COL32(50, 150, 255, 255);
                }

                size_t p_idx = ui_steps[s].path_idx;
                if (p_idx == 0 || p_idx >= flight_path.size()) continue;

                float fx1 = flight_path[p_idx - 1].x;
                float fy1 = flight_path[p_idx - 1].y;
                float fx2 = flight_path[p_idx].x;
                float fy2 = flight_path[p_idx].y;

                float ex1 = map_origin.x + (fx1 + ui_steps[s].t_start * (fx2 - fx1) + 0.5f) * zoom;
                float ey1 = map_origin.y + (fy1 + ui_steps[s].t_start * (fy2 - fy1) + 0.5f) * zoom;
                float ex2 = map_origin.x + (fx1 + ui_steps[s].t_end * (fx2 - fx1) + 0.5f) * zoom;
                float ey2 = map_origin.y + (fy1 + ui_steps[s].t_end * (fy2 - fy1) + 0.5f) * zoom;

                draw_list->AddLine(ImVec2(ex1, ey1), ImVec2(ex2, ey2), segment_color, 3.0f);
            }
        }
        // ==========================================
        // PASS 2: DRAW WAYPOINT MARKERS (TOP LAYER)
        // ==========================================
        auto draw_marker = [&](float r_x, float r_y, const char* label, ImU32 color, bool is_center_dot) {
            float center_x = map_origin.x + (r_x + 0.5f) * zoom;
            float center_y = map_origin.y + (r_y + 0.5f) * zoom;

            if (center_x >= canvas_p0.x - 50 && center_x <= canvas_p1.x + 50 &&
                center_y >= canvas_p0.y - 50 && center_y <= canvas_p1.y + 50) {

                if (zoom > 15.0f && !is_center_dot) {
                    float p0_x = map_origin.x + r_x * zoom;
                    float p0_y = map_origin.y + r_y * zoom;
                    draw_list->AddRectFilled(ImVec2(p0_x, p0_y), ImVec2(p0_x + zoom, p0_y + zoom), color & 0x7FFFFFFF);
                    draw_list->AddRect(ImVec2(p0_x, p0_y), ImVec2(p0_x + zoom, p0_y + zoom), color, 0.0f, 0, 1.5f);
                }
                else {
                    draw_list->AddCircleFilled(ImVec2(center_x, center_y), is_center_dot ? 3.0f : 4.0f, color);
                    if (!is_center_dot) draw_list->AddCircle(ImVec2(center_x, center_y), 12.0f, outlineColor, 0, 1.5f);
                    draw_list->AddLine(ImVec2(center_x - (is_center_dot ? 8 : 20), center_y), ImVec2(center_x + (is_center_dot ? 8 : 20), center_y), color, 2.0f);
                    draw_list->AddLine(ImVec2(center_x, center_y - (is_center_dot ? 8 : 20)), ImVec2(center_x, center_y + (is_center_dot ? 8 : 20)), color, 2.0f);
                }
                ImVec2 text_sz = ImGui::CalcTextSize(label);
                draw_list->AddText(ImVec2(center_x + 10, center_y - 20), outlineColor, label);
            }
            };

        int display_wp_idx = 1;
        for (size_t i = 0; i < waypoints.size(); ) {
            if (waypoints[i].type == WpType::Standard) {
                draw_marker(waypoints[i].raw_x, waypoints[i].raw_y, ("WP " + std::to_string(display_wp_idx++)).c_str(), markerColor, false);
                i++;
            }
            else if (waypoints[i].type == WpType::Arc3Pt && i + 2 < waypoints.size()) {
                std::string pfx = "3C" + std::to_string(waypoints[i].group_id);
                draw_marker(waypoints[i].raw_x, waypoints[i].raw_y, (pfx + "_Start").c_str(), markerColor, false);

                bool is_flipped = (waypoints[i].center_angle > 0.5f);
                if (!is_flipped) {
                    draw_marker(waypoints[i + 1].raw_x, waypoints[i + 1].raw_y, (pfx + "_Mid").c_str(), markerColor, false);
                }

                draw_marker(waypoints[i + 2].raw_x, waypoints[i + 2].raw_y, (pfx + "_End").c_str(), markerColor, false);

                double x1 = waypoints[i].raw_x, y1 = waypoints[i].raw_y;
                double x2 = (double)waypoints[i + 1].raw_x - x1, y2 = (double)waypoints[i + 1].raw_y - y1;
                double x3 = (double)waypoints[i + 2].raw_x - x1, y3 = (double)waypoints[i + 2].raw_y - y1;
                double D = 2.0 * (x2 * y3 - x3 * y2);
                if (std::abs(D) > 0.1) {
                    double Xc = x1 + ((x2 * x2 + y2 * y2) * y3 - (x3 * x3 + y3 * y3) * y2) / D;
                    double Yc = y1 + ((x3 * x3 + y3 * y3) * x2 - (x2 * x2 + y2 * y2) * x3) / D;
                    draw_marker((float)Xc, (float)Yc, (pfx + "_Center").c_str(), IM_COL32(255, 165, 0, 255), true);
                }
                i += 3;
            }
            else if (waypoints[i].type == WpType::ArcCenterAnchor && i + 1 < waypoints.size()) {
                std::string pfx = "C" + std::to_string(waypoints[i].group_id);
                draw_marker(waypoints[i].raw_x, waypoints[i].raw_y, (pfx + "_Start").c_str(), markerColor, false);
                draw_marker(waypoints[i + 1].raw_x, waypoints[i + 1].raw_y, (pfx + "_Center").c_str(), IM_COL32(255, 165, 0, 255), true);

                double sx = waypoints[i].raw_x, sy = waypoints[i].raw_y;
                double cx = waypoints[i + 1].raw_x, cy = waypoints[i + 1].raw_y;
                double dx = sx - cx, dy = sy - cy;
                double R = std::sqrt(dx * dx + dy * dy);
                double end_angle = std::atan2(dy, dx) + (waypoints[i + 1].center_angle * (3.1415926535 / 180.0));
                draw_marker((float)(cx + R * std::cos(end_angle)), (float)(cy + R * std::sin(end_angle)), (pfx + "_End").c_str(), markerColor, false);

                i += 2;
            }
            else { i++; }
        }

        for (size_t i = 0; i < temp_arc_points.size(); i++) {
            draw_marker(temp_arc_points[i].raw_x, temp_arc_points[i].raw_y, "Pending", IM_COL32(255, 165, 0, 255), false);
        }
        draw_list->PopClipRect();
    }

    void RenderControlPanelUI() {
        ImGui::Separator();
        ImGui::Text("Flight Geometry:");

        if (current_state != AppState::IDLE) {
            if (ImGui::Button("Cancel to Line")) {
                current_state = AppState::IDLE;
                temp_arc_points.clear();
            }
            ImGui::SameLine();
            ImGui::BeginDisabled();
            ImGui::Button("3-Point Arc");
            ImGui::SameLine();
            ImGui::Button("Center Arc");
            ImGui::EndDisabled();

            if (current_state == AppState::ADJUSTING_CENTER_ARC && temp_arc_points.size() == 2) {
                ImGui::Separator();
                ImGui::TextColored(ImVec4(0.0f, 1.0f, 0.0f, 1.0f), "Adjusting Center Arc Angle:");

                // Live Slider & Input Box (Supports Multi-Turn 750°+)
                ImGui::SliderFloat("##sweep_slider", &temp_arc_points[1].center_angle, -1080.0f, 1080.0f, "%.1f deg");
                ImGui::InputFloat("##sweep_input", &temp_arc_points[1].center_angle, 1.0f, 15.0f, "%.1f");

                if (ImGui::Button("Commit Arc", ImVec2(120, 0))) {
                    for (const auto& p : temp_arc_points) waypoints.push_back(p);
                    current_arc_group++;
                    temp_arc_points.clear();
                    current_state = AppState::IDLE;
                    SaveSession();
                }
            }
            else {
                ImGui::TextColored(ImVec4(1.0f, 0.8f, 0.0f, 1.0f),
                    current_state <= AppState::WAITING_3PT_3 ? "Waiting for 3-Point Arc selection..." : "Waiting for Center Arc selection...");
            }
        }
        else {
            ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.2f, 0.5f, 0.8f, 1.0f));
            ImGui::Button("Straight Line");
            ImGui::PopStyleColor();
            ImGui::SameLine();
            if (ImGui::Button("3-Point Arc")) {
                current_state = AppState::WAITING_3PT_1;
                temp_arc_points.clear();
            }
            ImGui::SameLine();
            if (ImGui::Button("Center Arc")) {
                current_state = AppState::WAITING_CENTER_1;
                temp_arc_points.clear();
            }
        }

        ImGui::Separator();
        ImGui::Text("Mission Generation:");

        if (ImGui::SliderInt("Mode Threshold", &mode_threshold, 5, 100, "%d m")) SaveSession();
        if (ImGui::SliderInt("Tile Offset", &tile_offset, 0, 20, "%d tiles")) SaveSession();
        if (ImGui::SliderInt("Step Grouping", &step_grouping, 1, 20, "%d steps")) SaveSession();

        ImGui::Separator();
        if (waypoints.size() >= 2 && map_loaded.load()) {
            if (ImGui::Button("Export F-Code")) {
                ExportMissionFCode();
            }
        }
        else {
            ImGui::BeginDisabled();
            ImGui::Button("Export F-Code");
            ImGui::EndDisabled();
            if (!map_loaded.load()) {
                ImGui::SameLine();
                ImGui::TextColored(ImVec4(1.0f, 0.6f, 0.0f, 1.0f), " (Parsing Terrain...)");
            }
        }

        if (!waypoints.empty()) {
            ImGui::Separator();
            ImGui::Text("Selected Waypoints:");

            if (ImGui::BeginChild("WPList", ImVec2(0, 150), true, ImGuiWindowFlags_HorizontalScrollbar)) {
                int display_wp_idx = 1;
                for (size_t i = 0; i < waypoints.size(); ) {
                    int lon = (26 * 3600) + (int)waypoints[i].raw_x;
                    int lat = (42 * 3600) - (int)waypoints[i].raw_y;
                    float elev = (elevation_data != nullptr) ? elevation_data[(size_t)waypoints[i].raw_y * 68400 + (size_t)waypoints[i].raw_x] : 0.0f;

                    if (waypoints[i].type == WpType::Arc3Pt) {
                        double x1 = waypoints[i].raw_x, y1 = waypoints[i].raw_y;
                        double x2 = (double)waypoints[i + 1].raw_x - x1, y2 = (double)waypoints[i + 1].raw_y - y1;
                        double x3 = (double)waypoints[i + 2].raw_x - x1, y3 = (double)waypoints[i + 2].raw_y - y1;
                        double D = 2.0 * (x2 * y3 - x3 * y2);
                        double display_angle = 0.0;
                        if (std::abs(D) > 0.1) {
                            double Xc = ((x2 * x2 + y2 * y2) * y3 - (x3 * x3 + y3 * y3) * y2) / D;
                            double Yc = ((x3 * x3 + y3 * y3) * x2 - (x2 * x2 + y2 * y2) * x3) / D;
                            double start_angle = std::atan2(-Yc, -Xc);
                            double mid_angle = std::atan2(y2 - Yc, x2 - Xc);
                            double end_angle = std::atan2(y3 - Yc, x3 - Xc);
                            double angle_diff = end_angle - start_angle;
                            while (angle_diff <= -3.14159265) angle_diff += 6.2831853;
                            while (angle_diff > 3.14159265) angle_diff -= 6.2831853;
                            double mid_diff = mid_angle - start_angle;
                            while (mid_diff <= -3.14159265) mid_diff += 6.2831853;
                            while (mid_diff > 3.14159265) mid_diff -= 6.2831853;
                            bool missed_opposite = ((angle_diff > 0.0 && mid_diff < 0.0) || (angle_diff < 0.0 && mid_diff > 0.0));
                            bool missed_overshoot = ((angle_diff > 0.0 && mid_diff > 0.0 && mid_diff > angle_diff) ||
                                (angle_diff < 0.0 && mid_diff < 0.0 && mid_diff < angle_diff));
                            if (missed_opposite || missed_overshoot) angle_diff += (angle_diff > 0.0) ? -6.2831853072 : 6.2831853072;
                            if (waypoints[i].center_angle > 0.5f) angle_diff += (angle_diff > 0.0) ? -6.2831853072 : 6.2831853072;
                            display_angle = angle_diff * (180.0 / 3.1415926535);
                        }

                        ImGui::Text("3P-Arc (3C%d): %d\xC2\xB0%02d'%02d\"N  %d\xC2\xB0%02d'%02d\"E | %.1f\xC2\xB0",
                            waypoints[i].group_id, lat / 3600, (lat % 3600) / 60, lat % 60, lon / 3600, (lon % 3600) / 60, lon % 60, display_angle);
                        i += 3;
                    }
                    else if (waypoints[i].type == WpType::ArcCenterAnchor) {
                        ImGui::Text("Cent-Arc (C%d): %d\xC2\xB0%02d'%02d\"N  %d\xC2\xB0%02d'%02d\"E | %.1f\xC2\xB0",
                            waypoints[i].group_id, lat / 3600, (lat % 3600) / 60, lat % 60, lon / 3600, (lon % 3600) / 60, lon % 60, waypoints[i + 1].center_angle);
                        i += 2;
                    }
                    else {
                        ImGui::Text("WP %d:         %d\xC2\xB0%02d'%02d\"N  %d\xC2\xB0%02d'%02d\"E  |  %.1fm",
                            display_wp_idx++, lat / 3600, (lat % 3600) / 60, lat % 60, lon / 3600, (lon % 3600) / 60, lon % 60, elev);
                        i++;
                    }
                }
            }
            ImGui::EndChild();

            ImGui::Spacing();
            float total_distance = 0.0f;
            float last_x = waypoints[0].raw_x;
            float last_y = waypoints[0].raw_y;

            for (size_t i = 0; i < waypoints.size(); ) {
                if (waypoints[i].type == WpType::Standard) {
                    if (i > 0) {
                        float dist = std::sqrt(std::pow(waypoints[i].raw_x - last_x, 2) + std::pow(waypoints[i].raw_y - last_y, 2)) * 30.0f;
                        ImGui::Text("Line: %.1f m", dist);
                        total_distance += dist;
                    }
                    last_x = waypoints[i].raw_x;
                    last_y = waypoints[i].raw_y;
                    i++;
                }
                else if (waypoints[i].type == WpType::Arc3Pt && i + 2 < waypoints.size()) {
                    // 3P-Arc Straight Line Bridge Restored
                    if (i > 0) {
                        float dist = std::sqrt(std::pow(waypoints[i].raw_x - last_x, 2) + std::pow(waypoints[i].raw_y - last_y, 2)) * 30.0f;
                        if (dist > 0.1f) {
                            ImGui::Text("Line: %.1f m", dist);
                            total_distance += dist;
                        }
                    }

                    double x1 = waypoints[i].raw_x, y1 = waypoints[i].raw_y;
                    double x2 = (double)waypoints[i + 1].raw_x - x1, y2 = (double)waypoints[i + 1].raw_y - y1;
                    double x3 = (double)waypoints[i + 2].raw_x - x1, y3 = (double)waypoints[i + 2].raw_y - y1;
                    double D = 2.0 * (x2 * y3 - x3 * y2);
                    float arc_len = 0.0f;

                    if (std::abs(D) > 0.1) {
                        double Xc = ((x2 * x2 + y2 * y2) * y3 - (x3 * x3 + y3 * y3) * y2) / D;
                        double Yc = ((x3 * x3 + y3 * y3) * x2 - (x2 * x2 + y2 * y2) * x3) / D;
                        double R = std::sqrt(Xc * Xc + Yc * Yc);
                        double start_angle = std::atan2(-Yc, -Xc);
                        double mid_angle = std::atan2(y2 - Yc, x2 - Xc);
                        double end_angle = std::atan2(y3 - Yc, x3 - Xc);
                        double angle_diff = end_angle - start_angle;

                        while (angle_diff <= -3.14159265) angle_diff += 6.2831853;
                        while (angle_diff > 3.14159265) angle_diff -= 6.2831853;
                        double mid_diff = mid_angle - start_angle;
                        while (mid_diff <= -3.14159265) mid_diff += 6.2831853;
                        while (mid_diff > 3.14159265) mid_diff -= 6.2831853;

                        bool missed_opposite = ((angle_diff > 0.0 && mid_diff < 0.0) || (angle_diff < 0.0 && mid_diff > 0.0));
                        bool missed_overshoot = ((angle_diff > 0.0 && mid_diff > 0.0 && mid_diff > angle_diff) ||
                            (angle_diff < 0.0 && mid_diff < 0.0 && mid_diff < angle_diff));
                        if (missed_opposite || missed_overshoot) angle_diff += (angle_diff > 0.0) ? -6.2831853072 : 6.2831853072;
                        if (waypoints[i].center_angle > 0.5f) angle_diff += (angle_diff > 0.0) ? -6.2831853072 : 6.2831853072;

                        arc_len = (float)(R * std::abs(angle_diff)) * 30.0f;
                    }

                    ImGui::Text("3P-Arc (3C%d): %.1f m", waypoints[i].group_id, arc_len);

                    ImGui::SameLine(ImGui::GetWindowWidth() - 60.0f);
                    ImGui::PushID((int)i);
                    if (ImGui::Button("Flip")) {
                        waypoints[i].center_angle = (waypoints[i].center_angle > 0.5f) ? 0.0f : 1.0f;
                        SaveSession();
                    }
                    ImGui::PopID();

                    total_distance += arc_len;
                    last_x = waypoints[i + 2].raw_x;
                    last_y = waypoints[i + 2].raw_y;
                    i += 3;
                }
                else if (waypoints[i].type == WpType::ArcCenterAnchor && i + 1 < waypoints.size()) {
                    // Center Arc Straight Line Bridge
                    if (i > 0) {
                        float dist = std::sqrt(std::pow(waypoints[i].raw_x - last_x, 2) + std::pow(waypoints[i].raw_y - last_y, 2)) * 30.0f;
                        if (dist > 0.1f) {
                            ImGui::Text("Line: %.1f m", dist);
                            total_distance += dist;
                        }
                    }
                    double sx = waypoints[i].raw_x, sy = waypoints[i].raw_y;
                    double cx = waypoints[i + 1].raw_x, cy = waypoints[i + 1].raw_y;
                    double dx = sx - cx, dy = sy - cy;
                    double R = std::sqrt(dx * dx + dy * dy);
                    double sweep_rads = waypoints[i + 1].center_angle * (3.1415926535 / 180.0);

                    float arc_len = (float)(R * std::abs(sweep_rads)) * 30.0f;
                    ImGui::Text("Cent-Arc (C%d): %.1f m", waypoints[i].group_id, arc_len);

                    ImGui::SameLine(ImGui::GetWindowWidth() - 60.0f);
                    ImGui::PushID((int)i);
                    if (ImGui::Button("Flip")) {
                        waypoints[i + 1].center_angle = -waypoints[i + 1].center_angle;
                        SaveSession();
                    }
                    ImGui::PopID();

                    total_distance += arc_len;

                    double start_angle = std::atan2(dy, dx);
                    double end_angle = start_angle + sweep_rads;
                    last_x = (float)(cx + R * std::cos(end_angle));
                    last_y = (float)(cy + R * std::sin(end_angle));
                    i += 2;
                }
                else { i++; }
            }
            ImGui::Separator();
            ImGui::TextColored(ImVec4(0.0f, 1.0f, 0.0f, 1.0f), "Total Distance: %.1f m", total_distance);
        }
        else {
            ImGui::Separator();
            ImGui::TextColored(ImVec4(0.5f, 0.5f, 0.5f, 1.0f), "No waypoints deployed.");
        }
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

    std::vector<ImVec2> GenerateFlightPath() {
        std::vector<ImVec2> path;

        // Combine locked points and the live pending points for real-time rendering
        std::vector<Waypoint> all_wps = waypoints;
        all_wps.insert(all_wps.end(), temp_arc_points.begin(), temp_arc_points.end());

        if (all_wps.empty()) return path;

        for (size_t i = 0; i < all_wps.size(); ) {
            if (all_wps[i].type == WpType::Arc3Pt) {
                if (i + 2 < all_wps.size() && all_wps[i + 1].type == WpType::Arc3Pt && all_wps[i + 2].type == WpType::Arc3Pt) {
                    double x1 = 0.0, y1 = 0.0;
                    double x2 = (double)all_wps[i + 1].raw_x - (double)all_wps[i].raw_x;
                    double y2 = (double)all_wps[i + 1].raw_y - (double)all_wps[i].raw_y;
                    double x3 = (double)all_wps[i + 2].raw_x - (double)all_wps[i].raw_x;
                    double y3 = (double)all_wps[i + 2].raw_y - (double)all_wps[i].raw_y;
                    double D_double = 2.0 * (x2 * y3 - x3 * y2);

                    if (std::abs(D_double) < 0.1) {
                        path.push_back(ImVec2(all_wps[i].raw_x, all_wps[i].raw_y));
                        path.push_back(ImVec2(all_wps[i + 1].raw_x, all_wps[i + 1].raw_y));
                        path.push_back(ImVec2(all_wps[i + 2].raw_x, all_wps[i + 2].raw_y));
                    }
                    else {
                        double Xc_rel = ((x2 * x2 + y2 * y2) * y3 - (x3 * x3 + y3 * y3) * y2) / D_double;
                        double Yc_rel = ((x3 * x3 + y3 * y3) * x2 - (x2 * x2 + y2 * y2) * x3) / D_double;
                        double Xc = (double)all_wps[i].raw_x + Xc_rel;
                        double Yc = (double)all_wps[i].raw_y + Yc_rel;
                        double R = std::sqrt(Xc_rel * Xc_rel + Yc_rel * Yc_rel);

                        double start_angle = std::atan2((double)all_wps[i].raw_y - Yc, (double)all_wps[i].raw_x - Xc);
                        double mid_angle = std::atan2((double)all_wps[i + 1].raw_y - Yc, (double)all_wps[i + 1].raw_x - Xc);
                        double end_angle = std::atan2((double)all_wps[i + 2].raw_y - Yc, (double)all_wps[i + 2].raw_x - Xc);

                        double angle_diff = end_angle - start_angle;
                        while (angle_diff <= -3.1415926535) angle_diff += 6.2831853072;
                        while (angle_diff > 3.1415926535)  angle_diff -= 6.2831853072;

                        double mid_diff = mid_angle - start_angle;
                        while (mid_diff <= -3.1415926535) mid_diff += 6.2831853072;
                        while (mid_diff > 3.1415926535)  mid_diff -= 6.2831853072;

                        bool missed_opposite = ((angle_diff > 0.0 && mid_diff < 0.0) || (angle_diff < 0.0 && mid_diff > 0.0));
                        bool missed_overshoot = ((angle_diff > 0.0 && mid_diff > 0.0 && mid_diff > angle_diff) ||
                            (angle_diff < 0.0 && mid_diff < 0.0 && mid_diff < angle_diff));

                        if (missed_opposite || missed_overshoot) angle_diff += (angle_diff > 0.0) ? -6.2831853072 : 6.2831853072;
                        if (all_wps[i].center_angle > 0.5f) angle_diff += (angle_diff > 0.0) ? -6.2831853072 : 6.2831853072;

                        int segments = (int)(std::max)(10.0, std::abs(angle_diff) * 30.0);
                        for (int j = 0; j <= segments; j++) {
                            double t = (double)j / (double)segments;
                            double current_angle = start_angle + (angle_diff * t);
                            path.push_back(ImVec2((float)(Xc + R * std::cos(current_angle)), (float)(Yc + R * std::sin(current_angle))));
                        }
                    }
                    i += 3;
                    continue;
                }
            }
            else if (all_wps[i].type == WpType::ArcCenterAnchor) {
                if (i + 1 < all_wps.size() && all_wps[i + 1].type == WpType::ArcCenterEnd) {
                    double sx = (double)all_wps[i].raw_x;
                    double sy = (double)all_wps[i].raw_y;
                    double cx = (double)all_wps[i + 1].raw_x;
                    double cy = (double)all_wps[i + 1].raw_y;
                    double dx = sx - cx;
                    double dy = sy - cy;
                    double R = std::sqrt(dx * dx + dy * dy);

                    double start_angle = std::atan2(dy, dx);
                    // Live parametric angle tracking, supporting > 360 degree multi-turns
                    double sweep_angle = all_wps[i + 1].center_angle * (3.1415926535 / 180.0);

                    int segments = (int)(std::max)(10.0, std::abs(sweep_angle) * 30.0);
                    for (int j = 0; j <= segments; j++) {
                        double t = (double)j / (double)segments;
                        double current_angle = start_angle + (sweep_angle * t);
                        path.push_back(ImVec2((float)(cx + R * std::cos(current_angle)), (float)(cy + R * std::sin(current_angle))));
                    }
                    i += 2;
                    continue;
                }
            }

            path.push_back(ImVec2(all_wps[i].raw_x, all_wps[i].raw_y));
            i++;
        }
        return path;
    }

    void ExportMissionFCode() {
        if (waypoints.size() < 2) return;

        // 1. Generate the true mathematical curve (Lines + Arcs)
        std::vector<ImVec2> flight_path = GenerateFlightPath();
        if (flight_path.size() < 2) return;

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
            std::ofstream file(ofn.lpstrFile);
            if (!file.is_open()) return;

            file << "%\n";
            file << "F43; Z50;\n";

            // --- 2. Global Supercover Raycast (Synchronized to Renderer) ---
            struct UITile { int x, y; };
            struct UIFlightStep {
                std::vector<UITile> tiles;
                float sum_elev;
                int tile_count;
            };
            std::vector<UIFlightStep> steps;

            UIFlightStep current_step;
            current_step.sum_elev = 0.0f;
            current_step.tile_count = 0;
            int prev_parent = -1;

            auto add_tile_to_step = [&](int cx, int cy, bool is_x_parent) {
                int current_parent = is_x_parent ? cx : cy;
                if (prev_parent == -1) prev_parent = current_parent;
                if (current_parent != prev_parent) {
                    if (current_step.tile_count > 0) steps.push_back(current_step);
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

            for (size_t i = 1; i < flight_path.size(); i++) {
                int start_x = (int)flight_path[i - 1].x;
                int start_y = (int)flight_path[i - 1].y;
                int end_x = (int)flight_path[i].x;
                int end_y = (int)flight_path[i].y;

                if (start_x == end_x && start_y == end_y) {
                    add_tile_to_step(start_x, start_y, true);
                    continue;
                }

                int dx = end_x - start_x;
                int dy = end_y - start_y;
                int step_x = (dx > 0) ? 1 : ((dx < 0) ? -1 : 0);
                int step_y = (dy > 0) ? 1 : ((dy < 0) ? -1 : 0);
                bool is_x_parent = std::abs(dx) >= std::abs(dy);

                double abs_dx = std::abs((double)dx), abs_dy = std::abs((double)dy);
                double tDeltaX = (abs_dx == 0) ? 1e30 : 1.0 / abs_dx;
                double tDeltaY = (abs_dy == 0) ? 1e30 : 1.0 / abs_dy;
                double tMaxX = (abs_dx == 0) ? 1e30 : 0.5 / abs_dx;
                double tMaxY = (abs_dy == 0) ? 1e30 : 0.5 / abs_dy;

                int current_x = start_x, current_y = start_y;
                int total_crossings = (int)(abs_dx + abs_dy);

                if (i == 1) add_tile_to_step(current_x, current_y, is_x_parent);

                for (int step = 0; step < total_crossings; step++) {
                    if (current_x == end_x && current_y == end_y) break;
                    if (std::abs(tMaxX - tMaxY) < 1e-8) {
                        current_x += step_x; current_y += step_y;
                        tMaxX += tDeltaX; tMaxY += tDeltaY;
                        step++; add_tile_to_step(current_x, current_y, is_x_parent);
                    }
                    else if (tMaxX < tMaxY) {
                        current_x += step_x; tMaxX += tDeltaX; add_tile_to_step(current_x, current_y, is_x_parent);
                    }
                    else {
                        current_y += step_y; tMaxY += tDeltaY; add_tile_to_step(current_x, current_y, is_x_parent);
                    }
                }
            }
            if (current_step.tile_count > 0) steps.push_back(current_step);

            // --- 3. Mode Calculation ---
            std::vector<int> raw_modes(steps.size(), 0);
            int current_mode = 0;
            float prev_group_elev = 0.0f;
            for (size_t i = 0; i < steps.size(); i += step_grouping) {
                float group_sum_elev = 0.0f;
                int group_tile_count = 0;
                size_t end_idx = (i + step_grouping < steps.size()) ? i + step_grouping : steps.size();
                for (size_t s = i; s < end_idx; s++) {
                    group_sum_elev += steps[s].sum_elev;
                    group_tile_count += steps[s].tile_count;
                }
                float current_group_elev = group_sum_elev / (float)group_tile_count;
                if (i == 0) prev_group_elev = current_group_elev;
                float delta_z = current_group_elev - prev_group_elev;

                if (current_mode == 0) {
                    if (delta_z > (float)mode_threshold) current_mode = 1;
                    else if (delta_z < -(float)mode_threshold) current_mode = 2;
                }
                else if (current_mode == 1) {
                    if (delta_z < ((float)mode_threshold * 0.5f)) current_mode = 0;
                }
                else if (current_mode == 2) {
                    if (delta_z > -((float)mode_threshold * 0.5f)) current_mode = 0;
                }
                for (size_t s = i; s < end_idx; s++) raw_modes[s] = current_mode;
                prev_group_elev = current_group_elev;
            }

            // --- 4. Dynamic Feed-Forward Offset ---
            std::vector<int> final_modes = raw_modes;
            for (int s = 1; s < (int)steps.size(); s++) {
                if (raw_modes[s] != raw_modes[s - 1]) {
                    int upcoming_mode = raw_modes[s];
                    for (int b = 1; b <= tile_offset; b++) {
                        if (s - b >= 0) final_modes[s - b] = upcoming_mode;
                    }
                }
            }

            // --- 5. Flatten, Deduplicate, and Write F-CODE ---
            struct FlatTile { int x, y, mode; };
            std::vector<FlatTile> flat_tiles;
            int last_x = -1, last_y = -1;

            for (size_t s = 0; s < steps.size(); s++) {
                int mode = final_modes[s];
                for (const auto& t : steps[s].tiles) {
                    // Stripping overlap guarantees perfect sequential waypoints
                    if (t.x == last_x && t.y == last_y) continue;
                    flat_tiles.push_back({ t.x, t.y, mode });
                    last_x = t.x;
                    last_y = t.y;
                }
            }

            current_mode = -1;
            for (size_t f = 0; f < flat_tiles.size(); f++) {
                int next_mode = flat_tiles[f].mode;
                int total_lon_sec = (26 * 3600) + flat_tiles[f].x;
                int total_lat_sec = (42 * 3600) - flat_tiles[f].y;
                int lon_d = total_lon_sec / 3600, lon_m = (total_lon_sec % 3600) / 60, lon_s = total_lon_sec % 60;
                int lat_d = total_lat_sec / 3600, lat_m = (total_lat_sec % 3600) / 60, lat_s = total_lat_sec % 60;

                char line[128];
                if (f == 0) {
                    snprintf(line, sizeof(line), "F91; X%02d%02d%02d; Y%02d%02d%02d; N40;\n", lon_d, lon_m, lon_s, lat_d, lat_m, lat_s);
                    current_mode = next_mode;
                }
                else if (f == flat_tiles.size() - 1) {
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

            file << "%\n";
            file.close();
        }
    }
}
