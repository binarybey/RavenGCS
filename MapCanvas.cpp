#define IMGUI_DEFINE_MATH_OPERATORS
#define NOMINMAX                    
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
#include <algorithm>                
#include <cstdint>
#include <climits>
#include <cstring>

namespace MapCanvas {

    static_assert(sizeof(size_t) >= 8,
        "MapCanvas requires a 64-bit build: the 5.9 GB elevation map cannot be "
        "mapped or indexed in a 32-bit address space.");

    template <typename T> static inline T Clamp(T v, T lo, T hi) {
        return (v < lo) ? lo : ((v > hi) ? hi : v);
    }

    constexpr int   MAP_W = 68400;
    constexpr int   MAP_H = 21600;
    constexpr int   ORIGIN_LON_DEG = 26;
    constexpr int   ORIGIN_LAT_DEG = 42;
    constexpr int   TILE_W = 3600;
    constexpr int   TILE_H = 10800;
    constexpr int   CHUNK_ROWS = 2;
    constexpr int   CHUNK_COLS = 19;
    constexpr int   CHUNK_TOTAL = CHUNK_ROWS * CHUNK_COLS;
    constexpr float PRECISION_ZOOM = 15.0f;
    constexpr float ZOOM_MIN = 0.01f;
    constexpr float ZOOM_MAX = 30.0f;
    constexpr float METERS_PER_TILE = 30.87f;

    constexpr int MODE_LEVEL = 0;
    constexpr int MODE_CLIMB = 1;
    constexpr int MODE_DESCEND = 2;

    enum class AppState {
        IDLE,
        WAITING_3PT_1,
        WAITING_3PT_2,
        WAITING_3PT_3,
        WAITING_CENTER_1,
        WAITING_CENTER_2,
        ADJUSTING_CENTER_ARC
    };

    static AppState current_state = AppState::IDLE;
    static int current_arc_group = 1;
    static std::vector<Waypoint> temp_arc_points;

    bool show_minute_grid = true;
    bool show_second_grid = true;
    std::vector<Waypoint> waypoints;

    const std::vector<Waypoint>& GetWaypoints() { return waypoints; }

    static ID3D11ShaderResourceView* texMap[CHUNK_ROWS][CHUNK_COLS] = { nullptr };

    static std::atomic<int>  loaded_chunks = 0;
    static std::atomic<bool> map_loaded = false;
    static std::atomic<bool> shutdown_requested = false;
    static std::thread       loader_thread;

    static HANDLE hMapFile = NULL;
    static float* elevation_data = nullptr;
    static bool   elevation_ready = false;
    static std::string elevation_error;

    static int mode_threshold = 15;
    static int tile_offset = 3;
    static int step_grouping = 3;

    static float  zoom = 0.012f;
    static ImVec2 scroll_pos = ImVec2(0.0f, 0.0f);

    bool precisionLockMet = false;

    static inline float SampleElevation(int x, int y) {
        if (!elevation_data) return 0.0f;
        if (x < 0) x = 0; else if (x >= MAP_W) x = MAP_W - 1;
        if (y < 0) y = 0; else if (y >= MAP_H) y = MAP_H - 1;
        return elevation_data[(size_t)y * (size_t)MAP_W + (size_t)x];
    }

    static inline bool InMap(int x, int y) {
        return x >= 0 && x < MAP_W && y >= 0 && y < MAP_H;
    }

    static inline int TileOf(float v) { return (int)std::floor(v); }

    bool LoadTextureFromFile(ID3D11Device* d3dDevice, const char* filename,
        ID3D11ShaderResourceView** out_srv, int* out_width,
        int* out_height, unsigned char** out_cpu_data)
    {
        int image_width = 0, image_height = 0;
        unsigned char* image_data = stbi_load(filename, &image_width, &image_height, NULL, 4);
        if (image_data == NULL) return false;

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

        ID3D11Texture2D* pTexture = NULL;
        D3D11_SUBRESOURCE_DATA subResource;
        subResource.pSysMem = image_data;
        subResource.SysMemPitch = desc.Width * 4;
        subResource.SysMemSlicePitch = 0;

        HRESULT hr = d3dDevice->CreateTexture2D(&desc, &subResource, &pTexture);
        if (FAILED(hr) || pTexture == nullptr) {
            stbi_image_free(image_data);
            return false;
        }

        d3dDevice->CreateShaderResourceView(pTexture, NULL, out_srv);
        pTexture->Release();

        *out_cpu_data = image_data;
        *out_width = image_width;
        *out_height = image_height;
        return true;
    }

    void LoadMapTexturesAsync(ID3D11Device* d3dDevice) {
        for (int r = 0; r < CHUNK_ROWS; r++) {
            for (int c = 0; c < CHUNK_COLS; c++) {
                if (shutdown_requested.load(std::memory_order_acquire)) return;

                char filename[256];
                snprintf(filename, sizeof(filename), "map_chunks_19x2\\chunk_%c_%02d.png",
                    r == 0 ? 'N' : 'S', c + 1);

                int w, h;
                unsigned char* cpu_ram = nullptr;
                LoadTextureFromFile(d3dDevice, filename, &texMap[r][c], &w, &h, &cpu_ram);
                if (cpu_ram) stbi_image_free(cpu_ram);

                loaded_chunks++;
            }
        }
        map_loaded.store(true, std::memory_order_release);
    }

    void LoadElevationData() {
        elevation_ready = false;
        elevation_error.clear();

        HANDLE hFile = CreateFileA("terrain_data.bin", GENERIC_READ, FILE_SHARE_READ,
            NULL, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL);
        if (hFile == INVALID_HANDLE_VALUE) {
            elevation_error = "terrain_data.bin not found next to the executable.";
            return;
        }

        LARGE_INTEGER sz{};
        const long long expected = (long long)MAP_W * (long long)MAP_H * (long long)sizeof(float);
        if (!GetFileSizeEx(hFile, &sz) || sz.QuadPart < expected) {
            char buf[256];
            snprintf(buf, sizeof(buf),
                "terrain_data.bin is %.2f GB, expected %.2f GB. Re-run the Python generator.",
                (double)sz.QuadPart / 1e9, (double)expected / 1e9);
            elevation_error = buf;
            CloseHandle(hFile);
            return;
        }

        hMapFile = CreateFileMappingA(hFile, NULL, PAGE_READONLY, 0, 0, NULL);
        if (hMapFile != NULL) {
            elevation_data = (float*)MapViewOfFile(hMapFile, FILE_MAP_READ, 0, 0, 0);
            if (!elevation_data)
                elevation_error = "MapViewOfFile failed. Are you building x64?";
        }
        else {
            elevation_error = "CreateFileMapping failed.";
        }
        CloseHandle(hFile);
        elevation_ready = (elevation_data != nullptr);
    }

    void Initialize(ID3D11Device* d3dDevice) {
        LoadSession();
        LoadElevationData();
        loader_thread = std::thread(LoadMapTexturesAsync, d3dDevice);
    }

    void Shutdown() {
        shutdown_requested.store(true, std::memory_order_release);
        if (loader_thread.joinable()) loader_thread.join();

        for (int r = 0; r < CHUNK_ROWS; r++)
            for (int c = 0; c < CHUNK_COLS; c++)
                if (texMap[r][c]) { texMap[r][c]->Release(); texMap[r][c] = nullptr; }

        if (elevation_data) { UnmapViewOfFile(elevation_data); elevation_data = nullptr; }
        if (hMapFile) { CloseHandle(hMapFile); hMapFile = NULL; }
        elevation_ready = false;
    }

    static std::vector<ImVec2> cached_path;
    static uint64_t cached_path_key = ~0ull;

    static uint64_t HashInputs(bool include_profile_params) {
        uint64_t h = 1469598103934665603ull;
        auto mix = [&h](uint64_t v) { h ^= v; h *= 1099511628211ull; };
        auto mixf = [&](float f) { uint32_t b; memcpy(&b, &f, 4); mix(b); };

        auto mix_wp = [&](const Waypoint& w) {
            mixf(w.raw_x); mixf(w.raw_y); mixf(w.center_angle);
            mix((uint64_t)(int)w.type); mix((uint64_t)w.group_id);
            };
        mix(waypoints.size());
        for (const auto& w : waypoints) mix_wp(w);
        mix(temp_arc_points.size());
        for (const auto& w : temp_arc_points) mix_wp(w);

        if (include_profile_params) {
            mix((uint64_t)mode_threshold);
            mix((uint64_t)tile_offset);
            mix((uint64_t)step_grouping);
        }
        return h;
    }

    static bool Arc3PtGeometry(const Waypoint& a, const Waypoint& b, const Waypoint& c,
        double& Xc, double& Yc, double& R,
        double& start_angle, double& sweep)
    {
        const double lat_rad = (ORIGIN_LAT_DEG - (a.raw_y / 3600.0)) * (3.1415926535 / 180.0);
        const double cos_lat = std::cos(lat_rad);

        const double x1 = a.raw_x * cos_lat, y1 = a.raw_y;
        const double x2 = (double)b.raw_x * cos_lat - x1, y2 = (double)b.raw_y - y1;
        const double x3 = (double)c.raw_x * cos_lat - x1, y3 = (double)c.raw_y - y1;
        const double D = 2.0 * (x2 * y3 - x3 * y2);
        if (std::abs(D) <= 0.1) return false;

        const double Xc_rel = ((x2 * x2 + y2 * y2) * y3 - (x3 * x3 + y3 * y3) * y2) / D;
        const double Yc_rel = ((x3 * x3 + y3 * y3) * x2 - (x2 * x2 + y2 * y2) * x3) / D;

        Xc = (x1 + Xc_rel) / cos_lat;
        Yc = y1 + Yc_rel;
        R = std::sqrt(Xc_rel * Xc_rel + Yc_rel * Yc_rel);

        start_angle = std::atan2(-Yc_rel, -Xc_rel);
        const double mid_angle = std::atan2(y2 - Yc_rel, x2 - Xc_rel);
        const double end_angle = std::atan2(y3 - Yc_rel, x3 - Xc_rel);

        double diff = end_angle - start_angle;
        while (diff <= -3.1415926535) diff += 6.2831853072;
        while (diff > 3.1415926535) diff -= 6.2831853072;
        double mid_diff = mid_angle - start_angle;
        while (mid_diff <= -3.1415926535) mid_diff += 6.2831853072;
        while (mid_diff > 3.1415926535) mid_diff -= 6.2831853072;

        const bool missed_opposite = ((diff > 0.0 && mid_diff < 0.0) || (diff < 0.0 && mid_diff > 0.0));
        const bool missed_overshoot = ((diff > 0.0 && mid_diff > 0.0 && mid_diff > diff) ||
            (diff < 0.0 && mid_diff < 0.0 && mid_diff < diff));
        if (missed_opposite || missed_overshoot) diff += (diff > 0.0) ? -6.2831853072 : 6.2831853072;
        if (a.center_angle > 0.5f)                diff += (diff > 0.0) ? -6.2831853072 : 6.2831853072;

        sweep = diff;
        return true;
    }

    static void BuildFlightPath(std::vector<ImVec2>& path) {
        path.clear();

        std::vector<Waypoint> all_wps;
        all_wps.reserve(waypoints.size() + temp_arc_points.size());
        all_wps.insert(all_wps.end(), waypoints.begin(), waypoints.end());
        all_wps.insert(all_wps.end(), temp_arc_points.begin(), temp_arc_points.end());
        if (all_wps.empty()) return;

        for (size_t i = 0; i < all_wps.size(); ) {
            if (all_wps[i].type == WpType::Arc3Pt &&
                i + 2 < all_wps.size() &&
                all_wps[i + 1].type == WpType::Arc3Pt &&
                all_wps[i + 2].type == WpType::Arc3Pt)
            {
                double Xc, Yc, R, sa, sweep;
                if (Arc3PtGeometry(all_wps[i], all_wps[i + 1], all_wps[i + 2], Xc, Yc, R, sa, sweep)) {
                    const double lat_rad = (ORIGIN_LAT_DEG - (all_wps[i].raw_y / 3600.0)) * (3.1415926535 / 180.0);
                    const double cos_lat = std::cos(lat_rad);

                    double arc_len_tiles = R * std::abs(sweep);
                    int segments = (int)Clamp(arc_len_tiles * 0.5, 16.0, 4000.0);

                    for (int j = 0; j <= segments; j++) {
                        double t = (double)j / (double)segments;
                        double a = sa + sweep * t;
                        path.push_back(ImVec2((float)(Xc + (R * std::cos(a)) / cos_lat), (float)(Yc + R * std::sin(a))));
                    }
                }
                else {
                    path.push_back(ImVec2(all_wps[i].raw_x, all_wps[i].raw_y));
                    path.push_back(ImVec2(all_wps[i + 1].raw_x, all_wps[i + 1].raw_y));
                    path.push_back(ImVec2(all_wps[i + 2].raw_x, all_wps[i + 2].raw_y));
                }
                i += 3;
                continue;
            }
            else if (all_wps[i].type == WpType::ArcCenterAnchor &&
                i + 1 < all_wps.size() &&
                all_wps[i + 1].type == WpType::ArcCenterEnd)
            {
                const double lat_rad = (ORIGIN_LAT_DEG - (all_wps[i].raw_y / 3600.0)) * (3.1415926535 / 180.0);
                const double cos_lat = std::cos(lat_rad);

                double sx_iso = (double)all_wps[i].raw_x * cos_lat, sy = (double)all_wps[i].raw_y;
                double cx_iso = (double)all_wps[i + 1].raw_x * cos_lat, cy = (double)all_wps[i + 1].raw_y;
                double dx = sx_iso - cx_iso, dy = sy - cy;
                double R = std::sqrt(dx * dx + dy * dy);

                double start_angle = std::atan2(dy, dx);
                double sweep_angle = all_wps[i + 1].center_angle * (3.1415926535 / 180.0);

                double arc_len_tiles = R * std::abs(sweep_angle);
                int segments = (int)Clamp(arc_len_tiles * 0.5, 16.0, 8000.0);

                for (int j = 0; j <= segments; j++) {
                    double t = (double)j / (double)segments;
                    double a = start_angle + sweep_angle * t;
                    path.push_back(ImVec2((float)((cx_iso + R * std::cos(a)) / cos_lat), (float)(cy + R * std::sin(a))));
                }
                i += 2;
                continue;
            }

            path.push_back(ImVec2(all_wps[i].raw_x, all_wps[i].raw_y));
            i++;
        }
    }

    static const std::vector<ImVec2>& GetCachedFlightPath() {
        uint64_t key = HashInputs(false);
        if (key != cached_path_key) {
            BuildFlightPath(cached_path);
            cached_path_key = key;
        }
        return cached_path;
    }

    std::vector<ImVec2> GenerateFlightPath() { return GetCachedFlightPath(); }

    struct ProfileTile {
        int   x, y;
        int   seg;
        float t;
    };
    struct ProfileStep {
        int   first, last;
        float sum_elev;
        int   count;
    };
    struct MissionProfile {
        std::vector<ProfileTile> tiles;
        std::vector<ProfileStep> steps;
        std::vector<int>         modes;
        bool  left_map = false;
        void clear() { tiles.clear(); steps.clear(); modes.clear(); left_map = false; }
    };

    static MissionProfile cached_profile;
    static uint64_t       cached_profile_key = ~0ull;

    static void RaycastPath(const std::vector<ImVec2>& path, MissionProfile& mp) {
        if (path.size() < 2) return;

        mp.tiles.reserve(path.size() * 2);

        int  prev_x = INT_MIN, prev_y = INT_MIN;
        int  prev_parent = INT_MIN;
        bool prev_axis_is_x = true;
        bool have_step = false;
        ProfileStep cur{};

        auto close_step = [&]() {
            if (have_step && cur.count > 0) mp.steps.push_back(cur);
            have_step = false;
            };

        auto emit = [&](int cx, int cy, bool axis_is_x, int seg, float t) {
            if (cx == prev_x && cy == prev_y) return;
            if (!InMap(cx, cy)) mp.left_map = true;

            int parent = axis_is_x ? cx : cy;

            if (!have_step) {
                cur = ProfileStep{ (int)mp.tiles.size(), (int)mp.tiles.size(), 0.0f, 0 };
                have_step = true;
                prev_parent = parent;
                prev_axis_is_x = axis_is_x;
            }
            else if (parent != prev_parent || axis_is_x != prev_axis_is_x) {
                close_step();
                cur = ProfileStep{ (int)mp.tiles.size(), (int)mp.tiles.size(), 0.0f, 0 };
                have_step = true;
                prev_parent = parent;
                prev_axis_is_x = axis_is_x;
            }

            mp.tiles.push_back(ProfileTile{ cx, cy, seg, t });
            cur.last = (int)mp.tiles.size() - 1;
            cur.sum_elev += SampleElevation(cx, cy);
            cur.count++;

            prev_x = cx; prev_y = cy;
            };

        for (size_t i = 1; i < path.size(); i++) {
            const int sx = TileOf(path[i - 1].x), sy = TileOf(path[i - 1].y);
            const int ex = TileOf(path[i].x), ey = TileOf(path[i].y);

            const int dx = ex - sx, dy = ey - sy;
            const bool axis_is_x = std::abs(dx) >= std::abs(dy);

            if (dx == 0 && dy == 0) {
                emit(sx, sy, axis_is_x, (int)i, 0.0f);
                continue;
            }

            const int step_x = (dx > 0) ? 1 : ((dx < 0) ? -1 : 0);
            const int step_y = (dy > 0) ? 1 : ((dy < 0) ? -1 : 0);

            const double adx = std::abs((double)dx), ady = std::abs((double)dy);
            const double tDeltaX = (adx == 0.0) ? 1e30 : 1.0 / adx;
            const double tDeltaY = (ady == 0.0) ? 1e30 : 1.0 / ady;
            double tMaxX = (adx == 0.0) ? 1e30 : 0.5 / adx;
            double tMaxY = (ady == 0.0) ? 1e30 : 0.5 / ady;

            int cx = sx, cy = sy;
            const int total_crossings = (int)(adx + ady);

            emit(cx, cy, axis_is_x, (int)i, 0.0f);

            for (int s = 0; s < total_crossings; s++) {
                if (cx == ex && cy == ey) break;

                const double t_cross = (tMaxX < tMaxY) ? tMaxX : tMaxY;

                if (std::abs(tMaxX - tMaxY) < 1e-8) {
                    cx += step_x; cy += step_y;
                    tMaxX += tDeltaX; tMaxY += tDeltaY;
                    s++;
                }
                else if (tMaxX < tMaxY) { cx += step_x; tMaxX += tDeltaX; }
                else { cy += step_y; tMaxY += tDeltaY; }

                emit(cx, cy, axis_is_x, (int)i, (float)Clamp(t_cross, 0.0, 1.0));
            }
        }
        close_step();
    }

    static void ClassifyModes(MissionProfile& mp) {
        const size_t n = mp.steps.size();
        mp.modes.assign(n, MODE_LEVEL);
        if (n == 0) return;

        const int grouping = std::max(1, step_grouping);

        std::vector<int> raw(n, MODE_LEVEL);
        int   mode = MODE_LEVEL;
        float prev_group_elev = 0.0f;

        for (size_t i = 0; i < n; i += grouping) {
            float sum = 0.0f; int cnt = 0;
            const size_t end_idx = std::min(i + (size_t)grouping, n);
            for (size_t s = i; s < end_idx; s++) { sum += mp.steps[s].sum_elev; cnt += mp.steps[s].count; }

            const float group_elev = (cnt > 0) ? (sum / (float)cnt) : prev_group_elev;
            if (i == 0) prev_group_elev = group_elev;
            const float delta_z = group_elev - prev_group_elev;

            const float up = (float)mode_threshold;
            const float rel = (float)mode_threshold * 0.5f;

            if (mode == MODE_CLIMB) {
                if (delta_z < -up)      mode = MODE_DESCEND;
                else if (delta_z < rel) mode = MODE_LEVEL;
            }
            else if (mode == MODE_DESCEND) {
                if (delta_z > up)        mode = MODE_CLIMB;
                else if (delta_z > -rel) mode = MODE_LEVEL;
            }
            else {
                if (delta_z > up)       mode = MODE_CLIMB;
                else if (delta_z < -up) mode = MODE_DESCEND;
            }

            for (size_t s = i; s < end_idx; s++) raw[s] = mode;
            prev_group_elev = group_elev;
        }

        const int off = std::max(0, tile_offset);

        auto paint_runs = [&](int target, int shift_start, int shift_end) {
            size_t i = 0;
            while (i < n) {
                if (raw[i] != target) { i++; continue; }
                size_t a = i;
                while (a + 1 < n && raw[a + 1] == target) a++;
                const long long lo = std::max(0LL, (long long)i + shift_start);
                const long long hi = std::min((long long)n - 1, (long long)a + shift_end);
                for (long long s = lo; s <= hi; s++) mp.modes[(size_t)s] = target;
                i = a + 1;
            }
            };

        paint_runs(MODE_DESCEND, +off, +off);
        paint_runs(MODE_CLIMB, -off, 0);
    }

    static const MissionProfile& GetMissionProfile() {
        const std::vector<ImVec2>& path = GetCachedFlightPath();
        const uint64_t key = HashInputs(true);
        if (key != cached_profile_key) {
            cached_profile.clear();
            RaycastPath(path, cached_profile);
            ClassifyModes(cached_profile);
            cached_profile_key = key;
        }
        return cached_profile;
    }

    static void InvalidateProfile() { cached_profile_key = ~0ull; cached_path_key = ~0ull; }

    struct BandRun { int x0, x1, y; };
    static std::vector<BandRun> band_runs;
    static int bk_fx = INT_MIN, bk_lx = 0, bk_fy = 0, bk_ly = 0, bk_step = 0, bk_tier = 0;
    constexpr float MIN_CELL_PX = 3.0f;

    static void RebuildBandRuns(int first_x, int last_x, int first_y, int last_y,
        int map_step, int tier_interval)
    {
        band_runs.clear();
        for (int y = first_y; y < last_y; y += map_step) {
            int run_x0 = INT_MIN;
            for (int x = first_x; x < last_x; x += map_step) {
                const float elev = SampleElevation(x, y);
                const int   tier = (int)(elev / (float)tier_interval);
                const bool  shaded = ((tier & 1) == 0);

                if (shaded) { if (run_x0 == INT_MIN) run_x0 = x; }
                else if (run_x0 != INT_MIN) { band_runs.push_back({ run_x0, x, y }); run_x0 = INT_MIN; }
            }
            if (run_x0 != INT_MIN) band_runs.push_back({ run_x0, last_x, y });
        }
    }

    void RenderInteractiveMap() {
        if (!map_loaded.load(std::memory_order_acquire)) {
            const float window_width = ImGui::GetWindowSize().x;
            const float window_height = ImGui::GetWindowSize().y;

            char text_buf[128];
            snprintf(text_buf, sizeof(text_buf), "Parsing Topography Elements (%d/%d)",
                loaded_chunks.load(), CHUNK_TOTAL);
            const float text_width = ImGui::CalcTextSize(text_buf).x;
            const float bar_width = 400.0f;

            ImGui::SetCursorPosY((window_height * 0.5f) - 30.0f);
            ImGui::SetCursorPosX((window_width - text_width) * 0.5f);
            ImGui::TextUnformatted(text_buf);
            ImGui::SetCursorPosX((window_width - bar_width) * 0.5f);
            ImGui::ProgressBar((float)loaded_chunks / (float)CHUNK_TOTAL, ImVec2(bar_width, 30.0f));
            return;
        }

        ImGuiIO& io = ImGui::GetIO();

        if (io.KeyCtrl && ImGui::IsKeyPressed(ImGuiKey_M, false)) { show_minute_grid = !show_minute_grid; SaveSession(); }
        if (io.KeyCtrl && ImGui::IsKeyPressed(ImGuiKey_S, false)) { show_second_grid = !show_second_grid; SaveSession(); }

        ImDrawList* draw_list = ImGui::GetWindowDrawList();

        ImVec2 canvas_p0 = ImGui::GetCursorScreenPos();
        ImVec2 canvas_sz = ImGui::GetContentRegionAvail();
        if (canvas_sz.x < 50.0f) canvas_sz.x = 50.0f;
        if (canvas_sz.y < 50.0f) canvas_sz.y = 50.0f;
        ImVec2 canvas_p1 = ImVec2(canvas_p0.x + canvas_sz.x, canvas_p0.y + canvas_sz.y);

        ImGui::InvisibleButton("canvas", canvas_sz,
            ImGuiButtonFlags_MouseButtonLeft | ImGuiButtonFlags_MouseButtonRight);
        const bool is_hovered = ImGui::IsItemHovered();

        if (is_hovered && ImGui::IsMouseDragging(ImGuiMouseButton_Middle)) {
            scroll_pos.x += io.MouseDelta.x / zoom;
            scroll_pos.y += io.MouseDelta.y / zoom;
        }

        if (is_hovered && io.MouseWheel != 0.0f) {
            const ImVec2 mouse_in_canvas = ImVec2(io.MousePos.x - canvas_p0.x, io.MousePos.y - canvas_p0.y);
            const float origin_x = (mouse_in_canvas.x / zoom) - scroll_pos.x;
            const float origin_y = (mouse_in_canvas.y / zoom) - scroll_pos.y;

            const float new_zoom = Clamp(zoom * powf(1.1f, io.MouseWheel), ZOOM_MIN, ZOOM_MAX);

            scroll_pos.x = (mouse_in_canvas.x / new_zoom) - origin_x;
            scroll_pos.y = (mouse_in_canvas.y / new_zoom) - origin_y;
            zoom = new_zoom;
        }

        const float half_w = (canvas_sz.x * 0.5f) / zoom;
        const float half_h = (canvas_sz.y * 0.5f) / zoom;
        scroll_pos.x = Clamp(scroll_pos.x, -(float)MAP_W + half_w, half_w);
        scroll_pos.y = Clamp(scroll_pos.y, -(float)MAP_H + half_h, half_h);

        draw_list->PushClipRect(canvas_p0, canvas_p1, true);

        const ImVec2 map_origin = ImVec2(canvas_p0.x + scroll_pos.x * zoom,
            canvas_p0.y + scroll_pos.y * zoom);

        for (int r = 0; r < CHUNK_ROWS; r++) {
            for (int c = 0; c < CHUNK_COLS; c++) {
                if (!texMap[r][c]) continue;
                const ImVec2 p0 = ImVec2(map_origin.x + (c * TILE_W) * zoom, map_origin.y + (r * TILE_H) * zoom);
                const ImVec2 p1 = ImVec2(p0.x + TILE_W * zoom, p0.y + TILE_H * zoom);
                if (p1.x > canvas_p0.x && p0.x < canvas_p1.x && p1.y > canvas_p0.y && p0.y < canvas_p1.y)
                    draw_list->AddImage((ImTextureID)texMap[r][c], p0, p1);
            }
        }

        if (ImGui::IsKeyPressed(ImGuiKey_Escape) && current_state != AppState::IDLE) {
            current_state = AppState::IDLE;
            temp_arc_points.clear();
        }

        if (is_hovered) {
            if (zoom > PRECISION_ZOOM && ImGui::IsMouseClicked(ImGuiMouseButton_Left)) {
                const float click_x = (io.MousePos.x - map_origin.x) / zoom;
                const float click_y = (io.MousePos.y - map_origin.y) / zoom;

                if (click_x >= 0 && click_x < MAP_W && click_y >= 0 && click_y < MAP_H) {
                    const int tile_x = TileOf(click_x);
                    const int tile_y = TileOf(click_y);

                    bool exists = false;
                    for (const auto& w : waypoints)
                        if (TileOf(w.raw_x) == tile_x && TileOf(w.raw_y) == tile_y) { exists = true; break; }
                    for (const auto& w : temp_arc_points)
                        if (TileOf(w.raw_x) == tile_x && TileOf(w.raw_y) == tile_y) { exists = true; break; }

                    if (!exists) {
                        Waypoint wp;
                        wp.raw_x = (float)tile_x;
                        wp.raw_y = (float)tile_y;
                        wp.lon_sec = (ORIGIN_LON_DEG * 3600) + tile_x;
                        wp.lat_sec = (ORIGIN_LAT_DEG * 3600) - tile_y;

                        auto next_group = [&]() {
                            int mx = 0;
                            for (const auto& w : waypoints)       mx = std::max(mx, w.group_id);
                            for (const auto& w : temp_arc_points) mx = std::max(mx, w.group_id);
                            return mx + 1;
                            };

                        switch (current_state) {
                        case AppState::IDLE:
                            wp.type = WpType::Standard;
                            waypoints.push_back(wp);
                            SaveSession();
                            break;

                        case AppState::WAITING_3PT_1:
                            current_arc_group = next_group();
                            wp.type = WpType::Arc3Pt; wp.group_id = current_arc_group; wp.group_index = 1;
                            temp_arc_points.push_back(wp);
                            current_state = AppState::WAITING_3PT_2;
                            break;

                        case AppState::WAITING_3PT_2:
                            wp.type = WpType::Arc3Pt; wp.group_id = current_arc_group; wp.group_index = 2;
                            temp_arc_points.push_back(wp);
                            current_state = AppState::WAITING_3PT_3;
                            break;

                        case AppState::WAITING_3PT_3:
                            wp.type = WpType::Arc3Pt; wp.group_id = current_arc_group; wp.group_index = 3;
                            temp_arc_points.push_back(wp);
                            waypoints.insert(waypoints.end(), temp_arc_points.begin(), temp_arc_points.end());
                            temp_arc_points.clear();
                            current_state = AppState::IDLE;
                            SaveSession();
                            break;

                        case AppState::WAITING_CENTER_1:
                            current_arc_group = next_group();
                            wp.type = WpType::ArcCenterAnchor; wp.group_id = current_arc_group; wp.group_index = 1;
                            temp_arc_points.push_back(wp);
                            current_state = AppState::WAITING_CENTER_2;
                            break;

                        case AppState::WAITING_CENTER_2:
                            wp.type = WpType::ArcCenterEnd; wp.group_id = current_arc_group; wp.group_index = 2;
                            wp.center_angle = 90.0f;
                            temp_arc_points.push_back(wp);
                            current_state = AppState::ADJUSTING_CENTER_ARC;
                            break;

                        default: break;
                        }
                    }
                }
            }

            if (ImGui::IsMouseClicked(ImGuiMouseButton_Right)) {
                const int click_tile_x = TileOf((io.MousePos.x - map_origin.x) / zoom);
                const int click_tile_y = TileOf((io.MousePos.y - map_origin.y) / zoom);

                for (auto it = waypoints.begin(); it != waypoints.end(); ++it) {
                    bool hit = false;
                    if (zoom > PRECISION_ZOOM) {
                        hit = (TileOf(it->raw_x) == click_tile_x && TileOf(it->raw_y) == click_tile_y);
                    }
                    else {
                        const float cxs = map_origin.x + (it->raw_x + 0.5f) * zoom;
                        const float cys = map_origin.y + (it->raw_y + 0.5f) * zoom;
                        const float dx = io.MousePos.x - cxs, dy = io.MousePos.y - cys;
                        hit = (dx * dx + dy * dy) < (15.0f * 15.0f);
                    }
                    if (!hit) continue;

                    if (it->type != WpType::Standard && it->group_id != -1) {
                        const int target_id = it->group_id;
                        waypoints.erase(
                            std::remove_if(waypoints.begin(), waypoints.end(),
                                [target_id](const Waypoint& w) { return w.group_id == target_id; }),
                            waypoints.end());
                    }
                    else {
                        waypoints.erase(it);
                    }
                    SaveSession();
                    break;
                }
            }
        }

        const float degStep = 3600.0f * zoom;
        const float minStep = 60.0f * zoom;
        const float secStep = 1.0f * zoom;

        if (degStep > 20.0f) {
            const ImU32 degColor = IM_COL32(255, 255, 255, 180);
            const ImU32 textColor = IM_COL32(255, 255, 255, 255);

            const int first_x = (int)((canvas_p0.x - map_origin.x) / degStep);
            const int last_x = (int)((canvas_p1.x - map_origin.x) / degStep);
            for (int px = first_x; px <= last_x; px++) {
                const float x = map_origin.x + px * degStep;
                if (x < canvas_p0.x || x > canvas_p1.x) continue;
                draw_list->AddLine(ImVec2(x, canvas_p0.y), ImVec2(x, canvas_p1.y), degColor, 2.0f);
                char label[16];
                snprintf(label, sizeof(label), "%d\xC2\xB0 E", ORIGIN_LON_DEG + px);
                draw_list->AddText(ImVec2(x + 5, canvas_p0.y + 5), textColor, label);
            }

            const int first_y = (int)((canvas_p0.y - map_origin.y) / degStep);
            const int last_y = (int)((canvas_p1.y - map_origin.y) / degStep);
            for (int py = first_y; py <= last_y; py++) {
                const float y = map_origin.y + py * degStep;
                if (y < canvas_p0.y || y > canvas_p1.y) continue;
                draw_list->AddLine(ImVec2(canvas_p0.x, y), ImVec2(canvas_p1.x, y), degColor, 2.0f);
                char label[16];
                snprintf(label, sizeof(label), "%d\xC2\xB0 N", ORIGIN_LAT_DEG - py);
                draw_list->AddText(ImVec2(canvas_p0.x + 5, y + 5), textColor, label);
            }
        }

        if (show_minute_grid && minStep > 10.0f) {
            const ImU32 minColor = IM_COL32(239, 63, 255, 127);
            const ImU32 textColor = IM_COL32(191, 191, 191, 220);
            const float center_x = canvas_p0.x + canvas_sz.x * 0.5f;
            const float center_y = canvas_p0.y + canvas_sz.y * 0.5f;

            int label_step = 1;
            if (minStep < 60.0f) label_step = 5;
            if (minStep < 20.0f) label_step = 15;

            const int first_x = (int)((canvas_p0.x - map_origin.x) / minStep);
            const int last_x = (int)((canvas_p1.x - map_origin.x) / minStep);
            for (int px = first_x; px <= last_x; px++) {
                if (px % 60 == 0) continue;
                const float x = map_origin.x + px * minStep;
                if (x < canvas_p0.x || x > canvas_p1.x) continue;
                draw_list->AddLine(ImVec2(x, canvas_p0.y), ImVec2(x, canvas_p1.y), minColor, 1.5f);
                if (px % label_step == 0) {
                    const int total = (ORIGIN_LON_DEG * 60) + px;
                    char label[32];
                    snprintf(label, sizeof(label), "E %d\xC2\xB0%02d'", total / 60, total % 60);
                    const ImVec2 ts = ImGui::CalcTextSize(label);
                    draw_list->AddText(ImVec2(x - ts.x * 0.5f, center_y - ts.y - 4.0f), textColor, label);
                }
            }

            const int first_y = (int)((canvas_p0.y - map_origin.y) / minStep);
            const int last_y = (int)((canvas_p1.y - map_origin.y) / minStep);
            for (int py = first_y; py <= last_y; py++) {
                if (py % 60 == 0) continue;
                const float y = map_origin.y + py * minStep;
                if (y < canvas_p0.y || y > canvas_p1.y) continue;
                draw_list->AddLine(ImVec2(canvas_p0.x, y), ImVec2(canvas_p1.x, y), minColor, 1.5f);
                if (py % label_step == 0) {
                    const int total = (ORIGIN_LAT_DEG * 60) - py;
                    char label[32];
                    snprintf(label, sizeof(label), "N %d\xC2\xB0%02d'", total / 60, total % 60);
                    const ImVec2 ts = ImGui::CalcTextSize(label);
                    draw_list->AddText(ImVec2(center_x + 6.0f, y - ts.y * 0.5f), textColor, label);
                }
            }
        }

        static float hover_timer = 0.0f;

        if (secStep > PRECISION_ZOOM) {
            precisionLockMet = true;
            ImGui::SetMouseCursor(ImGuiMouseCursor_Hand);

            if (show_second_grid) {
                const ImU32 secColor = IM_COL32(192, 96, 16, 95);
                const int first_x = (int)((canvas_p0.x - map_origin.x) / secStep);
                const int last_x = (int)((canvas_p1.x - map_origin.x) / secStep);
                for (int px = first_x; px <= last_x; px++) {
                    if (px % 60 == 0) continue;
                    const float x = map_origin.x + px * secStep;
                    if (x >= canvas_p0.x && x <= canvas_p1.x)
                        draw_list->AddLine(ImVec2(x, canvas_p0.y), ImVec2(x, canvas_p1.y), secColor, 1.0f);
                }
                const int first_y = (int)((canvas_p0.y - map_origin.y) / secStep);
                const int last_y = (int)((canvas_p1.y - map_origin.y) / secStep);
                for (int py = first_y; py <= last_y; py++) {
                    if (py % 60 == 0) continue;
                    const float y = map_origin.y + py * secStep;
                    if (y >= canvas_p0.y && y <= canvas_p1.y)
                        draw_list->AddLine(ImVec2(canvas_p0.x, y), ImVec2(canvas_p1.x, y), secColor, 1.0f);
                }
            }

            if (is_hovered) {
                if (io.MouseDelta.x == 0.0f && io.MouseDelta.y == 0.0f) hover_timer += io.DeltaTime;
                else hover_timer = 0.0f;

                if (hover_timer > 0.2f) {
                    const int raw_x = TileOf((io.MousePos.x - map_origin.x) / zoom);
                    const int raw_y = TileOf((io.MousePos.y - map_origin.y) / zoom);
                    if (InMap(raw_x, raw_y)) {
                        const int lon = (ORIGIN_LON_DEG * 3600) + raw_x;
                        const int lat = (ORIGIN_LAT_DEG * 3600) - raw_y;
                        ImGui::BeginTooltip();
                        ImGui::Text("%d\xC2\xB0 %d' %d\" N\n%d\xC2\xB0 %d' %d\" E\nElev: %.1f m",
                            lat / 3600, (lat % 3600) / 60, lat % 60,
                            lon / 3600, (lon % 3600) / 60, lon % 60,
                            SampleElevation(raw_x, raw_y));
                        ImGui::EndTooltip();
                    }
                }
            }
            else hover_timer = 0.0f;
        }
        else {
            precisionLockMet = false;
            hover_timer = 0.0f;
        }

        const float meters_per_pixel = METERS_PER_TILE / zoom;
        const int tier_interval = std::max(50, (int)std::lround(meters_per_pixel / 10.0f) * 50);

        if (minStep > 10.0f && elevation_data != nullptr) {
            const int map_step = std::max(1, (int)std::ceil(MIN_CELL_PX / secStep));

            int first_x = std::max(0, (int)std::floor((canvas_p0.x - map_origin.x) / secStep));
            int last_x = std::min(MAP_W - 1, (int)std::ceil((canvas_p1.x - map_origin.x) / secStep));
            int first_y = std::max(0, (int)std::floor((canvas_p0.y - map_origin.y) / secStep));
            int last_y = std::min(MAP_H - 1, (int)std::ceil((canvas_p1.y - map_origin.y) / secStep));

            first_x -= (first_x % map_step);
            first_y -= (first_y % map_step);

            if (last_x > first_x && last_y > first_y) {
                if (first_x != bk_fx || last_x != bk_lx || first_y != bk_fy ||
                    last_y != bk_ly || map_step != bk_step || tier_interval != bk_tier)
                {
                    RebuildBandRuns(first_x, last_x, first_y, last_y, map_step, tier_interval);
                    bk_fx = first_x; bk_lx = last_x; bk_fy = first_y; bk_ly = last_y;
                    bk_step = map_step; bk_tier = tier_interval;
                }

                const ImU32 bandColor = IM_COL32(0, 0, 0, 35);
                for (const BandRun& r : band_runs) {
                    const float x1 = std::floor(map_origin.x + (float)r.x0 * secStep);
                    const float x2 = std::floor(map_origin.x + (float)r.x1 * secStep);
                    const float y1 = std::floor(map_origin.y + (float)r.y * secStep);
                    const float y2 = std::floor(map_origin.y + (float)(r.y + map_step) * secStep);
                    draw_list->AddRectFilled(ImVec2(x1, y1), ImVec2(x2, y2), bandColor);
                }
            }
        }

        {
            char legend_buf[64];
            snprintf(legend_buf, sizeof(legend_buf), "Contour Interval: %d m", tier_interval);
            const ImVec2 lp = ImVec2(canvas_p0.x + 15.0f, canvas_p1.y - 35.0f);
            draw_list->AddRectFilled(lp, ImVec2(lp.x + 180.0f, lp.y + 25.0f), IM_COL32(30, 30, 30, 200), 4.0f);
            draw_list->AddRect(lp, ImVec2(lp.x + 180.0f, lp.y + 25.0f), IM_COL32(100, 100, 100, 255), 4.0f);
            draw_list->AddText(ImVec2(lp.x + 10.0f, lp.y + 4.0f), IM_COL32(255, 255, 255, 255), legend_buf);
        }

        if (!elevation_ready) {
            const ImVec2 wp_ = ImVec2(canvas_p0.x + 15.0f, canvas_p0.y + 15.0f);
            const char* msg = elevation_error.empty() ? "Elevation data unavailable" : elevation_error.c_str();
            const ImVec2 ts = ImGui::CalcTextSize(msg);
            draw_list->AddRectFilled(wp_, ImVec2(wp_.x + ts.x + 20.0f, wp_.y + 26.0f), IM_COL32(120, 20, 20, 220), 4.0f);
            draw_list->AddText(ImVec2(wp_.x + 10.0f, wp_.y + 5.0f), IM_COL32(255, 220, 220, 255), msg);
        }

        const ImU32 outlineColor = IM_COL32(255, 255, 255, 200);
        const ImU32 markerColor = IM_COL32(255, 50, 50, 255);

        const std::vector<ImVec2>& flight_path = GetCachedFlightPath();
        const MissionProfile& mp = GetMissionProfile();

        auto mode_color = [](int m, int alpha) -> ImU32 {
            if (m == MODE_CLIMB)   return IM_COL32(255, 50, 50, alpha);
            if (m == MODE_DESCEND) return IM_COL32(50, 150, 255, alpha);
            return IM_COL32(50, 255, 50, alpha);
            };

        if (!mp.steps.empty() && flight_path.size() > 1) {

            if (zoom > PRECISION_ZOOM) {
                for (size_t s = 0; s < mp.steps.size(); s++) {
                    const ImU32 col = mode_color(mp.modes[s], 100);
                    const ProfileStep& st = mp.steps[s];
                    for (int ti = st.first; ti <= st.last; ti++) {
                        const ProfileTile& t = mp.tiles[ti];
                        const float sx = map_origin.x + t.x * zoom;
                        const float sy = map_origin.y + t.y * zoom;
                        if (sx + zoom < canvas_p0.x || sx > canvas_p1.x ||
                            sy + zoom < canvas_p0.y || sy > canvas_p1.y) continue;
                        draw_list->AddRectFilled(ImVec2(sx, sy), ImVec2(sx + zoom, sy + zoom), col);
                    }
                }
            }

            auto point_at = [&](int seg, float t) -> ImVec2 {
                const ImVec2& a = flight_path[seg - 1];
                const ImVec2& b = flight_path[seg];
                return ImVec2(map_origin.x + (a.x + (b.x - a.x) * t + 0.5f) * zoom,
                    map_origin.y + (a.y + (b.y - a.y) * t + 0.5f) * zoom);
                };
            auto vertex_at = [&](int idx) -> ImVec2 {
                return ImVec2(map_origin.x + (flight_path[idx].x + 0.5f) * zoom,
                    map_origin.y + (flight_path[idx].y + 0.5f) * zoom);
                };

            for (size_t s = 0; s < mp.steps.size(); s++) {
                const ProfileStep& st = mp.steps[s];
                const ProfileTile& t0 = mp.tiles[st.first];
                if (t0.seg <= 0 || t0.seg >= (int)flight_path.size()) continue;

                int   seg_b; float t_b;
                if (st.last + 1 < (int)mp.tiles.size()) {
                    seg_b = mp.tiles[st.last + 1].seg;
                    t_b = mp.tiles[st.last + 1].t;
                }
                else { seg_b = mp.tiles[st.last].seg; t_b = 1.0f; }

                if (seg_b < t0.seg || seg_b >= (int)flight_path.size()) { seg_b = t0.seg; t_b = 1.0f; }

                const ImU32 col = mode_color(mp.modes[s], 255);

                if (seg_b == t0.seg) {
                    draw_list->AddLine(point_at(t0.seg, t0.t), point_at(t0.seg, t_b), col, 3.0f);
                }
                else {
                    draw_list->AddLine(point_at(t0.seg, t0.t), vertex_at(t0.seg), col, 3.0f);
                    for (int sg = t0.seg + 1; sg < seg_b; sg++)
                        draw_list->AddLine(vertex_at(sg - 1), vertex_at(sg), col, 3.0f);
                    draw_list->AddLine(vertex_at(seg_b - 1), point_at(seg_b, t_b), col, 3.0f);
                }
            }
        }

        auto draw_marker = [&](float r_x, float r_y, const char* label, ImU32 color, bool is_center_dot) {
            const float cx = map_origin.x + (r_x + 0.5f) * zoom;
            const float cy = map_origin.y + (r_y + 0.5f) * zoom;
            if (cx < canvas_p0.x - 50 || cx > canvas_p1.x + 50 ||
                cy < canvas_p0.y - 50 || cy > canvas_p1.y + 50) return;

            if (zoom > PRECISION_ZOOM && !is_center_dot) {
                const float p0x = map_origin.x + std::floor(r_x) * zoom;
                const float p0y = map_origin.y + std::floor(r_y) * zoom;
                draw_list->AddRectFilled(ImVec2(p0x, p0y), ImVec2(p0x + zoom, p0y + zoom), color & 0x7FFFFFFF);
                draw_list->AddRect(ImVec2(p0x, p0y), ImVec2(p0x + zoom, p0y + zoom), color, 0.0f, 0, 1.5f);
            }
            else {
                const float arm = is_center_dot ? 4.0f : 10.0f;

                draw_list->AddCircleFilled(ImVec2(cx, cy), is_center_dot ? 2.0f : 2.5f, color);

                if (!is_center_dot) draw_list->AddCircle(ImVec2(cx, cy), 6.0f, outlineColor, 0, 1.5f);

                draw_list->AddLine(ImVec2(cx - arm, cy), ImVec2(cx + arm, cy), color, 2.0f);
                draw_list->AddLine(ImVec2(cx, cy - arm), ImVec2(cx, cy + arm), color, 2.0f);
            }

            draw_list->AddText(ImVec2(cx + 6, cy - 14), outlineColor, label);
            };

        int display_wp_idx = 1;
        for (size_t i = 0; i < waypoints.size(); ) {
            if (waypoints[i].type == WpType::Arc3Pt && i + 2 < waypoints.size()) {
                const std::string pfx = "3C" + std::to_string(waypoints[i].group_id);
                draw_marker(waypoints[i].raw_x, waypoints[i].raw_y, (pfx + "_Start").c_str(), markerColor, false);
                if (!(waypoints[i].center_angle > 0.5f))
                    draw_marker(waypoints[i + 1].raw_x, waypoints[i + 1].raw_y, (pfx + "_Mid").c_str(), markerColor, false);
                draw_marker(waypoints[i + 2].raw_x, waypoints[i + 2].raw_y, (pfx + "_End").c_str(), markerColor, false);

                double Xc, Yc, R, sa, sweep;
                if (Arc3PtGeometry(waypoints[i], waypoints[i + 1], waypoints[i + 2], Xc, Yc, R, sa, sweep)) {
                    draw_marker((float)Xc, (float)Yc, (pfx + "_Center").c_str(), IM_COL32(255, 165, 0, 255), true);
                }
                i += 3;
            }
            else if (waypoints[i].type == WpType::ArcCenterAnchor && i + 1 < waypoints.size()) {
                const std::string pfx = "C" + std::to_string(waypoints[i].group_id);
                draw_marker(waypoints[i].raw_x, waypoints[i].raw_y, (pfx + "_Start").c_str(), markerColor, false);
                draw_marker(waypoints[i + 1].raw_x, waypoints[i + 1].raw_y, (pfx + "_Center").c_str(), IM_COL32(255, 165, 0, 255), true);

                const double lat_rad = (ORIGIN_LAT_DEG - (waypoints[i].raw_y / 3600.0)) * (3.1415926535 / 180.0);
                const double cos_lat = std::cos(lat_rad);

                const double sx_iso = (double)waypoints[i].raw_x * cos_lat, sy = (double)waypoints[i].raw_y;
                const double cx_iso = (double)waypoints[i + 1].raw_x * cos_lat, cy = (double)waypoints[i + 1].raw_y;
                const double dx = sx_iso - cx_iso, dy = sy - cy;
                const double R = std::sqrt(dx * dx + dy * dy);
                const double ea = std::atan2(dy, dx) + (waypoints[i + 1].center_angle * (3.1415926535 / 180.0));

                draw_marker((float)((cx_iso + R * std::cos(ea)) / cos_lat), (float)(cy + R * std::sin(ea)), (pfx + "_End").c_str(), markerColor, false);
                i += 2;
            }
            else {
                draw_marker(waypoints[i].raw_x, waypoints[i].raw_y,
                    ("WP " + std::to_string(display_wp_idx++)).c_str(), markerColor, false);
                i++;
            }
        }

        for (const auto& p : temp_arc_points)
            draw_marker(p.raw_x, p.raw_y, "Pending", IM_COL32(255, 165, 0, 255), false);

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
                ImGui::SliderFloat("##sweep_slider", &temp_arc_points[1].center_angle, -1080.0f, 1080.0f, "%.1f deg");
                ImGui::InputFloat("##sweep_input", &temp_arc_points[1].center_angle, 1.0f, 15.0f, "%.1f");

                if (ImGui::Button("Commit Arc", ImVec2(120, 0))) {
                    waypoints.insert(waypoints.end(), temp_arc_points.begin(), temp_arc_points.end());
                    temp_arc_points.clear();
                    current_state = AppState::IDLE;
                    SaveSession();
                }
            }
            else {
                ImGui::TextColored(ImVec4(1.0f, 0.8f, 0.0f, 1.0f),
                    current_state <= AppState::WAITING_3PT_3 ? "Waiting for 3-Point Arc selection..."
                    : "Waiting for Center Arc selection...");
            }
        }
        else {
            ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.2f, 0.5f, 0.8f, 1.0f));
            ImGui::Button("Straight Line");
            ImGui::PopStyleColor();
            ImGui::SameLine();
            if (ImGui::Button("3-Point Arc")) { current_state = AppState::WAITING_3PT_1; temp_arc_points.clear(); }
            ImGui::SameLine();
            if (ImGui::Button("Center Arc")) { current_state = AppState::WAITING_CENTER_1; temp_arc_points.clear(); }
        }

        ImGui::Separator();
        ImGui::Text("Mission Generation Parameters:");

        ImGui::SliderInt("Mode Threshold", &mode_threshold, 5, 100, "%d m");
        if (ImGui::IsItemDeactivatedAfterEdit()) SaveSession();
        ImGui::SliderInt("Tile Offset", &tile_offset, 0, 20, "%d tiles");
        if (ImGui::IsItemDeactivatedAfterEdit()) SaveSession();
        ImGui::SliderInt("Step Grouping", &step_grouping, 1, 20, "%d steps");
        if (ImGui::IsItemDeactivatedAfterEdit()) SaveSession();


        ImGui::Separator();
        const bool can_export = waypoints.size() >= 2 &&
            map_loaded.load(std::memory_order_acquire) && elevation_ready;
        if (can_export) {
            if (ImGui::Button("Export F-Code")) ExportMissionFCode();
        }
        else {
            ImGui::BeginDisabled();
            ImGui::Button("Export F-Code");
            ImGui::EndDisabled();
            if (!map_loaded.load(std::memory_order_acquire)) {
                ImGui::SameLine();
                ImGui::TextColored(ImVec4(1.0f, 0.6f, 0.0f, 1.0f), " (Parsing Terrain...)");
            }
            else if (!elevation_ready) {
                ImGui::SameLine();
                ImGui::TextColored(ImVec4(1.0f, 0.3f, 0.3f, 1.0f), " (No elevation data)");
            }
        }

        if (waypoints.empty()) {
            ImGui::Separator();
            ImGui::TextColored(ImVec4(0.5f, 0.5f, 0.5f, 1.0f), "No waypoints deployed.");
            ImGui::Spacing();
            return;
        }

        ImGui::Separator();
        ImGui::Text("Selected Waypoints:");

        if (ImGui::BeginChild("WPList", ImVec2(0, 150), true, ImGuiWindowFlags_HorizontalScrollbar)) {
            int display_wp_idx = 1;
            for (size_t i = 0; i < waypoints.size(); ) {
                const int lon = (ORIGIN_LON_DEG * 3600) + TileOf(waypoints[i].raw_x);
                const int lat = (ORIGIN_LAT_DEG * 3600) - TileOf(waypoints[i].raw_y);
                const float elev = SampleElevation(TileOf(waypoints[i].raw_x), TileOf(waypoints[i].raw_y));

                if (waypoints[i].type == WpType::Arc3Pt && i + 2 < waypoints.size()) {
                    double Xc, Yc, R, sa, sweep;
                    const double deg = Arc3PtGeometry(waypoints[i], waypoints[i + 1], waypoints[i + 2],
                        Xc, Yc, R, sa, sweep) ? sweep * (180.0 / 3.1415926535) : 0.0;
                    ImGui::Text("3P-Arc (3C%d): %d\xC2\xB0%02d'%02d\"N  %d\xC2\xB0%02d'%02d\"E  | %.1f\xC2\xB0",
                        waypoints[i].group_id, lat / 3600, (lat % 3600) / 60, lat % 60,
                        lon / 3600, (lon % 3600) / 60, lon % 60, deg);
                    i += 3;
                }
                else if (waypoints[i].type == WpType::ArcCenterAnchor && i + 1 < waypoints.size()) {
                    ImGui::Text("Cent-Arc (C%d): %d\xC2\xB0%02d'%02d\"N  %d\xC2\xB0%02d'%02d\"E | %.1f\xC2\xB0",
                        waypoints[i].group_id, lat / 3600, (lat % 3600) / 60, lat % 60,
                        lon / 3600, (lon % 3600) / 60, lon % 60, waypoints[i + 1].center_angle);
                    i += 2;
                }
                else {
                    ImGui::Text("WP %d:         %d\xC2\xB0%02d'%02d\"N  %d\xC2\xB0%02d'%02d\"E  |  %.1fm",
                        display_wp_idx++, lat / 3600, (lat % 3600) / 60, lat % 60,
                        lon / 3600, (lon % 3600) / 60, lon % 60, elev);
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
            auto bridge = [&](size_t idx) {
                if (idx == 0) return;
                const double lat_rad = (ORIGIN_LAT_DEG - (waypoints[idx].raw_y / 3600.0)) * (3.1415926535 / 180.0);
                const double cos_lat = std::cos(lat_rad);
                const float ddx = (waypoints[idx].raw_x - last_x) * (float)cos_lat;
                const float ddy = waypoints[idx].raw_y - last_y;
                const float d = std::sqrt(ddx * ddx + ddy * ddy) * METERS_PER_TILE;
                if (d > 0.1f) { ImGui::Text("Line: %.1f m", d); total_distance += d; }
                };

            if (waypoints[i].type == WpType::Arc3Pt && i + 2 < waypoints.size()) {
                bridge(i);
                double Xc, Yc, R, sa, sweep;
                float arc_len = 0.0f;
                if (Arc3PtGeometry(waypoints[i], waypoints[i + 1], waypoints[i + 2], Xc, Yc, R, sa, sweep))
                    arc_len = (float)(R * std::abs(sweep)) * METERS_PER_TILE;

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
                bridge(i);

                const double lat_rad = (ORIGIN_LAT_DEG - (waypoints[i].raw_y / 3600.0)) * (3.1415926535 / 180.0);
                const double cos_lat = std::cos(lat_rad);
                const double sx_iso = (double)waypoints[i].raw_x * cos_lat, sy = (double)waypoints[i].raw_y;
                const double cx_iso = (double)waypoints[i + 1].raw_x * cos_lat, cy = (double)waypoints[i + 1].raw_y;
                const double dx = sx_iso - cx_iso, dy = sy - cy;
                const double R = std::sqrt(dx * dx + dy * dy);
                const double sweep = waypoints[i + 1].center_angle * (3.1415926535 / 180.0);
                const float arc_len = (float)(R * std::abs(sweep)) * METERS_PER_TILE;

                ImGui::Text("Cent-Arc (C%d): %.1f m", waypoints[i].group_id, arc_len);
                ImGui::SameLine(ImGui::GetWindowWidth() - 60.0f);
                ImGui::PushID((int)i);
                if (ImGui::Button("Flip")) {
                    waypoints[i + 1].center_angle = -waypoints[i + 1].center_angle;
                    SaveSession();
                }
                ImGui::PopID();

                total_distance += arc_len;
                const double ea = std::atan2(dy, dx) + sweep;
                last_x = (float)((cx_iso + R * std::cos(ea)) / cos_lat);
                last_y = (float)(cy + R * std::sin(ea));
                i += 2;
            }
            else {
                bridge(i);
                last_x = waypoints[i].raw_x;
                last_y = waypoints[i].raw_y;
                i++;
            }
        }

        ImGui::Separator();
        ImGui::TextColored(ImVec4(0.0f, 1.0f, 0.0f, 1.0f), "Total Distance: %.1f m", total_distance);

        const MissionProfile& mp = GetMissionProfile();
        if (!mp.steps.empty()) {
            int n_climb = 0, n_desc = 0, n_level = 0;
            for (size_t s = 0; s < mp.steps.size(); s++) {
                if (mp.modes[s] == MODE_CLIMB) n_climb += mp.steps[s].count;
                else if (mp.modes[s] == MODE_DESCEND) n_desc += mp.steps[s].count;
                else n_level += mp.steps[s].count;
            }
            ImGui::TextDisabled("%zu tiles / %zu steps\nflat %d  climb %d  descend %d",
                mp.tiles.size(), mp.steps.size(), n_level, n_climb, n_desc);
            if (mp.left_map)
                ImGui::TextColored(ImVec4(1.0f, 0.6f, 0.0f, 1.0f),
                    "Warning: path leaves the DEM coverage area.");
        }
        ImGui::Spacing();
    }

    static const uint32_t SESSION_MAGIC = 0x4E564152u;
    static const uint32_t SESSION_VERSION = 2u;
    static const uint64_t SESSION_MAX_WPS = 100000u;

    void SaveSession() {
        std::ofstream file("mission_cache.dat", std::ios::binary);
        if (!file) return;

        const uint32_t wp_stride = (uint32_t)sizeof(Waypoint);
        file.write((const char*)&SESSION_MAGIC, sizeof(SESSION_MAGIC));
        file.write((const char*)&SESSION_VERSION, sizeof(SESSION_VERSION));
        file.write((const char*)&wp_stride, sizeof(wp_stride));

        file.write((const char*)&show_minute_grid, sizeof(bool));
        file.write((const char*)&show_second_grid, sizeof(bool));
        file.write((const char*)&mode_threshold, sizeof(int));
        file.write((const char*)&tile_offset, sizeof(int));
        file.write((const char*)&step_grouping, sizeof(int));

        const uint64_t wp_count = (uint64_t)waypoints.size();
        file.write((const char*)&wp_count, sizeof(wp_count));
        if (wp_count) file.write((const char*)waypoints.data(), (std::streamsize)(wp_count * sizeof(Waypoint)));
    }

    void LoadSession() {
        std::ifstream file("mission_cache.dat", std::ios::binary);
        if (!file) return;

        uint32_t magic = 0, version = 0, stride = 0;
        file.read((char*)&magic, sizeof(magic));
        file.read((char*)&version, sizeof(version));
        file.read((char*)&stride, sizeof(stride));
        if (!file || magic != SESSION_MAGIC || version != SESSION_VERSION ||
            stride != (uint32_t)sizeof(Waypoint))
            return;

        file.read((char*)&show_minute_grid, sizeof(bool));
        file.read((char*)&show_second_grid, sizeof(bool));
        file.read((char*)&mode_threshold, sizeof(int));
        file.read((char*)&tile_offset, sizeof(int));
        file.read((char*)&step_grouping, sizeof(int));
        if (!file) return;

        uint64_t wp_count = 0;
        file.read((char*)&wp_count, sizeof(wp_count));
        if (!file || wp_count > SESSION_MAX_WPS) return;

        std::vector<Waypoint> loaded((size_t)wp_count);
        if (wp_count) {
            file.read((char*)loaded.data(), (std::streamsize)(wp_count * sizeof(Waypoint)));
            if (!file) return;
        }

        mode_threshold = Clamp(mode_threshold, 5, 100);
        tile_offset = Clamp(tile_offset, 0, 20);
        step_grouping = Clamp(step_grouping, 1, 20);

        for (auto& w : loaded) {
            w.raw_x = Clamp(w.raw_x, 0.0f, (float)(MAP_W - 1));
            w.raw_y = Clamp(w.raw_y, 0.0f, (float)(MAP_H - 1));
        }

        waypoints.swap(loaded);
        current_arc_group = 1;
        for (const auto& w : waypoints) current_arc_group = std::max(current_arc_group, w.group_id + 1);
        InvalidateProfile();
    }

    void ExportMissionFCode() {
        if (waypoints.size() < 2) return;
        if (!elevation_ready) return;

        auto format_wp = [](const Waypoint& wp) {
            const int lon = (ORIGIN_LON_DEG * 3600) + TileOf(wp.raw_x);
            const int lat = (ORIGIN_LAT_DEG * 3600) - TileOf(wp.raw_y);
            char buf[32];
            snprintf(buf, sizeof(buf), "%02d%02d%02dN%02d%02d%02dE",
                lat / 3600, (lat % 3600) / 60, lat % 60,
                lon / 3600, (lon % 3600) / 60, lon % 60);
            return std::string(buf);
            };

        const std::string default_name = format_wp(waypoints.front()) + "-" +
            format_wp(waypoints.back()) + "-" +
            std::to_string(waypoints.size()) + ".fcode";

        OPENFILENAMEA ofn;
        char szFile[MAX_PATH];
        snprintf(szFile, sizeof(szFile), "%s", default_name.c_str());

        ZeroMemory(&ofn, sizeof(ofn));
        ofn.lStructSize = sizeof(ofn);
        ofn.hwndOwner = NULL;
        ofn.lpstrFile = szFile;
        ofn.nMaxFile = sizeof(szFile);
        ofn.lpstrFilter = "F-Code Files\0*.fcode\0All Files\0*.*\0";
        ofn.nFilterIndex = 1;
        ofn.lpstrDefExt = "fcode";
        ofn.Flags = OFN_PATHMUSTEXIST | OFN_OVERWRITEPROMPT;

        if (GetSaveFileNameA(&ofn) != TRUE) return;

        std::ofstream file(ofn.lpstrFile);
        if (!file.is_open()) return;

        std::string out;
        out.reserve(waypoints.size() * 64 + 128);
        out += "%\n";

        char line[128];
        snprintf(line, sizeof(line), "F43 Z%d;\n", mode_threshold * 4);
        out += line;

        const int start_lon_sec = (ORIGIN_LON_DEG * 3600) + TileOf(waypoints[0].raw_x);
        const int start_lat_sec = (ORIGIN_LAT_DEG * 3600) - TileOf(waypoints[0].raw_y);
        snprintf(line, sizeof(line), "F90 X%02d%02d%02d Y%02d%02d%02d;\n",
            start_lon_sec / 3600, (start_lon_sec % 3600) / 60, start_lon_sec % 60,
            start_lat_sec / 3600, (start_lat_sec % 3600) / 60, start_lat_sec % 60);
        out += line;

        double current_x = waypoints[0].raw_x;
        double current_y = waypoints[0].raw_y;

        for (size_t i = 0; i < waypoints.size(); ) {
            const double lat_rad = (ORIGIN_LAT_DEG - (waypoints[i].raw_y / 3600.0)) * (3.1415926535 / 180.0);
            const double cos_lat = std::cos(lat_rad);

            if (waypoints[i].type == WpType::Arc3Pt && i + 2 < waypoints.size()) {
                double Xc, Yc, R, sa, sweep;
                if (Arc3PtGeometry(waypoints[i], waypoints[i + 1], waypoints[i + 2], Xc, Yc, R, sa, sweep)) {

                    int arc_type = (sweep >= 0.0) ? 2 : 3;
                    double R_meters = R * METERS_PER_TILE;
                    double sweep_deg = std::abs(sweep) * (180.0 / 3.1415926535);

                    double ex = waypoints[i + 2].raw_x;
                    double ey = waypoints[i + 2].raw_y;

                    int end_lon = (ORIGIN_LON_DEG * 3600) + TileOf((float)ex);
                    int end_lat = (ORIGIN_LAT_DEG * 3600) - TileOf((float)ey);

                    int c_lon = (ORIGIN_LON_DEG * 3600) + (int)std::round(Xc);
                    int c_lat = (ORIGIN_LAT_DEG * 3600) - (int)std::round(Yc);

                    double dx = waypoints[i].raw_x - current_x;
                    double dy = -(waypoints[i].raw_y - current_y);

                    if (std::abs(dx) > 0.1 || std::abs(dy) > 0.1) {
                        snprintf(line, sizeof(line), "F01 X%.1f Y%.1f;\n", dx, dy);
                        out += line;
                    }

                    snprintf(line, sizeof(line), "F%02d I%02d%02d%02d J%02d%02d%02d R%.1f A%.1f X%02d%02d%02d Y%02d%02d%02d;\n",
                        arc_type,
                        c_lon / 3600, (c_lon % 3600) / 60, c_lon % 60,
                        c_lat / 3600, (c_lat % 3600) / 60, c_lat % 60,
                        R_meters, sweep_deg,
                        end_lon / 3600, (end_lon % 3600) / 60, end_lon % 60,
                        end_lat / 3600, (end_lat % 3600) / 60, end_lat % 60);
                    out += line;

                    current_x = ex;
                    current_y = ey;
                }
                i += 3;
            }
            else if (waypoints[i].type == WpType::ArcCenterAnchor && i + 1 < waypoints.size()) {
                double sx_iso = (double)waypoints[i].raw_x * cos_lat, sy = (double)waypoints[i].raw_y;
                double cx_iso = (double)waypoints[i + 1].raw_x * cos_lat, cy = (double)waypoints[i + 1].raw_y;
                double dx_iso = sx_iso - cx_iso, dy_iso = sy - cy;
                double R = std::sqrt(dx_iso * dx_iso + dy_iso * dy_iso);

                double start_angle = std::atan2(dy_iso, dx_iso);
                double sweep_angle = waypoints[i + 1].center_angle * (3.1415926535 / 180.0);
                int arc_type = (sweep_angle >= 0.0) ? 2 : 3;
                double R_meters = R * METERS_PER_TILE;
                double sweep_deg = std::abs(waypoints[i + 1].center_angle);

                double end_a = start_angle + sweep_angle;
                double ex = (cx_iso + R * std::cos(end_a)) / cos_lat;
                double ey = cy + R * std::sin(end_a);

                int end_lon = (ORIGIN_LON_DEG * 3600) + TileOf((float)ex);
                int end_lat = (ORIGIN_LAT_DEG * 3600) - TileOf((float)ey);

                int c_lon = (ORIGIN_LON_DEG * 3600) + (int)std::round(waypoints[i + 1].raw_x);
                int c_lat = (ORIGIN_LAT_DEG * 3600) - (int)std::round(waypoints[i + 1].raw_y);

                double dx = waypoints[i].raw_x - current_x;
                double dy = -(waypoints[i].raw_y - current_y);

                if (std::abs(dx) > 0.1 || std::abs(dy) > 0.1) {
                    snprintf(line, sizeof(line), "F01 X%.1f Y%.1f;\n", dx, dy);
                    out += line;
                }

                snprintf(line, sizeof(line), "F%02d I%02d%02d%02d J%02d%02d%02d R%.1f A%.1f X%02d%02d%02d Y%02d%02d%02d;\n",
                    arc_type,
                    c_lon / 3600, (c_lon % 3600) / 60, c_lon % 60,
                    c_lat / 3600, (c_lat % 3600) / 60, c_lat % 60,
                    R_meters, sweep_deg,
                    end_lon / 3600, (end_lon % 3600) / 60, end_lon % 60,
                    end_lat / 3600, (end_lat % 3600) / 60, end_lat % 60);
                out += line;

                current_x = ex;
                current_y = ey;
                i += 2;
            }
            else {
                double dx = waypoints[i].raw_x - current_x;
                double dy = -(waypoints[i].raw_y - current_y);

                if (std::abs(dx) > 0.1 || std::abs(dy) > 0.1) {
                    snprintf(line, sizeof(line), "F01 X%.1f Y%.1f;\n", dx, dy);
                    out += line;
                }
                current_x = waypoints[i].raw_x;
                current_y = waypoints[i].raw_y;
                i++;
            }
        }

        out += "F39;\n%\n";
        file << out;
        file.close();
    }

}
