#include "imgui.h"
#include "AppUI.h"
#include "MapCanvas.h"
#include <cmath>

void RenderAppUI() {
    ImGuiViewport* viewport = ImGui::GetMainViewport();
    ImGui::SetNextWindowPos(viewport->WorkPos);
    ImGui::SetNextWindowSize(viewport->WorkSize);

    ImGuiWindowFlags window_flags = ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove;
    ImGui::Begin("F-Code Generator", nullptr, window_flags);

    // --- Build the Left Control Panel ---
    ImGui::BeginChild("ControlPanel", ImVec2(350, 0), true);
    ImGui::Text("Waypoint Configuration");
    ImGui::Separator();

    // Visual toggles linked to the Ctrl+M / Ctrl+S shortcuts
    ImGui::Checkbox("Arc-Minute Grid (Ctrl+M)", &MapCanvas::show_minute_grid);
    ImGui::Checkbox("Arc-Second Grid (Ctrl+S)", &MapCanvas::show_second_grid);
    ImGui::Separator();

    // --- Distance Calculation Engine ---
    const auto& wps = MapCanvas::GetWaypoints();

    if (wps.empty()) {
        ImGui::TextDisabled("No waypoints deployed.");
    }
    else {
        float total_distance = 0.0f;
        ImGui::Text("Flight Path Metrics:");
        ImGui::Spacing();
        MapCanvas::RenderControlPanelUI();

        for (size_t i = 1; i < wps.size(); i++) {
            int d_lon = wps[i].lon_sec - wps[i - 1].lon_sec;
            int d_lat = wps[i].lat_sec - wps[i - 1].lat_sec;

            // Calculate average latitude in decimal degrees
            float lat_avg_deg = (wps[i].lat_sec + wps[i - 1].lat_sec) / (2.0f * 3600.0f);

            // Convert degrees to radians for C++ cos() function
            float correction = std::cos(lat_avg_deg * 3.14159265f / 180.0f);

            // Apply the 31.08m per arc-second scale
            float dx = d_lon * 31.08f * correction;
            float dy = d_lat * 31.08f;

            float seg_dist = std::sqrt(dx * dx + dy * dy);
            total_distance += seg_dist;

            ImGui::Text(" WP %zu -> WP %zu:  %.1f m", i, i + 1, seg_dist);
        }

        ImGui::Separator();
        ImGui::TextColored(ImVec4(0.2f, 1.0f, 0.2f, 1.0f), "Total Distance: %.1f m", total_distance);
    }

    ImGui::EndChild();

    ImGui::SameLine();

    // --- Build the Right Map Panel ---
    ImGui::BeginChild("MapPanel", ImVec2(0, 0), true);
    MapCanvas::RenderInteractiveMap();
    ImGui::EndChild();

    ImGui::End();
}
