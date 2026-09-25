#include "imgui.h"
#include "AppUI.h"
#include "MapCanvas.h"

void RenderAppUI() {
    ImGuiViewport* viewport = ImGui::GetMainViewport();
    ImGui::SetNextWindowPos(viewport->WorkPos);
    ImGui::SetNextWindowSize(viewport->WorkSize);

    ImGuiWindowFlags window_flags = ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove;
    ImGui::Begin("F-Code Generator", nullptr, window_flags);

    // --- Build the Left Control Panel ---
    ImGui::BeginChild("ControlPanel", ImVec2(400, 0), true);
    ImGui::Text("Waypoint Configuration");
    ImGui::Separator();

    // Visual toggles linked to the Ctrl+M / Ctrl+S shortcuts
    ImGui::Checkbox("Arc-Minute Grid (Ctrl+M)", &MapCanvas::show_minute_grid);
    ImGui::Checkbox("Arc-Second Grid (Ctrl+S)", &MapCanvas::show_second_grid);
    ImGui::Separator();

    // --- Render the Unified MapCanvas UI ---
    MapCanvas::RenderControlPanelUI();

    ImGui::EndChild();

    ImGui::SameLine();

    // --- Build the Right Map Panel ---
    ImGui::BeginChild("MapPanel", ImVec2(0, 0), true);
    MapCanvas::RenderInteractiveMap();
    ImGui::EndChild();

    ImGui::End();
}
