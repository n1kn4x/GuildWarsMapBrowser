#include "pch.h"
#include "draw_route_planner_panel.h"

#include "GuiGlobalConstants.h"
#include "draw_pathfinding_panel.h"
#include "FFNA_MapFile.h"
#include <algorithm>
#include <commdlg.h>
#include <filesystem>

extern FFNA_MapFile selected_ffna_map_file;
extern FileType selected_file_type;
extern int selected_map_file_index;

namespace {
    constexpr float kAggroRadius = 120.0f;
    constexpr float kCoverageSpacingFactor = 0.5f; // 50% overlap so coverage matches the full bubble

    std::wstring OpenSaveFileDialog(const std::wstring& default_name, const std::wstring& extension) {
        OPENFILENAMEW ofn;
        wchar_t szFile[260] = {0};
        wcscpy_s(szFile, default_name.c_str());

        ZeroMemory(&ofn, sizeof(ofn));
        ofn.lStructSize = sizeof(ofn);
        ofn.hwndOwner = NULL;
        ofn.lpstrFile = szFile;
        ofn.nMaxFile = sizeof(szFile) / sizeof(wchar_t);

        std::wstring filter = L"CSV Files\0*.csv\0All Files\0*.*\0";
        ofn.lpstrFilter = filter.c_str();
        ofn.nFilterIndex = 1;
        ofn.lpstrDefExt = extension.c_str();
        ofn.Flags = OFN_PATHMUSTEXIST | OFN_OVERWRITEPROMPT;

        if (GetSaveFileNameW(&ofn)) {
            return std::wstring(szFile);
        }
        return L"";
    }

    bool WriteCsv(const std::wstring& save_path, const std::vector<RouteWaypoint>& waypoints) {
        std::ofstream file(std::filesystem::path(save_path));
        if (!file.is_open()) {
            return false;
        }
        file << "index,x,y\n";
        for (size_t i = 0; i < waypoints.size(); ++i) {
            file << i << "," << waypoints[i].x << "," << waypoints[i].y << "\n";
        }
        return true;
    }

    bool TryAddWaypointFromImageClick(const PathfindingVisualizer& visualizer,
                                      const ImVec2& image_min,
                                      const ImVec2& image_size,
                                      const ImVec2& mouse_pos,
                                      RouteWaypoint& out_waypoint) {
        if (image_size.x <= 0.0f || image_size.y <= 0.0f) {
            return false;
        }

        const float rel_x = (mouse_pos.x - image_min.x) / image_size.x;
        const float rel_y = (mouse_pos.y - image_min.y) / image_size.y;

        if (rel_x < 0.0f || rel_x > 1.0f || rel_y < 0.0f || rel_y > 1.0f) {
            return false;
        }

        const float img_px = rel_x * image_size.x;
        const float img_py = rel_y * image_size.y;

        const float raw_x = visualizer.GetMinX() + (img_px / image_size.x) *
            (visualizer.GetMaxX() - visualizer.GetMinX());
        const float raw_y = visualizer.GetMinY() + ((image_size.y - img_py) / image_size.y) *
            (visualizer.GetMaxY() - visualizer.GetMinY());

        out_waypoint.x = raw_x;
        out_waypoint.y = raw_y;
        return true;
    }

    void DrawWaypointOverlay(const std::vector<RouteWaypoint>& waypoints,
                             const PathfindingVisualizer& visualizer,
                             const ImVec2& image_min,
                             const ImVec2& image_size,
                             bool show_lines,
                             bool show_coverage) {
        if (image_size.x <= 0.0f || image_size.y <= 0.0f) {
            return;
        }

        const float span_x = visualizer.GetMaxX() - visualizer.GetMinX();
        const float span_y = visualizer.GetMaxY() - visualizer.GetMinY();
        if (span_x <= 0.0f || span_y <= 0.0f) {
            return;
        }

        const float px_per_world_x = image_size.x / span_x;
        const float px_per_world_y = image_size.y / span_y;
        const float aggro_radius_px = kAggroRadius * std::min(px_per_world_x, px_per_world_y);
        const float coverage_radius_px = aggro_radius_px * kCoverageSpacingFactor;

        ImDrawList* draw_list = ImGui::GetWindowDrawList();
        for (size_t i = 0; i < waypoints.size(); ++i) {
            const float px = (waypoints[i].x - visualizer.GetMinX()) * px_per_world_x;
            const float py = (visualizer.GetMaxY() - waypoints[i].y) * px_per_world_y;
            const ImVec2 center = ImVec2(image_min.x + px, image_min.y + py);

            if (show_coverage) {
                draw_list->AddCircle(center, aggro_radius_px, IM_COL32(255, 180, 0, 120), 40, 2.0f);
                draw_list->AddCircleFilled(center, coverage_radius_px, IM_COL32(255, 140, 0, 40), 40);
            }

            draw_list->AddCircleFilled(center, 5.0f, IM_COL32(0, 200, 255, 200), 12);
            draw_list->AddText(ImVec2(center.x + 6.0f, center.y - 10.0f),
                               IM_COL32(255, 255, 255, 220),
                               std::to_string(i + 1).c_str());

            if (show_lines && i > 0) {
                const float prev_px = (waypoints[i - 1].x - visualizer.GetMinX()) * px_per_world_x;
                const float prev_py = (visualizer.GetMaxY() - waypoints[i - 1].y) * px_per_world_y;
                const ImVec2 prev_center = ImVec2(image_min.x + prev_px, image_min.y + prev_py);
                draw_list->AddLine(prev_center, center, IM_COL32(0, 220, 255, 180), 2.0f);
            }
        }
    }
}

void draw_route_planner_panel(MapRenderer* map_renderer) {
    if (!GuiGlobalConstants::is_route_planner_panel_open) {
        return;
    }

    const float min_width = 420.0f;
    const float min_height = 220.0f;
    ImGui::SetNextWindowSizeConstraints(ImVec2(min_width, min_height), ImVec2(FLT_MAX, FLT_MAX));

    static std::vector<RouteWaypoint> waypoints;
    static bool show_lines = true;
    static bool show_coverage = true;
    static bool click_to_add = true;
    static bool has_last_click = false;
    static RouteWaypoint last_click;

    if (ImGui::Begin("Route Planner", &GuiGlobalConstants::is_route_planner_panel_open, ImGuiWindowFlags_NoFocusOnAppearing)) {
        GuiGlobalConstants::ClampWindowToScreen();

        if (selected_file_type != FFNA_Type3) {
            ImGui::TextWrapped("No pathfinding data loaded.");
            ImGui::TextWrapped("Load a map file (FFNA Type3) from the DAT browser to plan routes.");
            ImGui::End();
            return;
        }

        auto* visualizer = GetPathfindingVisualizer();
        if (!visualizer->IsReady()) {
            if (selected_ffna_map_file.pathfinding_chunk.valid) {
                visualizer->GenerateImage(selected_ffna_map_file.pathfinding_chunk, 1024);
                visualizer->CreateTexture(map_renderer->GetTextureManager());
            }
        }

        ImGui::Text("Waypoints: %zu", waypoints.size());
        ImGui::SameLine();
        ImGui::Text("Aggro radius: %.0f", kAggroRadius);

        ImGui::Separator();

        ImGui::Checkbox("Click on map to add waypoint", &click_to_add);
        ImGui::Checkbox("Show route lines", &show_lines);
        ImGui::Checkbox("Show aggro coverage", &show_coverage);
        ImGui::TextWrapped("Clicks report map world coordinates (X/Y) based on the pathfinding data.");
        ImGui::Separator();

        int tex_id = visualizer->GetTextureId();
        if (tex_id >= 0 && visualizer->IsReady()) {
            ID3D11ShaderResourceView* texture = map_renderer->GetTextureManager()->GetTexture(tex_id);
            if (texture) {
                ImVec2 window_size = ImGui::GetContentRegionAvail();
                float img_width = static_cast<float>(visualizer->GetWidth());
                float img_height = static_cast<float>(visualizer->GetHeight());

                float scale_x = (window_size.x - 20) / img_width;
                float scale_y = (window_size.y - 120) / img_height;
                float scale = std::min(scale_x, scale_y);
                scale = std::max(0.1f, scale);

                ImVec2 scaled_size(img_width * scale, img_height * scale);
                ImVec2 image_min = ImGui::GetCursorScreenPos();
                ImGui::Image((ImTextureID)texture, scaled_size);
                ImVec2 image_max = ImVec2(image_min.x + scaled_size.x, image_min.y + scaled_size.y);

                DrawWaypointOverlay(waypoints, *visualizer, image_min, scaled_size, show_lines, show_coverage);

                if (ImGui::IsItemHovered() && ImGui::IsMouseClicked(ImGuiMouseButton_Left)) {
                    RouteWaypoint waypoint;
                    if (TryAddWaypointFromImageClick(*visualizer, image_min, scaled_size, ImGui::GetIO().MousePos, waypoint)) {
                        has_last_click = true;
                        last_click = waypoint;
                        if (click_to_add) {
                            waypoints.push_back(waypoint);
                        }
                    }
                }

                if (ImGui::BeginPopupContextItem("route_context_menu")) {
                    if (ImGui::MenuItem("Clear all waypoints")) {
                        waypoints.clear();
                    }
                    ImGui::EndPopup();
                }
            }
        } else {
            ImGui::Text("Generating visualization...");
        }

        ImGui::Separator();

        if (has_last_click) {
            ImGui::Text("Last click: (%.2f, %.2f)", last_click.x, last_click.y);
        } else {
            ImGui::Text("Last click: (n/a)");
        }

        ImGui::Separator();

        if (ImGui::Button("Undo last")) {
            if (!waypoints.empty()) {
                waypoints.pop_back();
            }
        }
        ImGui::SameLine();
        if (ImGui::Button("Clear")) {
            waypoints.clear();
        }
        ImGui::SameLine();
        if (ImGui::Button("Export CSV")) {
            if (!waypoints.empty()) {
                std::wstring default_name = std::format(L"route_waypoints_{}", selected_map_file_index);
                std::wstring save_path = OpenSaveFileDialog(default_name, L"csv");
                if (!save_path.empty()) {
                    WriteCsv(save_path, waypoints);
                }
            }
        }

        if (!waypoints.empty()) {
            ImGui::Separator();
            if (ImGui::BeginChild("route_waypoint_list", ImVec2(0, 120), true)) {
                for (size_t i = 0; i < waypoints.size(); ++i) {
                    ImGui::Text("%zu: (%.2f, %.2f)", i + 1, waypoints[i].x, waypoints[i].y);
                }
            }
            ImGui::EndChild();
        }
    }
    ImGui::End();
}
