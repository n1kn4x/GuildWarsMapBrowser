#include "pch.h"
#include "draw_route_planner_panel.h"

#include "DeviceResources.h"
#include "GuiGlobalConstants.h"
#include "draw_pathfinding_panel.h"
#include "FFNA_MapFile.h"
#include <algorithm>
#include <commdlg.h>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <sstream>

extern FFNA_MapFile selected_ffna_map_file;
extern FileType selected_file_type;
extern int selected_map_file_index;

namespace {
    constexpr float kSpellcastingRadius = 1085.0f;
    constexpr float kRouteHeightOffset = 35.0f;
    constexpr int kCircleSegments = 48;
    constexpr float kPi = 3.14159265358979323846f;
    constexpr float kTopDownCameraHeight = 80000.0f;
    constexpr float kTopDownNearZ = 100.0f;
    constexpr float kTopDownFarZ = 200000.0f;
    constexpr int kRouteMapImageSize = 1024;

    struct CameraStateSnapshot {
        CameraType camera_type;
        DirectX::XMFLOAT3 position;
        float pitch;
        float yaw;
        float fov;
        float view_width;
        float view_height;
        float near_z;
        float far_z;
    };

    CameraStateSnapshot CaptureCameraState(const Camera* camera) {
        CameraStateSnapshot snapshot{};
        snapshot.camera_type = camera->GetCameraType();
        snapshot.position = camera->GetPosition3f();
        snapshot.pitch = camera->GetPitch();
        snapshot.yaw = camera->GetYaw();
        snapshot.fov = camera->GetFovY();
        snapshot.view_width = camera->GetViewWidth();
        snapshot.view_height = camera->GetViewHeight();
        snapshot.near_z = camera->GetNearZ();
        snapshot.far_z = camera->GetFarZ();
        return snapshot;
    }

    void RestoreCameraState(MapRenderer* map_renderer, Camera* camera, const CameraStateSnapshot& snapshot) {
        if (snapshot.camera_type == CameraType::Orthographic) {
            map_renderer->SetFrustumAsOrthographic(snapshot.view_width, snapshot.view_height, snapshot.near_z,
                                                   snapshot.far_z);
        } else {
            const float aspect_ratio = camera->GetAspectRatio();
            map_renderer->SetFrustumAsPerspective(snapshot.fov, aspect_ratio, snapshot.near_z, snapshot.far_z);
        }
        camera->SetPosition(snapshot.position.x, snapshot.position.y, snapshot.position.z);
        camera->SetOrientation(snapshot.pitch, snapshot.yaw);
        map_renderer->Update(0.0f);
    }

    bool CaptureTextureToRgba(MapRenderer* map_renderer,
                              ID3D11Texture2D* texture,
                              std::vector<RGBA>& out_data,
                              int& out_width,
                              int& out_height) {
        if (!map_renderer || !texture) {
            return false;
        }

        DirectX::ScratchImage captured;
        HRESULT hr = DirectX::CaptureTexture(map_renderer->GetDevice(),
                                             map_renderer->GetDeviceContext(),
                                             texture, captured);
        if (FAILED(hr)) {
            return false;
        }

        const DirectX::Image* image = captured.GetImage(0, 0, 0);
        if (!image) {
            return false;
        }

        DirectX::ScratchImage converted;
        const DirectX::Image* final_image = image;
        if (image->format != DXGI_FORMAT_B8G8R8A8_UNORM) {
            HRESULT hr_convert = DirectX::Convert(*image, DXGI_FORMAT_B8G8R8A8_UNORM, DirectX::TEX_FILTER_DEFAULT,
                                                  DirectX::TEX_THRESHOLD_DEFAULT, converted);
            if (FAILED(hr_convert)) {
                return false;
            }
            final_image = converted.GetImage(0, 0, 0);
            if (!final_image) {
                return false;
            }
        }

        out_width = static_cast<int>(final_image->width);
        out_height = static_cast<int>(final_image->height);
        out_data.resize(static_cast<size_t>(out_width * out_height));
        std::memcpy(out_data.data(), final_image->pixels, out_data.size() * sizeof(RGBA));
        return true;
    }

    bool UpdateRoutePlannerMapTexture(MapRenderer* map_renderer,
                                      DX::DeviceResources* device_resources,
                                      PathfindingVisualizer& visualizer,
                                      int& route_map_texture_id) {
        if (!map_renderer || !device_resources) {
            return false;
        }
        auto* terrain = map_renderer->GetTerrain();
        if (!terrain) {
            return false;
        }

        if (!visualizer.IsMaskReady()) {
            if (selected_ffna_map_file.pathfinding_chunk.valid) {
                visualizer.GenerateMask(selected_ffna_map_file.pathfinding_chunk, kRouteMapImageSize);
            }
        }

        if (!visualizer.IsMaskReady()) {
            return false;
        }

        const int mask_width = visualizer.GetMaskWidth();
        const int mask_height = visualizer.GetMaskHeight();
        if (mask_width <= 0 || mask_height <= 0) {
            return false;
        }

        const float aspect_ratio = static_cast<float>(mask_width) / static_cast<float>(mask_height);
        device_resources->UpdateOffscreenResources(mask_width, mask_height, aspect_ratio, true);

        Camera* camera = map_renderer->GetCamera();
        const CameraStateSnapshot camera_snapshot = CaptureCameraState(camera);

        const bool prev_should_render_sky = map_renderer->GetShouldRenderSky();
        const bool prev_should_render_fog = map_renderer->GetShouldRenderFog();
        const bool prev_should_render_shadows = map_renderer->GetShouldRenderShadows();
        const bool prev_should_render_model_shadows = map_renderer->GetShouldRenderShadowsForModels();

        map_renderer->SetShouldRenderSky(false);
        map_renderer->SetShouldRenderFog(false);
        map_renderer->SetShouldRenderShadows(false);
        map_renderer->SetShouldRenderShadowsForModels(false);

        const float view_width = visualizer.GetMaxX() - visualizer.GetMinX();
        const float view_height = visualizer.GetMaxY() - visualizer.GetMinY();
        const float center_x = (visualizer.GetMinX() + visualizer.GetMaxX()) * 0.5f;
        const float center_z = (visualizer.GetMinY() + visualizer.GetMaxY()) * 0.5f;

        map_renderer->SetFrustumAsOrthographic(view_width, view_height, kTopDownNearZ, kTopDownFarZ);
        camera->SetOrientation(-90.0f * XM_PI / 180.0f, 0.0f);
        camera->SetPosition(center_x, kTopDownCameraHeight, center_z);
        map_renderer->Update(0.0f);

        auto* context = device_resources->GetD3DDeviceContext();
        auto* render_target = device_resources->GetOffscreenRenderTargetView();
        auto* depth_stencil = device_resources->GetOffscreenDepthStencilView();

        const auto& clear_color = map_renderer->GetClearColor();
        context->ClearRenderTargetView(render_target, (float*)(&clear_color));
        context->ClearDepthStencilView(depth_stencil, D3D11_CLEAR_DEPTH | D3D11_CLEAR_STENCIL, 1.0f, 0);

        context->OMSetRenderTargets(1, &render_target, depth_stencil);
        auto const viewport = device_resources->GetOffscreenViewport();
        context->RSSetViewports(1, &viewport);

        map_renderer->Render(render_target, nullptr, depth_stencil);
        context->Flush();

        std::vector<RGBA> map_rgba;
        int map_width = 0;
        int map_height = 0;
        ID3D11Texture2D* offscreen_texture = device_resources->GetOffscreenRenderTarget();
        const bool capture_ok = CaptureTextureToRgba(map_renderer, offscreen_texture, map_rgba, map_width, map_height);

        map_renderer->SetShouldRenderSky(prev_should_render_sky);
        map_renderer->SetShouldRenderFog(prev_should_render_fog);
        map_renderer->SetShouldRenderShadows(prev_should_render_shadows);
        map_renderer->SetShouldRenderShadowsForModels(prev_should_render_model_shadows);
        RestoreCameraState(map_renderer, camera, camera_snapshot);

        if (!capture_ok || map_width != mask_width || map_height != mask_height) {
            return false;
        }

        const auto& mask_data = visualizer.GetMaskData();
        if (mask_data.size() != map_rgba.size()) {
            return false;
        }

        for (size_t i = 0; i < map_rgba.size(); ++i) {
            map_rgba[i].a = mask_data[i].a;
        }

        auto* texture_manager = map_renderer->GetTextureManager();
        if (route_map_texture_id >= 0) {
            texture_manager->RemoveTexture(route_map_texture_id);
            route_map_texture_id = -1;
        }

        HRESULT hr = texture_manager->CreateTextureFromRGBA(mask_width, mask_height, map_rgba.data(),
                                                            &route_map_texture_id, -1);
        return SUCCEEDED(hr) && route_map_texture_id >= 0;
    }

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

    std::wstring OpenLoadFileDialog(const std::wstring& extension) {
        OPENFILENAMEW ofn;
        wchar_t szFile[260] = {0};

        ZeroMemory(&ofn, sizeof(ofn));
        ofn.lStructSize = sizeof(ofn);
        ofn.hwndOwner = NULL;
        ofn.lpstrFile = szFile;
        ofn.nMaxFile = sizeof(szFile) / sizeof(wchar_t);

        std::wstring filter = L"CSV Files\0*.csv\0All Files\0*.*\0";
        ofn.lpstrFilter = filter.c_str();
        ofn.nFilterIndex = 1;
        ofn.lpstrDefExt = extension.c_str();
        ofn.Flags = OFN_PATHMUSTEXIST | OFN_FILEMUSTEXIST;

        if (GetOpenFileNameW(&ofn)) {
            return std::wstring(szFile);
        }
        return L"";
    }

    bool WriteCsv(const std::wstring& save_path, const std::vector<RouteWaypoint>& waypoints) {
        std::ofstream file;
        file.open(std::filesystem::path(save_path));
        if (!file.is_open()) {
            return false;
        }
        file << "index,x,y\n";
        for (size_t i = 0; i < waypoints.size(); ++i) {
            file << i << "," << waypoints[i].x << "," << waypoints[i].y << "\n";
        }
        return true;
    }

    bool ReadCsv(const std::wstring& load_path, std::vector<RouteWaypoint>& out_waypoints) {
        std::ifstream file;
        file.open(std::filesystem::path(load_path));
        if (!file.is_open()) {
            return false;
        }

        std::string line;
        bool is_first_line = true;
        std::vector<RouteWaypoint> loaded;
        while (std::getline(file, line)) {
            if (line.empty()) {
                continue;
            }
            if (is_first_line) {
                is_first_line = false;
                continue;
            }

            std::stringstream line_stream(line);
            std::string token;
            RouteWaypoint waypoint{};

            std::getline(line_stream, token, ','); // index, ignored
            if (!std::getline(line_stream, token, ',')) {
                continue;
            }
            waypoint.x = static_cast<float>(std::stod(token));
            if (!std::getline(line_stream, token, ',')) {
                continue;
            }
            waypoint.y = static_cast<float>(std::stod(token));
            loaded.push_back(waypoint);
        }

        out_waypoints = std::move(loaded);
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
                             bool show_coverage,
                             int selected_index) {
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
        const float spellcasting_radius_px = kSpellcastingRadius * std::min(px_per_world_x, px_per_world_y);

        ImDrawList* draw_list = ImGui::GetWindowDrawList();

        for (size_t i = 0; i < waypoints.size(); ++i) {
            const float px = (waypoints[i].x - visualizer.GetMinX()) * px_per_world_x;
            const float py = (visualizer.GetMaxY() - waypoints[i].y) * px_per_world_y;
            const ImVec2 center = ImVec2(image_min.x + px, image_min.y + py);

            if (show_coverage) {
                draw_list->AddCircle(center, spellcasting_radius_px, IM_COL32(255, 180, 0, 120), 40, 2.0f);
            }

            const bool is_selected = static_cast<int>(i) == selected_index;
            const ImU32 marker_color = is_selected ? IM_COL32(255, 120, 0, 230) : IM_COL32(0, 200, 255, 200);
            const float marker_radius = is_selected ? 7.0f : 5.0f;
            draw_list->AddCircleFilled(center, marker_radius, marker_color, 12);
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

    int FindWaypointAtScreenPos(const std::vector<RouteWaypoint>& waypoints,
                                const PathfindingVisualizer& visualizer,
                                const ImVec2& image_min,
                                const ImVec2& image_size,
                                const ImVec2& mouse_pos,
                                float max_distance_px) {
        if (image_size.x <= 0.0f || image_size.y <= 0.0f) {
            return -1;
        }

        const float span_x = visualizer.GetMaxX() - visualizer.GetMinX();
        const float span_y = visualizer.GetMaxY() - visualizer.GetMinY();
        if (span_x <= 0.0f || span_y <= 0.0f) {
            return -1;
        }

        const float px_per_world_x = image_size.x / span_x;
        const float px_per_world_y = image_size.y / span_y;

        int best_index = -1;
        float best_distance_sq = max_distance_px * max_distance_px;
        for (size_t i = 0; i < waypoints.size(); ++i) {
            const float px = (waypoints[i].x - visualizer.GetMinX()) * px_per_world_x;
            const float py = (visualizer.GetMaxY() - waypoints[i].y) * px_per_world_y;
            const float dx = (image_min.x + px) - mouse_pos.x;
            const float dy = (image_min.y + py) - mouse_pos.y;
            const float dist_sq = dx * dx + dy * dy;
            if (dist_sq <= best_distance_sq) {
                best_distance_sq = dist_sq;
                best_index = static_cast<int>(i);
            }
        }
        return best_index;
    }

    void RemoveMeshList(MapRenderer* map_renderer, std::vector<int>& mesh_ids) {
        if (!map_renderer) {
            return;
        }
        auto* mesh_manager = map_renderer->GetMeshManager();
        if (!mesh_manager) {
            return;
        }
        for (int mesh_id : mesh_ids) {
            mesh_manager->RemoveMesh(mesh_id);
        }
        mesh_ids.clear();
    }

    void UpdateRouteOverlayMeshes(MapRenderer* map_renderer,
                                  const std::vector<RouteWaypoint>& waypoints,
                                  bool show_lines,
                                  bool show_coverage,
                                  std::vector<int>& route_line_mesh_ids,
                                  std::vector<int>& coverage_mesh_ids) {
        auto* mesh_manager = map_renderer ? map_renderer->GetMeshManager() : nullptr;
        auto* terrain = map_renderer ? map_renderer->GetTerrain() : nullptr;
        if (!mesh_manager || !terrain || waypoints.empty()) {
            RemoveMeshList(map_renderer, route_line_mesh_ids);
            RemoveMeshList(map_renderer, coverage_mesh_ids);
            return;
        }

        RemoveMeshList(map_renderer, route_line_mesh_ids);
        RemoveMeshList(map_renderer, coverage_mesh_ids);

        const DirectX::XMFLOAT4 route_color(0.0f, 0.85f, 1.0f, 0.95f);
        const DirectX::XMFLOAT4 coverage_color(1.0f, 0.65f, 0.15f, 0.7f);

        if (show_lines && waypoints.size() > 1) {
            for (size_t i = 1; i < waypoints.size(); ++i) {
                const auto& start = waypoints[i - 1];
                const auto& end = waypoints[i];
                const float start_height = terrain->get_height_at(start.x, start.y) + kRouteHeightOffset;
                const float end_height = terrain->get_height_at(end.x, end.y) + kRouteHeightOffset;
                DirectX::XMFLOAT3 start_pos(start.x, start_height, start.y);
                DirectX::XMFLOAT3 end_pos(end.x, end_height, end.y);

                int line_id = mesh_manager->AddLine(start_pos, end_pos, PixelShaderType::OldModel);
                if (line_id >= 0) {
                    mesh_manager->SetMeshColor(line_id, route_color);
                    route_line_mesh_ids.push_back(line_id);
                }
            }
        }

        if (show_coverage) {
            const float angle_step = (2.0f * kPi) / static_cast<float>(kCircleSegments);
            for (const auto& waypoint : waypoints) {
                float prev_x = waypoint.x + kSpellcastingRadius;
                float prev_y = waypoint.y;
                for (int seg = 1; seg <= kCircleSegments; ++seg) {
                    const float angle = angle_step * static_cast<float>(seg);
                    const float next_x = waypoint.x + kSpellcastingRadius * std::cos(angle);
                    const float next_y = waypoint.y + kSpellcastingRadius * std::sin(angle);

                    const float prev_height = terrain->get_height_at(prev_x, prev_y) + kRouteHeightOffset;
                    const float next_height = terrain->get_height_at(next_x, next_y) + kRouteHeightOffset;
                    DirectX::XMFLOAT3 start_pos(prev_x, prev_height, prev_y);
                    DirectX::XMFLOAT3 end_pos(next_x, next_height, next_y);

                    int line_id = mesh_manager->AddLine(start_pos, end_pos, PixelShaderType::OldModel);
                    if (line_id >= 0) {
                        mesh_manager->SetMeshColor(line_id, coverage_color);
                        coverage_mesh_ids.push_back(line_id);
                    }

                    prev_x = next_x;
                    prev_y = next_y;
                }
            }
        }
    }
}

void draw_route_planner_panel(MapRenderer* map_renderer, DX::DeviceResources* device_resources) {
    if (!GuiGlobalConstants::is_route_planner_panel_open) {
        return;
    }

    const float min_width = 420.0f;
    const float min_height = 220.0f;
    ImGui::SetNextWindowSizeConstraints(ImVec2(min_width, min_height), ImVec2(FLT_MAX, FLT_MAX));

    static std::vector<RouteWaypoint> waypoints;
    static bool show_lines = true;
    static bool show_coverage = true;
    static float map_zoom = 1.0f;
    static bool click_to_add = true;
    static bool has_last_click = false;
    static RouteWaypoint last_click;
    static int selected_waypoint = -1;
    static bool is_dragging_waypoint = false;
    static int dragging_waypoint_index = -1;
    static int route_map_texture_id = -1;
    static int route_map_map_index = -1;
    static int route_mask_map_index = -1;
    static std::vector<int> route_line_mesh_ids;
    static std::vector<int> coverage_mesh_ids;
    static std::vector<RouteWaypoint> last_overlay_waypoints;
    static bool last_overlay_show_lines = false;
    static bool last_overlay_show_coverage = false;
    static int last_overlay_map_index = -1;

    ImGuiIO& io = ImGui::GetIO();
    const bool prev_move_title_only = io.ConfigWindowsMoveFromTitleBarOnly;
    io.ConfigWindowsMoveFromTitleBarOnly = true;

    if (ImGui::Begin("Route Planner", &GuiGlobalConstants::is_route_planner_panel_open,
                     ImGuiWindowFlags_NoFocusOnAppearing | ImGuiWindowFlags_HorizontalScrollbar)) {
        GuiGlobalConstants::ClampWindowToScreen();

        if (selected_file_type != FFNA_Type3) {
            ImGui::TextWrapped("No pathfinding data loaded.");
            ImGui::TextWrapped("Load a map file (FFNA Type3) from the DAT browser to plan routes.");
            RemoveMeshList(map_renderer, route_line_mesh_ids);
            RemoveMeshList(map_renderer, coverage_mesh_ids);
            ImGui::End();
            return;
        }

        auto* visualizer = GetPathfindingVisualizer();
        if (route_mask_map_index != selected_map_file_index || !visualizer->IsMaskReady()) {
            if (selected_ffna_map_file.pathfinding_chunk.valid) {
                visualizer->GenerateMask(selected_ffna_map_file.pathfinding_chunk, kRouteMapImageSize);
                route_mask_map_index = selected_map_file_index;
            }
        }

        ImGui::Text("Waypoints: %zu", waypoints.size());
        ImGui::SameLine();
        ImGui::Text("Spellcasting range: %.0f", kSpellcastingRadius);

        ImGui::Separator();

        ImGui::Checkbox("Click on map to add waypoint", &click_to_add);
        ImGui::Checkbox("Show route lines", &show_lines);
        ImGui::Checkbox("Show spellcasting coverage", &show_coverage);
        ImGui::SliderFloat("Map zoom", &map_zoom, 0.25f, 4.0f, "%.2fx");
        if (ImGui::IsItemHovered()) {
            ImGui::SetTooltip("Use mouse wheel while hovering the map to zoom.");
        }
        ImGui::SameLine();
        if (ImGui::Button("Reset zoom")) {
            map_zoom = 1.0f;
        }
        ImGui::TextWrapped("Clicks report map world coordinates (X/Y) based on the pathfinding data.");
        ImGui::Separator();

        if (route_map_map_index != selected_map_file_index && route_map_texture_id >= 0) {
            map_renderer->GetTextureManager()->RemoveTexture(route_map_texture_id);
            route_map_texture_id = -1;
        }

        if (route_map_texture_id < 0 || route_map_map_index != selected_map_file_index) {
            if (UpdateRoutePlannerMapTexture(map_renderer, device_resources, *visualizer, route_map_texture_id)) {
                route_map_map_index = selected_map_file_index;
            }
        }

        if (route_map_texture_id >= 0 && visualizer->IsMaskReady()) {
            ID3D11ShaderResourceView* texture = map_renderer->GetTextureManager()->GetTexture(route_map_texture_id);
            if (texture) {
                ImVec2 window_size = ImGui::GetContentRegionAvail();
                float img_width = static_cast<float>(visualizer->GetMaskWidth());
                float img_height = static_cast<float>(visualizer->GetMaskHeight());

                float scale_x = (window_size.x - 20) / img_width;
                float scale_y = (window_size.y - 120) / img_height;
                float scale = std::min(scale_x, scale_y);
                scale = std::max(0.1f, scale);
                scale *= map_zoom;

                ImVec2 scaled_size(img_width * scale, img_height * scale);
                ImVec2 image_min = ImGui::GetCursorScreenPos();
                ImGui::Image((ImTextureID)texture, scaled_size);

                DrawWaypointOverlay(waypoints, *visualizer, image_min, scaled_size, show_lines, show_coverage, selected_waypoint);

                if (ImGui::IsItemHovered()) {
                    if (io.MouseWheel != 0.0f) {
                        map_zoom = std::clamp(map_zoom + io.MouseWheel * 0.1f, 0.25f, 4.0f);
                    }
                    const ImVec2 mouse_pos = ImGui::GetIO().MousePos;
                    const int hit_index = FindWaypointAtScreenPos(waypoints, *visualizer, image_min, scaled_size, mouse_pos, 10.0f);

                    if (ImGui::IsMouseClicked(ImGuiMouseButton_Left)) {
                        if (hit_index >= 0) {
                            selected_waypoint = hit_index;
                            is_dragging_waypoint = true;
                            dragging_waypoint_index = hit_index;
                        } else {
                            RouteWaypoint waypoint;
                            if (TryAddWaypointFromImageClick(*visualizer, image_min, scaled_size, mouse_pos, waypoint)) {
                                has_last_click = true;
                                last_click = waypoint;
                                if (click_to_add) {
                                    waypoints.push_back(waypoint);
                                    selected_waypoint = static_cast<int>(waypoints.size() - 1);
                                }
                            }
                        }
                    }

                    if (is_dragging_waypoint && dragging_waypoint_index >= 0 && ImGui::IsMouseDown(ImGuiMouseButton_Left)) {
                        RouteWaypoint waypoint;
                        if (TryAddWaypointFromImageClick(*visualizer, image_min, scaled_size, mouse_pos, waypoint)) {
                            waypoints[dragging_waypoint_index] = waypoint;
                            has_last_click = true;
                            last_click = waypoint;
                        }
                    }

                    if (ImGui::IsMouseReleased(ImGuiMouseButton_Left)) {
                        is_dragging_waypoint = false;
                        dragging_waypoint_index = -1;
                    }
                }

                if (ImGui::BeginPopupContextItem("route_context_menu")) {
                    if (ImGui::MenuItem("Clear all waypoints")) {
                        waypoints.clear();
                        selected_waypoint = -1;
                    }
                    ImGui::EndPopup();
                }
            }
        } else {
            ImGui::Text("Generating route map...");
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
                if (selected_waypoint >= static_cast<int>(waypoints.size())) {
                    selected_waypoint = static_cast<int>(waypoints.size()) - 1;
                }
            }
        }
        ImGui::SameLine();
        if (ImGui::Button("Clear")) {
            waypoints.clear();
            selected_waypoint = -1;
        }
        ImGui::SameLine();
        if (ImGui::Button("Import CSV")) {
            std::wstring load_path = OpenLoadFileDialog(L"csv");
            if (!load_path.empty()) {
                ReadCsv(load_path, waypoints);
                selected_waypoint = waypoints.empty() ? -1 : 0;
            }
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
                    char label[128];
                    snprintf(label, sizeof(label), "%zu: (%.2f, %.2f)", i + 1, waypoints[i].x, waypoints[i].y);
                    if (ImGui::Selectable(label, selected_waypoint == static_cast<int>(i))) {
                        selected_waypoint = static_cast<int>(i);
                    }
                }
            }
            ImGui::EndChild();

            if (ImGui::IsWindowFocused(ImGuiFocusedFlags_RootAndChildWindows) &&
                ImGui::IsKeyPressed(ImGuiKey_Delete) &&
                selected_waypoint >= 0 &&
                selected_waypoint < static_cast<int>(waypoints.size())) {
                waypoints.erase(waypoints.begin() + selected_waypoint);
                if (selected_waypoint >= static_cast<int>(waypoints.size())) {
                    selected_waypoint = static_cast<int>(waypoints.size()) - 1;
                }
            }
        }

        if (selected_map_file_index != last_overlay_map_index ||
            show_lines != last_overlay_show_lines ||
            show_coverage != last_overlay_show_coverage ||
            waypoints.size() != last_overlay_waypoints.size() ||
            !std::equal(waypoints.begin(), waypoints.end(), last_overlay_waypoints.begin(),
                        [](const RouteWaypoint& a, const RouteWaypoint& b) {
                            return a.x == b.x && a.y == b.y;
                        })) {
            UpdateRouteOverlayMeshes(map_renderer, waypoints, show_lines, show_coverage,
                                     route_line_mesh_ids, coverage_mesh_ids);
            last_overlay_waypoints = waypoints;
            last_overlay_show_lines = show_lines;
            last_overlay_show_coverage = show_coverage;
            last_overlay_map_index = selected_map_file_index;
        }
    }
    ImGui::End();
    io.ConfigWindowsMoveFromTitleBarOnly = prev_move_title_only;
}
