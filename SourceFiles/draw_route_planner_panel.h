#pragma once

#include "MapRenderer.h"
#include "draw_pathfinding_panel.h"
#include <vector>

struct RouteWaypoint {
    float x = 0.0f;
    float y = 0.0f;
};

void draw_route_planner_panel(MapRenderer* map_renderer, DX::DeviceResources* device_resources);
