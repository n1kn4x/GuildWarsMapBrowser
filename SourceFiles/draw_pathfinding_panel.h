#pragma once
#include "MapRenderer.h"
#include "FFNA_MapFile.h"

// Manages the pathfinding visualization texture
class PathfindingVisualizer {
public:
    PathfindingVisualizer() = default;
    ~PathfindingVisualizer() = default;

    // Generate RGBA image from trapezoids
    void GenerateImage(const PathfindingChunk& pathfinding_chunk, int image_size = 1024);

    // Generate an alpha mask for walkable areas
    void GenerateMask(const PathfindingChunk& pathfinding_chunk, int image_size = 1024);

    // Create texture from generated image
    int CreateTexture(TextureManager* texture_manager);

    // Create texture from generated mask
    int CreateMaskTexture(TextureManager* texture_manager);

    // Get texture ID
    int GetTextureId() const { return m_texture_id; }
    int GetMaskTextureId() const { return m_mask_texture_id; }

    // Get image dimensions
    int GetWidth() const { return m_width; }
    int GetHeight() const { return m_height; }
    int GetMaskWidth() const { return m_mask_width; }
    int GetMaskHeight() const { return m_mask_height; }

    // Check if visualization is ready
    bool IsReady() const { return m_image_ready; }
    bool IsMaskReady() const { return m_mask_ready; }

    // Coordinate transform helpers
    float GetMinX() const { return m_min_x; }
    float GetMinY() const { return m_min_y; }
    float GetMaxX() const { return m_max_x; }
    float GetMaxY() const { return m_max_y; }
    float GetScaleX() const { return m_scale_x; }
    float GetScaleY() const { return m_scale_y; }

    // Get raw RGBA data for export
    const std::vector<RGBA>& GetImageData() const { return m_image_data; }
    const std::vector<RGBA>& GetMaskData() const { return m_mask_data; }

    // Clear the visualization
    void Clear();

    // Get trapezoid/plane counts
    size_t GetTrapezoidCount() const { return m_trapezoid_count; }
    size_t GetPlaneCount() const { return m_plane_count; }

private:
    std::vector<RGBA> m_image_data;
    std::vector<RGBA> m_mask_data;
    int m_width = 0;
    int m_height = 0;
    int m_texture_id = -1;
    bool m_image_ready = false;
    int m_mask_width = 0;
    int m_mask_height = 0;
    int m_mask_texture_id = -1;
    bool m_mask_ready = false;
    size_t m_trapezoid_count = 0;
    size_t m_plane_count = 0;
    float m_min_x = 0.0f;
    float m_min_y = 0.0f;
    float m_max_x = 0.0f;
    float m_max_y = 0.0f;
    float m_scale_x = 1.0f;
    float m_scale_y = 1.0f;

    // HSV to RGB conversion for coloring trapezoids
    RGBA HsvToRgb(float h, float s, float v, uint8_t a = 255);

    // Draw a filled trapezoid on the image
    void DrawTrapezoid(const PathfindingTrapezoid& trap,
                       float min_x, float min_y, float scale_x, float scale_y,
                       RGBA fill_color, RGBA outline_color);

    // Draw a line on the image (Bresenham's algorithm)
    void DrawLine(int x0, int y0, int x1, int y1, RGBA color);

    // Fill a convex polygon (scanline fill)
    void FillPolygon(const std::vector<std::pair<int, int>>& points, RGBA color);
};

// Draw the pathfinding visualization panel
void draw_pathfinding_panel(MapRenderer* map_renderer);

// Expose visualization for other panels (route planner, etc.)
PathfindingVisualizer* GetPathfindingVisualizer();
