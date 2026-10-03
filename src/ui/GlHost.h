// GlHost.h : our own thin host layer over GLFW + OpenGL3 + Dear ImGui.
// Owns the main OS window, the GL context, and the ImGui/ImPlot lifecycle.
// Multi-viewport is enabled, so any ImGui window placed outside the main
// window becomes its own real OS window.
#pragma once

#include <string>

struct GLFWwindow;

namespace firedog {

// Where the main OS window sits. The rect is the CLIENT area in virtual-screen
// pixels — the same coordinates GLFW reports and accepts, so it round-trips
// without a conversion — and `maximized` is carried beside it rather than
// folded into the rect, because a maximized window still has a restored size
// to come back to.
struct WindowPlacement {
    int  x = 0, y = 0;
    int  w = 0, h = 0;
    bool maximized = false;

    bool Valid() const { return w > 0 && h > 0; }
};

class GlHost {
public:
    // `restore` is the placement a previous run was left at; an invalid one
    // (the default) means first run — the window is created at width x height
    // wherever the window manager puts it. A saved rect that no longer lands
    // on any connected monitor is ignored for POSITION but still honoured for
    // size: a screen can be unplugged between runs, and a window nobody can
    // reach with the mouse is worse than one in the wrong place.
    bool Init(const char* title, int width, int height,
              const WindowPlacement& restore = WindowPlacement{});
    void Shutdown();

    // The placement to save. While the window is maximized this reports the
    // last un-maximized rect, so the restored geometry survives a run that was
    // started, maximized and closed.
    const WindowPlacement& Placement() const { return placement_; }

    bool ShouldClose() const;
    // Override the GLFW should-close flag: pass false to swallow an OS close
    // request (X button / Alt+F4) so the app can confirm first, true to
    // actually end the run loop.
    void SetShouldClose(bool close);
    void BeginFrame();   // poll events + start an ImGui frame
    void EndFrame();     // render main window + platform windows, then swap

    double Time() const; // seconds since Init (for frame dt)

    // --- Fonts ---------------------------------------------------------------
    // Change the application typeface and/or size. `ttfPath` is a .ttf path
    // (empty = ImGui's built-in font); `sizePx` is the pixel height. Safe to
    // call any time — even mid-draw — because the atlas rebuild is deferred to
    // the next frame boundary. Falls back to the built-in font at the requested
    // size if the file can't be loaded.
    void  SetFont(const std::string& ttfPath, float sizePx);
    void  SetFontSize(float sizePx);            // keep the current typeface
    float FontSize() const { return fontSize_; }

    // Upload an RGBA8 image and return its GL texture id (use with ImGui::Image).
    unsigned int MakeTextureRGBA(int width, int height, const void* rgba);

    // Create-or-update an RGB8 texture (pass 0 to allocate a new id). Use for
    // streaming camera frames: call each new frame with the returned id.
    unsigned int EnsureTextureRGB(unsigned int tex, int width, int height, const void* rgb);

    // Same, for RGBA8 — a streaming overlay that has to be transparent where
    // nothing was found (the vision teach page's filter overlay).
    unsigned int EnsureTextureRGBA(unsigned int tex, int width, int height, const void* rgba);

    // Free a texture made by the functions above (0 is ignored). For a view
    // that pages textures in and out — the Mosaic page holds a few hundred
    // tiles and cannot keep every full frame on the GPU at once.
    void DeleteTexture(unsigned int tex);

private:
    void ApplyFont();        // apply pending size/typeface at a frame boundary
    void TrackPlacement();   // remember where the window is, each frame

    GLFWwindow*     window_ = nullptr;
    WindowPlacement placement_;

    std::string iniPath_;              // imgui.ini under the machine root (ImGui keeps the pointer)
    std::string fontPath_;             // requested typeface; empty = built-in font
    std::string loadedPath_;           // typeface currently in the atlas
    float       fontSize_  = 14.0f;
    bool        fontDirty_ = false;    // a change is pending for the next frame
};

} // namespace firedog
