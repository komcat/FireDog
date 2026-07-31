// GlHost.h : our own thin host layer over GLFW + OpenGL3 + Dear ImGui.
// Owns the main OS window, the GL context, and the ImGui/ImPlot lifecycle.
// Multi-viewport is enabled, so any ImGui window placed outside the main
// window becomes its own real OS window.
#pragma once

#include <string>

struct GLFWwindow;

namespace firedog {

class GlHost {
public:
    bool Init(const char* title, int width, int height);
    void Shutdown();

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

private:
    void ApplyFont();   // apply pending size/typeface at a frame boundary

    GLFWwindow* window_ = nullptr;

    std::string fontPath_;             // requested typeface; empty = built-in font
    std::string loadedPath_;           // typeface currently in the atlas
    float       fontSize_  = 14.0f;
    bool        fontDirty_ = false;    // a change is pending for the next frame
};

} // namespace firedog
