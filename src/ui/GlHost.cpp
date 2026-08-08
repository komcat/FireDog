// GlHost.cpp
#include "ui/GlHost.h"

#include "imgui.h"
#include "implot.h"
#include "backends/imgui_impl_glfw.h"
#include "backends/imgui_impl_opengl3.h"

#include <GLFW/glfw3.h>   // pulls in <GL/gl.h> for the base GL calls below

#include <cstdio>

namespace firedog {

bool GlHost::Init(const char* title, int width, int height)
{
    if (!glfwInit()) return false;

    // GL 3.0 + GLSL 130 — the configuration the ImGui GLFW/GL3 example ships
    // with on Windows; multi-viewport works fine on it.
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);

    window_ = glfwCreateWindow(width, height, title, nullptr, nullptr);
    if (!window_) { glfwTerminate(); return false; }
    glfwMakeContextCurrent(window_);
    glfwSwapInterval(1);   // vsync

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImPlot::CreateContext();

    ImGuiIO& io = ImGui::GetIO();
    io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;
    io.ConfigFlags |= ImGuiConfigFlags_ViewportsEnable;   // real extra OS windows

    ImGui::StyleColorsDark();
    // When viewports are on, keep window backgrounds opaque and corners square
    // so detached OS windows look right.
    ImGuiStyle& style = ImGui::GetStyle();
    if (io.ConfigFlags & ImGuiConfigFlags_ViewportsEnable) {
        style.WindowRounding = 0.0f;
        style.Colors[ImGuiCol_WindowBg].w = 1.0f;
    }

    ImGui_ImplGlfw_InitForOpenGL(window_, true);
    ImGui_ImplOpenGL3_Init("#version 130");
    return true;
}

void GlHost::Shutdown()
{
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImPlot::DestroyContext();
    ImGui::DestroyContext();
    if (window_) glfwDestroyWindow(window_);
    glfwTerminate();
    window_ = nullptr;
}

bool GlHost::ShouldClose() const
{
    return window_ == nullptr || glfwWindowShouldClose(window_);
}

void GlHost::SetShouldClose(bool close)
{
    if (window_) glfwSetWindowShouldClose(window_, close ? GLFW_TRUE : GLFW_FALSE);
}

void GlHost::BeginFrame()
{
    glfwPollEvents();
    // Apply any pending font change here — between frames, when no ImGui frame
    // is in progress and the previous one is fully rendered/swapped.
    if (fontDirty_) { ApplyFont(); fontDirty_ = false; }
    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();
}

void GlHost::SetFont(const std::string& ttfPath, float sizePx)
{
    fontPath_ = ttfPath;
    if (sizePx > 0.0f) fontSize_ = sizePx;
    fontDirty_ = true;   // deferred to the next BeginFrame
}

void GlHost::SetFontSize(float sizePx)
{
    SetFont(fontPath_, sizePx);
}

// ImGui 1.92 dynamic fonts: size is driven by style.FontSizeBase (no atlas
// rebuild), and the backend owns the font texture (ImGuiBackendFlags_
// RendererHasTextures) — so we only reload the atlas when the typeface changes.
void GlHost::ApplyFont()
{
    ImGuiIO&    io    = ImGui::GetIO();
    ImGuiStyle& style = ImGui::GetStyle();

    style.FontSizeBase = fontSize_;   // dynamic size — cheap, applies every frame

    if (fontPath_ == loadedPath_ && io.Fonts->Fonts.Size > 0)
        return;                       // typeface unchanged (e.g. size-only tweak)

    // Typeface changed: swap the atlas. Clear() between frames is safe; the
    // backend recreates its texture automatically on the next NewFrame().
    io.Fonts->Clear();
    ImFont* font = nullptr;
    if (!fontPath_.empty()) {
        // Probe the file first: AddFontFromFileTTF hits IM_ASSERT (a crash in a
        // debug build) if the path can't be opened, rather than returning null.
        if (FILE* f = std::fopen(fontPath_.c_str(), "rb")) {
            std::fclose(f);
            font = io.Fonts->AddFontFromFileTTF(fontPath_.c_str(), fontSize_);
        }
    }
    if (!font) {                      // missing file, or built-in requested
        io.Fonts->AddFontDefault();   // embedded scalable font
        fontPath_.clear();            // don't retry a path that failed to load
    }
    io.FontDefault = nullptr;         // -> Fonts[0], the one just added
    loadedPath_    = fontPath_;
}

void GlHost::EndFrame()
{
    ImGui::Render();

    int fbw = 0, fbh = 0;
    glfwGetFramebufferSize(window_, &fbw, &fbh);
    glViewport(0, 0, fbw, fbh);
    glClearColor(0.07f, 0.07f, 0.08f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT);
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

    // Render every detached viewport window, then restore our context.
    ImGuiIO& io = ImGui::GetIO();
    if (io.ConfigFlags & ImGuiConfigFlags_ViewportsEnable) {
        GLFWwindow* backup = glfwGetCurrentContext();
        ImGui::UpdatePlatformWindows();
        ImGui::RenderPlatformWindowsDefault();
        glfwMakeContextCurrent(backup);
    }

    glfwSwapBuffers(window_);
}

double GlHost::Time() const
{
    return glfwGetTime();
}

unsigned int GlHost::MakeTextureRGBA(int width, int height, const void* rgba)
{
    GLuint tex = 0;
    glGenTextures(1, &tex);
    glBindTexture(GL_TEXTURE_2D, tex);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, width, height, 0,
                 GL_RGBA, GL_UNSIGNED_BYTE, rgba);
    return static_cast<unsigned int>(tex);
}

unsigned int GlHost::EnsureTextureRGB(unsigned int tex, int width, int height, const void* rgb)
{
    GLuint id = static_cast<GLuint>(tex);
    if (id == 0) {
        glGenTextures(1, &id);
        glBindTexture(GL_TEXTURE_2D, id);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    } else {
        glBindTexture(GL_TEXTURE_2D, id);
    }
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);   // RGB rows aren't 4-byte aligned
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width, height, 0,
                 GL_RGB, GL_UNSIGNED_BYTE, rgb);
    return static_cast<unsigned int>(id);
}

unsigned int GlHost::EnsureTextureRGBA(unsigned int tex, int width, int height,
                                       const void* rgba)
{
    GLuint id = static_cast<GLuint>(tex);
    if (id == 0) {
        glGenTextures(1, &id);
        glBindTexture(GL_TEXTURE_2D, id);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    } else {
        glBindTexture(GL_TEXTURE_2D, id);
    }
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, width, height, 0,
                 GL_RGBA, GL_UNSIGNED_BYTE, rgba);
    return static_cast<unsigned int>(id);
}

} // namespace firedog
