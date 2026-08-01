# CMakeList.txt : CMake project for FireDog.
# Lightweight controller for a universal active-alignment machine.
#
cmake_minimum_required(VERSION 3.14)
project("FireDog" LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Enable Hot Reload for MSVC compilers if supported.
if (POLICY CMP0141)
  cmake_policy(SET CMP0141 NEW)
  set(CMAKE_MSVC_DEBUG_INFORMATION_FORMAT "$<IF:$<AND:$<C_COMPILER_ID:MSVC>,$<CXX_COMPILER_ID:MSVC>>,$<$<CONFIG:Debug,RelWithDebInfo>:EditAndContinue>,$<$<CONFIG:Debug,RelWithDebInfo>:ProgramDatabase>>")
endif()

include(FetchContent)

# ---- Windowing: GLFW (we own the host layer; no raylib/rlImGui) ----
set(GLFW_BUILD_DOCS     OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_TESTS    OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(GLFW_INSTALL        OFF CACHE BOOL "" FORCE)
FetchContent_Declare(glfw
    GIT_REPOSITORY https://github.com/glfw/glfw.git
    GIT_TAG 3.4 GIT_SHALLOW TRUE)
FetchContent_MakeAvailable(glfw)

# ---- Dear ImGui (docking branch -> multi-viewport = real 2nd OS window) ----
FetchContent_Declare(imgui
    GIT_REPOSITORY https://github.com/ocornut/imgui.git
    GIT_TAG docking GIT_SHALLOW TRUE)
FetchContent_MakeAvailable(imgui)

# ---- ImPlot (realtime charts) ----
FetchContent_Declare(implot
    GIT_REPOSITORY https://github.com/epezent/implot.git
    GIT_TAG master GIT_SHALLOW TRUE)
FetchContent_MakeAvailable(implot)

# ---- nlohmann/json (hardware config) ----
FetchContent_Declare(nlohmann_json
    GIT_REPOSITORY https://github.com/nlohmann/json.git
    GIT_TAG v3.11.3 GIT_SHALLOW TRUE)
FetchContent_MakeAvailable(nlohmann_json)

# ---- spdlog (layer-0 logging; bundles fmt) ----
set(SPDLOG_BUILD_EXAMPLE OFF CACHE BOOL "" FORCE)
set(SPDLOG_BUILD_TESTS   OFF CACHE BOOL "" FORCE)
FetchContent_Declare(spdlog
    GIT_REPOSITORY https://github.com/gabime/spdlog.git
    GIT_TAG v1.14.1 GIT_SHALLOW TRUE)
FetchContent_MakeAvailable(spdlog)

# ---- Lua 5.4 + sol2 (layer 2.5 Lua program tier) ----
# CMake-wrapped Lua mirror (builds lua_static as C — trivial under MSVC);
# sol2 is header-only. See docs/lua-tier-design.md.
set(LUA_ENABLE_TESTING OFF CACHE BOOL "" FORCE)
set(LUA_BUILD_BINARY   OFF CACHE BOOL "" FORCE)
set(LUA_BUILD_COMPILER OFF CACHE BOOL "" FORCE)
FetchContent_Declare(lua
    GIT_REPOSITORY https://github.com/walterschell/Lua.git
    GIT_TAG master GIT_SHALLOW TRUE)
FetchContent_MakeAvailable(lua)

set(SOL2_ENABLE_INSTALL OFF CACHE BOOL "" FORCE)
FetchContent_Declare(sol2
    GIT_REPOSITORY https://github.com/ThePhD/sol2.git
    GIT_TAG v3.3.0 GIT_SHALLOW TRUE)
FetchContent_MakeAvailable(sol2)

# ---- stb (single-header PNG writing for dut_capture_image) ----
# Header-only collection, no CMakeLists — MakeAvailable just populates.
FetchContent_Declare(stb
    GIT_REPOSITORY https://github.com/nothings/stb.git
    GIT_TAG master GIT_SHALLOW TRUE)
FetchContent_MakeAvailable(stb)

# ---- Basler Pylon SDK (layer 1 camera) — optional ----
# Prefer a vendored copy under external/, else a system Basler install (PYLON_DEV_DIR).
set(PYLON_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/external/pylon 8")
if(EXISTS "${PYLON_ROOT}/Development/include")
    set(PYLON_INCLUDE_DIR "${PYLON_ROOT}/Development/include")
    set(PYLON_LIB_DIR     "${PYLON_ROOT}/Development/lib/x64")
    set(PYLON_RUNTIME_DIR "${PYLON_ROOT}/Runtime/x64")
elseif(DEFINED ENV{PYLON_DEV_DIR})
    set(PYLON_INCLUDE_DIR "$ENV{PYLON_DEV_DIR}/include")
    set(PYLON_LIB_DIR     "$ENV{PYLON_DEV_DIR}/lib/x64")
    set(PYLON_RUNTIME_DIR "")   # system install keeps runtime DLLs on PATH
endif()

if(PYLON_INCLUDE_DIR AND EXISTS "${PYLON_INCLUDE_DIR}")
    file(GLOB PYLON_LIBRARIES
        "${PYLON_LIB_DIR}/PylonBase_v*.lib"
        "${PYLON_LIB_DIR}/PylonUtility_v*.lib"
        "${PYLON_LIB_DIR}/GCBase_*.lib"
        "${PYLON_LIB_DIR}/GenApi_*.lib")
    set(PYLON_FOUND TRUE)
    message(STATUS "Pylon SDK found: ${PYLON_INCLUDE_DIR}")
else()
    set(PYLON_FOUND FALSE)
    message(WARNING "Pylon SDK not found - camera runs in test-pattern stub mode")
endif()

# ---- NI-VISA (Keithley / GPIB instruments) — optional ----
# Prefer a vendored copy under external/visa, else a system IVI/NI-VISA install.
set(VISA_DIR "${CMAKE_CURRENT_SOURCE_DIR}/external/visa")
if(EXISTS "${VISA_DIR}/include/visa.h")
    set(VISA_INCLUDE_DIR "${VISA_DIR}/include")
    set(VISA_LIBRARY     "${VISA_DIR}/Lib_x64/visa64.lib")
elseif(EXISTS "C:/Program Files/IVI Foundation/VISA/Win64/Include/visa.h")
    set(VISA_INCLUDE_DIR "C:/Program Files/IVI Foundation/VISA/Win64/Include")
    set(VISA_LIBRARY     "C:/Program Files/IVI Foundation/VISA/Win64/Lib_x64/msc/visa64.lib")
endif()

if(VISA_INCLUDE_DIR AND EXISTS "${VISA_INCLUDE_DIR}/visa.h" AND EXISTS "${VISA_LIBRARY}")
    set(VISA_FOUND TRUE)
    message(STATUS "VISA SDK found: ${VISA_INCLUDE_DIR}")
else()
    set(VISA_FOUND FALSE)
    message(WARNING "VISA SDK not found - Keithley runs in simulated stub mode")
endif()

# ---- PI GCS2 SDK (layer 1 PI hexapod) — optional ----
# Vendored under external/PI (header + import lib + runtime DLL alongside).
set(PI_SDK_DIR "${CMAKE_CURRENT_SOURCE_DIR}/external/PI")
if(EXISTS "${PI_SDK_DIR}/PI_GCS2_DLL.h" AND EXISTS "${PI_SDK_DIR}/PI_GCS2_DLL_x64.lib")
    set(PI_FOUND TRUE)
    set(PI_LIBRARY "${PI_SDK_DIR}/PI_GCS2_DLL_x64.lib")
    message(STATUS "PI GCS2 SDK found: ${PI_SDK_DIR}")
else()
    set(PI_FOUND FALSE)
    message(WARNING "PI GCS2 SDK not found - PI hexapod runs in simulated stub mode")
endif()

# ---- ACS SPiiPlus SDK (layer 1 ACS gantry) — optional ----
# Vendored under external/ACS (header + import lib). Links ws2_32 (winsock).
set(ACS_SDK_DIR "${CMAKE_CURRENT_SOURCE_DIR}/external/ACS")
if(EXISTS "${ACS_SDK_DIR}/ACSC.h" AND EXISTS "${ACS_SDK_DIR}/ACSCL_x64.LIB")
    set(ACS_FOUND TRUE)
    set(ACS_LIBRARY "${ACS_SDK_DIR}/ACSCL_x64.LIB")
    message(STATUS "ACS SDK found: ${ACS_SDK_DIR}")
else()
    set(ACS_FOUND FALSE)
    message(WARNING "ACS SDK not found - ACS gantry runs in simulated stub mode")
endif()

# ---- FASTECH EziMOTIONPlusE SDK (layer 1 Ezi-IO digital IO) — optional ----
# Vendored under external/eziio (header + import lib + runtime DLL alongside).
# WAGO needs no SDK (hand-rolled Modbus TCP over winsock).
set(EZIIO_SDK_DIR "${CMAKE_CURRENT_SOURCE_DIR}/external/eziio")
if(EXISTS "${EZIIO_SDK_DIR}/FAS_EziMOTIONPlusE.h" AND EXISTS "${EZIIO_SDK_DIR}/EziMOTIONPlusE.lib")
    set(EZIIO_FOUND TRUE)
    set(EZIIO_LIBRARY "${EZIIO_SDK_DIR}/EziMOTIONPlusE.lib")
    message(STATUS "EziMOTIONPlusE SDK found: ${EZIIO_SDK_DIR}")
else()
    set(EZIIO_FOUND FALSE)
    message(WARNING "EziMOTIONPlusE SDK not found - Ezi-IO runs in simulated stub mode")
endif()

# Dear ImGui core + GLFW/OpenGL3 backends.
add_library(imgui_lib STATIC
    ${imgui_SOURCE_DIR}/imgui.cpp
    ${imgui_SOURCE_DIR}/imgui_draw.cpp
    ${imgui_SOURCE_DIR}/imgui_tables.cpp
    ${imgui_SOURCE_DIR}/imgui_widgets.cpp
    ${imgui_SOURCE_DIR}/imgui_demo.cpp
    ${imgui_SOURCE_DIR}/backends/imgui_impl_glfw.cpp
    ${imgui_SOURCE_DIR}/backends/imgui_impl_opengl3.cpp)
target_include_directories(imgui_lib PUBLIC
    ${imgui_SOURCE_DIR}
    ${imgui_SOURCE_DIR}/backends)
target_link_libraries(imgui_lib PUBLIC glfw)

# ImPlot (needs ImGui).
add_library(implot_lib STATIC
    ${implot_SOURCE_DIR}/implot.cpp
    ${implot_SOURCE_DIR}/implot_items.cpp)
target_include_directories(implot_lib PUBLIC ${implot_SOURCE_DIR})
target_link_libraries(implot_lib PUBLIC imgui_lib)

# ---- firedog_hw : the UI-free layers (core + hardware + workflow) ----
# Both executables (the FireDog client and the FireDogRig console) share these,
# so they are compiled once here instead of twice. Nothing under these three
# directories includes ImGui/GLFW/OpenGL/Lua — keep it that way.
#
# The vendor-SDK gating lives on this library and propagates PUBLIC:
# HardwareManager.h pulls PylonCamera.h & co into the UI translation units, so
# the HAS_* definitions must reach every consumer, not just the drivers.
file(GLOB_RECURSE FIREDOG_HW_SOURCES CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/src/core/*.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/core/*.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/hardware/*.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/hardware/*.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/workflow/*.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/workflow/*.h")

add_library(firedog_hw STATIC ${FIREDOG_HW_SOURCES})

target_include_directories(firedog_hw PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/src)

# ws2_32 for the WAGO Modbus TCP client (always compiled — no vendor SDK).
target_link_libraries(firedog_hw PUBLIC
    nlohmann_json::nlohmann_json spdlog::spdlog ws2_32)

# Pylon camera (real SDK if found; otherwise PylonCamera.cpp compiles its stub).
if(PYLON_FOUND)
    target_compile_definitions(firedog_hw PUBLIC HAS_PYLON)
    target_include_directories(firedog_hw PUBLIC "${PYLON_INCLUDE_DIR}")
    target_link_libraries(firedog_hw PUBLIC ${PYLON_LIBRARIES})
endif()

# Keithley VISA (real driver if found; otherwise Keithley6482.cpp compiles its stub).
if(VISA_FOUND)
    target_compile_definitions(firedog_hw PUBLIC HAS_VISA)
    target_include_directories(firedog_hw PUBLIC "${VISA_INCLUDE_DIR}")
    target_link_libraries(firedog_hw PUBLIC "${VISA_LIBRARY}")
endif()

# PI hexapod (real GCS2 driver if found; otherwise PIHexapod.cpp compiles its stub).
if(PI_FOUND)
    target_compile_definitions(firedog_hw PUBLIC HAS_PI)
    target_include_directories(firedog_hw PUBLIC "${PI_SDK_DIR}")
    target_link_libraries(firedog_hw PUBLIC "${PI_LIBRARY}")
endif()

# ACS gantry (real ACSC driver if found; otherwise ACSController.cpp compiles its stub).
if(ACS_FOUND)
    target_compile_definitions(firedog_hw PUBLIC HAS_ACS)
    target_include_directories(firedog_hw PUBLIC "${ACS_SDK_DIR}")
    target_link_libraries(firedog_hw PUBLIC "${ACS_LIBRARY}" ws2_32)
endif()

# Ezi-IO (real FASTECH driver if found; otherwise EziIoDevice.cpp compiles its stub).
# The SDK header auto-links via #pragma comment(lib, "EziMOTIONPlusE.lib"), so the
# link directory must be known in addition to the explicit library path.
if(EZIIO_FOUND)
    target_compile_definitions(firedog_hw PUBLIC HAS_EZIIO)
    target_include_directories(firedog_hw PUBLIC "${EZIIO_SDK_DIR}")
    target_link_directories(firedog_hw PUBLIC "${EZIIO_SDK_DIR}")
    target_link_libraries(firedog_hw PUBLIC "${EZIIO_LIBRARY}")
endif()

# ---- Runtime staging, shared by every executable ----
# config/ next to the exe (every config path in the code is cwd-relative) plus
# the vendor runtime DLLs. Both exes land in the same output directory, so the
# second call is a copy_if_different no-op — but building only one target must
# still stage everything that target needs to run.
function(firedog_stage_runtime tgt)
    add_custom_command(TARGET ${tgt} POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_directory
                "${CMAKE_CURRENT_SOURCE_DIR}/config" "$<TARGET_FILE_DIR:${tgt}>/config"
        COMMENT "Copying config to output directory")

    if(PYLON_FOUND AND PYLON_RUNTIME_DIR AND EXISTS "${PYLON_RUNTIME_DIR}")
        add_custom_command(TARGET ${tgt} POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_directory
                    "${PYLON_RUNTIME_DIR}" "$<TARGET_FILE_DIR:${tgt}>"
            COMMENT "Copying Pylon runtime DLLs")
    endif()

    if(PI_FOUND AND EXISTS "${PI_SDK_DIR}/PI_GCS2_DLL_x64.dll")
        add_custom_command(TARGET ${tgt} POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
                    "${PI_SDK_DIR}/PI_GCS2_DLL_x64.dll" "$<TARGET_FILE_DIR:${tgt}>"
            COMMENT "Copying PI GCS2 runtime DLL")
    endif()

    if(EZIIO_FOUND AND EXISTS "${EZIIO_SDK_DIR}/EziMOTIONPlusE.dll")
        add_custom_command(TARGET ${tgt} POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
                    "${EZIIO_SDK_DIR}/EziMOTIONPlusE.dll" "$<TARGET_FILE_DIR:${tgt}>"
            COMMENT "Copying EziMOTIONPlusE runtime DLL")
    endif()
endfunction()

# ---- FireDog executable (the GUI client) ----
# Sources are globbed from src/ so new files are picked up automatically.
# CONFIGURE_DEPENDS re-globs at build time (Ninja/MSBuild), so adding or removing
# a file under src/ reconfigures on the next build — no need to edit this list.
file(GLOB_RECURSE FIREDOG_SOURCES CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/*.h")

# core/, hardware/ and workflow/ are compiled into firedog_hw and linked below —
# compiling them a second time here would duplicate every symbol.
list(FILTER FIREDOG_SOURCES EXCLUDE REGEX "/src/(core|hardware|workflow)/")

add_executable(FireDog
    FireDog.cpp
    FireDog.h
    ${FIREDOG_SOURCES})

# The project include dirs arrive PUBLIC from firedog_hw; stb is UI/command-only.
target_include_directories(FireDog PRIVATE ${stb_SOURCE_DIR})

# opengl32 for the base GL calls in GlHost (texture upload, clear).
target_link_libraries(FireDog PRIVATE firedog_hw
    glfw imgui_lib implot_lib opengl32 lua_static sol2::sol2)
# sol2 safety checks (argument/type validation on every boundary crossing) —
# cheap insurance for a layer that commands real hardware.
target_compile_definitions(FireDog PRIVATE SOL_ALL_SAFETIES_ON=1)

firedog_stage_runtime(FireDog)

# ---- FireDogRig executable (console hardware bring-up rig) ----
# Same drivers, same Workflow::Start() bring-up as the client, no UI. Lives in
# tools/ rather than src/ so the FireDog glob above never sees its main().
# See docs/rig-console.md.
file(GLOB_RECURSE FIREDOG_RIG_SOURCES CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/tools/rig/*.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/tools/rig/*.h")

add_executable(FireDogRig ${FIREDOG_RIG_SOURCES})
target_link_libraries(FireDogRig PRIVATE firedog_hw)

firedog_stage_runtime(FireDogRig)

# TODO (layer 1 hardware SDKs still to port) — mirror the detection blocks above:
#   PI GCS2  -> external/PI/PI_GCS2_DLL_x64.lib   (DONE: HAS_PI, copies DLL)
#   ACS ACSC -> external/ACS/ACSCL_x64.LIB        (DONE: HAS_ACS, links ws2_32)
#   NI-VISA  -> CLD101x + UDP3305S PSU (DONE: HAS_VISA); SMU still a SimulatedDevice
# Detect each with if(EXISTS ...) and gate the real adapter behind a HAS_* compile def.
