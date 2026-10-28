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
# PINNED to a release tag, never the moving `docking` branch: every configure
# re-fetches a branch, so a build tree silently picked up 1.93.1 WIP on
# 2026-10-27 - whose restyled StyleColorsDark() made input boxes invisible on
# our themes and whose ImDrawList moved line/rect strokes by half a pixel.
# Upgrade deliberately: bump the tag, read the CHANGELOG's Breaking Changes,
# check every theme (ui/Theme.cpp) and the custom overlays.
FetchContent_Declare(imgui
    GIT_REPOSITORY https://github.com/ocornut/imgui.git
    GIT_TAG v1.92.9b-docking GIT_SHALLOW TRUE)
FetchContent_MakeAvailable(imgui)

# ---- ImPlot (realtime charts) ----
# Pinned by commit for the same reason (ImPlot has no release since it began
# tracking ImGui 1.92); this one guards its 1.93-only paths by version.
FetchContent_Declare(implot
    GIT_REPOSITORY https://github.com/epezent/implot.git
    GIT_TAG 72de27832d1e6299bcc930ce01073d899d420392)   # 2026-10-25
FetchContent_MakeAvailable(implot)

# ---- nlohmann/json (hardware config) ----
# Release tarball, not the git repo: even a shallow clone drags ~170 MB of test
# data per build tree, which reads as a hung configure. The tarball is 111 KB
# and ships the same CMakeLists.txt.
FetchContent_Declare(nlohmann_json
    URL https://github.com/nlohmann/json/releases/download/v3.11.3/json.tar.xz
    URL_HASH SHA256=d6c65aca6b1ed68e7a182f4757257b107ae403032760ed6ef121c9d55e81757d
    DOWNLOAD_EXTRACT_TIMESTAMP FALSE)
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

# ---- Velopack (software update: Setup.exe install, update from GitHub Releases) ----
# The prebuilt C/C++ library from Velopack's own release, pinned by hash, and
# the SAME version /release packs with (vpk) - the two must stay equal. The
# zip carries every platform (~30 MB); only include/ and the Windows DLL are
# used. No CMakeLists inside, so MakeAvailable just populates.
# docs/software-update-design.md.
FetchContent_Declare(velopack_libc
    URL https://github.com/velopack/velopack/releases/download/1.2.161/velopack_libc_1.2.161.zip
    URL_HASH SHA256=31de8f2cf05a47bd5328a9f98b0c41aec1582156f6075f1058b26941f7f17c20
    DOWNLOAD_EXTRACT_TIMESTAMP FALSE)
FetchContent_MakeAvailable(velopack_libc)

# ---- Basler Pylon SDK (layer 1 camera) — optional ----
# The install root is NOT fixed: Basler versions the folder on some machines
# ("Basler/pylon 8") and leaves it bare on others ("Basler/pylon"), and a client
# can sit a major version behind. That is not cosmetic — the import libs and
# DLLs carry the major in their name (PylonBase_v9), so an exe built against one
# does not load against the other. Whichever pylon THIS machine has is the one
# to build against, so the search must find it without anyone editing this file.
#
# Note for whoever comes here from a client machine: the SDK is Development/ and
# Runtime/. "Applications/x64/bin" is the pylon Viewer — Qt and viewer DLLs, no
# PylonBase, no headers. Pointing this at it finds nothing.
#
# Arch follows OUR build, never the environment — same rule as HALCON below.
if(CMAKE_SIZEOF_VOID_P EQUAL 8)
    set(_pylon_arch "x64")
else()
    set(_pylon_arch "Win32")
endif()

# Candidate roots, in priority order; the first one COMPLETE for our arch wins.
#   1. -DPYLON_ROOT=<dir>     explicit override (CI, or a pinned version)
#   2. external/pylon*        vendored copy, same convention as VISA/PI/ACS
#   3. $PYLON_DEV_DIR         what the installer sets (points AT Development)
#   4. $PYLON_ROOT            what older installers set (points at the root)
#   5. Basler/pylon*          whatever is installed, newest first
set(PYLON_ROOT "" CACHE PATH "Basler pylon install dir (empty = auto-detect)")
set(_pylon_candidates "")
if(PYLON_ROOT)
    file(TO_CMAKE_PATH "${PYLON_ROOT}" _pc)
    list(APPEND _pylon_candidates "${_pc}")
endif()
file(GLOB _pylon_vendored "${CMAKE_CURRENT_SOURCE_DIR}/external/pylon*")
list(SORT _pylon_vendored)
list(REVERSE _pylon_vendored)               # newest version folder first
list(APPEND _pylon_candidates ${_pylon_vendored})
if(DEFINED ENV{PYLON_DEV_DIR})
    file(TO_CMAKE_PATH "$ENV{PYLON_DEV_DIR}" _pc)
    string(REGEX REPLACE "/+$" "" _pc "${_pc}")      # the installer sets a trailing slash
    get_filename_component(_pc "${_pc}" DIRECTORY)   # Development -> root
    list(APPEND _pylon_candidates "${_pc}")
endif()
if(DEFINED ENV{PYLON_ROOT})
    file(TO_CMAKE_PATH "$ENV{PYLON_ROOT}" _pc)
    list(APPEND _pylon_candidates "${_pc}")
endif()
foreach(_base "$ENV{ProgramFiles}/Basler" "$ENV{ProgramW6432}/Basler"
              "$ENV{ProgramFiles\(x86\)}/Basler" "C:/Program Files/Basler")
    file(TO_CMAKE_PATH "${_base}" _base)
    file(GLOB _pylon_installed "${_base}/pylon*")
    list(SORT _pylon_installed)
    list(REVERSE _pylon_installed)
    list(APPEND _pylon_candidates ${_pylon_installed})
endforeach()
list(REMOVE_DUPLICATES _pylon_candidates)

set(PYLON_FOUND FALSE)
foreach(_root IN LISTS _pylon_candidates)
    # A root is complete only with BOTH headers and import libs for our arch;
    # a headers-only hit would configure clean and fail at link. Probe the
    # include/pylon DIRECTORY, not a named header — which headers ship varies
    # by version, and a client can be a major behind.
    if(IS_DIRECTORY "${_root}/Development/include/pylon"
       AND IS_DIRECTORY "${_root}/Development/lib/${_pylon_arch}")
        set(PYLON_INCLUDE_DIR "${_root}/Development/include")
        set(PYLON_LIB_DIR     "${_root}/Development/lib/${_pylon_arch}")
        set(PYLON_RUNTIME_DIR "${_root}/Runtime/${_pylon_arch}")
        if(_root MATCHES "^${CMAKE_CURRENT_SOURCE_DIR}/external/")
            set(_pylon_is_vendored TRUE)
        endif()
        set(PYLON_FOUND TRUE)
        break()
    endif()
endforeach()

if(PYLON_FOUND)
    file(GLOB PYLON_LIBRARIES
        "${PYLON_LIB_DIR}/PylonBase_v*.lib"
        "${PYLON_LIB_DIR}/PylonUtility_v*.lib"
        "${PYLON_LIB_DIR}/GCBase_*.lib"
        "${PYLON_LIB_DIR}/GenApi_*.lib")
    if(NOT PYLON_LIBRARIES)
        set(PYLON_FOUND FALSE)
        message(WARNING "Pylon SDK at ${PYLON_LIB_DIR} has no import libs for ${_pylon_arch}"
                        " - camera runs in test-pattern stub mode")
    else()
        message(STATUS "Pylon SDK found: ${PYLON_INCLUDE_DIR}")
    endif()
else()
    message(WARNING "Pylon SDK not found - camera runs in test-pattern stub mode."
                    " Searched: ${_pylon_candidates}")
endif()

# Whether the pylon runtime travels WITH the exe. A vendored copy always does —
# nothing else on that machine provides it. A system install does not by default,
# because its runtime is already on PATH and a stale copy beside the exe would
# shadow the installed pylon after an upgrade, which is how a runtime and its
# camera drivers drift apart.
#
# Turn it on to hand the output folder to a machine that has no pylon 8:
#   cmake --preset x64-release -DFIREDOG_STAGE_PYLON=ON
# It makes the exe START there. It cannot make a camera WORK there — the USB3
# and GigE drivers are installed, never copied — so a machine that must talk to
# a camera still wants Basler's own installer.
option(FIREDOG_STAGE_PYLON "Copy the pylon runtime DLLs beside the exe (deployment)" OFF)
set(PYLON_DEPLOY_FILES "")
if(PYLON_FOUND AND (FIREDOG_STAGE_PYLON OR _pylon_is_vendored) AND EXISTS "${PYLON_RUNTIME_DIR}")
    # The set FireDog needs, not the whole 213 MB runtime: the DLLs the exe
    # imports, what they pull in turn, and the transport layers plus producers
    # that are loaded by name at camera-enumeration time and so appear in no
    # import table. Globbed on version, since every one of these names carries a
    # major that moves (PylonBase_v9, gxapi_v15, uxapi_v14).
    file(GLOB PYLON_DEPLOY_FILES
        "${PYLON_RUNTIME_DIR}/PylonBase_v*.dll"
        "${PYLON_RUNTIME_DIR}/PylonUtility_v*.dll"
        "${PYLON_RUNTIME_DIR}/*_Basler_pylon*.dll"   # GenICam: *_Basler_pylon.dll (pylon 7), *_Basler_pylon_v1.dll (pylon 8)
        "${PYLON_RUNTIME_DIR}/Pylon*_TL.dll"
        "${PYLON_RUNTIME_DIR}/Producer*.cti"
        "${PYLON_RUNTIME_DIR}/gxapi_v*.dll"
        "${PYLON_RUNTIME_DIR}/uxapi_v*.dll"
        "${PYLON_RUNTIME_DIR}/uxtopapi_v*.dll")
    list(LENGTH PYLON_DEPLOY_FILES _n)
    message(STATUS "Pylon runtime staged beside the exe: ${_n} files from ${PYLON_RUNTIME_DIR}")
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

# ---- MVTec HALCON (vision) — optional ----
# No fixed version and no single fixed path: HALCON is licensed per machine, so
# whichever release the operator installed is the one to build against. The
# search must therefore survive an upgrade (24.05 -> 25.05 -> ...) and a machine
# that keeps several releases side by side, without anyone editing this file.
#
# Absence is normal and must stay cheap: FireDog links HALCON with /DELAYLOAD
# (below), so an exe built WITH the SDK still starts on a machine that has no
# HALCON at all. vision/HalconEngine probes the DLLs before calling anything.

# Arch follows OUR build, never the environment. HALCONARCH describes the
# machine's default install; an x86 preset must still link the 32-bit HALCON, and
# silently linking the x64 libs into it is a link error at best.
if(CMAKE_SIZEOF_VOID_P EQUAL 8)
    set(HALCON_ARCH "x64-win64")
else()
    set(HALCON_ARCH "x86sse2-win32")
endif()

# Candidate roots, in priority order; the first one COMPLETE for our arch wins.
#   1. -DHALCON_ROOT=<dir>          explicit override (CI, or a pinned version)
#   2. external/halcon              vendored copy, same convention as Pylon/VISA
#   3. $HALCONROOT                  what the installer sets on this machine
#   4. newest HALCON-* under the standard install bases
set(HALCON_ROOT "" CACHE PATH "MVTec HALCON install dir (empty = auto-detect)")
set(_halcon_candidates "")
if(HALCON_ROOT)
    file(TO_CMAKE_PATH "${HALCON_ROOT}" _hc)
    list(APPEND _halcon_candidates "${_hc}")
endif()
list(APPEND _halcon_candidates "${CMAKE_CURRENT_SOURCE_DIR}/external/halcon")
if(DEFINED ENV{HALCONROOT})
    file(TO_CMAKE_PATH "$ENV{HALCONROOT}" _hc)
    list(APPEND _halcon_candidates "${_hc}")
endif()

# Standard install bases. MVTec's per-user installer lands in LOCALAPPDATA; the
# machine-wide one in Program Files. Both are globbed for HALCON-<version>-<edition>
# (e.g. HALCON-24.05-Progress, HALCON-22.11-Steady).
foreach(_base "$ENV{LOCALAPPDATA}/Programs/MVTec" "$ENV{ProgramW6432}/MVTec"
              "C:/Program Files/MVTec" "C:/Program Files (x86)/MVTec")
    if(_base AND IS_DIRECTORY "${_base}")
        file(GLOB _halcon_installs "${_base}/HALCON-*")
        # Plain lexicographic sort, then reverse = newest first. HALCON versions
        # have been two-digit YY.MM since 17.12, so this orders them correctly
        # without needing COMPARE NATURAL (CMake 3.18+, above our floor of 3.14).
        list(SORT _halcon_installs)
        list(REVERSE _halcon_installs)
        list(APPEND _halcon_candidates ${_halcon_installs})
    endif()
endforeach()
list(REMOVE_DUPLICATES _halcon_candidates)

set(HALCON_FOUND FALSE)
set(_halcon_wrong_arch "")
foreach(_cand IN LISTS _halcon_candidates)
    if(NOT EXISTS "${_cand}/include/halconcpp/HalconCpp.h")
        continue()
    endif()
    if(EXISTS "${_cand}/lib/${HALCON_ARCH}/halconcpp.lib" AND
       EXISTS "${_cand}/lib/${HALCON_ARCH}/halcon.lib")
        set(HALCON_ROOT_DIR    "${_cand}")
        set(HALCON_INCLUDE_DIR "${_cand}/include")
        set(HALCON_LIB_DIR     "${_cand}/lib/${HALCON_ARCH}")
        set(HALCON_FOUND TRUE)
        break()
    endif()
    # An install WITH headers but WITHOUT our arch is the interesting failure —
    # remember it so the message can say so instead of "not found".
    if(NOT _halcon_wrong_arch)
        set(_halcon_wrong_arch "${_cand}")
    endif()
endforeach()

if(HALCON_FOUND)
    message(STATUS "HALCON SDK found: ${HALCON_ROOT_DIR} (${HALCON_ARCH})")
elseif(_halcon_wrong_arch)
    message(STATUS "HALCON found at ${_halcon_wrong_arch} but it has no ${HALCON_ARCH} "
                   "libraries - HALCON-backed vision disabled for this preset "
                   "(auto-focus is unaffected)")
else()
    message(STATUS "HALCON SDK not found (no HALCONROOT, no external/halcon, nothing "
                   "under the MVTec install dirs) - HALCON-backed vision disabled; "
                   "auto-focus is unaffected")
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
    "${CMAKE_CURRENT_SOURCE_DIR}/src/vision/*.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/vision/*.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/workflow/*.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/workflow/*.h")

add_library(firedog_hw STATIC ${FIREDOG_HW_SOURCES})

target_include_directories(firedog_hw PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/src)

# Build identity (core/BuildInfo): the source tree's git state, re-read on EVERY
# build by a script target — a configure-time answer names the wrong commit as
# soon as somebody commits and rebuilds. The header is only rewritten when it
# changes, so a clean rebuild recompiles nothing. Run once here too, so the
# header exists before the first build.
find_package(Git QUIET)
set(FIREDOG_GITINFO_H "${CMAKE_CURRENT_BINARY_DIR}/generated/firedog_gitinfo.h")
set(_gitinfo_args -DSRC=${CMAKE_CURRENT_SOURCE_DIR} -DOUT=${FIREDOG_GITINFO_H}
                  -DGIT_EXECUTABLE=${GIT_EXECUTABLE})
execute_process(COMMAND ${CMAKE_COMMAND} ${_gitinfo_args}
                        -P ${CMAKE_CURRENT_SOURCE_DIR}/cmake/GitInfo.cmake)
add_custom_target(firedog_gitinfo
    COMMAND ${CMAKE_COMMAND} ${_gitinfo_args} -P ${CMAKE_CURRENT_SOURCE_DIR}/cmake/GitInfo.cmake
    BYPRODUCTS ${FIREDOG_GITINFO_H}
    COMMENT "Reading git state for the build info"
    VERBATIM)
add_dependencies(firedog_hw firedog_gitinfo)
target_include_directories(firedog_hw PRIVATE ${CMAKE_CURRENT_BINARY_DIR}/generated)

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

# HALCON (vision/HalconEngine.cpp is the only TU that includes a HALCON header;
# without HAS_HALCON it compiles its "built without the SDK" answer instead).
#
# /DELAYLOAD is the whole point of the arrangement: the DLLs are resolved on
# first use, not at process start, so an exe built here still runs on a machine
# with no HALCON installed. The link options are INTERFACE so both executables
# inherit them from this library — per the project rule, SDK gating never gets
# attached to an executable directly.
if(HALCON_FOUND)
    # The root and arch are baked in as the runtime's first DLL-probe candidate:
    # they name the exact install these import libs came from, which is the one
    # whose DLLs are certain to match. The environment is only the fallback.
    target_compile_definitions(firedog_hw PUBLIC HAS_HALCON
                               "FIREDOG_HALCON_ROOT=\"${HALCON_ROOT_DIR}\""
                               "FIREDOG_HALCON_ARCH=\"${HALCON_ARCH}\"")
    target_include_directories(firedog_hw SYSTEM PUBLIC "${HALCON_INCLUDE_DIR}")
    target_link_libraries(firedog_hw PUBLIC
        "${HALCON_LIB_DIR}/halconcpp.lib" "${HALCON_LIB_DIR}/halcon.lib")
    if(MSVC)
        # version.lib: the probe reads the DLLs' version resource BEFORE loading
        # them, to refuse a runtime from another release than these import libs.
        target_link_libraries(firedog_hw PUBLIC delayimp.lib version.lib)
        target_link_options(firedog_hw INTERFACE
            "/DELAYLOAD:halconcpp.dll" "/DELAYLOAD:halcon.dll")
    endif()
endif()

# ---- Runtime staging, shared by every executable ----
# The SHIPPED config/ next to the exe (scripts + defaults; the machine's own
# state lives under the machine root - docs/machine-root-design.md) plus the
# vendor runtime DLLs. Both exes land in the same output directory, so the
# second call is a copy_if_different no-op — but building only one target must
# still stage everything that target needs to run.
function(firedog_stage_runtime tgt)
    # Machine config is NEVER overwritten: a file is staged only when missing
    # (scripts/ excepted, always refreshed). See cmake/StageConfig.cmake.
    add_custom_command(TARGET ${tgt} POST_BUILD
        COMMAND ${CMAKE_COMMAND}
                "-DSRC=${CMAKE_CURRENT_SOURCE_DIR}/config"
                "-DDST=$<TARGET_FILE_DIR:${tgt}>/config"
                -P "${CMAKE_CURRENT_SOURCE_DIR}/cmake/StageConfig.cmake"
        COMMENT "Staging config (missing files only; scripts refreshed)")

    # UI artwork (the pt1/pt2 mark icons, the recipe row icons). Separate from
    # config/ because it is NOT runtime-writable: nothing edits these, so they
    # never get the xxx.local.json treatment and a rebuild simply refreshes
    # them. The whole tree is copied, so a new icon needs no CMake edit.
    if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/assets")
        add_custom_command(TARGET ${tgt} POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_directory
                    "${CMAKE_CURRENT_SOURCE_DIR}/assets" "$<TARGET_FILE_DIR:${tgt}>/assets"
            COMMENT "Copying assets to output directory")
    endif()

    if(PYLON_DEPLOY_FILES)
        add_custom_command(TARGET ${tgt} POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
                    ${PYLON_DEPLOY_FILES} "$<TARGET_FILE_DIR:${tgt}>"
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

# core/, hardware/, vision/ and workflow/ are compiled into firedog_hw and linked
# below — compiling them a second time here would duplicate every symbol.
list(FILTER FIREDOG_SOURCES EXCLUDE REGEX "/src/(core|hardware|vision|workflow)/")

# The app icon, compiled in as a Win32 resource (see resources/FireDog.rc for
# why it is named GLFW_ICON). RC is enabled here rather than in project()
# because it is the only thing in the build that needs it.
if(WIN32)
    enable_language(RC)
    set(FIREDOG_ICON_RC "${CMAKE_CURRENT_SOURCE_DIR}/resources/FireDog.rc")
else()
    set(FIREDOG_ICON_RC "")
endif()

add_executable(FireDog
    FireDog.cpp
    ${FIREDOG_ICON_RC}
    ${FIREDOG_SOURCES})

# The project include dirs arrive PUBLIC from firedog_hw; stb is UI/command-only.
target_include_directories(FireDog PRIVATE ${stb_SOURCE_DIR})

# opengl32 for the base GL calls in GlHost (texture upload, clear).
target_link_libraries(FireDog PRIVATE firedog_hw
    glfw imgui_lib implot_lib opengl32 lua_static sol2::sol2)
# sol2 safety checks (argument/type validation on every boundary crossing) —
# cheap insurance for a layer that commands real hardware.
target_compile_definitions(FireDog PRIVATE SOL_ALL_SAFETIES_ON=1)

# Velopack: the client only (update/AppUpdater.cpp is its one includer). It is
# not a vendor SDK with a stub - it is always fetched - and FireDogRig has no
# business updating anything, so it stays off firedog_hw.
if(CMAKE_SIZEOF_VOID_P EQUAL 8)
    set(_vpk_dll "${velopack_libc_SOURCE_DIR}/lib/velopack_libc_win_x64_msvc.dll")
else()
    set(_vpk_dll "${velopack_libc_SOURCE_DIR}/lib/velopack_libc_win_x86_msvc.dll")
endif()
target_include_directories(FireDog PRIVATE "${velopack_libc_SOURCE_DIR}/include")
target_link_libraries(FireDog PRIVATE "${_vpk_dll}.lib")
# RENAMED on the way: the import lib names the DLL "velopack_libc.dll" (dumpbin
# /dependents FireDog.exe), whatever the file in the zip is called. Copied under
# its own name, the exe does not start - "velopack_libc.dll was not found".
add_custom_command(TARGET FireDog POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different "${_vpk_dll}"
            "$<TARGET_FILE_DIR:FireDog>/velopack_libc.dll"
    COMMENT "Copying the Velopack runtime DLL (as velopack_libc.dll)")

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
