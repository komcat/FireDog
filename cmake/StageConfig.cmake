# StageConfig.cmake — run as a POST_BUILD script:
#   cmake -DSRC=<repo>/config -DDST=<exe dir>/config -P StageConfig.cmake
#
# The machine's configuration now lives under the machine root, not here
# (src/core/Paths.h, docs/machine-root-design.md): the app only READS the
# exe-side config/ as shipped defaults. But on a machine that predates that,
# the exe-side hardware.json / motion_transforms.json / *.local.json ARE the
# machine's state until its first start copies them into the root - and a
# machine builds before it runs. So a build still must not overwrite them with
# the repository's copy, which describes some other machine (it used to, with
# copy_directory, and every build silently reverted the machine's wiring).
#
#   config/*.json etc.  copied only when MISSING (a fresh build tree gets the
#                       repo's defaults, an existing machine keeps its own).
#                       When the repo copy differs, a line says so — merge by
#                       hand if the repo gained something the machine needs.
#   config/scripts/     always refreshed (copy-if-different). The standard
#                       scripts change in step with the commands they call
#                       (IsSystemScript); a stale copy silently skips the
#                       newest step.
if(NOT SRC OR NOT DST)
    message(FATAL_ERROR "StageConfig.cmake needs -DSRC=... -DDST=...")
endif()

file(GLOB_RECURSE _files RELATIVE "${SRC}" "${SRC}/*")
foreach(_f IN LISTS _files)
    set(_src "${SRC}/${_f}")
    set(_dst "${DST}/${_f}")
    get_filename_component(_dir "${_dst}" DIRECTORY)
    file(MAKE_DIRECTORY "${_dir}")

    if(_f MATCHES "^scripts/")
        file(COPY_FILE "${_src}" "${_dst}" ONLY_IF_DIFFERENT)
    elseif(NOT EXISTS "${_dst}")
        file(COPY_FILE "${_src}" "${_dst}")
        message(STATUS "config: staged new ${_f}")
    else()
        file(SHA256 "${_src}" _a)
        file(SHA256 "${_dst}" _b)
        if(NOT _a STREQUAL _b)
            message(STATUS "config: kept the machine's ${_f} (differs from the repo copy - "
                           "merge by hand if the repo gained something this machine needs)")
        endif()
    endif()
endforeach()
