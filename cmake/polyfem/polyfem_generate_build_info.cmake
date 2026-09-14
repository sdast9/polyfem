# Build-time generator of src/polyfem/io/BuildInfo.hpp's data (RB-12 run
# identity). Run in script mode (`cmake -P`) by the polyfem_build_info target
# before every build of the library, so that the identity compiled into the
# binary is the identity of the sources actually built: the effective PolyFEM,
# IPC Toolkit and PolySolve checkouts (commit, dirty state, a hash of the
# uncommitted patch), the declared dependency pins next to them, the compiler,
# configuration and options. The output is rewritten only when it changed, so
# an unchanged identity does not relink anything.
#
# Inputs (-D): OUTPUT, GIT_EXECUTABLE, POLYFEM_SOURCE_DIR, IPC_SOURCE_DIR,
# IPC_DECLARED_REPO, IPC_DECLARED_PIN, IPC_SOURCE_OVERRIDE, POLYSOLVE_SOURCE_DIR,
# POLYSOLVE_DECLARED_REPO, POLYSOLVE_DECLARED_PIN, POLYSOLVE_SOURCE_OVERRIDE,
# DATA_DECLARED_PIN, CXX_COMPILER_ID, CXX_COMPILER_VERSION, CXX_COMPILER,
# BUILD_TYPE, CXX_FLAGS, CXX_FLAGS_CONFIG, GENERATOR, CMAKE_VERSION_STRING,
# SYSTEM_NAME, SYSTEM_PROCESSOR, HOST_SYSTEM, THREADING, OPTIONS (a list of
# NAME=VALUE).

cmake_minimum_required(VERSION 3.25)

# JSON string literal of a CMake string.
function(_bi_quote out value)
    string(REPLACE "\\" "\\\\" value "${value}")
    string(REPLACE "\"" "\\\"" value "${value}")
    string(REPLACE "\n" "\\n" value "${value}")
    string(REPLACE "\r" "\\r" value "${value}")
    string(REPLACE "\t" "\\t" value "${value}")
    set(${out} "\"${value}\"" PARENT_SCOPE)
endfunction()

# JSON object describing one git checkout. `declared_pin` may be empty.
function(_bi_describe_repo out dir declared_repo declared_pin override)
    set(json "{}")
    _bi_quote(q "${dir}")
    string(JSON json SET "${json}" "path" "${q}")
    if(declared_repo)
        _bi_quote(q "${declared_repo}")
        string(JSON json SET "${json}" "declared_repository" "${q}")
    else()
        string(JSON json SET "${json}" "declared_repository" "null")
    endif()
    if(declared_pin)
        _bi_quote(q "${declared_pin}")
        string(JSON json SET "${json}" "declared_pin" "${q}")
    else()
        string(JSON json SET "${json}" "declared_pin" "null")
    endif()
    if(override)
        _bi_quote(q "${override}")
        string(JSON json SET "${json}" "source_override" "${q}")
    else()
        string(JSON json SET "${json}" "source_override" "null")
    endif()

    if(NOT dir OR NOT IS_DIRECTORY "${dir}")
        string(JSON json SET "${json}" "state" "\"unavailable\"")
        string(JSON json SET "${json}" "unavailable_reason" "\"The source directory does not exist at build time\"")
        set(${out} "${json}" PARENT_SCOPE)
        return()
    endif()
    if(NOT GIT_EXECUTABLE)
        string(JSON json SET "${json}" "state" "\"unavailable\"")
        string(JSON json SET "${json}" "unavailable_reason" "\"git was not found when the build was configured\"")
        set(${out} "${json}" PARENT_SCOPE)
        return()
    endif()
    execute_process(
        COMMAND "${GIT_EXECUTABLE}" -C "${dir}" rev-parse --show-toplevel
        OUTPUT_VARIABLE toplevel OUTPUT_STRIP_TRAILING_WHITESPACE
        ERROR_QUIET RESULT_VARIABLE result)
    if(NOT result EQUAL 0)
        string(JSON json SET "${json}" "state" "\"not a git checkout\"")
        string(JSON json SET "${json}" "unavailable_reason" "\"The directory is not inside a git work tree (a downloaded archive, an exported tree)\"")
        set(${out} "${json}" PARENT_SCOPE)
        return()
    endif()
    # A source directory that is not itself a checkout but sits inside one
    # (an extracted archive dropped into a repository) is reported with the
    # enclosing work tree, so a reader can tell the commit does not describe it.
    file(REAL_PATH "${toplevel}" toplevel)
    file(REAL_PATH "${dir}" real_dir)
    _bi_quote(q "${toplevel}")
    string(JSON json SET "${json}" "git_toplevel" "${q}")
    if(real_dir STREQUAL toplevel)
        string(JSON json SET "${json}" "is_toplevel" "true")
    else()
        string(JSON json SET "${json}" "is_toplevel" "false")
    endif()

    execute_process(COMMAND "${GIT_EXECUTABLE}" -C "${dir}" rev-parse HEAD
        OUTPUT_VARIABLE commit OUTPUT_STRIP_TRAILING_WHITESPACE ERROR_QUIET RESULT_VARIABLE result)
    if(NOT result EQUAL 0)
        string(JSON json SET "${json}" "state" "\"unavailable\"")
        string(JSON json SET "${json}" "unavailable_reason" "\"git rev-parse HEAD failed (no commits?)\"")
        set(${out} "${json}" PARENT_SCOPE)
        return()
    endif()
    execute_process(COMMAND "${GIT_EXECUTABLE}" -C "${dir}" rev-parse --abbrev-ref HEAD
        OUTPUT_VARIABLE branch OUTPUT_STRIP_TRAILING_WHITESPACE ERROR_QUIET)
    execute_process(COMMAND "${GIT_EXECUTABLE}" -C "${dir}" show -s --format=%cI HEAD
        OUTPUT_VARIABLE commit_time OUTPUT_STRIP_TRAILING_WHITESPACE ERROR_QUIET)
    execute_process(COMMAND "${GIT_EXECUTABLE}" -C "${dir}" status --porcelain --untracked-files=all
        OUTPUT_VARIABLE status ERROR_QUIET)
    execute_process(COMMAND "${GIT_EXECUTABLE}" -C "${dir}" diff HEAD --no-color --no-ext-diff --binary
        OUTPUT_VARIABLE diff ERROR_QUIET)

    set(tracked_changes 0)
    set(untracked "")
    if(status)
        string(REPLACE ";" "\\;" status_escaped "${status}")
        string(REGEX REPLACE "\n" ";" lines "${status_escaped}")
        foreach(line IN LISTS lines)
            if(line MATCHES "^\\?\\? (.*)$")
                list(APPEND untracked "${CMAKE_MATCH_1}")
            elseif(line)
                math(EXPR tracked_changes "${tracked_changes} + 1")
            endif()
        endforeach()
    endif()
    list(LENGTH untracked untracked_count)
    if(tracked_changes GREATER 0 OR untracked_count GREATER 0)
        set(dirty true)
    else()
        set(dirty false)
    endif()

    # Source-patch hash: the tracked diff against HEAD followed by one line
    # per untracked file with its content hash, so two dirty trees with the
    # same edits get the same hash. Untracked hashing is capped; the count of
    # hashed files is recorded next to the total.
    set(patch "${diff}")
    set(hashed 0)
    set(untracked_names "[]")
    set(index 0)
    foreach(path IN LISTS untracked)
        if(hashed LESS 1000 AND EXISTS "${dir}/${path}" AND NOT IS_DIRECTORY "${dir}/${path}")
            file(SHA256 "${dir}/${path}" file_hash)
            string(APPEND patch "untracked ${path} ${file_hash}\n")
            math(EXPR hashed "${hashed} + 1")
        endif()
        if(index LESS 20)
            _bi_quote(q "${path}")
            string(JSON untracked_names SET "${untracked_names}" "${index}" "${q}")
            math(EXPR index "${index} + 1")
        endif()
    endforeach()
    if(dirty)
        string(SHA256 patch_hash "${patch}")
        string(JSON json SET "${json}" "patch_sha256" "\"${patch_hash}\"")
    else()
        string(JSON json SET "${json}" "patch_sha256" "null")
    endif()

    string(JSON json SET "${json}" "state" "\"git\"")
    string(JSON json SET "${json}" "commit" "\"${commit}\"")
    _bi_quote(q "${branch}")
    string(JSON json SET "${json}" "branch" "${q}")
    _bi_quote(q "${commit_time}")
    string(JSON json SET "${json}" "commit_time" "${q}")
    string(JSON json SET "${json}" "dirty" "${dirty}")
    string(JSON json SET "${json}" "tracked_changes" "${tracked_changes}")
    string(JSON json SET "${json}" "untracked_count" "${untracked_count}")
    string(JSON json SET "${json}" "untracked_hashed" "${hashed}")
    string(JSON json SET "${json}" "untracked_first" "${untracked_names}")
    if(declared_pin)
        string(TOLOWER "${declared_pin}" pin_lower)
        string(TOLOWER "${commit}" commit_lower)
        if(commit_lower STREQUAL pin_lower)
            string(JSON json SET "${json}" "matches_declared_pin" "true")
        else()
            string(JSON json SET "${json}" "matches_declared_pin" "false")
        endif()
    else()
        string(JSON json SET "${json}" "matches_declared_pin" "null")
    endif()
    set(${out} "${json}" PARENT_SCOPE)
endfunction()

set(info "{}")
string(JSON info SET "${info}" "schema" "\"polyfem.build-info\"")
string(JSON info SET "${info}" "version" "1")

set(sources "{}")
_bi_describe_repo(repo "${POLYFEM_SOURCE_DIR}" "" "" "")
string(JSON sources SET "${sources}" "polyfem" "${repo}")
_bi_describe_repo(repo "${IPC_SOURCE_DIR}" "${IPC_DECLARED_REPO}" "${IPC_DECLARED_PIN}" "${IPC_SOURCE_OVERRIDE}")
string(JSON sources SET "${sources}" "ipc_toolkit" "${repo}")
_bi_describe_repo(repo "${POLYSOLVE_SOURCE_DIR}" "${POLYSOLVE_DECLARED_REPO}" "${POLYSOLVE_DECLARED_PIN}" "${POLYSOLVE_SOURCE_OVERRIDE}")
string(JSON sources SET "${sources}" "polysolve" "${repo}")
string(JSON info SET "${info}" "sources" "${sources}")
if(DATA_DECLARED_PIN)
    string(JSON info SET "${info}" "test_data_declared_pin" "\"${DATA_DECLARED_PIN}\"")
else()
    string(JSON info SET "${info}" "test_data_declared_pin" "null")
endif()

set(build "{}")
_bi_quote(q "${CXX_COMPILER_ID} ${CXX_COMPILER_VERSION}")
string(JSON build SET "${build}" "compiler" "${q}")
_bi_quote(q "${CXX_COMPILER}")
string(JSON build SET "${build}" "compiler_path" "${q}")
_bi_quote(q "${BUILD_TYPE}")
string(JSON build SET "${build}" "configuration" "${q}")
_bi_quote(q "${CXX_FLAGS}")
string(JSON build SET "${build}" "cxx_flags" "${q}")
_bi_quote(q "${CXX_FLAGS_CONFIG}")
string(JSON build SET "${build}" "cxx_flags_configuration" "${q}")
_bi_quote(q "${GENERATOR}")
string(JSON build SET "${build}" "generator" "${q}")
_bi_quote(q "${CMAKE_VERSION_STRING}")
string(JSON build SET "${build}" "cmake" "${q}")
_bi_quote(q "${SYSTEM_NAME}-${SYSTEM_PROCESSOR}")
string(JSON build SET "${build}" "target_system" "${q}")
_bi_quote(q "${HOST_SYSTEM}")
string(JSON build SET "${build}" "host_system" "${q}")
_bi_quote(q "${THREADING}")
string(JSON build SET "${build}" "threading" "${q}")
set(options "{}")
foreach(entry IN LISTS OPTIONS)
    if(entry MATCHES "^([^=]+)=(.*)$")
        _bi_quote(q "${CMAKE_MATCH_2}")
        string(JSON options SET "${options}" "${CMAKE_MATCH_1}" "${q}")
    endif()
endforeach()
string(JSON build SET "${build}" "options" "${options}")
string(JSON info SET "${info}" "build" "${build}")

set(content "// Generated at build time by cmake/polyfem/polyfem_generate_build_info.cmake (RB-12); do not edit.
#include <polyfem/io/BuildInfo.hpp>

namespace polyfem::io
{
	const json &build_info()
	{
		static const json info = json::parse(R\"pfbuildinfo(
${info}
)pfbuildinfo\");
		return info;
	}
} // namespace polyfem::io
")
file(WRITE "${OUTPUT}.tmp" "${content}")
execute_process(COMMAND "${CMAKE_COMMAND}" -E copy_if_different "${OUTPUT}.tmp" "${OUTPUT}")
file(REMOVE "${OUTPUT}.tmp")
