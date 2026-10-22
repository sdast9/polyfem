# json spec engine (https://github.com/geometryprocessing/json-spec-engine)
# License: MIT

if(TARGET jse::jse)
    return()
endif()

message(STATUS "Third-party: creating target 'jse::jse'")

include(CPM)
# The patch looks rules up by pointer instead of scanning the whole spec: it
# groups the rules by pointer once per verify_json / inject_defaults call (in
# spec order, so every lookup returns what the scan returned, in the same
# order) and iterates matches by reference. Upstream copied every rule of the
# spec (4,571 after includes) at every input node and again for every optional
# field it filled with a default, which took ~2.7 s per call -- two calls, most
# of PolyFEM_bin's start-up. Validation results, messages and injected
# defaults are unchanged. CPM keys its source cache on the patch's path, not
# its content: rename the patch when changing it.
CPMAddPackage(
    URI "gh:geometryprocessing/json-spec-engine#81c4dc4092115f37a841285dede3d39559f7fb86"
    PATCHES "${CMAKE_CURRENT_LIST_DIR}/patches/jse-rule-index.patch"
)
