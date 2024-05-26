if(NOT TARGET JsonCpp::JsonCpp)
  # Attempt to find JsonCpp library - modify paths as needed
  find_library(JSONCPP_LIBRARY NAMES jsoncpp PATHS /user/local/Cellar/jsoncpp/1.9.5/lib /usr/local/lib)
  find_path(JSONCPP_INCLUDE_DIR NAMES json/json.h PATHS /user/local/Cellar/jsoncpp/1.9.5/include /usr/local/include)

  # Create an imported target
  add_library(JsonCpp::JsonCpp SHARED IMPORTED)
  set_target_properties(JsonCpp::JsonCpp PROPERTIES
    IMPORTED_LOCATION "${JSONCPP_LIBRARY}"
    INTERFACE_INCLUDE_DIRECTORIES "${JSONCPP_INCLUDE_DIR}")
endif()