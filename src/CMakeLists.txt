set(UTILS ${CMAKE_CURRENT_SOURCE_DIR}/utils)

add_library(bionautlabs-libs
  ${UTILS}/simUtils.cpp
  ${UTILS}/controlUtils.cpp
)

target_include_directories(bionautlabs-libs PUBLIC
    ${PROJECT_BASE_DIR}/include
    ${PROJECT_BASE_DIR}/thirdParty/eigen
)

target_link_libraries(bionautlabs-libs PUBLIC
  eigen
)