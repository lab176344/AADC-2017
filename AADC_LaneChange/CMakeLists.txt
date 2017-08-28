# add the include directory of the filter
include_directories(../include)
# declare our filter
set(FILTER_NAME user_LaneChange)

adtf_add_filter(${FILTER_NAME}
    stdafx.h
    cLaneChange.h
    cLaneChange.cpp
)

adtf_set_folder(${FILTER_NAME} LaneChange) 

# Specify where it should be installed to
adtf_install_plugin(${FILTER_NAME} ${CMAKE_INSTALL_BINARY})
#adtf_install_plugin(${FILTER_NAME} ${AADC_INSTALL_DIR})
