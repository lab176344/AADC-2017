# add the include directory of the filter
include_directories(../include)
# declare our filter
set(FILTER_NAME user_Crossing)

adtf_add_filter(${FILTER_NAME}
    stdafx.h
    cCrossing.h
    cCrossing.cpp
)

adtf_set_folder(${FILTER_NAME} Crossing) 

# Specify where it should be installed to
#adtf_install_plugin(${FILTER_NAME} ${CMAKE_INSTALL_BINARY})
adtf_install_plugin(${FILTER_NAME} ${AADC_INSTALL_DIR})
