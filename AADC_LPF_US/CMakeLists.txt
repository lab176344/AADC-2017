# add the include directory of the filter
include_directories(../include)

set(FILTER_NAME user_lpf_us)

# declare our filter
adtf_add_filter(${FILTER_NAME}          
    stdafx.h            
    stdafx.cpp            
    lpfus.h            
    lpfus.cpp
)

adtf_install_plugin(${FILTER_NAME} ${CMAKE_INSTALL_BINARY})
#adtf_install_plugin(${FILTER_NAME} ${AADC_INSTALL_DIR})
