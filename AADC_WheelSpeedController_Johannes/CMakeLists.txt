set(FILTER_NAME aadc_wheelSpeedController_Johannes)
 
# Specify the filter module target
adtf_add_filter(${FILTER_NAME}
                stdafx.h
                cWheelSpeedController_Johannes.h
                cWheelSpeedController_Johannes.cpp
)

# set folder (only needed and available for MSVC)
adtf_set_folder(${FILTER_NAME} aadc_wheelSpeedController_Johannes) 

# Specify where it should be installed to
adtf_install_plugin(${FILTER_NAME} ${CMAKE_INSTALL_BINARY})
#adtf_install_plugin(${FILTER_NAME} ${AADC_INSTALL_DIR})
