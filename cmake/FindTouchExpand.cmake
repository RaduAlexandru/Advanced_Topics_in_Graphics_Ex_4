# - Try to find the TouchExpand library
# Once done this will define
#
#  TouchExpand_FOUND - system has TouchExpand
#  TouchExpand_INCLUDE_DIR - **the** TouchExpand include directory
#  TouchExpand_INCLUDE_DIRS - TouchExpand include directories
#  TouchExpand_SOURCES - the TouchExpand source files

if(NOT TouchExpand_FOUND)
	find_path(TouchExpand_INCLUDE_DIR
		NAMES TouchExpand.h
	   	PATHS ${PROJECT_SOURCE_DIR}/externals/TouchExpand/
		DOC "The TouchExpand include directory"
		NO_DEFAULT_PATH)

	if(TouchExpand_INCLUDE_DIR)
	   set(TouchExpand_FOUND TRUE)
	   set(TouchExpand_INCLUDE_DIRS ${TouchExpand_INCLUDE_DIR})
	else()
	   message("+-----------------------------------------------------------------------------+")
	   message("| TouchExpand not found, please unpack the externals/TouchExpand.zip |")
	   message("+-----------------------------------------------------------------------------+")
	   message(FATAL_ERROR "")
	endif()
endif()
