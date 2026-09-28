find_path(Glog_INCLUDE_DIRS NAMES glog/logging.h)
find_library(Glog_LIBRARY NAMES glog)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(Glog DEFAULT_MSG
  Glog_INCLUDE_DIRS Glog_LIBRARY)

if(Glog_FOUND)
  set(Glog_LIBRARIES "${Glog_LIBRARY}")
  set(Glog_INCLUDE_DIRS "${Glog_INCLUDE_DIRS}")
endif()

mark_as_advanced(Glog_INCLUDE_DIRS Glog_LIBRARY)
