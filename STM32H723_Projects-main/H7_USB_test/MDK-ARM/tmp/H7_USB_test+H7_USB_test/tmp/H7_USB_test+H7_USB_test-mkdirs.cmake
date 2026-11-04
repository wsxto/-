# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION ${CMAKE_VERSION}) # this file comes with cmake

# If CMAKE_DISABLE_SOURCE_CHANGES is set to true and the source directory is an
# existing directory in our source tree, calling file(MAKE_DIRECTORY) on it
# would cause a fatal error, even though it would be a no-op.
if(NOT EXISTS "F:/spareE/My_projects/H7_USB_test/MDK-ARM/tmp/H7_USB_test+H7_USB_test")
  file(MAKE_DIRECTORY "F:/spareE/My_projects/H7_USB_test/MDK-ARM/tmp/H7_USB_test+H7_USB_test")
endif()
file(MAKE_DIRECTORY
  "F:/spareE/My_projects/H7_USB_test/MDK-ARM/tmp/1"
  "F:/spareE/My_projects/H7_USB_test/MDK-ARM/tmp/H7_USB_test+H7_USB_test"
  "F:/spareE/My_projects/H7_USB_test/MDK-ARM/tmp/H7_USB_test+H7_USB_test/tmp"
  "F:/spareE/My_projects/H7_USB_test/MDK-ARM/tmp/H7_USB_test+H7_USB_test/src/H7_USB_test+H7_USB_test-stamp"
  "F:/spareE/My_projects/H7_USB_test/MDK-ARM/tmp/H7_USB_test+H7_USB_test/src"
  "F:/spareE/My_projects/H7_USB_test/MDK-ARM/tmp/H7_USB_test+H7_USB_test/src/H7_USB_test+H7_USB_test-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "F:/spareE/My_projects/H7_USB_test/MDK-ARM/tmp/H7_USB_test+H7_USB_test/src/H7_USB_test+H7_USB_test-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "F:/spareE/My_projects/H7_USB_test/MDK-ARM/tmp/H7_USB_test+H7_USB_test/src/H7_USB_test+H7_USB_test-stamp${cfgdir}") # cfgdir has leading slash
endif()
