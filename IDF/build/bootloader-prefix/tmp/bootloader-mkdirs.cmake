# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "C:/Users/sky/esp/v5.2.1/esp-idf/components/bootloader/subproject"
  "F:/Dian/IDF/build/bootloader"
  "F:/Dian/IDF/build/bootloader-prefix"
  "F:/Dian/IDF/build/bootloader-prefix/tmp"
  "F:/Dian/IDF/build/bootloader-prefix/src/bootloader-stamp"
  "F:/Dian/IDF/build/bootloader-prefix/src"
  "F:/Dian/IDF/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "F:/Dian/IDF/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "F:/Dian/IDF/build/bootloader-prefix/src/bootloader-stamp${cfgdir}") # cfgdir has leading slash
endif()
