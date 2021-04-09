// Copyright 2021 The Autoware Foundation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef RUBIS_1__VISIBILITY_CONTROL_HPP_
#define RUBIS_1__VISIBILITY_CONTROL_HPP_

////////////////////////////////////////////////////////////////////////////////
#if defined(__WIN32)
  #if defined(RUBIS_1_BUILDING_DLL) || defined(RUBIS_1_EXPORTS)
    #define RUBIS_1_PUBLIC __declspec(dllexport)
    #define RUBIS_1_LOCAL
  #else  // defined(RUBIS_1_BUILDING_DLL) || defined(RUBIS_1_EXPORTS)
    #define RUBIS_1_PUBLIC __declspec(dllimport)
    #define RUBIS_1_LOCAL
  #endif  // defined(RUBIS_1_BUILDING_DLL) || defined(RUBIS_1_EXPORTS)
#elif defined(__linux__)
  #define RUBIS_1_PUBLIC __attribute__((visibility("default")))
  #define RUBIS_1_LOCAL __attribute__((visibility("hidden")))
#elif defined(__APPLE__)
  #define RUBIS_1_PUBLIC __attribute__((visibility("default")))
  #define RUBIS_1_LOCAL __attribute__((visibility("hidden")))
#else
  #error "Unsupported Build Configuration"
#endif

#endif  // RUBIS_1__VISIBILITY_CONTROL_HPP_