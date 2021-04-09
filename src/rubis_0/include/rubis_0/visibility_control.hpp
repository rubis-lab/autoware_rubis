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

#ifndef RUBIS_0__VISIBILITY_CONTROL_HPP_
#define RUBIS_0__VISIBILITY_CONTROL_HPP_

////////////////////////////////////////////////////////////////////////////////
#if defined(__WIN32)
  #if defined(RUBIS_0_BUILDING_DLL) || defined(RUBIS_0_EXPORTS)
    #define RUBIS_0_PUBLIC __declspec(dllexport)
    #define RUBIS_0_LOCAL
  #else  // defined(RUBIS_0_BUILDING_DLL) || defined(RUBIS_0_EXPORTS)
    #define RUBIS_0_PUBLIC __declspec(dllimport)
    #define RUBIS_0_LOCAL
  #endif  // defined(RUBIS_0_BUILDING_DLL) || defined(RUBIS_0_EXPORTS)
#elif defined(__linux__)
  #define RUBIS_0_PUBLIC __attribute__((visibility("default")))
  #define RUBIS_0_LOCAL __attribute__((visibility("hidden")))
#elif defined(__APPLE__)
  #define RUBIS_0_PUBLIC __attribute__((visibility("default")))
  #define RUBIS_0_LOCAL __attribute__((visibility("hidden")))
#else
  #error "Unsupported Build Configuration"
#endif

#endif  // RUBIS_0__VISIBILITY_CONTROL_HPP_
