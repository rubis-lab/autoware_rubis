// Copyright 2020 The Autoware Foundation
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

#ifndef TRAJECTORY_SMOOTHER__VISIBILITY_CONTROL_HPP_
#define TRAJECTORY_SMOOTHER__VISIBILITY_CONTROL_HPP_

////////////////////////////////////////////////////////////////////////////////
#if defined(__WIN32)
  #if defined(TRAJECTORY_SMOOTHER_BUILDING_DLL) || defined(TRAJECTORY_SMOOTHER_EXPORTS)
    #define TRAJECTORY_SMOOTHER_PUBLIC __declspec(dllexport)
    #define TRAJECTORY_SMOOTHER_LOCAL
  #else  // defined(TRAJECTORY_SMOOTHER_BUILDING_DLL) || defined(TRAJECTORY_SMOOTHER_EXPORTS)
    #define TRAJECTORY_SMOOTHER_PUBLIC __declspec(dllimport)
    #define TRAJECTORY_SMOOTHER_LOCAL
  #endif  // defined(TRAJECTORY_SMOOTHER_BUILDING_DLL) || defined(TRAJECTORY_SMOOTHER_EXPORTS)
#elif defined(__linux__)
  #define TRAJECTORY_SMOOTHER_PUBLIC __attribute__((visibility("default")))
  #define TRAJECTORY_SMOOTHER_LOCAL __attribute__((visibility("hidden")))
#elif defined(__APPLE__)
  #define TRAJECTORY_SMOOTHER_PUBLIC __attribute__((visibility("default")))
  #define TRAJECTORY_SMOOTHER_LOCAL __attribute__((visibility("hidden")))
#else
  #error "Unsupported Build Configuration"
#endif

#endif  // TRAJECTORY_SMOOTHER__VISIBILITY_CONTROL_HPP_
