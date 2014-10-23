/** @file
    @brief Header

    @date 2014

    @author
    Ryan Pavlik
    <ryan@sensics.com>;
    <http://sensics.com>
*/

// Copyright 2014 Sensics, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
// 	http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef INCLUDED_GenericDeleter_h_GUID_FF10B285_AE30_45B6_BFD4_D84F98FB1EF0
#define INCLUDED_GenericDeleter_h_GUID_FF10B285_AE30_45B6_BFD4_D84F98FB1EF0

namespace ogvr {
namespace detail {
    /// Generic deleter function - the kind of thing easily hidden in a
    /// header-only
    /// C++ wrapper.
    template <typename T> inline void generic_deleter(void *obj) {
        T *o = static_cast<T *>(obj);
        delete o;
    }
} // end of namespace detail
} // end of anonymous namespace

#endif // INCLUDED_GenericDeleter_h_GUID_FF10B285_AE30_45B6_BFD4_D84F98FB1EF0