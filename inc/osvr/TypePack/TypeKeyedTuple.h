/** @file
    @brief Header

    @date 2015

    @author
    Sensics, Inc.
    <http://sensics.com/osvr>
*/

// Copyright 2015 Sensics, Inc.
// TypePack is part of OSVR-Core.
//
// Use, modification and distribution is subject to the
// Boost Software License, Version 1.0. (See accompanying
// file LICENSE_1_0.txt or copy at
// http://www.boost.org/LICENSE_1_0.txt)

#ifndef INCLUDED_TypeKeyedTuple_h_GUID_04E79266_BD2F_458D_B7AC_DF5F35CC6EC4
#define INCLUDED_TypeKeyedTuple_h_GUID_04E79266_BD2F_458D_B7AC_DF5F35CC6EC4

// Internal Includes
#include "Contains.h"
#include "FindFirst.h"
#include "Apply.h"
#include "Transform.h"
#include "ApplyList.h"
#include "Quote.h"

// Library/third-party includes
// - none

// Standard includes
#include <tuple>

namespace osvr {
namespace typepack {
    template <typename KeyList, typename ComputeValueTypes>
    class TypeKeyedTuple;
    namespace detail {
        template <typename Key, typename TypeKeyedContainer>
        struct GetIndexInTypeKeyedImpl {
            using key_types = typename TypeKeyedContainer::key_types;
            static_assert(contains<key_types, Key>::value,
                          "key type given not in the list of key types for "
                          "this container");
            using type = find_first<key_types, Key>;
        };
        template <typename Key, typename TypeKeyedContainer>
        using GetIndexInTypeKeyed =
            t_<GetIndexInTypeKeyedImpl<Key, TypeKeyedContainer>>;

        template <typename TypeKeyedContainer>
        struct TypeKeyedContainerStorageImpl;

        template <typename KeyList, typename ComputeValueTypes>
        struct TypeKeyedContainerStorageImpl<
            TypeKeyedTuple<KeyList, ComputeValueTypes>> {
            using value_types = transform<KeyList, ComputeValueTypes>;
            using type = apply_list<quote<std::tuple>, value_types>;
        };

        template <typename TypeKeyedContainer>
        using TypeKeyedContainerStorage =
            t_<TypeKeyedContainerStorageImpl<TypeKeyedContainer>>;

        template <typename Key, typename TypeKeyedContainer>
        struct ValueAtTypeKeyedImpl;
        template <typename Key, typename KeyList, typename ComputeValueTypes>
        struct ValueAtTypeKeyedImpl<
            Key, TypeKeyedTuple<KeyList, ComputeValueTypes>> {
            using container = TypeKeyedTuple<KeyList, ComputeValueTypes>;
            using tuple = TypeKeyedContainerStorage<container>;
            using index = GetIndexInTypeKeyed<Key, container>;
            using type = t_<std::tuple_element<index::value, tuple>>;
        };
        template <typename Key, typename TypeKeyedContainer>
        using ValueAtTypeKeyed =
            t_<ValueAtTypeKeyedImpl<Key, TypeKeyedContainer>>;
        template <typename Key, typename TypeKeyedContainer>
        using RefAtTypeKeyed = t_<std::add_lvalue_reference<
            ValueAtTypeKeyed<Key, TypeKeyedContainer>>>;

        template <typename Key, typename TypeKeyedContainer>
        using ConstRefAtTypeKeyed = t_<std::add_lvalue_reference<
            t_<std::add_const<ValueAtTypeKeyed<Key, TypeKeyedContainer>>>>>;
        template <typename Key, typename TypeKeyedContainer>
        using RRefAtTypeKeyed = t_<std::add_rvalue_reference<
            ValueAtTypeKeyed<Key, TypeKeyedContainer>>>;
    } // namespace detail
    /// @brief Provides a data structure where a value of heterogeneous data
    /// types may be stored at runtime for each of the "key" types in a list.
    /// The runtime data type stored is computed by an alias class.
    ///
    /// Vaguely replaces the functionality of a boost::fusion::map.
    ///
    /// Values can be accessed with a templated getElement member function, a
    /// subscript operator overload, as well as the nonmember get() free
    /// function templates.
    ///
    /// Element access performance is equal to `get()` on a std::tuple (should
    /// be constant)
    template <typename KeyList, typename ComputeValueTypes>
    class TypeKeyedTuple {
        using type = TypeKeyedTuple<KeyList, ComputeValueTypes>;

      public:
        using key_types = KeyList;
        using container_type = detail::TypeKeyedContainerStorage<type>;
        /// @brief Accessor by key type - either cast nullptr to a Key const *,
        /// or explicitly specify a template argument.
        template <typename Key>
        auto getElement(Key const * = nullptr)
            -> detail::RefAtTypeKeyed<Key, type> {
            using index = detail::GetIndexInTypeKeyed<Key, type>;
            return std::get<index::value>(container_);
        }

        /// @brief Const accessor by key type - either cast nullptr to a Key
        /// const *, or explicitly specify a template argument.
        template <typename Key>
        auto getElement(Key const * = nullptr) const
            -> detail::ConstRefAtTypeKeyed<Key, type> {
            using index = detail::GetIndexInTypeKeyed<Key, type>;
            return std::get<index::value>(container_);
        }

      private:
        container_type container_;
    };

    /// @brief Nonmember accessor for typepack::TypeKeyedTuple - explicitly
    /// specify your key type.
    template <typename Key, typename KL, typename CVT>
    inline detail::RefAtTypeKeyed<Key, TypeKeyedTuple<KL, CVT>>
    get(TypeKeyedTuple<KL, CVT> &c) {
        return c.template getElement<Key>();
    }

    /// @brief Nonmember const accessor for typepack::TypeKeyedTuple -
    /// explicitly specify your key type.
    template <typename Key, typename KL, typename CVT>
    inline detail::ConstRefAtTypeKeyed<Key, TypeKeyedTuple<KL, CVT>>
    get(const TypeKeyedTuple<KL, CVT> &c) {
        return c.template getElement<Key>();
    }
} // namespace typepack
} // namespace osvr

#endif // INCLUDED_TypeKeyedTuple_h_GUID_04E79266_BD2F_458D_B7AC_DF5F35CC6EC4
