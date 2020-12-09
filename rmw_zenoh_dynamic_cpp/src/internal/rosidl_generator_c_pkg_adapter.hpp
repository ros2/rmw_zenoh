// Copyright 2020 Continental AG
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

//From Foxy distro rosidl_generator_c has been renamed to rosidl_generator_c,
//purpose of this header file is to alias old C type names to new ones, that way the
//rest of the code doesn't have to care about old package name regardless of distro.
//any rosidl_generator_c to rosidl_runtime_c adaptions should be added here

#if ROS_DISTRO >= FOXY

#include <rosidl_runtime_c/primitives_sequence.h>
#include <rosidl_runtime_c/primitives_sequence_functions.h>
#include <rosidl_runtime_c/string.h>
#include <rosidl_runtime_c/string_functions.h>

#else

#include <rosidl_generator_c/primitives_sequence.h>
#include <rosidl_generator_c/primitives_sequence_functions.h>
#include <rosidl_generator_c/string.h>
#include <rosidl_generator_c/string_functions.h>

#define ADAPT_PRIMITIVE_SEQUENCE_NAMESPACE(STRUCT_NAME) \
  using rosidl_runtime_c__##STRUCT_NAME##__Sequence = rosidl_generator_c__##STRUCT_NAME##__Sequence;

ADAPT_PRIMITIVE_SEQUENCE_NAMESPACE(float)
ADAPT_PRIMITIVE_SEQUENCE_NAMESPACE(double)
ADAPT_PRIMITIVE_SEQUENCE_NAMESPACE(long_double)
ADAPT_PRIMITIVE_SEQUENCE_NAMESPACE(char)
ADAPT_PRIMITIVE_SEQUENCE_NAMESPACE(wchar)
ADAPT_PRIMITIVE_SEQUENCE_NAMESPACE(boolean)
ADAPT_PRIMITIVE_SEQUENCE_NAMESPACE(octet)
ADAPT_PRIMITIVE_SEQUENCE_NAMESPACE(uint8)
ADAPT_PRIMITIVE_SEQUENCE_NAMESPACE(int8)
ADAPT_PRIMITIVE_SEQUENCE_NAMESPACE(uint16)
ADAPT_PRIMITIVE_SEQUENCE_NAMESPACE(int16)
ADAPT_PRIMITIVE_SEQUENCE_NAMESPACE(uint32)
ADAPT_PRIMITIVE_SEQUENCE_NAMESPACE(int32)
ADAPT_PRIMITIVE_SEQUENCE_NAMESPACE(uint64)
ADAPT_PRIMITIVE_SEQUENCE_NAMESPACE(int64)
ADAPT_PRIMITIVE_SEQUENCE_NAMESPACE(String)

#undef ADAPT_PRIMITIVE_SEQUENCE_NAMESPACE

using rosidl_runtime_c__String = rosidl_generator_c__String;

#define rosidl_runtime_c__float__Sequence__fini(sequence) rosidl_generator_c__float__Sequence__fini(sequence)
#define rosidl_runtime_c__float__Sequence__init(sequence, size) rosidl_generator_c__float__Sequence__init(sequence, size)

#define rosidl_runtime_c__double__Sequence__fini(sequence) rosidl_generator_c__double__Sequence__fini(sequence)
#define rosidl_runtime_c__double__Sequence__init(sequence, size) rosidl_generator_c__double__Sequence__init(sequence, size)

#define rosidl_runtime_c__long_double__Sequence__fini(sequence) rosidl_generator_c__long_double__Sequence__fini(sequence)
#define rosidl_runtime_c__long_double__Sequence__init(sequence, size) rosidl_generator_c__long_double__Sequence__init(sequence, size)

#define rosidl_runtime_c__char__Sequence__fini(sequence) rosidl_generator_c__char__Sequence__fini(sequence)
#define rosidl_runtime_c__char__Sequence__init(sequence, size) rosidl_generator_c__char__Sequence__init(sequence, size)

#define rosidl_runtime_c__wchar__Sequence__fini(sequence) rosidl_generator_c__wchar__Sequence__fini(sequence)
#define rosidl_runtime_c__wchar__Sequence__init(sequence, size) rosidl_generator_c__wchar__Sequence__init(sequence, size)

#define rosidl_runtime_c__boolean__Sequence__fini(sequence) rosidl_generator_c__boolean__Sequence__fini(sequence)
#define rosidl_runtime_c__boolean__Sequence__init(sequence, size) rosidl_generator_c__boolean__Sequence__init(sequence, size)

#define rosidl_runtime_c__octet__Sequence__fini(sequence) rosidl_generator_c__octet__Sequence__fini(sequence)
#define rosidl_runtime_c__octet__Sequence__init(sequence, size) rosidl_generator_c__octet__Sequence__init(sequence, size)

#define rosidl_runtime_c__uint8__Sequence__fini(sequence) rosidl_generator_c__uint8__Sequence__fini(sequence)
#define rosidl_runtime_c__uint8__Sequence__init(sequence, size) rosidl_generator_c__uint8__Sequence__init(sequence, size)

#define rosidl_runtime_c__int8__Sequence__fini(sequence) rosidl_generator_c__int8__Sequence__fini(sequence)
#define rosidl_runtime_c__int8__Sequence__init(sequence, size) rosidl_generator_c__int8__Sequence__init(sequence, size)

#define rosidl_runtime_c__uint16__Sequence__fini(sequence) rosidl_generator_c__uint16__Sequence__fini(sequence)
#define rosidl_runtime_c__uint16__Sequence__init(sequence, size) rosidl_generator_c__uint16__Sequence__init(sequence, size)

#define rosidl_runtime_c__int16__Sequence__fini(sequence) rosidl_generator_c__int16__Sequence__fini(sequence)
#define rosidl_runtime_c__int16__Sequence__init(sequence, size) rosidl_generator_c__int16__Sequence__init(sequence, size)

#define rosidl_runtime_c__uint32__Sequence__fini(sequence) rosidl_generator_c__uint32__Sequence__fini(sequence)
#define rosidl_runtime_c__uint32__Sequence__init(sequence, size) rosidl_generator_c__uint32__Sequence__init(sequence, size)

#define rosidl_runtime_c__int32__Sequence__fini(sequence) rosidl_generator_c__int32__Sequence__fini(sequence)
#define rosidl_runtime_c__int32__Sequence__init(sequence, size) rosidl_generator_c__int32__Sequence__init(sequence, size)

#define rosidl_runtime_c__uint64__Sequence__fini(sequence) rosidl_generator_c__uint64__Sequence__fini(sequence)
#define rosidl_runtime_c__uint64__Sequence__init(sequence, size) rosidl_generator_c__uint64__Sequence__init(sequence, size)

#define rosidl_runtime_c__int64__Sequence__fini(sequence) rosidl_generator_c__int64__Sequence__fini(sequence)
#define rosidl_runtime_c__int64__Sequence__init(sequence, size) rosidl_generator_c__int64__Sequence__init(sequence, size)

#define rosidl_runtime_c__bool__Sequence__init(sequence, size) rosidl_generator_c__bool__Sequence__init(sequence, size)
#define rosidl_runtime_c__bool__Sequence__fini(sequence) rosidl_generator_c__bool__Sequence__init(sequence)

#define rosidl_runtime_c__byte__Sequence__init(sequence, size) rosidl_generator_c__byte__Sequence__init(sequence, size)
#define rosidl_runtime_c__byte__Sequence__fini(sequence) rosidl_generator_c__byte__Sequence__fini(sequence)

#define rosidl_runtime_c__float32__Sequence__init(sequence, size) rosidl_generator_c__float32__Sequence__init(sequence, size)
#define rosidl_runtime_c__float32__Sequence__fini(sequence) rosidl_generator_c__float32__Sequence__fini(sequence)

#define rosidl_runtime_c__float64__Sequence__init(sequence, size) rosidl_generator_c__float64__Sequence__init(sequence, size)
#define rosidl_runtime_c__float64__Sequence__fini(sequence) rosidl_generator_c__float64__Sequence__fini(sequence)

#define rosidl_runtime_c__String__init(str) rosidl_generator_c__String__init(str)
#define rosidl_runtime_c__String__fini(str) rosidl_generator_c__String__fini(str)
#define rosidl_runtime_c__String__assignn(str, value, n) rosidl_generator_c__String__assignn(str, value, n)
#define rosidl_runtime_c__String__assign(str, value) rosidl_generator_c__String__assign(str, value)
#define rosidl_runtime_c__String__Sequence__init(sequence, size) rosidl_generator_c__String__Sequence__init(sequence, size)
#define rosidl_runtime_c__String__Sequence__fini(sequence) rosidl_generator_c__String__Sequence__fini(sequence)
#define rosidl_runtime_c__String__Sequence__create(size) rosidl_generator_c__String__Sequence__create(size)
#define rosidl_runtime_c__String__Sequence__destroy(sequence) rosidl_generator_c__String__Sequence__destroy(sequence)

#endif
