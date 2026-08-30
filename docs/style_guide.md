# Integrated Motor Controller Firmware — C++ Style Guide

This guide describes the conventions used across the Integrated Motor Controller firmware codebase.
It is the source of truth for code formatting and naming. Update this file to
reflect your preferences; ask Claude to reformat any file to match the current
state of this guide.

---

## 1. Language Standard and Compiler Flags

- Compile as **C++20** (`-std=c++20`).

  C++20 was adopted deliberately, for two features. Use them; the rest of the
  standard's additions are not endorsed by default.

  - **Designated initializers.** Aggregate initialisation names its fields:
    `PidGains{.kp = 1.2f, .ki = 0.0f, .kd = 0.05f}`. Prefer this anywhere a
    struct carries more than two fields, and especially throughout
    `app/tactical/config/`, whose whole job is to be read rather than executed.
  - **`std::span`.** Replaces every `(pointer, length)` parameter pair. A
    `std::span<uint8_t>` cannot silently ignore its length the way a raw pointer
    plus a `len` argument can.

  Also fine where they read better: `<bit>` (`std::bit_cast`,
  `std::countl_zero`) in place of `reinterpret_cast` or `memcpy` for register
  and protocol bit-twiddling; `constinit` on statically allocated objects;
  `using enum` inside switch-heavy state machines.

  **Do not use**, notwithstanding that the standard offers them:

  - **Coroutines** — the frame is heap-allocated by default, which contradicts
    the no-heap-after-boot rule. The custom-allocator workaround costs more
    clarity than the plain state machine it would replace.
  - **`<ranges>` and `std::format`** — both carry real code size on
    newlib-nano. Use a `snprintf`-style formatter.
  - **Modules** — not usable with this CMake and toolchain setup.

  C++23 is deliberately not adopted: `std::expected` would suit the `bool init()`
  convention well, but arm-none-eabi-gcc 13.2 (what `ubuntu:24.04` ships) does not
  have it, and pinning a newer toolchain for one feature is not worth the CI
  fragility. Revisit when the base image moves.
- Always compile with **`-fno-exceptions`** and **`-fno-rtti`**. No code in
  this codebase may throw or catch exceptions, or use `typeid` / `dynamic_cast`.
- Fixed-width integer types (`uint8_t`, `int32_t`, etc.) from `<cstdint>` are
  preferred over `int`, `long`, etc. everywhere.

---

## 2. File Layout

### 2.1 Header files (`.hpp`)

```
/*******************************************************************************
 * @file ClassName.hpp
 * @brief One-sentence summary.
 *
 * Longer description: purpose, hardware it owns, pin assignments,
 * usage example, relevant notes.
 *
 * @author Integrated Motor Controller firmware team
 ******************************************************************************/

#pragma once

extern "C"
{
#include "stm32h5xx_hal.h"
}

#include <cstdint>
// ... other standard includes

// Class definition follows
```

- Use **`#pragma once`** (not `#ifndef` guards).
- HAL / CMSIS headers are always wrapped in `extern "C" { }`.
- Standard library includes (`<cstdint>`, `<cstring>`, etc.) come after the
  `extern "C"` block.
- No implementation code in headers (exception: trivial one-liners that are
  explicitly performance-critical and noted as such).

### 2.2 Source files (`.cpp`)

```
/*******************************************************************************
 * @file ClassName.cpp
 * @brief One-sentence summary (mirrors header).
 ******************************************************************************/

#include "ClassName.hpp"

#include <algorithm>
// ... other standard includes
```

- The class's own header is always the **first** `#include`.
- File-local constants are declared `static constexpr` at file scope (not
  `#define`).
- File-local helper constants that form a logical group (e.g. a register map)
  go in a named `namespace` rather than loose at file scope.

### 2.3 End of File

End the file with a separator comment and a new line.
```cpp
/* EOF -----------------------------------------------------------------------*/
```

---

## 3. Naming

| Entity | Convention | Example |
|---|---|---|
| Class | `PascalCase` | `CRSFReceiver`, `Motor` |
| Public method | `camelCase` | `readMillivolts()`, `setPulseUs()` |
| Private method | `camelCase` | `csAssert()`, `checkCrc()` |
| Private member variable | leading `m_` + `camelCase` | `m_htim`, `m_accelScale` |
| Constructor parameter | `camelCase` (no trailing `_`) | `htim`, `csPort` |
| Local variable | `camelCase` | `delta`, `pageOffset` |
| Class constant (`static constexpr`) | `kCamelCase` | `kWhoAmIValue`, `kMaxFrameLen` |
| File-local constant (`static constexpr`) | `kCamelCase` | `kSpiTimeout`, `kReadFlag` |
| Named namespace (register maps, etc.) | `PascalCase` | `namespace Reg { }` |
| `enum class` enumerators | `PascalCase` | `Direction::Forward` |
| Macro (avoid; HAL macros only) | `ALL_CAPS` | `GPIO_PIN_SET` |
| Pointers | leading `p` + `PascalCase` | `pI2CHandle`, `m_pClassData` |

- **No Hungarian notation.** Type information belongs in the type, not the name.
- Boolean variables and methods that return `bool` should read as a question:
  `isFaulted()`, `hasNewData()`, `m_lineReady`.

---

## 4. Classes

### 4.1 Member order (top to bottom within `public:` / `private:`)

1. `static constexpr` constants
2. Nested types (`enum class`, inner `struct`)
3. Constructor(s)
4. Destructor (only if needed)
5. Public methods in logical groups (init, then operations, then queries)
6. Private methods
7. Private data members

### 4.2 Constructors

- Use an **initializer list** for every member. One member per line, leading
  comma style:

```cpp
Motor::Motor(TIM_HandleTypeDef* pwmTimer, ...)
    : pwmTimer_(pwmTimer)
    , pwmChannel_(pwmChannel)
    , timerPeriod_(0U)
{}
```

- Initialise every member in the constructor — never leave members
  uninitialised.
- Prefer zero-initialisation literals: `0U`, `0.0f`, `nullptr`, `false`.

### 4.3 `enum class`

- Always use `enum class` (scoped enum), not plain `enum`.
- Specify the underlying type explicitly when the size matters:

```cpp
enum class Direction : uint8_t
{
  Forward = 0,
  Reverse = 1
};
```

---

## 5. Functions and Methods

### 5.1 Brace style

Opening brace on a **new line** for function / method definitions:

```cpp
bool Motor::init()
{
    ...
}
```

Opening brace on the **new line** for control flow:

```cpp
if (condition)
{
    ...
}
else
{
    ...
}

for (uint8_t i = 0U; i < n; ++i)
{
    ...
}
```

Single-statement bodies still use braces:

```cpp
if (val < 0)
{
  return false;
}   // acceptable for compact guard clauses
```

### 5.2 Long parameter lists

When a constructor or function call does not fit on one line, align parameters
under the opening parenthesis:

```cpp
Motor::Motor(TIM_HandleTypeDef* pwmTimer,
             uint32_t           pwmChannel,
             GPIO_TypeDef*      phPort,
             uint16_t           phPin)
```

### 5.3 Return values and error handling

- Functions that can fail return `bool` (`true` = success).
- Functions that return data and can fail either use an output parameter
  (`bool read(AccelGyro& out)`) or a sentinel value (`-1` for signed, `0` for
  unsigned where 0 is never a valid result).
- **No exceptions.** Never use `throw`.

### 5.4 Early returns / guard clauses

Validate preconditions at the top of a function and return early:

```cpp
bool EEPROM::readByte(uint16_t address, uint8_t& out)
{
  if (address >= kSizeBytes)
  {
    return false;
  }
  ...
}
```

---

## 6. Types and Literals

### 6.1 Integer literals

All unsigned integer literals carry the `U` suffix; floating-point literals
carry `.0f`:

```cpp
uint32_t kAdcMax = 4095U;
float    kVdda   = 3.3f;
uint8_t  i       = 0U;
```

### 6.2 Casts

Always use C++ named casts. Never use C-style casts.

```cpp
static_cast<uint32_t>(someFloat)      // numeric conversions
static_cast<uint8_t>(reg | kReadFlag) // narrowing
reinterpret_cast<uint8_t*>(str)       // pointer reinterpretation (sparingly)
const_cast<uint8_t*>(buf)             // only when HAL forces it
```

### 6.3 `nullptr`

Always `nullptr`, never `NULL` or `0` for pointer contexts.

### 6.4 Fixed-width integer types

Prefer `uint8_t`, `uint16_t`, `uint32_t`, `int16_t`, `int32_t`, `int64_t`
over `int`, `unsigned`, `long`, etc. Use `size_t` only where a standard API
requires it.

---

## 7. Constants

- Use `static constexpr` — never `#define` — for numeric and string constants.
- Class-scoped constants go inside the class body under `public:` if they are
  part of the API, or `private:` if internal.
- File-scoped constants (used only in one `.cpp`) are declared `static constexpr`
  at file scope before first use.
- Named namespaces group related file-scoped constants (e.g. a device register
  map):

```cpp
namespace Reg
{
  static constexpr uint8_t WHO_AM_I = 0x75U;
  static constexpr uint8_t PWR_MGMT0 = 0x4EU;
}
```

---

## 8. Comments

### 8.1 Doxygen

Every header file, every class, every public method, every public constant, and
every public struct field gets a Doxygen comment. Use `/** */` for multi-line,
`///` for trailing single-line:

```cpp
/*******************************************************************************
 * @brief Read the battery voltage.
 *
 * Performs a single-conversion ADC read, applies the divider ratio, and
 * returns the result in millivolts.
 *
 * @return Battery voltage in millivolts, or 0 on ADC error.
 ******************************************************************************/
uint32_t readMillivolts();

uint16_t currentPulseUs_; ///< Currently commanded pulse width (µs).
```

Required Doxygen tags per entity:

| Entity | Required tags |
|---|---|
| File (`.hpp` and `.cpp`) | `@file`, `@brief`, `@author` |
| Class | `@brief` + descriptive body |
| Public method | `@brief`, `@param` (each), `@return` (if non-void) |
| Public constant | trailing `///< description` |
| Public struct field | trailing `///< description` |
| Private member | trailing `///< description` (brief only) |
| `@todo` | Use for known gaps (e.g. values to fill in from schematic) |
| `@note` | Use for important caveats (interrupt context, timing, etc.) |
| `@see` | Use to cross-reference specs or related classes |

### 8.2 Inline comments

- Use `//` inline comments to explain *why*, not *what*. The code says what;
  the comment says why.
- Section dividers in long `.cpp` files include a descriptor and '-' to make line 80 chars:

```cpp
/* Private Helpers -----------------------------------------------------------*/
```

---

## 9. Includes

Order within a file (each group separated by a blank line):

1. The class's own header (`.cpp` files only, always first)
2. Other project headers (`"..."`)
3. `extern "C" { }` block for HAL / CMSIS (`.hpp` files) or a single-line
   `extern "C"` forward declaration (`.cpp` files when needed)
4. C++ standard library headers (`<algorithm>`, `<cstdint>`, `<cstring>`, etc.)

No `using namespace std;` anywhere in the codebase.

---

## 10. Memory and Resource Management

- **No heap allocation** in interrupt context or time-critical paths.
- Prefer stack allocation and static storage; avoid `new` / `delete`.
- All buffers have compile-time-known sizes (`static constexpr` or template
  parameter).
- HAL handles are owned by the CubeMX-generated `main.c`; driver classes hold
  **non-owning pointers** to them. Drivers do not free HAL resources.

---

## 11. Embedded-Specific Rules

- **No `std::vector`, `std::string`, `std::map`** or other heap-backed
  containers. Use fixed-size arrays.
- **No `std::function`** (heap allocation inside). Use plain function pointers
  or template parameters.
- **No virtual functions** in performance-critical paths (vtable overhead +
  indirect branch). Mark classes `final` when subclassing is not intended.
- Floating-point is acceptable on the STM32H563 (Cortex-M33 with FPU), but
  avoid it in ISRs unless the FPU context is saved.
- Prefer `++i` over `i++` in loop increments.
- Always use `HAL_GetTick()` for millisecond timestamps; never roll your own
  tick counter.

---

## 12. Formatting Summary (Quick Reference)

| Rule | Value |
|---|---|
| Indentation | 2 spaces (no tabs) |
| Line length | 100 characters soft limit, 120 hard limit |
| Brace style | Allman |
| Pointer/reference alignment | Attached to type: `uint8_t* buf`, `AccelGyro& out` |
| Blank lines between methods | 1 blank line |
| Trailing whitespace | None |
| Newline at end of file | Yes |
