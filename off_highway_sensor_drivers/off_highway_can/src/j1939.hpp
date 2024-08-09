// Copyright 2024 Robert Bosch GmbH and its subsidiaries
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

#pragma once

#include <cstdint>

namespace off_highway_can
{

static constexpr uint8_t kJ1939SourceAddressMask{0xFF};
static constexpr uint16_t kJ1939PgnMask{0xFFFF};
static constexpr uint8_t kJ1939PgnShift{8};

/**
 * \brief Extract J1939 source address from received message id
 *
 * \param id CAN id of received message
 * \return J1939 source address
 */
inline uint8_t get_j1939_source_address(uint32_t id)
{
  return id & kJ1939SourceAddressMask;
}

/**
 * \brief Extract J1939 parameter group number (PGN) from received message id
 *
 * \param id CAN id of received message
 * \return J1939 parameter group number (PGN)
 */
inline uint16_t get_j1939_pgn(uint32_t id)
{
  return (id >> kJ1939PgnShift) & kJ1939PgnMask;
}
}  // namespace off_highway_can
