// Parser for flat JSON objects of numbers, as used by preset files:
//   { "J": 0.9, "pulse_nm": 150, "pulse_period_ms": 700 }
// Nested objects, arrays and string values are rejected. No heap.
#pragma once

namespace simcore {

// Called once per member. Return false to stop with an error.
typedef bool (*JsonMemberFn)(const char *key, float value, void *ctx);

// Returns nullptr on success, otherwise a short error message.
const char *json_flat_parse(const char *text, JsonMemberFn fn, void *ctx);

} // namespace simcore
