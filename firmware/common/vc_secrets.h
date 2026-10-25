// Loads config/secrets.h when present, otherwise the placeholder example.
// -D VC_FORCE_PLACEHOLDER_SECRETS=1 ignores secrets.h (used by tools/verify.py
// and CI so that verification never depends on, or embeds, real credentials).
#pragma once

#if __has_include("secrets.h") && !defined(VC_FORCE_PLACEHOLDER_SECRETS)
#include "secrets.h"
#define VC_SECRETS_SOURCE "config/secrets.h"
#else
#include "secrets.example.h"
#define VC_SECRETS_SOURCE "config/secrets.example.h (placeholders)"
#endif
