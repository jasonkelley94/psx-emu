#!/usr/bin/env bash
# run_tests.sh — automated ps1-tests regression runner
#
# Usage:
#   tests/run_tests.sh [--emu PATH] [--tests-dir PATH]
#
# Environment overrides (flags take precedence):
#   PSX_EMU       path to psx_emu binary  (default: build/psx_emu)
#   PS1_TESTS_DIR path to ps1-tests dir   (default: tests/ps1-tests)
#
# Exit code: 0 if all tests pass (or are skipped), non-zero if any fail.

set -euo pipefail

# ── Defaults ──────────────────────────────────────────────────────────────────
REPO_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
EMU="${PSX_EMU:-$REPO_ROOT/build/psx_emu}"
TESTS_DIR="${PS1_TESTS_DIR:-$REPO_ROOT/tests/ps1-tests}"

# ── Argument parsing ───────────────────────────────────────────────────────────
while [[ $# -gt 0 ]]; do
    case "$1" in
        --emu)       EMU="$2";       shift 2 ;;
        --tests-dir) TESTS_DIR="$2"; shift 2 ;;
        *) echo "Unknown argument: $1" >&2; exit 1 ;;
    esac
done

# ── Resolve the emulator ──────────────────────────────────────────────────────
if [[ ! -x "$EMU" ]]; then
    echo "error: emulator not found at '$EMU'" >&2
    echo "       Build first: cmake -B build && cmake --build build" >&2
    echo "       Or set PSX_EMU=/path/to/psx_emu" >&2
    exit 1
fi

if [[ ! -d "$TESTS_DIR" ]]; then
    echo "error: ps1-tests directory not found at '$TESTS_DIR'" >&2
    echo "       Run: bash tests/fetch_tests.sh" >&2
    echo "       Or set PS1_TESTS_DIR=/path/to/ps1-tests" >&2
    exit 1
fi

# ── Test table ────────────────────────────────────────────────────────────────
# Format: "display_name|exe_path|cycles|must_contain|check_no_fail"
# check_no_fail: "1" = combined output must not contain "fail -"
#                "0" = skip that check (test has expected "failed" substrings)
TESTS=(
    "gte/test-all|gte/test-all/test-all.exe|20000000|Failed tests: 0|0"
    "dma/otc-test|dma/otc-test/otc-test.exe|5000000|Done.|1"
    "dma/chopping|dma/chopping/chopping.exe|35000000|Done.|1"
    "dma/chain-looping|dma/chain-looping/chain-looping.exe|5000000|finished = false, irq = false|0"
    "gpu/gp0-e1|gpu/gp0-e1/gp0-e1.exe|5000000|Done.|1"
    "gpu/mask-bit|gpu/mask-bit/mask-bit.exe|5000000|Done.|1"
    "cpu/cop|cpu/cop/cop.exe|5000000|Done.|1"
    "cpu/code-in-io|cpu/code-in-io/code-in-io.exe|5000000|Done.|1"
    "cpu/io-access-bitwidth|cpu/io-access-bitwidth/io-access-bitwidth.exe|5000000|Done.|1"
    "timers/timers|timers/timers.exe|50000000|Done.|1"
    "cdrom/getloc|cdrom/getloc/getloc.exe|30000000|Test passed|0"
)

# ── Runner ────────────────────────────────────────────────────────────────────
pass=0; fail=0; skip=0
results=()

for entry in "${TESTS[@]}"; do
    IFS='|' read -r name exe cycles must_contain check_no_fail <<< "$entry"
    exe_path="$TESTS_DIR/$exe"

    if [[ ! -f "$exe_path" ]]; then
        results+=("SKIP  $name")
        skip=$((skip + 1))
        continue
    fi

    output=$("$EMU" --headless --cycles "$cycles" "$exe_path" 2>&1) || true

    ok=1
    # Must contain the pass string
    if ! echo "$output" | grep -qF "$must_contain"; then
        ok=0
    fi
    # Must not contain "fail -" (for psxtest-framework tests)
    if [[ "$check_no_fail" == "1" ]] && echo "$output" | grep -qF "fail -"; then
        ok=0
    fi

    if [[ $ok -eq 1 ]]; then
        results+=("PASS  $name")
        pass=$((pass + 1))
    else
        results+=("FAIL  $name")
        fail=$((fail + 1))
    fi
done

# ── Report ────────────────────────────────────────────────────────────────────
echo ""
for r in "${results[@]}"; do
    echo "$r"
done
echo ""
echo "Results: $pass passed, $fail failed, $skip skipped"

[[ $fail -eq 0 ]]
