// Copyright 2018 The Fuchsia Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#pragma once

#include <cstdio>

#include <fbl/string.h>
#include <fbl/vector.h>
#include <lib/zx/time.h>
#include <zxtest/base/observer.h>

namespace zxtest {

// Forward declarations.
class Runner;
class TestCase;
class TestInfo;

namespace internal {

// Helper class to measure a timer interval.
class Timer {
public:
    Timer() : start_(zx::ticks::now()) {}

    void Reset() { start_ = zx::ticks::now(); }

    // Gets the amount of milliseconds since |start_|.
    int64_t GetElapsedTime() const {
        return (zx::ticks::now() - start_) / (zx::ticks::per_second() / 1000);
    }

private:
    zx::ticks start_;
};

// Summary about test results.
struct IterationSummary {

    void Reset();

    int64_t failed = 0;
    int64_t passed = 0;
    int64_t skipped = 0;

    // List of TestCase.Test that will reported on iteration end.
    fbl::Vector<fbl::String> failed_tests;
};

} // namespace internal

// Reports test lifecycle progress.
class Reporter : public LifecycleObserver {
public:
    Reporter() = delete;
    // Prints output to |stream|. If |stream| is |nullptr| it will behave as writing to /dev/null.
    explicit Reporter(FILE* stream);
    Reporter(const Reporter&) = delete;
    Reporter(Reporter&&) = default;
    ~Reporter() final = default;

    Reporter& operator=(const Reporter&) = delete;
    Reporter& operator=(Reporter&&) = delete;

    // Reports before any test activity starts.
    void OnProgramStart(const Runner& runner) final;

    // Reports before every test iteration begins.
    void OnIterationStart(const Runner& runner, int iteration) final;

    // Reports before any environment setup is called.
    void OnEnvironmentSetUp(const Runner& runner) final;

    // Reports before every TestCase is set up.
    void OnTestCaseStart(const TestCase& test_case) final;

    // Reports before every test starts.
    void OnTestStart(const TestCase& test_case, const TestInfo& test) final;

    // Reports before every test starts.
    void OnTestSkip(const TestCase& test_case, const TestInfo& test) final;

    // Reports before every TestCase is set up.
    void OnTestFailure(const TestCase& test_case, const TestInfo& test) final;

    // Reports before every TestCase is set up.
    void OnTestSuccess(const TestCase& test_case, const TestInfo& test) final;

    // Reports before every TestCase is torn down.
    void OnTestCaseEnd(const TestCase& test_case) final;

    // Reports before any environment setup is called.
    void OnEnvironmentTearDown(const Runner& runner) final;

    // Reports after each test iteration completes.
    void OnIterationEnd(const Runner& runner, int iteration) final;

    // Reports after all test activity is completed.
    void OnProgramEnd(const Runner& runner) final;

    // Returns the stream where the reporter is writing to.
    FILE* stream() const { return stream_; }

private:
    // Pointer to where report should be written to.
    FILE* stream_ = nullptr;

    struct {
        internal::Timer program;
        internal::Timer iteration;
        internal::Timer test_case;
        internal::Timer test;
    } timers_;

    // The counters reset on every iteration.
    internal::IterationSummary iteration_summary_;
};

} // namespace zxtest
