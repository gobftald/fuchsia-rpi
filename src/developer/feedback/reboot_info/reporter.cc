// Copyright 2019 The Fuchsia Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "src/developer/feedback/reboot_info/reporter.h"

#include <lib/async/cpp/task.h>
#include <lib/fit/result.h>
#include <lib/zx/time.h>
#include <zircon/types.h>

#include <string>
#include <vector>

#include "src/developer/feedback/reboot_info/reboot_reason.h"
#include "src/developer/feedback/utils/errors.h"
#include "src/lib/files/file.h"
#include "src/lib/fsl/vmo/file.h"
#include "src/lib/fsl/vmo/strings.h"
#include "src/lib/fxl/logging.h"
#include "src/lib/syslog/cpp/logger.h"

namespace feedback {
namespace {

constexpr char kHasReportedOnPath[] = "/tmp/has_reported_on_reboot_log.txt";

}  // namespace

Reporter::Reporter(async_dispatcher_t* dispatcher, std::shared_ptr<sys::ServiceDirectory> services)
    : dispatcher_(dispatcher),
      executor_(dispatcher),
      crash_reporter_(dispatcher, services),
      cobalt_(dispatcher, services) {}

void Reporter::ReportOn(const RebootLog& reboot_log, zx::duration crash_reporting_delay) {
  if (files::IsFile(kHasReportedOnPath)) {
    FX_LOGS(INFO)
        << "Reboot log has already been reported on in another instance of this component "
           "for this boot cycle";
    return;
  }

  if (!files::WriteFile(kHasReportedOnPath, /*data=*/"", /*size=*/0)) {
    FX_LOGS(ERROR) << "Failed to record reboot log as reported on";
  }

  // TODO(49689): Start logging Cobalt events and filing crash reports for non-parseable reboot
  // logs.
  if (reboot_log.RebootReason() == RebootReason::kNotParseable && !reboot_log.HasRebootLogStr()) {
    FX_LOGS(ERROR) << "Error parsing reboot log";
    return;
  }

  cobalt_.LogOccurrence(ToCobaltRebootReason(reboot_log.RebootReason()));

  // We don't want to file a crash report on graceful  or cold reboots.
  if (IsGraceful(reboot_log.RebootReason()) || reboot_log.RebootReason() == RebootReason::kCold) {
    return;
  }

  executor_.schedule_task(FileCrashReport(reboot_log, crash_reporting_delay));
}

namespace {

fuchsia::feedback::CrashReport CreateCrashReport(const RebootLog& reboot_log) {
  // Build the crash report.
  fuchsia::feedback::GenericCrashReport generic_report;
  generic_report.set_crash_signature(ToCrashSignature(reboot_log.RebootReason()));
  fuchsia::feedback::SpecificCrashReport specific_report;
  specific_report.set_generic(std::move(generic_report));
  fuchsia::feedback::CrashReport report;
  report.set_program_name(ToCrashProgramName(reboot_log.RebootReason()));
  if (reboot_log.HasUptime()) {
    report.set_program_uptime(reboot_log.Uptime().get());
  }
  report.set_specific_report(std::move(specific_report));

  // Build the crash report attachments.
  if (reboot_log.HasRebootLogStr()) {
    fsl::SizedVmo vmo;
    if (fsl::VmoFromString(reboot_log.RebootLogStr(), &vmo)) {
      std::vector<fuchsia::feedback::Attachment> attachments(1);
      attachments.back().key = "reboot_crash_log";
      attachments.back().value = std::move(vmo).ToTransport();
      report.set_attachments(std::move(attachments));
    }
  }

  return report;
}

}  // namespace

::fit::promise<void> Reporter::FileCrashReport(const RebootLog& reboot_log,
                                               const zx::duration delay) {
  auto report = CreateCrashReport(reboot_log);

  delayed_crash_reporting_.Reset([this, report = std::move(report)]() mutable {
    crash_reporter_->File(std::move(report), [this](::fit::result<void, zx_status_t> result) {
      if (crash_reporter_.IsAlreadyDone()) {
        return;
      }

      if (result.is_error()) {
        FX_PLOGS(ERROR, result.error()) << "fuchsia.feedback.CrashReporter/File returned an error";
        crash_reporter_.CompleteError(Error::kBadValue);
      } else {
        crash_reporter_.CompleteOk();
      }
    });
  });

  if (const zx_status_t status = async::PostDelayedTask(
          dispatcher_, [cb = delayed_crash_reporting_.callback()] { cb(); }, delay);
      status != ZX_OK) {
    FX_PLOGS(ERROR, status) << "Failed to post delayed task, no crash reporting";
    crash_reporter_.CompleteError(Error::kAsyncTaskPostFailure);
  }

  return crash_reporter_.WaitForDone().then([this](const ::fit::result<void, Error>& result) {
    delayed_crash_reporting_.Cancel();
    if (result.is_error()) {
      FX_LOGS(ERROR) << "Failed to file a crash report: " << ToString(result.error());
    }

    return ::fit::ok();
  });
}

}  // namespace feedback