// Copyright 2018 The Fuchsia Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "src/developer/feedback/testing/fakes/last_reboot_info_provider.h"

#include <fuchsia/feedback/cpp/fidl.h>

namespace feedback {
namespace fakes {

using namespace fuchsia::feedback;

void LastRebootInfoProvider::Get(GetCallback callback) {
  LastReboot last_reboot;

  last_reboot.set_graceful(true)
      .set_reason(RebootReason::GENERIC_GRACEFUL)
      .set_uptime(zx::hour(1).get());

  callback(std::move(last_reboot));
}

}  // namespace fakes
}  // namespace feedback