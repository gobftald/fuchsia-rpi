// Copyright 2019 The Fuchsia Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "src/developer/shell/console/executor.h"

#include <lib/fdio/spawn.h>
#include <lib/zx/process.h>
#include <zircon/compiler.h>
#include <zircon/status.h>

#include <iostream>
#include <map>
#include <memory>
#include <vector>

#include "src/developer/shell/common/result.h"

namespace shell::console {

Executor::Executor(llcpp::fuchsia::shell::Shell::SyncClient* client)
    : context_id_(0), client_(client) {}

Executor::~Executor() = default;

Err Executor::Execute(std::unique_ptr<Command> command,
                      fit::function<void(const std::string&)> out_callback,
                      fit::function<void(const std::string&)> err_callback,
                      fit::callback<void()> done_callback) {
  if (!command->parse_error().ok()) {
    std::string msg = "Invalid command: ";
    msg.append(command->parse_error().msg);
    err_callback(msg);
    return Err(ZX_ERR_NEXT, zx_status_get_string(ZX_ERR_NEXT));
  }
  if (command->nodes().empty()) {
    return Err(ZX_ERR_NEXT, zx_status_get_string(ZX_ERR_NEXT));
  }
  bool done = false;
  context_id_ += 1;
  llcpp::fuchsia::shell::Shell::ResultOf::CreateExecutionContext create_result =
      client_->CreateExecutionContext(context_id_);
  if (!create_result.ok()) {
    return Err(create_result.status(), create_result.error());
  }

  // TODO: Make sure that add_result is small enough to fit in a single FIDL message.  Otherwise,
  // split it.
  llcpp::fuchsia::shell::Shell::ResultOf::AddNodes add_result =
      client_->AddNodes(context_id_, command->nodes().DefsAsVectorView());
  if (!add_result.ok()) {
    return Err(add_result.status(), add_result.error());
  }

  llcpp::fuchsia::shell::Shell::ResultOf::ExecuteExecutionContext execute_result =
      client_->ExecuteExecutionContext(context_id_);
  if (!execute_result.ok()) {
    return Err(execute_result.status(), execute_result.error());
  }

  while (!done) {
    llcpp::fuchsia::shell::Shell::EventHandlers handlers;
    handlers.on_text_result = [&out_callback](uint64_t context_id, ::fidl::StringView result,
                                              bool partial_result) {
      out_callback(result.data());
      return ZX_OK;
    };
    handlers.on_execution_done = [&done](uint64_t context_id,
                                         ::llcpp::fuchsia::shell::ExecuteResult result) {
      done = true;
      return ZX_OK;
    };
    handlers.on_error = [&err_callback](
                            uint64_t context_id,
                            ::fidl::VectorView<::llcpp::fuchsia::shell::Location> locations,
                            ::fidl::StringView error_message) {
      err_callback(error_message.data());
      return ZX_OK;
    };
    handlers.on_result = [&err_callback, &out_callback](
                             uint64_t context_id,
                             fidl::VectorView<llcpp::fuchsia::shell::Node> nodes,
                             bool partial_result) -> zx_status_t {
      std::string msg;
      if (partial_result) {
        err_callback("Result too large: partial results not supported");
        return ZX_OK;
      }
      std::stringstream ss;
      shell::common::DeserializeResult deserialize;
      deserialize.Deserialize(nodes)->Dump(ss);
      out_callback(ss.str());
      return ZX_OK;
    };
    zx_status_t result = client_->HandleEvents(std::move(handlers));
    if (result != ZX_OK) {
      return Err(result, zx_status_get_string(result));
    }
  }
  if (done_callback != nullptr) {
    done_callback();
  }

  return Err(ZX_ERR_NEXT, zx_status_get_string(ZX_ERR_NEXT));
}

void Executor::KillForegroundTask() {
  // TODO(fidl-tools-team): What happens when we hit ^C?
}

}  // namespace shell::console
