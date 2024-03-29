// Copyright 2020 the Fuchsia Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "sc_stage_1_passkey.h"

#include "lib/fit/result.h"
#include "lib/gtest/test_loop_fixture.h"
#include "src/connectivity/bluetooth/core/bt-host/common/byte_buffer.h"
#include "src/connectivity/bluetooth/core/bt-host/common/random.h"
#include "src/connectivity/bluetooth/core/bt-host/common/uint128.h"
#include "src/connectivity/bluetooth/core/bt-host/hci/connection.h"
#include "src/connectivity/bluetooth/core/bt-host/sm/fake_phase_listener.h"
#include "src/connectivity/bluetooth/core/bt-host/sm/packet.h"
#include "src/connectivity/bluetooth/core/bt-host/sm/sc_stage_1.h"
#include "src/connectivity/bluetooth/core/bt-host/sm/smp.h"
#include "src/connectivity/bluetooth/core/bt-host/sm/util.h"
#include "third_party/googletest/googletest/include/gtest/gtest.h"

namespace bt {
namespace sm {
namespace {

struct ScStage1Args {
  Role role = Role::kInitiator;
  UInt256 local_pub_key_x = {1};
  UInt256 peer_pub_key_x = {2};
  PairingMethod method = PairingMethod::kPasskeyEntryDisplay;
};

class SMP_ScStage1PasskeyTest : public ::gtest::TestLoopFixture {
 public:
  SMP_ScStage1PasskeyTest() = default;
  ~SMP_ScStage1PasskeyTest() = default;

 protected:
  using ConfirmCallback = FakeListener::ConfirmCallback;
  using PasskeyResponseCallback = FakeListener::PasskeyResponseCallback;

  void SetUp() override { NewScStage1Passkey(); }
  void TearDown() override { stage_1_ = nullptr; }
  void NewScStage1Passkey(ScStage1Args args = ScStage1Args()) {
    args_ = args;
    listener_ = std::make_unique<FakeListener>();
    stage_1_ = std::make_unique<ScStage1Passkey>(
        listener_->as_weak_ptr(), args.role, args.local_pub_key_x, args.peer_pub_key_x, args.method,
        [this](ByteBufferPtr sent_packet) {
          auto maybe_reader = ValidPacketReader::ParseSdu(sent_packet);
          ASSERT_TRUE(maybe_reader.is_ok())
              << "Failed to parse packet "
              << ProtocolErrorTraits<sm::ErrorCode>::ToString(maybe_reader.error());
          last_packet_ = maybe_reader.value();
          last_packet_internal_ = std::move(sent_packet);
        },
        [this](fit::result<ScStage1::Output, ErrorCode> out) { last_results_ = out; });
  }

  UInt128 GenerateConfirmValue(const UInt128& random, bool gen_initiator_confirm, uint8_t r) const {
    UInt256 initiator_key = args_.local_pub_key_x, responder_key = args_.peer_pub_key_x;
    if (args_.role == Role::kResponder) {
      std::swap(responder_key, initiator_key);
    }

    return gen_initiator_confirm ? util::F4(initiator_key, responder_key, random, r).value()
                                 : util::F4(responder_key, initiator_key, random, r).value();
  }

  struct MatchingPair {
    UInt128 confirm;
    UInt128 random;
  };
  MatchingPair GenerateMatchingConfirmAndRandom(uint8_t r) const {
    MatchingPair pair;
    zx_cprng_draw(pair.random.data(), pair.random.size());
    // If the args_ has Role::kResponder, then we are testing responder flow, so the test code will
    // act in the initiator role, and vice versa if args_ has Role::kInitiator.
    pair.confirm = GenerateConfirmValue(pair.random, args_.role == Role::kResponder, r);
    return pair;
  }

  void DestroyStage1() { stage_1_ = nullptr; }
  ScStage1Passkey* stage_1() { return stage_1_.get(); }
  FakeListener* listener() { return listener_.get(); }
  std::optional<ValidPacketReader> last_packet() const { return last_packet_; }
  std::optional<fit::result<ScStage1::Output, ErrorCode>> last_results() const {
    return last_results_;
  }

 private:
  ScStage1Args args_;
  std::unique_ptr<FakeListener> listener_;
  std::unique_ptr<ScStage1Passkey> stage_1_;
  std::optional<ValidPacketReader> last_packet_ = std::nullopt;
  // To store the last sent SDU so that the the last_packet_ PacketReader points at valid data.
  ByteBufferPtr last_packet_internal_;
  std::optional<fit::result<ScStage1::Output, ErrorCode>> last_results_ = std::nullopt;
};

using SMP_ScStage1PasskeyDeathTest = SMP_ScStage1PasskeyTest;
TEST_F(SMP_ScStage1PasskeyDeathTest, InvalidMethodsDie) {
  ScStage1Args args;
  args.method = PairingMethod::kOutOfBand;
  ASSERT_DEATH_IF_SUPPORTED(NewScStage1Passkey(args), ".*method.*");
  args.method = PairingMethod::kNumericComparison;
  ASSERT_DEATH_IF_SUPPORTED(NewScStage1Passkey(args), ".*method.*");
  args.method = PairingMethod::kJustWorks;
  ASSERT_DEATH_IF_SUPPORTED(NewScStage1Passkey(args), ".*method.*");
}

TEST_F(SMP_ScStage1PasskeyTest, InitiatorPasskeyEntryDisplay) {
  ScStage1Args args;
  args.method = PairingMethod::kPasskeyEntryDisplay;
  args.role = Role::kInitiator;
  NewScStage1Passkey(args);
  uint64_t passkey;
  ConfirmCallback confirm_cb = nullptr;
  listener()->set_display_delegate(
      [&](uint64_t disp_passkey, Delegate::DisplayMethod method, ConfirmCallback cb) {
        ASSERT_EQ(Delegate::DisplayMethod::kPeerEntry, method);
        confirm_cb = std::move(cb);
        passkey = disp_passkey;
      });
  stage_1()->Run();
  ASSERT_TRUE(confirm_cb);
  confirm_cb(true);

  MatchingPair vals;
  UInt128 last_rand;
  for (size_t i = 0; i < 20; ++i) {
    ASSERT_EQ(kPairingConfirm, last_packet()->code());
    const uint8_t r = (passkey & (1 << i)) ? 0x81 : 0x80;
    vals = GenerateMatchingConfirmAndRandom(r);
    PairingConfirmValue init_confirm = last_packet()->payload<PairingConfirmValue>();
    stage_1()->OnPairingConfirm(vals.confirm);

    ASSERT_EQ(kPairingRandom, last_packet()->code());
    last_rand = last_packet()->payload<PairingRandomValue>();
    ASSERT_EQ(GenerateConfirmValue(last_rand, true /*gen_initiator_confirm*/, r), init_confirm);
    stage_1()->OnPairingRandom(vals.random);
  }
  UInt128 passkey_array{0};
  // Copy little-endian uint64 passkey to the UInt128 array needed for Stage 2
  std::memcpy(passkey_array.data(), &passkey, sizeof(uint64_t));
  ScStage1::Output expected_results{.initiator_r = passkey_array,
                                    .responder_r = passkey_array,
                                    .initiator_rand = last_rand,
                                    .responder_rand = vals.random};
  ASSERT_TRUE(last_results()->is_ok());
  ASSERT_EQ(expected_results, last_results()->value());
}

TEST_F(SMP_ScStage1PasskeyTest, InitiatorPasskeyEntryInput) {
  ScStage1Args args;
  args.method = PairingMethod::kPasskeyEntryInput;
  args.role = Role::kInitiator;
  NewScStage1Passkey(args);
  const uint64_t kPasskey = 123456;
  PasskeyResponseCallback passkey_cb = nullptr;
  listener()->set_request_passkey_delegate(
      [&](PasskeyResponseCallback cb) { passkey_cb = std::move(cb); });
  stage_1()->Run();
  ASSERT_TRUE(passkey_cb);
  passkey_cb(kPasskey);

  MatchingPair vals;
  UInt128 last_rand;
  for (size_t i = 0; i < 20; ++i) {
    ASSERT_EQ(kPairingConfirm, last_packet()->code());
    const uint8_t r = (kPasskey & (1 << i)) ? 0x81 : 0x80;
    vals = GenerateMatchingConfirmAndRandom(r);
    PairingConfirmValue init_confirm = last_packet()->payload<PairingConfirmValue>();
    stage_1()->OnPairingConfirm(vals.confirm);

    ASSERT_EQ(kPairingRandom, last_packet()->code());
    last_rand = last_packet()->payload<PairingRandomValue>();
    ASSERT_EQ(GenerateConfirmValue(last_rand, true /*gen_initiator_confirm*/, r), init_confirm);
    stage_1()->OnPairingRandom(vals.random);
  }
  UInt128 passkey_array{0};
  // Copy little-endian uint32 kPasskey to the UInt128 array needed for Stage 2
  std::memcpy(passkey_array.data(), &kPasskey, sizeof(uint64_t));
  ScStage1::Output expected_results{.initiator_r = passkey_array,
                                    .responder_r = passkey_array,
                                    .initiator_rand = last_rand,
                                    .responder_rand = vals.random};
  ASSERT_TRUE(last_results()->is_ok());
  EXPECT_EQ(expected_results, last_results()->value());
}

TEST_F(SMP_ScStage1PasskeyTest, InitiatorPeerConfirmBeforeUserInputFails) {
  ScStage1Args args;
  args.method = PairingMethod::kPasskeyEntryInput;
  args.role = Role::kInitiator;
  NewScStage1Passkey(args);
  PasskeyResponseCallback passkey_cb = nullptr;
  listener()->set_request_passkey_delegate(
      [&](PasskeyResponseCallback cb) { passkey_cb = std::move(cb); });
  stage_1()->Run();
  ASSERT_TRUE(passkey_cb);

  uint8_t r = 0x80;
  MatchingPair vals = GenerateMatchingConfirmAndRandom(r);
  stage_1()->OnPairingConfirm(vals.confirm);
  // The initiator expects to send the pairing confirm first, so it should now reject pairing
  ASSERT_TRUE(last_results()->is_error());
  EXPECT_EQ(ErrorCode::kUnspecifiedReason, last_results()->error());
}

TEST_F(SMP_ScStage1PasskeyTest, ReceiveRandomBeforePeerConfirm) {
  ScStage1Args args;
  args.method = PairingMethod::kPasskeyEntryInput;
  args.role = Role::kInitiator;
  NewScStage1Passkey(args);
  const uint64_t kPasskey = 123456;
  listener()->set_request_passkey_delegate([=](PasskeyResponseCallback cb) { cb(kPasskey); });
  stage_1()->Run();
  ASSERT_EQ(kPairingConfirm, last_packet()->code());

  const uint8_t r = (kPasskey & 1) ? 0x81 : 0x80;
  MatchingPair vals = GenerateMatchingConfirmAndRandom(r);
  stage_1()->OnPairingRandom(vals.random);
  ASSERT_TRUE(last_results()->is_error());
  EXPECT_EQ(ErrorCode::kUnspecifiedReason, last_results()->error());
}

TEST_F(SMP_ScStage1PasskeyTest, ListenerRejectsPasskeyEntryDisplay) {
  ScStage1Args args;
  args.method = PairingMethod::kPasskeyEntryDisplay;
  NewScStage1Passkey(args);
  ConfirmCallback user_confirm = nullptr;
  listener()->set_display_delegate(
      [&](uint32_t, Delegate::DisplayMethod method, ConfirmCallback cb) {
        ASSERT_EQ(Delegate::DisplayMethod::kPeerEntry, method);
        user_confirm = std::move(cb);
      });

  stage_1()->Run();
  ASSERT_TRUE(user_confirm);
  user_confirm(false);
  // No packets should be sent, but Stage 1 should end with kPasskeyEntryFailed
  EXPECT_FALSE(last_packet().has_value());
  EXPECT_EQ(ErrorCode::kPasskeyEntryFailed, last_results()->error());
}

TEST_F(SMP_ScStage1PasskeyTest, ListenerRejectsPasskeyEntryInput) {
  ScStage1Args args;
  args.method = PairingMethod::kPasskeyEntryInput;
  NewScStage1Passkey(args);
  PasskeyResponseCallback passkey_responder = nullptr;
  listener()->set_request_passkey_delegate(
      [&](PasskeyResponseCallback cb) { passkey_responder = std::move(cb); });

  stage_1()->Run();
  ASSERT_TRUE(passkey_responder);

  // Responding with a negative number indicates failure.
  passkey_responder(-12345);
  EXPECT_FALSE(last_packet().has_value());
  EXPECT_EQ(ErrorCode::kPasskeyEntryFailed, last_results()->error());
}

TEST_F(SMP_ScStage1PasskeyTest, StageDestroyedWhileWaitingForPasskeyEntryDisplay) {
  ScStage1Args args;
  args.method = PairingMethod::kPasskeyEntryDisplay;
  NewScStage1Passkey(args);
  ConfirmCallback user_confirm = nullptr;
  listener()->set_display_delegate(
      [&](uint32_t, Delegate::DisplayMethod method, ConfirmCallback cb) {
        ASSERT_EQ(Delegate::DisplayMethod::kPeerEntry, method);
        user_confirm = std::move(cb);
      });

  stage_1()->Run();
  ASSERT_TRUE(user_confirm);

  DestroyStage1();
  // No results should be reported after Stage 1 is destroyed.
  user_confirm(true);
  EXPECT_FALSE(last_packet().has_value());
  EXPECT_FALSE(last_results().has_value());
}

TEST_F(SMP_ScStage1PasskeyTest, StageDestroyedWhileWaitingForPasskeyEntryInput) {
  ScStage1Args args;
  args.method = PairingMethod::kPasskeyEntryInput;
  NewScStage1Passkey(args);
  PasskeyResponseCallback passkey_responder = nullptr;
  listener()->set_request_passkey_delegate(
      [&](PasskeyResponseCallback cb) { passkey_responder = std::move(cb); });

  stage_1()->Run();
  ASSERT_TRUE(passkey_responder);

  DestroyStage1();
  // No results should be reported after Stage 1 is destroyed.
  passkey_responder(12345);
  EXPECT_FALSE(last_packet().has_value());
  EXPECT_FALSE(last_results().has_value());
}

TEST_F(SMP_ScStage1PasskeyTest, ResponderPasskeyEntryDisplay) {
  ScStage1Args args;
  args.method = PairingMethod::kPasskeyEntryDisplay;
  args.role = Role::kResponder;
  NewScStage1Passkey(args);
  uint64_t passkey;
  ConfirmCallback confirm_cb = nullptr;
  listener()->set_display_delegate(
      [&](uint64_t disp_passkey, Delegate::DisplayMethod method, ConfirmCallback cb) {
        ASSERT_EQ(Delegate::DisplayMethod::kPeerEntry, method);
        confirm_cb = std::move(cb);
        passkey = disp_passkey;
      });
  stage_1()->Run();
  ASSERT_TRUE(confirm_cb);
  confirm_cb(true);

  MatchingPair vals;
  UInt128 last_rand;
  for (size_t i = 0; i < 20; ++i) {
    const uint8_t r = (passkey & (1 << i)) ? 0x81 : 0x80;
    vals = GenerateMatchingConfirmAndRandom(r);
    stage_1()->OnPairingConfirm(vals.confirm);
    ASSERT_EQ(kPairingConfirm, last_packet()->code());
    PairingConfirmValue init_confirm = last_packet()->payload<PairingConfirmValue>();

    stage_1()->OnPairingRandom(vals.random);
    ASSERT_EQ(kPairingRandom, last_packet()->code());
    last_rand = last_packet()->payload<PairingRandomValue>();
    ASSERT_EQ(GenerateConfirmValue(last_rand, false /*gen_initiator_confirm*/, r), init_confirm);
  }
  UInt128 passkey_array{0};
  // Copy little-endian uint64 passkey to the UInt128 array needed for Stage 2
  std::memcpy(passkey_array.data(), &passkey, sizeof(uint64_t));
  ScStage1::Output expected_results{.initiator_r = passkey_array,
                                    .responder_r = passkey_array,
                                    .initiator_rand = vals.random,
                                    .responder_rand = last_rand};
  ASSERT_TRUE(last_results()->is_ok());
  EXPECT_EQ(expected_results, last_results()->value());
}

TEST_F(SMP_ScStage1PasskeyTest, ResponderPasskeyEntryInput) {
  ScStage1Args args;
  args.method = PairingMethod::kPasskeyEntryInput;
  args.role = Role::kResponder;
  NewScStage1Passkey(args);
  const uint64_t kPasskey = 123456;
  PasskeyResponseCallback passkey_cb = nullptr;
  listener()->set_request_passkey_delegate(
      [&](PasskeyResponseCallback cb) { passkey_cb = std::move(cb); });
  stage_1()->Run();
  ASSERT_TRUE(passkey_cb);
  passkey_cb(kPasskey);

  MatchingPair vals;
  UInt128 last_rand;
  for (size_t i = 0; i < 20; ++i) {
    const uint8_t r = (kPasskey & (1 << i)) ? 0x81 : 0x80;
    vals = GenerateMatchingConfirmAndRandom(r);
    stage_1()->OnPairingConfirm(vals.confirm);
    ASSERT_EQ(kPairingConfirm, last_packet()->code());
    PairingConfirmValue init_confirm = last_packet()->payload<PairingConfirmValue>();

    stage_1()->OnPairingRandom(vals.random);
    ASSERT_EQ(kPairingRandom, last_packet()->code());
    last_rand = last_packet()->payload<PairingRandomValue>();
    ASSERT_EQ(GenerateConfirmValue(last_rand, false /*gen_initiator_confirm*/, r), init_confirm);
  }
  UInt128 passkey_array{0};
  // Copy little-endian uint64 passkey to the UInt128 array needed for Stage 2
  std::memcpy(passkey_array.data(), &kPasskey, sizeof(uint64_t));
  ScStage1::Output expected_results{.initiator_r = passkey_array,
                                    .responder_r = passkey_array,
                                    .initiator_rand = vals.random,
                                    .responder_rand = last_rand};
  ASSERT_TRUE(last_results()->is_ok());
  EXPECT_EQ(expected_results, last_results()->value());
}

TEST_F(SMP_ScStage1PasskeyTest, ResponderPeerConfirmBeforeUserInputOk) {
  ScStage1Args args;
  args.method = PairingMethod::kPasskeyEntryInput;
  args.role = Role::kResponder;
  NewScStage1Passkey(args);
  const uint64_t kPasskey = 123456;
  PasskeyResponseCallback passkey_cb = nullptr;
  listener()->set_request_passkey_delegate(
      [&](PasskeyResponseCallback cb) { passkey_cb = std::move(cb); });
  stage_1()->Run();
  ASSERT_TRUE(passkey_cb);

  // Do 0th iteration outside loop so we can notify the callback after receiving the confirm.
  uint8_t r = (kPasskey & 1) ? 0x81 : 0x80;
  MatchingPair vals = GenerateMatchingConfirmAndRandom(r);
  stage_1()->OnPairingConfirm(vals.confirm);
  ASSERT_FALSE(last_packet().has_value());

  passkey_cb(kPasskey);
  ASSERT_EQ(kPairingConfirm, last_packet()->code());
  PairingConfirmValue init_confirm = last_packet()->payload<PairingConfirmValue>();

  stage_1()->OnPairingRandom(vals.random);
  ASSERT_EQ(kPairingRandom, last_packet()->code());
  UInt128 last_rand = last_packet()->payload<PairingRandomValue>();
  ASSERT_EQ(GenerateConfirmValue(last_rand, false /*gen_initiator_confirm*/, r), init_confirm);
  for (size_t i = 1; i < 20; ++i) {
    r = (kPasskey & (1 << i)) ? 0x81 : 0x80;
    vals = GenerateMatchingConfirmAndRandom(r);
    stage_1()->OnPairingConfirm(vals.confirm);
    ASSERT_EQ(kPairingConfirm, last_packet()->code());
    init_confirm = last_packet()->payload<PairingConfirmValue>();

    stage_1()->OnPairingRandom(vals.random);
    ASSERT_EQ(kPairingRandom, last_packet()->code());
    last_rand = last_packet()->payload<PairingRandomValue>();
    ASSERT_EQ(GenerateConfirmValue(last_rand, false /*gen_initiator_confirm*/, r), init_confirm);
  }
  UInt128 passkey_array{0};
  // Copy little-endian uint64 passkey to the UInt128 array needed for Stage 2
  std::memcpy(passkey_array.data(), &kPasskey, sizeof(uint64_t));
  ScStage1::Output expected_results{.initiator_r = passkey_array,
                                    .responder_r = passkey_array,
                                    .initiator_rand = vals.random,
                                    .responder_rand = last_rand};
  ASSERT_TRUE(last_results()->is_ok());
  EXPECT_EQ(expected_results, last_results()->value());
}

TEST_F(SMP_ScStage1PasskeyTest, ReceiveTwoPairingConfirmsFails) {
  ScStage1Args args;
  args.method = PairingMethod::kPasskeyEntryInput;
  args.role = Role::kResponder;
  NewScStage1Passkey(args);
  const uint64_t kPasskey = 123456;
  listener()->set_request_passkey_delegate([=](PasskeyResponseCallback cb) { cb(kPasskey); });
  stage_1()->Run();

  const uint8_t r = (kPasskey & 1) ? 0x81 : 0x80;
  MatchingPair vals = GenerateMatchingConfirmAndRandom(r);
  stage_1()->OnPairingConfirm(vals.confirm);
  EXPECT_EQ(kPairingConfirm, last_packet()->code());

  stage_1()->OnPairingConfirm(vals.confirm);
  ASSERT_TRUE(last_results()->is_error());
  EXPECT_EQ(ErrorCode::kUnspecifiedReason, last_results()->error());
}

TEST_F(SMP_ScStage1PasskeyTest, ReceiveTwoPairingRandomsFails) {
  ScStage1Args args;
  args.method = PairingMethod::kPasskeyEntryInput;
  args.role = Role::kResponder;
  NewScStage1Passkey(args);
  const uint64_t kPasskey = 123456;
  listener()->set_request_passkey_delegate([=](PasskeyResponseCallback cb) { cb(kPasskey); });
  stage_1()->Run();

  const uint8_t r = (kPasskey & 1) ? 0x81 : 0x80;
  MatchingPair vals = GenerateMatchingConfirmAndRandom(r);
  stage_1()->OnPairingConfirm(vals.confirm);
  EXPECT_EQ(kPairingConfirm, last_packet()->code());

  stage_1()->OnPairingRandom(vals.random);
  EXPECT_EQ(kPairingRandom, last_packet()->code());

  stage_1()->OnPairingRandom(vals.confirm);
  ASSERT_TRUE(last_results()->is_error());
  EXPECT_EQ(ErrorCode::kUnspecifiedReason, last_results()->error());
}

TEST_F(SMP_ScStage1PasskeyTest, ReceiveMismatchedPairingConfirmFails) {
  ScStage1Args args;
  args.method = PairingMethod::kPasskeyEntryInput;
  args.role = Role::kResponder;
  NewScStage1Passkey(args);
  const uint64_t kPasskey = 123456;
  PasskeyResponseCallback passkey_cb = nullptr;
  listener()->set_request_passkey_delegate(
      [&](PasskeyResponseCallback cb) { passkey_cb = std::move(cb); });
  stage_1()->Run();
  ASSERT_TRUE(passkey_cb);
  passkey_cb(kPasskey);

  // Here we reverse the bit of the passkey used to generate the confirm, so the random and confirm
  // values do not match
  uint8_t r = (kPasskey & 1) ? 0x80 : 0x81;
  MatchingPair vals = GenerateMatchingConfirmAndRandom(r);
  stage_1()->OnPairingConfirm(vals.confirm);
  EXPECT_EQ(kPairingConfirm, last_packet()->code());

  stage_1()->OnPairingRandom(vals.random);
  ASSERT_TRUE(last_results()->is_error());
  EXPECT_EQ(ErrorCode::kConfirmValueFailed, last_results()->error());
}

}  // namespace
}  // namespace sm
}  // namespace bt
