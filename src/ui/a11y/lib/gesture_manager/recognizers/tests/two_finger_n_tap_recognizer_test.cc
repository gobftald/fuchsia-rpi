// Copyright 2020 The Fuchsia Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "src/ui/a11y/lib/gesture_manager/recognizers/two_finger_n_tap_recognizer.h"

#include <fuchsia/ui/input/accessibility/cpp/fidl.h>

#include <memory>

#include <gtest/gtest.h>

#include "src/lib/testing/loop_fixture/test_loop_fixture.h"
#include "src/ui/a11y/lib/gesture_manager/arena/tests/mocks/mock_contest_member.h"
#include "src/ui/a11y/lib/gesture_manager/gesture_util/util.h"
#include "src/ui/a11y/lib/testing/input.h"

namespace accessibility_test {

namespace {

constexpr int kNumberOfDoubleTaps = 2;
constexpr int kDefaultTaps = 1;

using Phase = fuchsia::ui::input::PointerEventPhase;

class TwoFingerNTapRecognizerTest : public gtest::TestLoopFixture {
 public:
  TwoFingerNTapRecognizerTest() = default;

  void SendPointerEvents(const std::vector<PointerParams>& events) const {
    for (const auto& event : events) {
      SendPointerEvent(event);
    }
  }

  // Constraints to keep in mind when simulating |GestureArena| behavior:
  // * Only send pointer events while a contest member is held.
  void SendPointerEvent(const PointerParams& event) const {
    if (member_.is_held()) {
      recognizer_->HandleEvent(ToPointerEvent(event, 0));
    }
  }

  void CreateGestureRecognizer(int number_of_taps) {
    recognizer_ = std::make_unique<a11y::TwoFingerNTapRecognizer>(
        [this](a11y::GestureContext context) {
          gesture_won_ = true;
          gesture_context_ = context;
        },
        number_of_taps);
  }

  MockContestMember member_;
  std::unique_ptr<a11y::TwoFingerNTapRecognizer> recognizer_;
  bool gesture_won_ = false;
  a11y::GestureContext gesture_context_;
};

// Tests Single tap Gesture Detection case.
TEST_F(TwoFingerNTapRecognizerTest, SingleTapWonAfterGestureDetected) {
  CreateGestureRecognizer(kDefaultTaps);

  recognizer_->OnContestStarted(member_.TakeInterface());

  // Send two-finger-tap event.
  SendPointerEvents(DownEvents(1, {}) + DownEvents(2, {}) + UpEvents(1, {}) + UpEvents(2, {}));

  EXPECT_FALSE(member_.is_held());
  EXPECT_EQ(member_.status(), a11y::ContestMember::Status::kAccepted);
}

// Tests Double tap Gesture Detection case.
TEST_F(TwoFingerNTapRecognizerTest, DoubleTapWonAfterGestureDetected) {
  CreateGestureRecognizer(kNumberOfDoubleTaps);

  recognizer_->OnContestStarted(member_.TakeInterface());

  // Send events for first tap.
  SendPointerEvents(DownEvents(1, {}) + DownEvents(2, {}) + UpEvents(1, {}) + UpEvents(2, {}));

  // Send event for the second tap.
  SendPointerEvents(DownEvents(1, {}) + DownEvents(2, {}) + UpEvents(1, {}) + UpEvents(2, {}));

  EXPECT_FALSE(member_.is_held());
  EXPECT_EQ(member_.status(), a11y::ContestMember::Status::kAccepted);
}

// Tests Single tap gesture detection case where gesture is declared a winner.
TEST_F(TwoFingerNTapRecognizerTest, SingleTapGestureDetectedWin) {
  CreateGestureRecognizer(kDefaultTaps);
  recognizer_->OnContestStarted(member_.TakeInterface());

  SendPointerEvents(DownEvents(1, {}) + DownEvents(2, {}) + UpEvents(1, {}) + UpEvents(2, {}));
  recognizer_->OnWin();

  EXPECT_TRUE(gesture_won_);

  // Wait for the timeout, to make sure the scheduled task doesn't execute and crash us.
  RunLoopFor(a11y::TwoFingerNTapRecognizer::kTapTimeout);
}

// Tests Double tap gesture detection case where gesture is declared a winner.
TEST_F(TwoFingerNTapRecognizerTest, DoubleTapGestureDetectedWin) {
  CreateGestureRecognizer(kNumberOfDoubleTaps);
  recognizer_->OnContestStarted(member_.TakeInterface());

  // Send events for first tap.
  SendPointerEvents(DownEvents(1, {}) + DownEvents(2, {}) + UpEvents(1, {}) + UpEvents(2, {}));

  // Send event for the second tap.
  SendPointerEvents(DownEvents(1, {}) + DownEvents(2, {}) + UpEvents(1, {}) + UpEvents(2, {}));

  recognizer_->OnWin();

  EXPECT_TRUE(gesture_won_);

  // Wait for the timeout, to make sure the scheduled task doesn't execute and crash us.
  RunLoopFor(a11y::TwoFingerNTapRecognizer::kTapTimeout);
}

// Tests Single tap gesture detection case where gesture is declared defeated.
TEST_F(TwoFingerNTapRecognizerTest, SingleTapGestureDetectedLoss) {
  CreateGestureRecognizer(kDefaultTaps);
  recognizer_->OnContestStarted(member_.TakeInterface());

  // Send events for first tap.
  SendPointerEvents(DownEvents(1, {}) + DownEvents(2, {}) + UpEvents(1, {}) + UpEvents(2, {}));

  recognizer_->OnDefeat();

  EXPECT_FALSE(gesture_won_);

  // Wait for the timeout, to make sure the scheduled task doesn't execute and crash us.
  RunLoopFor(a11y::TwoFingerNTapRecognizer::kTapTimeout);
}

// Tests Double tap gesture detection case where gesture is declared defeated.
TEST_F(TwoFingerNTapRecognizerTest, DoubleTapGestureDetectedLoss) {
  CreateGestureRecognizer(kNumberOfDoubleTaps);
  recognizer_->OnContestStarted(member_.TakeInterface());

  // Send events for first tap.
  SendPointerEvents(DownEvents(1, {}) + DownEvents(2, {}) + UpEvents(1, {}) + UpEvents(2, {}));

  // Send event for the second tap.
  SendPointerEvents(DownEvents(1, {}) + DownEvents(2, {}) + UpEvents(1, {}) + UpEvents(2, {}));

  recognizer_->OnDefeat();

  EXPECT_FALSE(gesture_won_);

  // Wait for the timeout, to make sure the scheduled task doesn't execute and crash us.
  RunLoopFor(a11y::TwoFingerNTapRecognizer::kTapTimeout);
}

// Tests Single tap gesture detection failure, where gesture detection times out because of long
// press.
TEST_F(TwoFingerNTapRecognizerTest, SingleTapGestureTimeout) {
  CreateGestureRecognizer(kDefaultTaps);
  recognizer_->OnContestStarted(member_.TakeInterface());

  SendPointerEvents((DownEvents(1, {}) + DownEvents(2, {})));

  // Wait until the timeout, after which the gesture should abandon.
  RunLoopFor(a11y::TwoFingerNTapRecognizer::kTapTimeout);

  EXPECT_EQ(member_.status(), a11y::ContestMember::Status::kRejected);
}

// Tests Double tap gesture detection failure, where gesture detection times out because second tap
// doesn't start under timeout_between_taps_.
TEST_F(TwoFingerNTapRecognizerTest, DoubleTapGestureTimeoutBetweenTaps) {
  CreateGestureRecognizer(kNumberOfDoubleTaps);
  recognizer_->OnContestStarted(member_.TakeInterface());

  // Send events for first tap.
  SendPointerEvents((DownEvents(1, {}) + DownEvents(2, {}) + UpEvents(1, {}) + UpEvents(2, {})));

  // Wait until the timeout, after which the gesture should abandon.
  RunLoopFor(a11y::TwoFingerNTapRecognizer::kTapTimeout);

  EXPECT_EQ(member_.status(), a11y::ContestMember::Status::kRejected);
}

// Tests Single tap gesture detection failure when multiple fingers are detected.
TEST_F(TwoFingerNTapRecognizerTest, SingleTapThirdFingerDetected) {
  CreateGestureRecognizer(kDefaultTaps);
  recognizer_->OnContestStarted(member_.TakeInterface());

  SendPointerEvents(DownEvents(1, {}) + DownEvents(2, {}));

  // New pointer ID added, but it did not make contact with the screen yet.
  SendPointerEvent({3, Phase::ADD, {}});

  EXPECT_EQ(member_.status(), a11y::ContestMember::Status::kUndecided);

  // Sends a down event with the second pointer ID, causing the gesture to be rejected.
  SendPointerEvent({3, Phase::DOWN, {}});

  EXPECT_EQ(member_.status(), a11y::ContestMember::Status::kRejected);
}

// Tests Double tap gesture detection failure when multiple fingers are detected.
TEST_F(TwoFingerNTapRecognizerTest, DoubleTapThirdFingerDetected) {
  CreateGestureRecognizer(kNumberOfDoubleTaps);
  recognizer_->OnContestStarted(member_.TakeInterface());

  // Send events for first tap.
  SendPointerEvents((DownEvents(1, {}) + DownEvents(2, {}) + UpEvents(1, {}) + UpEvents(2, {})));

  // New pointer ID added, but it did not make contact with the screen yet.
  SendPointerEvent({3, Phase::ADD, {}});

  EXPECT_EQ(member_.status(), a11y::ContestMember::Status::kUndecided);

  // Sends a down event with the second pointer ID, causing the gesture to be rejected.
  SendPointerEvent({3, Phase::DOWN, {}});

  EXPECT_EQ(member_.status(), a11y::ContestMember::Status::kRejected);
}

// Tests Single tap gesture detection when gesture is preformed with move under the allowed limit.
TEST_F(TwoFingerNTapRecognizerTest, SingleTapGestureWithMoveUnderThreshold) {
  CreateGestureRecognizer(kDefaultTaps);
  recognizer_->OnContestStarted(member_.TakeInterface());

  SendPointerEvents(DownEvents(1, {}) + DragEvents(2, {}, {a11y::kGestureMoveThreshold - .1f, 0}) +
                    UpEvents(1, {}));

  EXPECT_FALSE(member_.is_held());
  EXPECT_EQ(member_.status(), a11y::ContestMember::Status::kAccepted);
}

// Tests Single tap gesture detection failure when gesture is performed over a larger area(something
// like swipe).
TEST_F(TwoFingerNTapRecognizerTest, SingleTapGesturePerformedOverLargerArea) {
  CreateGestureRecognizer(kDefaultTaps);
  recognizer_->OnContestStarted(member_.TakeInterface());

  SendPointerEvents(DownEvents(1, {}) + DragEvents(2, {}, {a11y::kGestureMoveThreshold + .1f, 0}));

  EXPECT_EQ(member_.status(), a11y::ContestMember::Status::kRejected);
}

// This test makes sure that local coordinates are passed correctly through the gesture context to
// the callback.
TEST_F(TwoFingerNTapRecognizerTest, RecognizersPassesLocalCoordinatesToCallback) {
  CreateGestureRecognizer(kDefaultTaps);
  recognizer_->OnContestStarted(member_.TakeInterface());

  // Send first finger down event with location specified.
  {
    auto event = ToPointerEvent({1, Phase::ADD, {}}, 0);
    event.set_viewref_koid(100);
    event.set_local_point({1, 1});
    recognizer_->HandleEvent(event);
    event.set_phase(Phase::DOWN);
    recognizer_->HandleEvent(event);
  }

  // Send second finger down event with a different location than the first.
  // The recognizer should pass the location from this event through to the
  // callback.
  {
    auto event = ToPointerEvent({2, Phase::ADD, {}}, 0);
    event.set_viewref_koid(100);
    event.set_local_point({2, 2});
    recognizer_->HandleEvent(event);
    event.set_phase(Phase::DOWN);
    recognizer_->HandleEvent(event);
  }

  // Send up events.
  SendPointerEvents(UpEvents(1, {}) + UpEvents(2, {}));
  recognizer_->OnWin();

  EXPECT_TRUE(gesture_won_);
  EXPECT_EQ(gesture_context_.view_ref_koid, 100u);
  EXPECT_EQ(gesture_context_.local_point->x, 2);
  EXPECT_EQ(gesture_context_.local_point->y, 2);
}

TEST_F(TwoFingerNTapRecognizerTest, LiftAndReplaceSecondFingerIsNotRecognized) {
  CreateGestureRecognizer(kDefaultTaps);
  recognizer_->OnContestStarted(member_.TakeInterface());

  // Send events for holding one finger down, and double-tapping with the
  // other finger.
  SendPointerEvents(DownEvents(1, {}) + DownEvents(2, {}) + UpEvents(2, {}) + DownEvents(2, {}) +
                    UpEvents(1, {}) + UpEvents(2, {}));

  EXPECT_EQ(member_.status(), a11y::ContestMember::Status::kRejected);
}

TEST_F(TwoFingerNTapRecognizerTest, LiftFingerBeforePlacingSecondFingerOnScreen) {
  CreateGestureRecognizer(kDefaultTaps);
  recognizer_->OnContestStarted(member_.TakeInterface());

  // Send one-finger double tap.
  SendPointerEvents(2 * TapEvents(1, {}));

  EXPECT_EQ(member_.status(), a11y::ContestMember::Status::kRejected);
}

}  // namespace

}  // namespace accessibility_test
