// Copyright 2019 The Fuchsia Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:math';

import 'sl4f_client.dart';
import 'ssh.dart';

/// Clockwise rotation of the screen.
enum Rotation {
  degrees0,
  degrees90,
  degrees180,
  degrees270,
}

class Input {
  final Ssh ssh;
  final Rotation _screenRotation;
  final Sl4f _sl4f;

  /// Construct an [Input] object.
  ///
  /// You can change the default [screenRotation] to compensate for.
  Input(Sl4f sl4f, [this._screenRotation = Rotation.degrees0])
      : ssh = sl4f.ssh,
        _sl4f = sl4f;

  /// Taps on the screen at coordinates ([coord.x], [coord.y]).
  ///
  /// Coordinates must be in the range of [0, 1000] and are scaled to the screen
  /// size on the device, and they are rotated to compensate for the clockwise
  /// [screenRotation].
  ///
  /// [tap_event_count]: Number of tap events to send ([duration] is divided
  /// over the tap events). Defaults to 1.
  /// [duration]: Duration of the event(s) in milliseconds. Defaults to 0.
  /// These defaults are set in the input facade.
  Future<bool> tap(Point<int> coord,
      {Rotation screenRotation, int tapEventCount, int duration}) async {
    final tcoord = _rotate(coord, screenRotation ?? _screenRotation);
    final result = await _sl4f.request('input_facade.Tap', {
      'x': tcoord.x,
      'y': tcoord.y,
      if (tapEventCount != null) 'tap_event_count': tapEventCount,
      if (duration != null) 'duration': duration,
    });
    return result == 'Success';
  }

  /// Swipes on the screen from coordinates ([from.x], [from.y]) to ([to.x],
  /// [to.y]).
  ///
  /// Coordinates must be in the range of [0, 1000] are scaled to the screen
  /// size on the device, and they are rotated to compensate for the clockwise
  /// [screenRotation]. How long the swipe lasts can be controlled with
  /// [duration].
  Future<bool> swipe(Point<int> from, Point<int> to,
      {Duration duration = const Duration(milliseconds: 300),
      Rotation screenRotation}) async {
    final tfrom = _rotate(from, screenRotation);
    final tto = _rotate(to, screenRotation);
    final result = await _sl4f.request('input_facade.Swipe', {
      'x0': tfrom.x,
      'y0': tfrom.y,
      'x1': tto.x,
      'y1': tto.y,
      'duration': duration.inMilliseconds,
    });
    return result == 'Success';
  }

  /// Compensates for the given [screenRotation].
  ///
  /// If null is provided, the default specified in the constructor is used.
  Point<int> _rotate(Point<int> coord, Rotation screenRotation) {
    final rotation = screenRotation ?? _screenRotation;
    switch (rotation) {
      case Rotation.degrees0:
        return coord;
      case Rotation.degrees90:
        return Point<int>(1000 - coord.y, coord.x);
      case Rotation.degrees180:
        return Point<int>(1000 - coord.x, 1000 - coord.y);
      case Rotation.degrees270:
        return Point<int>(coord.y, 1000 - coord.x);
    }
    return coord;
  }
}
