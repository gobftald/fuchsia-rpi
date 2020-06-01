// Copyright 2020 The Fuchsia Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:test/test.dart';
import 'package:fxutils/fxutils.dart';
import './mock_process.dart';

void main() {
  group('ProcessLauncher', () {
    test('returns raw specific output', () async {
      final pl = ProcessLauncher(
        processStarter:
            returnGivenProcess(MockProcess(stdout: 'specific output\n')),
      );
      var processResult = await pl.run('whatever', [], outputEncoding: null);
      expect(processResult.stdout, [
        115,
        112,
        101,
        99,
        105,
        102,
        105,
        99,
        32,
        111,
        117,
        116,
        112,
        117,
        116,
        10,
      ]);
    });

    test('returns utf8 specific output', () async {
      final pl = ProcessLauncher(
        processStarter: returnGivenProcess(
          MockProcess(stdout: 'specific\noutput'),
        ),
      );
      // pl.stdout.listen(output.writeln);
      final result = await pl.run('whatever', []);
      expect(result.stdout.toString(), 'specific\noutput');
    });
  });
  group('fx wrapper', () {
    test('successfully returns process output', () async {
      var fx = Fx.mock(MockProcess(stdout: 'specific output\n'));
      expect(await fx.getSubCommandOutput('whatever'), 'specific output');
    });
    test('successfully returns special characters', () async {
      var fx = Fx.mock(MockProcess(stdout: '❌'));
      expect(await fx.getSubCommandOutput('whatever'), '❌');
    });
    test('leaves trailing newlines', () async {
      var fx = Fx.mock(MockProcess(stdout: 'with newline\n'));
      expect(
          await fx.getSubCommandOutput('whatever', shouldTrimTrailing: false),
          'with newline\n');
    });
    test('raises exp on failed command', () {
      var fx = Fx.mock(MockProcess(exitCode: 1));
      expect(() => fx.getDeviceName(),
          throwsA(TypeMatcher<FailedProcessException>()));
    });
  });
}
