// Copyright 2020 The Fuchsia Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "lib/fit/function.h"
#include "src/developer/shell/parser/parse_result.h"

#ifndef SRC_DEVELOPER_SHELL_PARSER_COMBINATORS_H_
#define SRC_DEVELOPER_SHELL_PARSER_COMBINATORS_H_

namespace shell::parser {

// Create a parser that runs a sequence of parsers consecutively.
template <typename... Args>
fit::function<ParseResultStream(ParseResultStream)> Seq(
    fit::function<ParseResultStream(ParseResultStream)> first, Args... args) {
  return Seq(std::move(first), Seq(std::move(args)...));
}

fit::function<ParseResultStream(ParseResultStream)> Seq(
    fit::function<ParseResultStream(ParseResultStream)> a,
    fit::function<ParseResultStream(ParseResultStream)> b);
fit::function<ParseResultStream(ParseResultStream)> Seq(
    fit::function<ParseResultStream(ParseResultStream)> first);

// Given a list of parsers, produce a parser which tries to parse each of them in sequence and
// returns the first successful result.
template <typename... Args>
fit::function<ParseResultStream(ParseResultStream)> Alt(
    fit::function<ParseResultStream(ParseResultStream)> first, Args... args) {
  return Alt(std::move(first), Alt(std::move(args)...));
}

fit::function<ParseResultStream(ParseResultStream)> Alt(
    fit::function<ParseResultStream(ParseResultStream)> a);
fit::function<ParseResultStream(ParseResultStream)> Alt(
    fit::function<ParseResultStream(ParseResultStream)> a,
    fit::function<ParseResultStream(ParseResultStream)> b);

// Parser which always returns success and consumes no output.
ParseResultStream Empty(ParseResultStream prefixes);

// End Of Stream. Parser which only succeeds if there is no more input.
ParseResultStream EOS(ParseResultStream prefixes);

// Produce a parser which runs the given parser, and returns its result, unless it fails in which
// case it returns an empty parse.
inline fit::function<ParseResultStream(ParseResultStream)> Maybe(
    fit::function<ParseResultStream(ParseResultStream)> child) {
  return Alt(std::move(child), Empty);
}

// Produce a parser which tries to parse the input with the given parser. If the given parser fails,
// the produced parser succeeds, and if the given parser succeeds, the produced parser fails. Either
// way the produced parser does not advance the parse position and produces no nodes on success, and
// one error node on failure.
fit::function<ParseResultStream(ParseResultStream)> Not(
    fit::function<ParseResultStream(ParseResultStream)> inv);

// Produce a parser which sequentially repeats a given parser between min and max times.
fit::function<ParseResultStream(ParseResultStream)> Multi(
    size_t min, size_t max, fit::function<ParseResultStream(ParseResultStream)> child);

// Produce a parser which sequentially repeats a given parser exactly count times.
fit::function<ParseResultStream(ParseResultStream)> Multi(
    size_t count, fit::function<ParseResultStream(ParseResultStream)> child);

// Produce a parser which sequentially repeats a given parser zero or more times.
inline fit::function<ParseResultStream(ParseResultStream)> ZeroPlus(
    fit::function<ParseResultStream(ParseResultStream)> child) {
  return Multi(0, std::numeric_limits<size_t>::max(), std::move(child));
}

// Produce a parser which sequentially repeats a given parser one or more times.
inline fit::function<ParseResultStream(ParseResultStream)> OnePlus(
    fit::function<ParseResultStream(ParseResultStream)> child) {
  return Multi(1, std::numeric_limits<size_t>::max(), std::move(child));
}

// Collect the results of the contained parse as a nonterminal and assign a name.
template <typename T>
fit::function<ParseResultStream(ParseResultStream)> NT(
    fit::function<ParseResultStream(ParseResultStream)> a) {
  return [a = std::move(a)](ParseResultStream prefixes) {
    return a(std::move(prefixes).Mark()).Reduce<T>();
  };
}

// Parse a left-associative sequence of non-terminals.
//
// This is best explained by example. Assume the parser "operand" parses A -> 'a' and the parser
// "continuation" parses B -> 'b'. If we built the parser:
//
//     LAssoc<Q>(operand, continuation)
//
// We would expect the following parses:
//
//    "a" -> A
//    "ab" -> Q(A B)
//    "abb" -> Q(Q(A B) B)
//    "abbb" -> Q(Q(Q(A B) B) B)
//
// Essentially we are parsing the rule:
//
//     Q -> Q B / A
//
// But that rule would break our combinator framework due to left recursion, so we instead parse:
//
//     Q -> A B*
//
// but insert some stack cleverness so we get the nonterminal structure we expect.
template <typename T>
fit::function<ParseResultStream(ParseResultStream)> LAssoc(
    fit::function<ParseResultStream(ParseResultStream)> operand,
    fit::function<ParseResultStream(ParseResultStream)> continuation) {
  auto combined =
      Seq(std::move(operand), ZeroPlus(Seq(std::move(continuation), [](ParseResultStream p) {
            return std::move(p).Reduce<T>(false);
          })));
  return [combined = std::move(combined)](ParseResultStream prefixes) {
    return combined(std::move(prefixes).Mark()).DropMarker();
  };
}

}  // namespace shell::parser

#endif  // SRC_DEVELOPER_SHELL_PARSER_COMBINATORS_H_
