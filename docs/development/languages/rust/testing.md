# Testing Rust code

This document describes best practices for writing tests for Rust code, and
paired with ["Running tests as components"][component_tests] describes how to
component-ize, package, and run these tests.

This document is targeted towards developers working inside of `fuchsia.git`,
and the workflow described is unlikely to work for IDK consumers.

The source code for this tutorial is available at
[`//examples/hello_world/rust`][example-src].

## Unit tests

### Adding tests to code

The idiomatic way for adding Rust unit tests works just as well inside of
Fuchsia as it does outside, and can be easily accomplished by dropping the
following snippet into the bottom of whatever test you want to write:

```rust
{% includecode gerrit_repo="fuchsia/fuchsia" gerrit_path="examples/hello_world/rust/src/main.rs" region_tag="test_mod" adjust_indentation="auto" %}
```

This will cause a new mod named `tests` to be created, and this mod will only be
included when building unit tests. Any functions annotated with `#[test]` will
be run as a test, and if the function successfully returns then the test passes.

For tests exercising asynchronous code, use the
`#[fasync::run_until_stalled(test)]` annotation as an alternative to
using an asynchronous executor.

```rust
{% includecode gerrit_repo="fuchsia/fuchsia" gerrit_path="examples/hello_world/rust/src/main.rs" region_tag="async_test" adjust_indentation="auto" %}
```

### Building tests

The unit tests can be automatically built by Rust targets (i.e. either
`rustc_binary` or `rustc_library`).  The approaches are by and large similar.

#### Building tests for a Rust binary

This section is useful if you are testing a rust *binary* (i.e. you have a
`main.rs`). If you have a library instead, see the next section.

Your `BUILD.gn` file first needs to make available the `rustc_binary` template
by importing it:

```gn
import("//build/rust/rustc_binary.gni")
```

Unit tests are built by the `rustc_binary` GN template only if the setting
`with_unit_tests = true` is added:

```gn
{% includecode gerrit_repo="fuchsia/fuchsia" gerrit_path="examples/hello_world/rust/BUILD.gn" region_tag="rustc_tests" adjust_indentation="auto" %}
```

Setting `with_unit_tests = true` causes this build rule to generate two
different executables, one with the provided and one with `_bin_test` appended
to the provided name.

In our example here, the executable names that are created are called:

* `hello_world_rust`; and
* `hello_world_rust_bin_test`.

#### Building tests for a Rust library

Your `BUILD.gn` file first needs to make available the `rustc_library` template
by importing it:

```gn
import("//build/rust/rustc_library.gni")
```

Unit tests are built by the `rustc_library` GN template only if the setting
`with_unit_tests = true` is added, similarly to how it is done in the case
of `rustc_binary` above.

In this case, however, a **differently named** test binary is created:

* `hello_world_rust_lib_test`.  Note that the name of the binary is different
  from the name generated by the library.

The binary names are important because they will be used in followup steps.

### Packaging and running tests

To run the tests that were generated by previous targets, they will need to be
packaged first.  This is currently a two step process, which includes writing
a package manifest and the package build target.

#### Writing the package manifest

A package manifest is currently required.  The manifest is placed in the
`meta/` subdirectory, immediately below the directory containing your `BUILD.gn`
file.

A minimal manifest file is shown below, and **must** be named the same as the
`name` attribute of the target that is being generated by the `rustc_binary` or
`rustc_library` above.

In case of a manifest for a `rustc_binary` given above, the resulting manifest
is:

```cmx
{
        "program": {
               "binary": "test/hello_world_rust_bin_test"
        }
}
```

Note:

* The binary name is based on the `name = "hello_world_rust"` line on the
  `rustc_binary` target.
* The binary is inside a `test/` subdirectory.  This placement is implicit in the
  `rustc_binary` build rule.
* You may have noticed that the package manifests are somewhat formulaic.
  In the future, we may find ourselves able to automatically generate the
  package manifests instead of having to write them out by hand.

In case of a manifest for a `rustc_library`, the manifest and the naming scheme
are similar.  But pay attention to the subtle naming difference in the value
for the stanza `program.binary`:

```cmx
{
        "program": {
               "binary": "test/hello_world_rust_lib_test"
        }
}
```

The `_lib_test` suffix is hard-coded in the `rustc_library` build rule, and
`hello_world_rust` again comes from the `name` attribute in the build rule.

#### Writing the package build target

For the Hello world binary example, the test package needs to reference the
generated targets, `bin_test` (based on target name `bin` and the implicit
suffix `_test`), and `hello_world_rust_bin_test` (based on the value of `name`
stanza).

If you are building a library instead,
then the library name will be `hello_world_rust_lib_test`.

```gn
{% includecode gerrit_repo="fuchsia/fuchsia" gerrit_path="examples/hello_world/rust/BUILD.gn" indented_block="^test_package\(\"hello_world_rust_tests\"\) {" %}
```

To run the tests run:

```sh
fx test hello_world_rust_tests
```

Note: that in order to use `fx test`, you can't override
`package_name="..."` in your `package`  or `test_package` declaration. This
issue is tracked by BLD-338.


For information on packaging and running tests, please refer to the
[documentation on running tests as components][component_tests].

### Helpful crates

The following in-tree third-party crates can help you write tests:

* [`matches`]: provides the macro `assert_matches!`, making pattern assertions ergonomic.
* [`pretty_assertions`]: provides an alternative `assert_eq!` macro that displays a colored diff
  when the assertion fails.

These can be included in your `BUILD.gn` under `test_deps`.

```gn
rustc_binary("bin") {
  name = "my_test"
  with_unit_tests = true
  edition = "2018"

  test_deps = [
    "//third_party/rust_crates:matches",
    "//third_party/rust_crates:pretty_assertions",
  ]
}
```

[component_tests]:/docs/development/testing/running_tests_as_components.md
[example-src]: /examples/hello_world/rust
[`matches`]: https://fuchsia-docs.firebaseapp.com/rust/matches/index.html
[`pretty_assertions`]: https://fuchsia-docs.firebaseapp.com/rust/pretty_assertions/index.html
