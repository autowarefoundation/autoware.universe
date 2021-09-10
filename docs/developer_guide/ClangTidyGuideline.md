# Clang-Tidy Guidelines

Weekly Clang-Tidy analysis report for `main` branch is [here](https://tier4.github.io/autoware-iv-metrics-dashboard/tidy/) (updated every Sunday).

**Information:** Due to the lack of the header files of CUDA or NVML, Perception modules and NVML GPU Monitor can not be analyzed correctly by clang-tidy-pr in autoware.iv repository. You may find some clang-errors and false positives.

This document follows the convention of [RFC2119](https://datatracker.ietf.org/doc/html/rfc2119).

---

## Severity Level

The severity level of check rules are defined by CodeChecker, not by Clang-Tidy itself. The level definitions of rules can be found [here](https://github.com/Ericsson/codechecker/blob/a621d3978fa7441c8b8bc517ff2bac29d3df7fa7/config/labels/analyzers/clang-tidy.json).

### High

We MUST fix the code problems pointed out by Clang-Tidy.

### Middle

We SHOULD fix the code problems pointed out by Clang-Tidy.

### Low

We MAY fix the code problems pointed out by Clang-Tidy. But some rules SHOULD NOT be ignored. See Rules section.

### Style

We MAY fix the code problems pointed out by Clang-Tidy. But some rules SHOULD NOT be ignored. See Rules section.

---

## Rules

Some rules are disabled by default to prevent false-positives that are annoying.

### `clang-diagnostic-*`

We MUST fix the code problems detected by clang-diagnostic-\* rules.

These rules come from Clang++ compile-time errors and warnings.

### `boost-*`

We MUST fix the code problems detected by boost-\_ rules if these are not false-positives.

#### boost-use-to-string [LOW] [(doc)](https://clang.llvm.org/extra/clang-tidy/checks/boost-use-to-string.html)

### `bugprone-*`

We SHOULD fix the code problems detected by bugprone-\* rules if these are not false-positives.

#### bugprone-argument-comment [LOW] [(doc)](https://clang.llvm.org/extra/clang-tidy/checks/bugprone-argument-comment.html)

#### bugprone-assert-side-effect [MEDIUM] [(doc)](https://clang.llvm.org/extra/clang-tidy/checks/bugprone-assert-side-effect.html)

#### bugprone-bad-signal-to-kill-thread [MEDIUM] [(doc)](https://clang.llvm.org/extra/clang-tidy/checks/bugprone-bad-signal-to-kill-thread.html)

#### bugprone-bool-pointer-implicit-conversion [LOW] [(doc)](https://clang.llvm.org/extra/clang-tidy/checks/bugprone-bool-pointer-implicit-conversion.html)

#### bugprone-branch-clone [LOW] [(doc)](https://clang.llvm.org/extra/clang-tidy/checks/bugprone-branch-clone.html)

Sometimes the clones are intentional as follows. We MAY ignore safely.
(Note that the following code violates the modernize-make-unique rule)

```cpp
} else if (type == autoware_perception_msgs::msg::Semantic::MOTORBIKE) {
model_ptr.reset(new normal::BoundingBoxModel);
} else if (type == autoware_perception_msgs::msg::Semantic::BICYCLE) {
model_ptr.reset(new normal::BoundingBoxModel);
}
```

#### bugprone-copy-constructor-init [MEDIUM] [(doc)](https://clang.llvm.org/extra/clang-tidy/checks/bugprone-copy-constructor-init.html)

#### bugprone-dangling-handle [HIGH] [(doc)](https://clang.llvm.org/extra/clang-tidy/checks/bugprone-dangling-handle.html)

#### bugprone-dynamic-static-initializers [MEDIUM] [(doc)](https://clang.llvm.org/extra/clang-tidy/checks/bugprone-dynamic-static-initializers.html)

#### bugprone-exception-escape [MEDIUM] [(doc)](https://clang.llvm.org/extra/clang-tidy/checks/bugprone-exception-escape.html)

#### bugprone-fold-init-type [HIGH] [(doc)](https://clang.llvm.org/extra/clang-tidy/checks/bugprone-fold-init-type.html)

#### bugprone-forward-declaration-namespace [LOW] [(doc)](https://clang.llvm.org/extra/clang-tidy/checks/bugprone-forward-declaration-namespace.html)

#### bugprone-forwarding-reference-overload [LOW] [(doc)](https://clang.llvm.org/extra/clang-tidy/checks/bugprone-forwarding-reference-overload.html)

#### bugprone-inaccurate-erase [HIGH] [(doc)](https://clang.llvm.org/extra/clang-tidy/checks/bugprone-inaccurate-erase.html)

#### bugprone-incorrect-roundings [HIGH] [(doc)](https://clang.llvm.org/extra/clang-tidy/checks/bugprone-incorrect-roundings.html)<!--- cspell:disable-line -->

#### (bugprone-infinite-loop) [MEDIUM] [(doc)](https://clang.llvm.org/extra/clang-tidy/checks/bugprone-infinite-loop.html)

Disabled because the rule falls into the infinite-loop when checking some ROS functions, such as `spin()`.

#### bugprone-integer-division [MEDIUM] [(doc)](https://clang.llvm.org/extra/clang-tidy/checks/bugprone-integer-division.html)

#### bugprone-lambda-function-name [LOW] [(doc)](https://clang.llvm.org/extra/clang-tidy/checks/bugprone-lambda-function-name.html)

#### bugprone-macro-parentheses [MEDIUM] [(doc)](https://clang.llvm.org/extra/clang-tidy/checks/bugprone-macro-parentheses.html)

#### bugprone-macro-repeated-side-effects [MEDIUM] [(doc)](https://clang.llvm.org/extra/clang-tidy/checks/bugprone-macro-repeated-side-effects.html)

#### bugprone-misplaced-operator-in-strlen-in-alloc [MEDIUM] [(doc)](https://clang.llvm.org/extra/clang-tidy/checks/bugprone-misplaced-operator-in-strlen-in-alloc.html)<!--- cspell:disable-line -->

#### (bugprone-misplaced-pointer-arithmetic-in-alloc) [MEDIUM] [(doc)](https://clang.llvm.org/extra/clang-tidy/checks/bugprone-misplaced-pointer-arithmetic-in-alloc.html)<!--- cspell:disable-line -->

Disabled.

#### bugprone-misplaced-widening-cast [HIGH] [(doc)](https://clang.llvm.org/extra/clang-tidy/checks/bugprone-misplaced-widening-cast.html)

#### bugprone-move-forwarding-reference [MEDIUM] [(doc)](https://clang.llvm.org/extra/clang-tidy/checks/bugprone-move-forwarding-reference.html)

#### bugprone-multiple-statement-macro [MEDIUM] [(doc)](https://clang.llvm.org/extra/clang-tidy/checks/bugprone-multiple-statement-macro.html)

#### (bugprone-no-escape) [undefined] [(doc)](https://clang.llvm.org/extra/clang-tidy/checks/bugprone-no-escape.html)

To be added to .clang-tidy.

#### bugprone-not-null-terminated-result [MEDIUM] [(doc)](https://clang.llvm.org/extra/clang-tidy/checks/bugprone-not-null-terminated-result.html)

#### bugprone-parent-virtual-call [MEDIUM] [(doc)](https://clang.llvm.org/extra/clang-tidy/checks/bugprone-parent-virtual-call.html)

#### bugprone-posix-return [undefined] [(doc)](https://clang.llvm.org/extra/clang-tidy/checks/bugprone-posix-return.html)

#### (bugprone-redundant-branch-condition) [LOW] [(doc)](https://clang.llvm.org/extra/clang-tidy/checks/bugprone-redundant-branch-condition.html)

Disabled.

#### (bugprone-reserved-identifier) [LOW] [(doc)](https://clang.llvm.org/extra/clang-tidy/checks/bugprone-reserved-identifier.html)

Disabled because ROS 2 coding naming style for inclusion guards violates the rule by default.

#### (bugprone-signal-handler) [undefined] [(doc)](https://clang.llvm.org/extra/clang-tidy/checks/bugprone-signal-handler.html)

To be added to .clang-tidy.

#### bugprone-signed-char-misuse [MEDIUM] [(doc)](https://clang.llvm.org/extra/clang-tidy/checks/bugprone-signed-char-misuse.html)

#### bugprone-sizeof-container [HIGH] [(doc)](https://clang.llvm.org/extra/clang-tidy/checks/bugprone-sizeof-container.html)

#### bugprone-sizeof-expression [HIGH] [(doc)](https://clang.llvm.org/extra/clang-tidy/checks/bugprone-sizeof-expression.html)

#### (bugprone-spuriously-wake-up-functions) [undefined] [(doc)](https://clang.llvm.org/extra/clang-tidy/checks/bugprone-spuriously-wake-up-functions.html)

#### bugprone-string-constructor [HIGH] [(doc)](https://clang.llvm.org/extra/clang-tidy/checks/bugprone-string-constructor.html)

#### bugprone-string-integer-assignment [LOW] [(doc)](https://clang.llvm.org/extra/clang-tidy/checks/bugprone-string-integer-assignment.html)

#### bugprone-string-literal-with-embedded-nul [MEDIUM] [(doc)](https://clang.llvm.org/extra/clang-tidy/checks/bugprone-string-literal-with-embedded-nul.html)

#### bugprone-suspicious-enum-usage [HIGH] [(doc)](https://clang.llvm.org/extra/clang-tidy/checks/bugprone-suspicious-enum-usage.html)

#### (bugprone-suspicious-include) [LOW] [(doc)](https://clang.llvm.org/extra/clang-tidy/checks/bugprone-suspicious-include.html)

Disabled.

#### bugprone-suspicious-memset-usage [HIGH] [(doc)](https://clang.llvm.org/extra/clang-tidy/checks/bugprone-suspicious-memset-usage.html)

#### bugprone-suspicious-missing-comma [HIGH] [(doc)](https://clang.llvm.org/extra/clang-tidy/checks/bugprone-suspicious-missing-comma.html)

#### bugprone-suspicious-semicolon [HIGH] [(doc)](https://clang.llvm.org/extra/clang-tidy/checks/bugprone-suspicious-semicolon.html)

#### bugprone-suspicious-string-compare [MEDIUM] [(doc)](https://clang.llvm.org/extra/clang-tidy/checks/bugprone-suspicious-string-compare.html)

#### bugprone-swapped-arguments [HIGH] [(doc)](https://clang.llvm.org/extra/clang-tidy/checks/bugprone-swapped-arguments.html)

#### bugprone-terminating-continue [MEDIUM] [(doc)](https://clang.llvm.org/extra/clang-tidy/checks/bugprone-terminating-continue.html)

#### bugprone-throw-keyword-missing [MEDIUM] [(doc)](https://clang.llvm.org/extra/clang-tidy/checks/bugprone-throw-keyword-missing.html)

#### bugprone-too-small-loop-variable [MEDIUM] [(doc)](https://clang.llvm.org/extra/clang-tidy/checks/bugprone-too-small-loop-variable.html)

#### bugprone-undefined-memory-manipulation [MEDIUM] [(doc)](https://clang.llvm.org/extra/clang-tidy/checks/bugprone-undefined-memory-manipulation.html)

#### bugprone-undelegated-constructor [MEDIUM] [(doc)](https://clang.llvm.org/extra/clang-tidy/checks/bugprone-undelegated-constructor.html)

#### (bugprone-unhandled-exception-at-new) [undefined] [(doc)](https://clang.llvm.org/extra/clang-tidy/checks/bugprone-unhandled-exception-at-new.html)

To be added.

#### bugprone-unhandled-self-assignment [MEDIUM] [(doc)](https://clang.llvm.org/extra/clang-tidy/checks/bugprone-unhandled-self-assignment.html)

#### bugprone-unused-raii [HIGH] [(doc)](https://clang.llvm.org/extra/clang-tidy/checks/bugprone-unused-raii.html)

#### bugprone-unused-return-value [MEDIUM] [(doc)](https://clang.llvm.org/extra/clang-tidy/checks/bugprone-unused-return-value.html)

#### bugprone-use-after-move [HIGH] [(doc)](https://clang.llvm.org/extra/clang-tidy/checks/bugprone-use-after-move.html)

#### bugprone-virtual-near-miss [MEDIUM] [(doc)](https://clang.llvm.org/extra/clang-tidy/checks/bugprone-virtual-near-miss.html)

### `cppcoreguidelines-*`

We SHOULD fix the code problems detected by `cppcoreguidelines-*` rules if these are not false-positives.

#### cppcoreguidelines-avoid-goto [STYLE] [(doc)](https://clang.llvm.org/extra/clang-tidy/checks/cppcoreguidelines-avoid-goto.html)

#### (cppcoreguidelines-avoid-non-const-global-variables) [LOW] [(doc)](https://clang.llvm.org/extra/clang-tidy/checks/cppcoreguidelines-avoid-non-const-global-variables.html)

Disabled.

#### cppcoreguidelines-init-variables [MEDIUM] [(doc)](https://clang.llvm.org/extra/clang-tidy/checks/cppcoreguidelines-init-variables.html)

#### cppcoreguidelines-interfaces-global-init [LOW] [(doc)](https://clang.llvm.org/extra/clang-tidy/checks/cppcoreguidelines-interfaces-global-init.html)

#### cppcoreguidelines-macro-usage [LOW] [(doc)](https://clang.llvm.org/extra/clang-tidy/checks/cppcoreguidelines-macro-usage.html)

#### cppcoreguidelines-narrowing-conversions [MEDIUM] [(doc)](https://clang.llvm.org/extra/clang-tidy/checks/cppcoreguidelines-narrowing-conversions.html)

#### cppcoreguidelines-no-malloc [LOW] [(doc)](https://clang.llvm.org/extra/clang-tidy/checks/cppcoreguidelines-no-malloc.html)

#### (cppcoreguidelines-owning-memory) [STYLE] [(doc)](https://clang.llvm.org/extra/clang-tidy/checks/cppcoreguidelines-owning-memory.html)

Disabled.

#### cppcoreguidelines-prefer-member-initializer [STYLE] [(doc)](https://clang.llvm.org/extra/clang-tidy/checks/cppcoreguidelines-prefer-member-initializer.html)

Disabled.

#### (cppcoreguidelines-pro-bounds-array-to-pointer-decay) [LOW] [(doc)](https://clang.llvm.org/extra/clang-tidy/checks/cppcoreguidelines-pro-bounds-array-to-pointer-decay.html)

Disabled.

#### (cppcoreguidelines-pro-bounds-constant-array-index) [LOW] [(doc)](https://clang.llvm.org/extra/clang-tidy/checks/cppcoreguidelines-pro-bounds-constant-array-index.html)

Disabled.

#### cppcoreguidelines-pro-bounds-pointer-arithmetic [LOW] [(doc)](https://clang.llvm.org/extra/clang-tidy/checks/cppcoreguidelines-pro-bounds-pointer-arithmetic.html)

#### cppcoreguidelines-pro-type-const-cast [LOW] [(doc)](https://clang.llvm.org/extra/clang-tidy/checks/cppcoreguidelines-pro-type-const-cast.html)

#### cppcoreguidelines-pro-type-cstyle-cast [LOW] [(doc)](https://clang.llvm.org/extra/clang-tidy/checks/cppcoreguidelines-pro-type-cstyle-cast.html)<!--- cspell:disable-line -->

#### cppcoreguidelines-pro-type-member-init [LOW] [(doc)](https://clang.llvm.org/extra/clang-tidy/checks/cppcoreguidelines-pro-type-member-init.html)

#### cppcoreguidelines-pro-type-reinterpret-cast [LOW] [(doc)](https://clang.llvm.org/extra/clang-tidy/checks/cppcoreguidelines-pro-type-reinterpret-cast.html)

#### cppcoreguidelines-pro-type-static-cast-downcast [LOW] [(doc)](https://clang.llvm.org/extra/clang-tidy/checks/cppcoreguidelines-pro-type-static-cast-downcast.html)

#### cppcoreguidelines-pro-type-union-access [LOW] [(doc)](https://clang.llvm.org/extra/clang-tidy/checks/cppcoreguidelines-pro-type-union-access.html)

#### (cppcoreguidelines-pro-type-vararg) [LOW] [(doc)](https://clang.llvm.org/extra/clang-tidy/checks/cppcoreguidelines-pro-type-vararg.html)<!--- cspell:disable-line -->

Disabled.

#### cppcoreguidelines-slicing [LOW] [(doc)](https://clang.llvm.org/extra/clang-tidy/checks/cppcoreguidelines-slicing.html)

#### cppcoreguidelines-special-member-functions [LOW] [(doc)](https://clang.llvm.org/extra/clang-tidy/checks/cppcoreguidelines-special-member-functions.html)

### `google-*`

We SHOULD fix the code problems detected by `google-*` rules if these are not false-positives.

#### google-build-explicit-make-pair [MEDIUM] [(doc)](https://clang.llvm.org/extra/clang-tidy/checks/google-build-explicit-make-pair.html)

#### google-build-namespaces [MEDIUM] [(doc)](https://clang.llvm.org/extra/clang-tidy/checks/google-build-namespaces.html)

#### google-build-using-namespace [STYLE] [(doc)](https://clang.llvm.org/extra/clang-tidy/checks/google-build-using-namespace.html)

#### (google-default-arguments) [LOW] [(doc)](https://clang.llvm.org/extra/clang-tidy/checks/google-default-arguments.html)

Disabled.

#### google-explicit-constructor [MEDIUM] [(doc)](https://clang.llvm.org/extra/clang-tidy/checks/google-explicit-constructor.html)

#### google-global-names-in-headers [STYLE] [(doc)](https://clang.llvm.org/extra/clang-tidy/checks/google-global-names-in-headers.html)

#### (google-objc-avoid-nsobject-new) [undefined] [(doc)](https://clang.llvm.org/extra/clang-tidy/checks/google-objc-avoid-nsobject-new.html)<!--- cspell:disable-line -->

Disabled.

#### (google-objc-avoid-throwing-exception) [undefined] [(doc)](https://clang.llvm.org/extra/clang-tidy/checks/google-objc-avoid-throwing-exception.html)<!--- cspell:disable-line -->

Disabled.

#### (google-objc-function-naming) [undefined] [(doc)](https://clang.llvm.org/extra/clang-tidy/checks/google-objc-function-naming.html)<!--- cspell:disable-line -->

Disabled.

#### (google-objc-global-variable-declaration) [undefined] [(doc)](https://clang.llvm.org/extra/clang-tidy/checks/google-objc-global-variable-declaration.html)<!--- cspell:disable-line -->

Disabled.

#### (google-readability-avoid-underscore-in-googletest-name) [STYLE] [(doc)](https://clang.llvm.org/extra/clang-tidy/checks/google-readability-avoid-underscore-in-googletest-name.html)<!--- cspell:disable-line -->

#### (google-readability-casting) [LOW] [(doc)](https://clang.llvm.org/extra/clang-tidy/checks/google-readability-casting.html)

Disabled.

#### (google-readability-todo) [STYLE] [(doc)](https://clang.llvm.org/extra/clang-tidy/checks/google-readability-todo.html)

Disabled.

#### (google-runtime-int) [LOW] [(doc)](https://clang.llvm.org/extra/clang-tidy/checks/google-runtime-int.html)

Disabled.

#### (google-runtime-operator) [MEDIUM] [(doc)](https://clang.llvm.org/extra/clang-tidy/checks/google-runtime-operator.html)

Disabled.

#### google-upgrade-googletest-case [STYLE] [(doc)](https://clang.llvm.org/extra/clang-tidy/checks/google-upgrade-googletest-case.html)<!--- cspell:disable-line -->

### `hicpp-*`<!--- cspell:disable-line -->

We SHOULD fix the code problems detected by `hicpp-*` rules if these are not false-positives.<!--- cspell:disable-line -->

#### hicpp-exception-baseclass [LOW] [(doc)](https://clang.llvm.org/extra/clang-tidy/checks/hicpp-exception-baseclass.html)<!--- cspell:disable-line -->

#### hicpp-multiway-paths-covered [STYLE] [(doc)](https://clang.llvm.org/extra/clang-tidy/checks/hicpp-multiway-paths-covered.html)<!--- cspell:disable-line -->

#### hicpp-no-assembler [LOW] [(doc)](https://clang.llvm.org/extra/clang-tidy/checks/hicpp-no-assembler.html)<!--- cspell:disable-line -->

#### hicpp-signed-bitwise [LOW] [(doc)](https://clang.llvm.org/extra/clang-tidy/checks/hicpp-signed-bitwise.html)<!--- cspell:disable-line -->

### `llvm-*`

We MUST fix the code problems detected by `llvm-*` rules if these are not false-positives.

#### llvm-namespace-comment [LOW] [(doc)](https://clang.llvm.org/extra/clang-tidy/checks/llvm-namespace-comment.html)

### `misc-*`

We SHOULD fix the code problems detected by `misc-*` rules if these are not false-positives.

#### misc-definitions-in-headers [MEDIUM] [(doc)](https://clang.llvm.org/extra/clang-tidy/checks/misc-definitions-in-headers.html)

#### misc-misplaced-const [LOW] [(doc)](https://clang.llvm.org/extra/clang-tidy/checks/misc-misplaced-const.html)

#### misc-new-delete-overloads [MEDIUM] [(doc)](https://clang.llvm.org/extra/clang-tidy/checks/misc-new-delete-overloads.html)

#### misc-no-recursion [LOW] [(doc)](https://clang.llvm.org/extra/clang-tidy/checks/misc-no-recursion.html)

#### misc-non-copyable-objects [HIGH] [(doc)](https://clang.llvm.org/extra/clang-tidy/checks/misc-non-copyable-objects.html)

#### (misc-non-private-member-variables-in-classes) [LOW] [(doc)](https://clang.llvm.org/extra/clang-tidy/checks/misc-non-private-member-variables-in-classes.html)

Disabled because of annoyance.

#### misc-redundant-expression [MEDIUM] [(doc)](https://clang.llvm.org/extra/clang-tidy/checks/misc-redundant-expression.html)

MUST.

#### misc-static-assert [LOW] [(doc)](https://clang.llvm.org/extra/clang-tidy/checks/misc-static-assert.html)

#### misc-throw-by-value-catch-by-reference [HIGH] [(doc)](https://clang.llvm.org/extra/clang-tidy/checks/misc-throw-by-value-catch-by-reference.html)

MUST.

#### misc-unconventional-assign-operator [MEDIUM] [(doc)](https://clang.llvm.org/extra/clang-tidy/checks/misc-unconventional-assign-operator.html)

#### misc-uniqueptr-reset-release [MEDIUM] [(doc)](https://clang.llvm.org/extra/clang-tidy/checks/misc-uniqueptr-reset-release.html)<!--- cspell:disable-line -->

#### misc-unused-alias-decls [LOW] [(doc)](https://clang.llvm.org/extra/clang-tidy/checks/misc-unused-alias-decls.html)

MUST.

#### misc-unused-parameters [LOW] [(doc)](https://clang.llvm.org/extra/clang-tidy/checks/misc-unused-parameters.html)

MUST.

#### misc-unused-using-decls [LOW] [(doc)](https://clang.llvm.org/extra/clang-tidy/checks/misc-unused-using-decls.html)

MUST.

### `modernize-*`

We SHOULD fix the code problems detected by `modernize-*` rules if these are not false-positives.

#### (modernize-avoid-bind) [STYLE] [(doc)](https://clang.llvm.org/extra/clang-tidy/checks/modernize-avoid-bind.html)

Disabled because of incompatibility with ROS.

#### modernize-avoid-c-arrays [LOW] [(doc)](https://clang.llvm.org/extra/clang-tidy/checks/modernize-avoid-c-arrays.html)

#### modernize-concat-nested-namespaces [STYLE] [(doc)](https://clang.llvm.org/extra/clang-tidy/checks/modernize-concat-nested-namespaces.html)

#### modernize-deprecated-headers [LOW] [(doc)](https://clang.llvm.org/extra/clang-tidy/checks/modernize-deprecated-headers.html)

MUST.

#### modernize-deprecated-ios-base-aliases [LOW] [(doc)](https://clang.llvm.org/extra/clang-tidy/checks/modernize-deprecated-ios-base-aliases.html)

#### modernize-loop-convert [STYLE] [(doc)](https://clang.llvm.org/extra/clang-tidy/checks/modernize-loop-convert.html)

#### modernize-make-shared [LOW] [(doc)](https://clang.llvm.org/extra/clang-tidy/checks/modernize-make-shared.html)

#### modernize-make-unique [LOW] [(doc)](https://clang.llvm.org/extra/clang-tidy/checks/modernize-make-unique.html)

#### modernize-pass-by-value [LOW] [(doc)](https://clang.llvm.org/extra/clang-tidy/checks/modernize-pass-by-value.html)

#### modernize-raw-string-literal [STYLE] [(doc)](https://clang.llvm.org/extra/clang-tidy/checks/modernize-raw-string-literal.html)

#### modernize-redundant-void-arg [STYLE] [(doc)](https://clang.llvm.org/extra/clang-tidy/checks/modernize-redundant-void-arg.html)

MUST.

#### modernize-replace-auto-ptr [LOW] [(doc)](https://clang.llvm.org/extra/clang-tidy/checks/modernize-replace-auto-ptr.html)

#### modernize-replace-disallow-copy-and-assign-macro [LOW] [(doc)](https://clang.llvm.org/extra/clang-tidy/checks/modernize-replace-disallow-copy-and-assign-macro.html#modernize-replace-disallow-copy-and-assign-macro)

#### modernize-replace-random-shuffle [LOW] [(doc)](https://clang.llvm.org/extra/clang-tidy/checks/modernize-replace-random-shuffle.html)

#### modernize-return-braced-init-list [STYLE] [(doc)](https://clang.llvm.org/extra/clang-tidy/checks/modernize-return-braced-init-list.html)

#### modernize-shrink-to-fit [STYLE] [(doc)](https://clang.llvm.org/extra/clang-tidy/checks/modernize-shrink-to-fit.html)

#### modernize-unary-static-assert [STYLE] [(doc)](https://clang.llvm.org/extra/clang-tidy/checks/modernize-unary-static-assert.html)

#### modernize-use-auto [STYLE] [(doc)](https://clang.llvm.org/extra/clang-tidy/checks/modernize-use-auto.html)

#### modernize-use-bool-literals [STYLE] [(doc)](https://clang.llvm.org/extra/clang-tidy/checks/modernize-use-bool-literals.html)

MUST.

#### modernize-use-default-member-init [STYLE] [(doc)](https://clang.llvm.org/extra/clang-tidy/checks/modernize-use-default-member-init.html)

#### modernize-use-emplace [STYLE] [(doc)](https://clang.llvm.org/extra/clang-tidy/checks/modernize-use-emplace.html)

MUST.

#### modernize-use-equals-default [STYLE] [(doc)](https://clang.llvm.org/extra/clang-tidy/checks/modernize-use-equals-default.html)

MUST.

#### modernize-use-equals-delete [STYLE] [(doc)](https://clang.llvm.org/extra/clang-tidy/checks/modernize-use-equals-delete.html)

MUST.

#### modernize-use-nodiscard [LOW] [(doc)](https://clang.llvm.org/extra/clang-tidy/checks/modernize-use-nodiscard.html)

We SHOULD follow the rule if we use C++17.

#### modernize-use-noexcept [LOW] [(doc)](https://clang.llvm.org/extra/clang-tidy/checks/modernize-use-noexcept.html)

#### modernize-use-nullptr [LOW] [(doc)](https://clang.llvm.org/extra/clang-tidy/checks/modernize-use-nullptr.html)

MUST.

#### modernize-use-override [LOW] [(doc)](https://clang.llvm.org/extra/clang-tidy/checks/modernize-use-override.html)

MUST.

#### (modernize-use-trailing-return-type) [LOW] [(doc)](https://clang.llvm.org/extra/clang-tidy/checks/modernize-use-trailing-return-type.html)

Disabled.

#### modernize-use-transparent-functors [LOW] [(doc)](https://clang.llvm.org/extra/clang-tidy/checks/modernize-use-transparent-functors.html)

#### modernize-use-uncaught-exceptions [STYLE] [(doc)](https://clang.llvm.org/extra/clang-tidy/checks/modernize-use-uncaught-exceptions.html)

#### modernize-use-using [STYLE] [(doc)](https://clang.llvm.org/extra/clang-tidy/checks/modernize-use-using.html)

MUST.

### `openmp-*`

#### openmp-use-default-none [undefined] [(doc)](https://clang.llvm.org/extra/clang-tidy/checks/openmp-use-default-none.html)

### `performance-*`

We SHOULD fix the code problems detected by `performance-*` rules if these are not false-positives.

#### performance-faster-string-find [LOW] [(doc)](https://clang.llvm.org/extra/clang-tidy/checks/performance-faster-string-find.html)

MUST.

#### performance-for-range-copy [LOW] [(doc)](https://clang.llvm.org/extra/clang-tidy/checks/performance-for-range-copy.html)

#### performance-implicit-conversion-in-loop [LOW] [(doc)](https://clang.llvm.org/extra/clang-tidy/checks/performance-implicit-conversion-in-loop.html)

#### performance-inefficient-algorithm [MEDIUM] [(doc)](https://clang.llvm.org/extra/clang-tidy/checks/performance-inefficient-algorithm.html)

MUST.

#### performance-inefficient-string-concatenation [LOW] [(doc)](https://clang.llvm.org/extra/clang-tidy/checks/performance-inefficient-string-concatenation.html)

#### performance-inefficient-vector-operation [LOW] [(doc)](https://clang.llvm.org/extra/clang-tidy/checks/performance-inefficient-vector-operation.html)

#### performance-move-const-arg [MEDIUM] [(doc)](https://clang.llvm.org/extra/clang-tidy/checks/performance-move-const-arg.html)

#### performance-move-constructor-init [MEDIUM] [(doc)](https://clang.llvm.org/extra/clang-tidy/checks/performance-move-constructor-init.html)

#### performance-no-automatic-move [LOW] [(doc)](https://clang.llvm.org/extra/clang-tidy/checks/performance-no-automatic-move.html)

#### performance-no-int-to-ptr [LOW] [(doc)](https://clang.llvm.org/extra/clang-tidy/checks/performance-no-int-to-ptr.html)

#### performance-noexcept-move-constructor [MEDIUM] [(doc)](https://clang.llvm.org/extra/clang-tidy/checks/performance-noexcept-move-constructor.html)

#### performance-trivially-destructible [LOW] [(doc)](https://clang.llvm.org/extra/clang-tidy/checks/performance-trivially-destructible.html)

#### performance-type-promotion-in-math-fn [LOW] [(doc)](https://clang.llvm.org/extra/clang-tidy/checks/performance-type-promotion-in-math-fn.html)

#### performance-unnecessary-copy-initialization [LOW] [(doc)](https://clang.llvm.org/extra/clang-tidy/checks/performance-unnecessary-copy-initialization.html)

#### performance-unnecessary-value-param [LOW] [(doc)](https://clang.llvm.org/extra/clang-tidy/checks/performance-unnecessary-value-param.html)

We MAY ignore the warning safely if Ptr, SharedPtr, and ConstSharedPtr are pointed out as follows (which is currently suppressed by the AllowedTypes option in .clang-tidy).

```sh
src/autoware/AutowareArchitectureProposal.iv/planning/scenario_planning/lane_driving/motion_planning/obstacle_stop_planner/src/adaptive_cruise_control.cpp:176:58:
warning: the const qualified parameter 'current_velocity_ptr' is copied for each invocation; consider making it a reference [performance-unnecessary-value-param]
const geometry_msgs::msg::TwistStamped::ConstSharedPtr current_velocity_ptr, bool _ need_to_stop,
^
&
```

### `portability-*`

#### portability-simd-intrinsics [STYLE] [(doc)](https://clang.llvm.org/extra/clang-tidy/checks/portability-simd-intrinsics.html)<!--- cspell:disable-line -->

### `readability-*`

We SHOULD fix the code problems detected by `bugprone-*` rules if these are not false-positives.

#### (readability-avoid-const-params-in-decls) [STYLE] [(doc)](https://clang.llvm.org/extra/clang-tidy/checks/readability-avoid-const-params-in-decls.html)

Disabled.

#### (readability-braces-around-statements) [STYLE] [(doc)](https://clang.llvm.org/extra/clang-tidy/checks/readability-braces-around-statements.html)

Disabled.

#### readability-const-return-type [LOW] [(doc)](https://clang.llvm.org/extra/clang-tidy/checks/readability-const-return-type.html)

#### readability-container-size-empty [STYLE] [(doc)](https://clang.llvm.org/extra/clang-tidy/checks/readability-container-size-empty.html)

#### readability-convert-member-functions-to-static [STYLE] [(doc)](https://clang.llvm.org/extra/clang-tidy/checks/readability-convert-member-functions-to-static.html)

Optional. It depends on the design of the class. We MAY ignore this warning safely.

#### readability-delete-null-pointer [STYLE] [(doc)](https://clang.llvm.org/extra/clang-tidy/checks/readability-delete-null-pointer.html)

#### readability-else-after-return [STYLE] [(doc)](https://clang.llvm.org/extra/clang-tidy/checks/readability-else-after-return.html)

#### readability-function-cognitive-complexity [STYLE] [(doc)](https://clang.llvm.org/extra/clang-tidy/checks/readability-function-cognitive-complexity.html)

#### (readability-function-size) [STYLE] [(doc)](https://clang.llvm.org/extra/clang-tidy/checks/readability-function-size.html)

#### (readability-identifier-length) [STYLE] [(doc)](https://clang.llvm.org/extra/clang-tidy/checks/readability-identifier-length.html)

To be added.

#### (readability-identifier-naming) [STYLE] [(doc)](https://clang.llvm.org/extra/clang-tidy/checks/readability-identifier-naming.html)

To be added.

#### (readability-implicit-bool-conversion) [STYLE] [(doc)](https://clang.llvm.org/extra/clang-tidy/checks/readability-implicit-bool-conversion.html)

#### readability-inconsistent-declaration-parameter-name [STYLE] [(doc)](https://clang.llvm.org/extra/clang-tidy/checks/readability-inconsistent-declaration-parameter-name.html)

#### readability-isolate-declaration [STYLE] [(doc)](https://clang.llvm.org/extra/clang-tidy/checks/readability-isolate-declaration.html)

#### (readability-magic-numbers) [STYLE] [(doc)](https://clang.llvm.org/extra/clang-tidy/checks/readability-magic-numbers.html)

Disabled because of annoyance.

#### readability-make-member-function-const [STYLE] [(doc)](https://clang.llvm.org/extra/clang-tidy/checks/readability-make-member-function-const.html)

MUST.

#### readability-misleading-indentation [LOW] [(doc)](https://clang.llvm.org/extra/clang-tidy/checks/readability-misleading-indentation.html)

MUST.

#### readability-misplaced-array-index [STYLE] [(doc)](https://clang.llvm.org/extra/clang-tidy/checks/readability-misplaced-array-index.html)

MUST.

#### (readability-named-parameter) [STYLE] [(doc)](https://clang.llvm.org/extra/clang-tidy/checks/readability-named-parameter.html)

#### readability-non-const-parameter [STYLE] [(doc)](https://clang.llvm.org/extra/clang-tidy/checks/readability-non-const-parameter.html)

#### (readability-qualified-auto) [LOW] [(doc)](https://clang.llvm.org/extra/clang-tidy/checks/readability-qualified-auto.html)

To be added.

#### readability-redundant-access-specifiers [STYLE] [(doc)](https://clang.llvm.org/extra/clang-tidy/checks/readability-redundant-access-specifiers.html)

#### readability-redundant-control-flow [STYLE] [(doc)](https://clang.llvm.org/extra/clang-tidy/checks/readability-redundant-control-flow.html)

#### readability-redundant-declaration [STYLE] [(doc)](https://clang.llvm.org/extra/clang-tidy/checks/readability-redundant-declaration.html)

#### readability-redundant-function-ptr-dereference [STYLE] [(doc)](https://clang.llvm.org/extra/clang-tidy/checks/readability-redundant-function-ptr-dereference.html)

#### readability-redundant-member-init [STYLE] [(doc)](https://clang.llvm.org/extra/clang-tidy/checks/readability-redundant-member-init.html)

#### (readability-redundant-preprocessor) [STYLE] [(doc)](https://clang.llvm.org/extra/clang-tidy/checks/readability-redundant-preprocessor.html)

#### readability-redundant-smartptr-get [STYLE] [(doc)](https://clang.llvm.org/extra/clang-tidy/checks/readability-redundant-smartptr-get.html)<!--- cspell:disable-line -->

#### readability-redundant-string-cstr [STYLE] [(doc)](https://clang.llvm.org/extra/clang-tidy/checks/readability-redundant-string-cstr.html)<!--- cspell:disable-line -->

#### readability-redundant-string-init [STYLE] [(doc)](https://clang.llvm.org/extra/clang-tidy/checks/readability-redundant-string-cstr.html)

#### readability-simplify-boolean-expr [MEDIUM] [(doc)](https://clang.llvm.org/extra/clang-tidy/checks/readability-simplify-boolean-expr.html)

#### readability-simplify-subscript-expr [STYLE] [(doc)](https://clang.llvm.org/extra/clang-tidy/checks/readability-simplify-subscript-expr.html)

#### readability-static-accessed-through-instance [STYLE] [(doc)](https://clang.llvm.org/extra/clang-tidy/checks/readability-static-accessed-through-instance.html)

#### readability-static-definition-in-anonymous-namespace [STYLE] [(doc)](https://clang.llvm.org/extra/clang-tidy/checks/readability-static-definition-in-anonymous-namespace.html)

#### readability-string-compare [LOW] [(doc)](https://clang.llvm.org/extra/clang-tidy/checks/readability-string-compare.html)

MUST.

#### (readability-suspicious-call-argument) [LOW] [(doc)](https://clang.llvm.org/extra/clang-tidy/checks/readability-suspicious-call-argument.html)

Disabled.

#### readability-uniqueptr-delete-release [STYLE] [(doc)](https://clang.llvm.org/extra/clang-tidy/checks/readability-uniqueptr-delete-release.html)<!--- cspell:disable-line -->

#### (readability-uppercase-literal-suffix) [STYLE] [(doc)](https://clang.llvm.org/extra/clang-tidy/checks/readability-uppercase-literal-suffix.html)

Disabled temporarily. Will be enabled to merge the code to Autoware.Auto.

#### readability-use-anyofallof [STYLE] [(doc)](https://clang.llvm.org/extra/clang-tidy/checks/readability-use-anyofallof.html)<!--- cspell:disable-line -->

Optional. We MAY ignore the warning safely.
