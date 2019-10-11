// Copyright 2019 The Fuchsia Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

package templates

const Header = `
{{- define "Header" -}}
// WARNING: This file is machine generated by fidlgen.

#pragma once

#include "lib/fidl/cpp/fuzzing/traits.h"
#include "lib/fidl/cpp/internal/header.h"

{{ range .Headers -}}
#include <{{ . }}>
{{- end }}
{{ if .LFHeaders -}}
{{ "" }}
{{ range .LFHeaders -}}
#include <{{ . }}>
{{ end -}}
{{ end -}}

namespace fuzzing {

{{ range .Decls }}
{{- if and (NEq .Kind Kinds.Interface) (NEq .Kind Kinds.Const) }}
using {{ .Name }} = {{ .Namespace }}::{{ .Name }};
{{- end }}
{{- end }}

{{ range .Decls }}
{{- if Eq .Kind Kinds.Bits }}{{ template "BitsSizeAndAlloc" . }}{{- end }}
{{- if Eq .Kind Kinds.Enum }}{{ template "EnumSizeAndAlloc" . }}{{- end }}
{{- if Eq .Kind Kinds.Struct }}{{ template "StructSizeAndAlloc" . }}{{- end }}
{{- if Eq .Kind Kinds.Table }}{{ template "TableSizeAndAlloc" . }}{{- end }}
{{- if Eq .Kind Kinds.Union }}{{ template "UnionSizeAndAlloc" . }}{{- end }}
{{- if Eq .Kind Kinds.XUnion }}{{ template "XUnionSizeAndAlloc" . }}{{- end }}
{{- end }}

}  // namespace fuzzing
{{ end }}
`
