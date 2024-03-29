// Copyright 2020 The Fuchsia Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

package templates

const Union = `
{{- define "UnionDefinition" -}}

{{ .Name }} [
       {{- range .Members }}
       {{ .Name }} {{ .Type }}
	   {{- end }}
] {{- if .VarLen -}} [varlen] {{- end -}}

{{- end -}}
`
