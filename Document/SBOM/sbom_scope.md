# M2L31 BSP SBOM Scope and Audit Evidence

The current package consists of separate CycloneDX 1.6 Product and Test Sample
SBOMs. The fixed-link Git artifacts are byte-identical to the Product and Test
Sample files in formal SVN release `V3.02.000-131-gbf6b5193` at revision 39.

## Product SBOM

The Product SBOM covers drivers, middleware, libraries, startup code, boot code,
binaries, and source components under `Library/` that are compiled, linked,
flashed, deployed, or reasonably expected to be integrated into product
firmware. It contains 30 components, including 23 exact file components.

## Test Sample SBOM

The Test Sample SBOM covers `SampleCode/`, `ThirdParty/`, and `Tool/` content
used for examples, demonstrations, validation, host-side utilities, or testing.
These components are not Product Runtime dependencies unless a user explicitly
integrates or redistributes them. It contains 73 components, including 26 exact
file components.

## Evidence

Canonical manual evidence is maintained under `Document/SBOM/components/`.
The Product view contains CMSIS evidence. The Test Sample view contains FatFs,
FreeRTOS, repository binary, and Windows binary evidence. Across these files,
3 vendored-source components close to exactly one SBOM component with source
hash and license evidence. Another 24 manual binary components close by exact
path, SHA-256, and license evidence reference. Total manual closure is 27/27.

All 47 Git-tracked `.a`, `.bin`, `.dll`, `.exe`, and `.lib` files close by exact
repository-relative path and SHA-256. The phase-one formal wrapper checksum list
closes 42 of 42 payload files. Manifest references and hashes resolve, and the
candidate package contains no local absolute paths.

## Validation result

The canonical aggregate audit result is `PASS (0 gaps)`. Both CycloneDX 1.6
documents validate. Both strict checker reports pass with 0 errors and 0
warnings. Binary exact closure is 47/47, manual component closure is 27/27, and
both Grype raw JSON reports pass native schema and metadata validation.

The Grype Product report contains 0 matches and the Test Sample report contains
0 matches. Each report therefore has 0 Critical, High, Medium, Low, Negligible,
and Unknown findings. The raw schema contains `matches`, `source`, `distro`, and
`descriptor`. Both reports record Grype 0.117.0 and database schema v6.1.9,
built `2026-08-31T06:37:31Z`, with `valid: true`.

CycloneDX input can omit identifiers required for vulnerability matching, and
offline or stale databases further limit coverage. Zero matches is not evidence
that the BSP is clean or that the assessment is complete. The raw reports are
preserved without deletion, hiding, or severity downgrade. No Product Security
disposition, approval, or VEX applicability conclusion is asserted; no VEX is
published and no `not_affected` assertion is made.
